// tools/maze_sim の CLI (maze_sim.c) と GUI 用の DLL (sim_api.c) で共有する部分。
// 本当の迷路を作る・読む、壁を観測する、指令を本当の迷路で実行する。
#include "sim_core.h"

#include <stdio.h>
#include <string.h>

const MazePos kSimGoals[MAZE_GOAL_COUNT] = MAZE_GOALS;
const MazePos kSimStart = { MAZE_START_X, MAZE_START_Y };

// 機体の FastBand(app/search_run.c)と同じ並び
typedef struct {
    float v, accel, decel, small_v;
    uint8_t type; // 1 SMALL、2 LARGE
} SimFastBand;
static const SimFastBand kSimBands[] = FAST_BANDS;

int SimCore_FastBandCount(void) {
    return (int)(sizeof(kSimBands) / sizeof(kSimBands[0]));
}

RunProfile SimCore_FastProfile(int band, bool *large) {
    if (band < 1) band = 1;
    if (band > SimCore_FastBandCount()) band = SimCore_FastBandCount();
    const SimFastBand *b = &kSimBands[band - 1];
    // 機体の ComputeFastTurns と同じ(減速度は帯の値)
    RunProfile prof = RunProfile_ForSpeeds(b->v, b->accel, b->small_v, NULL);
    prof.decel = b->decel;
    if (large != NULL) *large = (b->type == 2u);
    return prof;
}

// ------------------------------------------------------------
// 本当の迷路を作る
// ------------------------------------------------------------

// 全部の内壁を既知の「なし」にしてから、壁を立てていく
static void Truth_Clear(WallMap *m) {
    WallMap_Init(m);
    for (uint8_t y = 0; y < MAZE_SIZE; y++) {
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
            MazePos p = { x, y };
            WallMap_SetWall(m, p, DIR_NORTH, false);
            WallMap_SetWall(m, p, DIR_EAST, false);
        }
    }
}

bool SimCore_LoadMazeFile(const char *path, WallMap *m, MazePos *goals, uint8_t *goal_count) {
    enum { LINES = 2 * MAZE_SIZE + 1, WIDTH = 4 * MAZE_SIZE + 1 };
    static char lines[LINES][256];

    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        fprintf(stderr, "cannot open %s\n", path);
        return false;
    }
    int n = 0;
    size_t first_width = 0;
    while (n < LINES && fgets(lines[n], sizeof(lines[n]), fp) != NULL) {
        size_t len = strcspn(lines[n], "\r\n");
        if (n == 0) first_width = len;
        // 足りない分は空白で埋めて、同じ幅の行として読む
        while (len < WIDTH && len + 1 < sizeof(lines[n])) lines[n][len++] = ' ';
        lines[n][len] = '\0';
        n++;
    }
    fclose(fp);
    if (n < LINES) {
        fprintf(stderr, "%s: need %d lines, got %d\n", path, LINES, n);
        return false;
    }
    // 1行目(北の外周)の幅で大きさを確かめる(32×32 のハーフサイズの迷路などを読まないように)
    if (first_width != WIDTH) {
        fprintf(stderr, "%s: not a %dx%d maze (first line has %u chars, expected %d)\n",
                path, MAZE_SIZE, MAZE_SIZE, (unsigned)first_width, WIDTH);
        return false;
    }

    Truth_Clear(m);
    *goal_count = 0;
    for (uint8_t y = 0; y < MAZE_SIZE; y++) {
        int row_north = 2 * (MAZE_SIZE - 1 - y); // この区画の北の壁の行
        int row_cell = row_north + 1;            // この区画の西・東の壁の行
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
            MazePos p = { x, y };
            int col = 4 * x;
            if (lines[row_north][col + 2] == '-') WallMap_SetWall(m, p, DIR_NORTH, true);
            if (lines[row_cell][col] == '|') WallMap_SetWall(m, p, DIR_WEST, true);
            if (lines[row_cell][col + 4] == '|') WallMap_SetWall(m, p, DIR_EAST, true);
            if (y == 0 && lines[2 * MAZE_SIZE][col + 2] == '-') WallMap_SetWall(m, p, DIR_SOUTH, true);
            // 区画の真ん中の 'G' はゴール(入りきらない分は捨てる)
            if (lines[row_cell][col + 2] == 'G' && *goal_count < MAZE_GOAL_MAX) {
                goals[(*goal_count)++] = p;
            }
        }
    }
    return true;
}

// 環境に依らず同じ迷路になるよう、乱数は自前(xorshift32)
static uint32_t s_rng;
static uint32_t Rand(void) {
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

static bool IsStartEastWall(MazePos p, Direction d) {
    MazePos n;
    if (!MazePos_Step(p, d, &n)) return false;
    return (MazePos_Equal(p, kSimStart) && d == DIR_EAST) || (MazePos_Equal(n, kSimStart) && d == DIR_WEST);
}

// 穴掘り法(深さ優先)で全区画がつながった迷路を作り、ところどころ壁を抜いてループも作る。
// 規定どおり、スタート区画の東は壁、ゴールの4区画の間は壁なしにする。
void SimCore_MakeRandomMaze(uint32_t seed, WallMap *m) {
    s_rng = seed * 2654435761u + 1u;
    if (s_rng == 0) s_rng = 1;

    // 全部の壁を立てる
    Truth_Clear(m);
    for (uint8_t y = 0; y < MAZE_SIZE; y++) {
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
            MazePos p = { x, y };
            for (int d = 0; d < 4; d++) WallMap_SetWall(m, p, (Direction)d, true);
        }
    }

    static bool visited[MAZE_SIZE][MAZE_SIZE];
    static MazePos stack[MAZE_CELL_COUNT];
    memset(visited, 0, sizeof(visited));
    int top = 0;
    stack[top++] = kSimStart;
    visited[kSimStart.y][kSimStart.x] = true;

    while (top > 0) {
        MazePos p = stack[top - 1];
        Direction cand[4];
        int nc = 0;
        for (int d = 0; d < 4; d++) {
            MazePos n;
            if (!MazePos_Step(p, (Direction)d, &n)) continue;
            if (visited[n.y][n.x]) continue;
            if (IsStartEastWall(p, (Direction)d)) continue;
            cand[nc++] = (Direction)d;
        }
        if (nc == 0) {
            top--;
            continue;
        }
        Direction d = cand[Rand() % (uint32_t)nc];
        MazePos n;
        MazePos_Step(p, d, &n);
        WallMap_SetWall(m, p, d, false);
        visited[n.y][n.x] = true;
        stack[top++] = n;
    }

    // ループを作る(内壁のおよそ8%を抜く)
    for (uint8_t y = 0; y < MAZE_SIZE; y++) {
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
            MazePos p = { x, y };
            for (int d = 0; d < 2; d++) { // 北と東だけ見れば全部の内壁を1回ずつ見る
                MazePos n;
                if (!MazePos_Step(p, (Direction)d, &n)) continue;
                if (IsStartEastWall(p, (Direction)d)) continue;
                if (Rand() % 100u < 8u) WallMap_SetWall(m, p, (Direction)d, false);
            }
        }
    }

    // ゴールの区画どうしの間は壁なし
    for (int i = 0; i < MAZE_GOAL_COUNT; i++) {
        for (int d = 0; d < 4; d++) {
            MazePos n;
            if (!MazePos_Step(kSimGoals[i], (Direction)d, &n)) continue;
            if (MazePos_InList(n, kSimGoals, MAZE_GOAL_COUNT)) WallMap_SetWall(m, kSimGoals[i], (Direction)d, false);
        }
    }
}

// ------------------------------------------------------------
// 機体を動かす
// ------------------------------------------------------------

WallObservation SimCore_Sense(const WallMap *truth, MazePos p, Direction heading) {
    WallObservation o;
    o.front = WallMap_HasWall(truth, p, heading, WALL_VIEW_KNOWN);
    o.right = WallMap_HasWall(truth, p, Dir_Turn(heading, 1), WALL_VIEW_KNOWN);
    o.left = WallMap_HasWall(truth, p, Dir_Turn(heading, -1), WALL_VIEW_KNOWN);
    return o;
}

// 指令を本当の迷路で実行する。壁にぶつかったらfalse。
bool SimCore_Execute(const WallMap *truth, MazePos *pos, Direction *heading, Action a) {
    *heading = Dir_Turn(*heading, Action_QuarterTurns(a.type));
    for (uint8_t i = 0; i < a.cells; i++) {
        if (WallMap_HasWall(truth, *pos, *heading, WALL_VIEW_KNOWN)) return false;
        MazePos_Step(*pos, *heading, pos);
    }
    return true;
}

void SimCore_PlannerValues(const SearchPlanner *sp, MazeSolver *out) {
    if (sp->phase == SEARCH_PHASE_TO_GOAL) {
        Dijkstra_Compute(out, sp->map, WALL_VIEW_SEARCH, &sp->cost_to_goal, sp->goals, sp->goal_count);
    } else if (sp->phase == SEARCH_PHASE_FULL) {
        // 全面探索: 今の行き先(最短経路になりうる、まだ見ていない区画)へ
        Dijkstra_Compute(out, sp->map, WALL_VIEW_SEARCH, &sp->cost_to_goal, sp->full_targets, sp->full_target_count);
    } else {
        Dijkstra_Compute(out, sp->map, WALL_VIEW_SEARCH, &sp->cost_to_start, &sp->start, 1);
    }
}
