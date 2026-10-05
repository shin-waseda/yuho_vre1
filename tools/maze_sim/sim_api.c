// GUI (tools/maze_sim/gui.py) から ctypes で呼ぶための DLL の窓口。
// 探索の判断はファームウェアと同じ logic 層 (SearchPlanner) が行い、
// 本当の迷路での観測・移動は sim_core.c が行う。Python は描画と操作だけを受け持つ。
//
// 状態はこのファイルの static に1組だけ持つ(GUIは1つの迷路しか扱わない)。
// Python との受け渡しは整数とバイト列だけにして、構造体の並びに依存しないようにする。

#include <stddef.h>

#include "sim_core.h"
#include "logic/maze/dijkstra.h"
#include "logic/maze/step_map.h"
#include "logic/maze/search_planner.h"

#ifdef _WIN32
#define SIM_EXPORT __declspec(dllexport)
#else
#define SIM_EXPORT __attribute__((visibility("default")))
#endif

// sim_step() の戻り値
enum {
    SIM_MOVED = 0,     // 指令を1つ実行した
    SIM_AT_GOAL = 1,   // ゴールに着いて止まった(次の sim_step から帰り始める)
    SIM_DONE = 2,      // スタートに戻って探索が終わった
    SIM_FAILED = 3,    // 目的地へ行けない(分かっている壁でふさがれた)
    SIM_CRASH = 4,     // 指令どおりに動くと壁にぶつかる(logic層のバグ)
    SIM_LOST = 5,      // プランナーの位置とシミュレータの位置がずれた(logic層のバグ)
};

// sim_wall() の戻り値
enum { SIM_WALL_OPEN = 0, SIM_WALL_EXISTS = 1, SIM_WALL_UNKNOWN = 2 };

static WallMap s_truth;
static WallMap s_map;
static SearchPlanner s_planner;
static MazeSolver s_solver; // 最短経路の計算用(プランナーの作業領域とは別)
static CommandList s_route;

// 今の迷路のゴール。迷路ファイルに 'G' があればそれ、なければ params.h の MAZE_GOALS
static MazePos s_goals[MAZE_GOAL_MAX];
static uint8_t s_goal_count;

static void UseDefaultGoals(void) {
    for (uint8_t i = 0; i < MAZE_GOAL_COUNT; i++) s_goals[i] = kSimGoals[i];
    s_goal_count = MAZE_GOAL_COUNT;
}

static MazePos s_pos;
static Direction s_heading;
static int s_moves_to_goal;
static int s_moves_back;
static int s_status;
static SearchAlgo s_algo;
static bool s_has_values; // リセット後、プランナーが一度でも経路を計算したか(区画の値の表示用)

static MazeCost CostFromArray(const uint16_t *c, MazeCost fallback) {
    if (c == NULL) return fallback;
    MazeCost m = { c[0], c[1], c[2], c[3] };
    return m;
}

SIM_EXPORT int sim_maze_size(void) {
    return MAZE_SIZE;
}

SIM_EXPORT int sim_goals(uint8_t *xs, uint8_t *ys, int max) {
    if (s_goal_count == 0) UseDefaultGoals(); // まだ迷路を読んでいない
    int n = (s_goal_count < max) ? s_goal_count : max;
    for (int i = 0; i < n; i++) {
        xs[i] = s_goals[i].x;
        ys[i] = s_goals[i].y;
    }
    return n;
}

SIM_EXPORT void sim_start(uint8_t *x, uint8_t *y) {
    *x = kSimStart.x;
    *y = kSimStart.y;
}

SIM_EXPORT void sim_new_random(uint32_t seed) {
    SimCore_MakeRandomMaze(seed, &s_truth);
    UseDefaultGoals();
}

// 成功で1、失敗で0(32×32 など MAZE_SIZE でない迷路も失敗)。ゴールは迷路ファイルの 'G'。
SIM_EXPORT int sim_load_file(const char *path) {
    if (!SimCore_LoadMazeFile(path, &s_truth, s_goals, &s_goal_count)) return 0;
    if (s_goal_count == 0) UseDefaultGoals();
    return 1;
}

// 探索を最初からやり直す。costはそれぞれ {直進, 90°, 180°, 既知区画} の4要素(NULLなら既定値)。
// 13bitに収まらないコストなら0を返して何もしない。
SIM_EXPORT int sim_reset(int algo, const uint16_t *goal_cost, const uint16_t *back_cost) {
    s_algo = (algo == 1) ? SEARCH_ALGO_ADACHI : SEARCH_ALGO_DIJKSTRA;

    MazeCost back_default = MazeCost_Default();
    back_default.known_cell = MAZE_COST_KNOWN_CELL_RETURN;
    MazeCost to_goal = CostFromArray(goal_cost, MazeCost_Default());
    MazeCost to_start = CostFromArray(back_cost, back_default);
    if (to_goal.straight == 0 || to_start.straight == 0) return 0;
    if (!MAZE_COST_FITS(to_goal.straight, to_goal.turn90, to_goal.turn180, to_goal.known_cell)) return 0;
    if (!MAZE_COST_FITS(to_start.straight, to_start.turn90, to_start.turn180, to_start.known_cell)) return 0;

    WallMap_Init(&s_map);
    SearchPlanner_Init(&s_planner, &s_map, s_algo, kSimStart, DIR_NORTH, s_goals, s_goal_count);
    s_planner.cost_to_goal = to_goal;
    s_planner.cost_to_start = to_start;

    s_pos = kSimStart;
    s_heading = DIR_NORTH;
    s_moves_to_goal = 0;
    s_moves_back = 0;
    s_status = SIM_MOVED;
    s_has_values = false;
    return 1;
}

// 今の区画の壁を観測してプランナーに渡し、返ってきた指令を本当の迷路で実行する。
// 実行した指令を type/cells に書く(止まったときは ACTION_STOP)。
SIM_EXPORT int sim_step(uint8_t *type, uint8_t *cells) {
    *type = ACTION_STOP;
    *cells = 0;
    if (s_status == SIM_DONE || s_status == SIM_FAILED || s_status == SIM_CRASH || s_status == SIM_LOST) {
        return s_status;
    }

    WallObservation obs = SimCore_Sense(&s_truth, s_pos, s_heading);
    Action a = SearchPlanner_Step(&s_planner, obs);
    s_has_values = true;
    *type = a.type;
    *cells = a.cells;

    if (a.type == ACTION_STOP) {
        switch (s_planner.phase) {
            case SEARCH_PHASE_DONE:     s_status = SIM_DONE; break;
            case SEARCH_PHASE_FAILED:   s_status = SIM_FAILED; break;
            case SEARCH_PHASE_TO_START: s_status = SIM_AT_GOAL; break;
            default:                    s_status = SIM_MOVED; break;
        }
        return s_status;
    }

    if (!SimCore_Execute(&s_truth, &s_pos, &s_heading, a)) {
        s_status = SIM_CRASH;
        return s_status;
    }
    if (!MazePos_Equal(s_pos, s_planner.pos) || s_heading != s_planner.heading) {
        s_status = SIM_LOST;
        return s_status;
    }
    if (s_planner.phase == SEARCH_PHASE_TO_GOAL) s_moves_to_goal++;
    else s_moves_back++;
    s_status = SIM_MOVED;
    return s_status;
}

SIM_EXPORT void sim_pose(uint8_t *x, uint8_t *y, uint8_t *dir) {
    *x = s_pos.x;
    *y = s_pos.y;
    *dir = (uint8_t)s_heading;
}

// 0: ゴールへ向かう 1: スタートへ戻る 2: 終わり 3: 失敗(SearchPhaseと同じ)
SIM_EXPORT int sim_phase(void) {
    return (int)s_planner.phase;
}

SIM_EXPORT void sim_moves(int *to_goal, int *back) {
    *to_goal = s_moves_to_goal;
    *back = s_moves_back;
}

// which: 0 = 本当の迷路, 1 = 探索で分かった地図
SIM_EXPORT int sim_wall(int which, int x, int y, int dir) {
    const WallMap *m = (which == 0) ? &s_truth : &s_map;
    MazePos p = { (uint8_t)x, (uint8_t)y };
    if (!WallMap_IsKnown(m, p, (Direction)dir)) return SIM_WALL_UNKNOWN;
    return WallMap_HasWall(m, p, (Direction)dir, WALL_VIEW_KNOWN) ? SIM_WALL_EXISTS : SIM_WALL_OPEN;
}

// 区画の4方向の壁がすべて分かっているか(探索済みの表示用)
SIM_EXPORT int sim_cell_known(int x, int y) {
    MazePos p = { (uint8_t)x, (uint8_t)y };
    return WallMap_IsCellKnown(&s_map, p) ? 1 : 0;
}

// プランナーが最後に計算した、その区画の値(Dijkstraはゴールまでの最小コスト、足立法は歩数)。
// 行けない・まだ計算していなければ 0xFFFF。
SIM_EXPORT uint16_t sim_cell_value(int x, int y) {
    if (!s_has_values) return 0xFFFFu;
    MazePos p = { (uint8_t)x, (uint8_t)y };
    if (s_algo == SEARCH_ALGO_ADACHI) {
        return s_planner.work.step.step[p.y][p.x];
    }
    uint16_t best = MAZE_COST_INF;
    for (int d = 0; d < 4; d++) {
        uint16_t c = Dijkstra_Cost(&s_planner.work.solver, p, (Direction)d);
        if (c < best) best = c;
    }
    return (best == MAZE_COST_INF) ? 0xFFFFu : best;
}

// 探索で分かった壁だけで最短経路を計算する(最短走行のコスト = params.h)。
// 指令を types/cells に最大 max 個書いて個数を返す(行けなければ0)。
// cost_found: その経路のコスト、cost_best: 迷路を全部知っていたときの最短のコスト。
SIM_EXPORT int sim_route(uint8_t *types, uint8_t *cells, int max, uint16_t *cost_found, uint16_t *cost_best) {
    Dijkstra_Compute(&s_solver, &s_truth, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
    *cost_best = Dijkstra_Cost(&s_solver, kSimStart, DIR_NORTH);

    Dijkstra_Compute(&s_solver, &s_map, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
    *cost_found = Dijkstra_Cost(&s_solver, kSimStart, DIR_NORTH);
    if (!Dijkstra_BuildRoute(&s_solver, kSimStart, DIR_NORTH, true, &s_route)) return 0;

    int n = (s_route.count < max) ? s_route.count : max;
    for (int i = 0; i < n; i++) {
        types[i] = s_route.items[i].type;
        cells[i] = s_route.items[i].cells;
    }
    return n;
}
