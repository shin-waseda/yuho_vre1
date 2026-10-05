// yuho の迷路 logic 層 (Core/Src/logic/maze/*.c, Core/Src/logic/command.c) を
// PC で動かして確かめるシミュレータ。ファームウェアと同じソースをそのままコンパイルする。
//
// ビルド:  sh tools/maze_sim/build.sh
// 使い方:
//   tools/maze_sim/build/maze_sim --random 1          乱数(シード1)で作った迷路で探索する
//   tools/maze_sim/build/maze_sim maze.txt            テキストの迷路ファイルで探索する
//   tools/maze_sim/build/maze_sim --batch 200         シード1〜200の迷路で続けて試し、結果だけ出す
//   tools/maze_sim/build/maze_sim mazes/alljapan-*.txt  迷路ファイルが2つ以上なら、まとめて試す
//   (大会の迷路は sh tools/maze_sim/fetch_mazes.sh で tools/maze_sim/mazes/ に取ってくる)
// オプション:
//   --algo dijkstra|adachi   探索のアルゴリズム(既定 dijkstra)
//   --goal S,T90,T180,K      探索の行きのDijkstraのコスト(直進,90°旋回,180°旋回,既知区画の上乗せ)
//   --back S,T90,T180,K      探索の帰りのコスト(既定は params.h。K = MAZE_COST_KNOWN_CELL_RETURN)
//   --known-back K           帰りの既知区画の上乗せだけを変える
//   --verbose                1区画ごとに地図を表示する
//   --sizes                  構造体の大きさを表示して終わる
// 探索で見つけた経路の評価(最短走行のコスト)は、常に params.h のコストで計算する。
// 探索のコストを変えても、比べる物差しは変わらない。
//
// 迷路ファイルの形式 (micromouseonline/mazefiles の classic と同じ):
//   北(上)から 2*MAZE_SIZE+1 行。角は 'o' か '+'、横の壁は "---"、縦の壁は '|'。
//
// 流れ: 探索(ゴールへ行ってスタートへ戻る) → 分かった壁だけで最短経路を計算 →
//       その経路を本当の迷路で走らせて、壁にぶつからずゴールに着くか確かめる。

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "logic/command.h"
#include "logic/maze/maze_types.h"
#include "logic/maze/wall_map.h"
#include "logic/maze/dijkstra.h"
#include "logic/maze/priority_queue.h"
#include "logic/maze/search_planner.h"
#include "logic/maze/maze_print.h"
#include "sim_core.h"

#define STEP_LIMIT 5000 // 探索で指令をこれだけ実行しても終わらなければ打ち切る

// 大きな構造体は静的に置く
static WallMap s_truth;     // 本当の迷路(全部の壁が既知)
static WallMap s_map;       // 探索で分かった地図
static SearchPlanner s_planner;
static MazeSolver s_solver;
static CommandList s_route;

// 今の迷路のゴール。迷路ファイルに 'G' があればそれ、なければ params.h の MAZE_GOALS
static MazePos s_goals[MAZE_GOAL_MAX];
static uint8_t s_goal_count;

static void UseDefaultGoals(void) {
    for (uint8_t i = 0; i < MAZE_GOAL_COUNT; i++) s_goals[i] = kSimGoals[i];
    s_goal_count = MAZE_GOAL_COUNT;
}

static bool LoadMaze(const char *path) {
    if (!SimCore_LoadMazeFile(path, &s_truth, s_goals, &s_goal_count)) return false;
    if (s_goal_count == 0) UseDefaultGoals();
    return true;
}

static void MakeRandomMaze(uint32_t seed) {
    SimCore_MakeRandomMaze(seed, &s_truth);
    UseDefaultGoals();
}

// ------------------------------------------------------------
// 1つの迷路で試す
// ------------------------------------------------------------

typedef struct {
    bool ok;
    int moves_to_goal;    // 探索の行き(区画数)
    int moves_to_start;   // 探索の帰り(区画数)
    uint16_t cost_found;  // 探索で分かった壁だけでの最短コスト
    uint16_t cost_best;   // 迷路を全部知っていたときの最短コスト
} Result;

typedef struct {
    SearchAlgo algo;
    bool set_goal_cost;
    bool set_back_cost;
    MazeCost goal_cost;
    MazeCost back_cost;
} SimConfig;

static Result RunOne(const WallMap *truth, const SimConfig *cfg, bool verbose, bool quiet) {
    Result r = { false, 0, 0, MAZE_COST_INF, MAZE_COST_INF };
    SearchAlgo algo = cfg->algo;

    // --- 探索 ---
    WallMap_Init(&s_map);
    SearchPlanner_Init(&s_planner, &s_map, algo, kSimStart, DIR_NORTH, s_goals, s_goal_count);
    if (cfg->set_goal_cost) s_planner.cost_to_goal = cfg->goal_cost;
    if (cfg->set_back_cost) s_planner.cost_to_start = cfg->back_cost;
    MazePos pos = kSimStart;
    Direction heading = DIR_NORTH;

    for (int step = 0; step < STEP_LIMIT; step++) {
        WallObservation obs = SimCore_Sense(truth, pos, heading);
        SearchPhase before = s_planner.phase;
        Action a = SearchPlanner_Step(&s_planner, obs);

        if (a.type == ACTION_STOP) {
            if (s_planner.phase == SEARCH_PHASE_FAILED) {
                if (!quiet) printf("search FAILED at (%u,%u)\n", pos.x, pos.y);
                return r;
            }
            if (s_planner.phase == SEARCH_PHASE_DONE) break;
            if (!quiet && before == SEARCH_PHASE_TO_GOAL) {
                printf("reached goal (%u,%u) after %d moves\n", pos.x, pos.y, r.moves_to_goal);
            }
            continue;
        }

        if (!SimCore_Execute(truth, &pos, &heading, a)) {
            if (!quiet) printf("CRASH: %s at (%u,%u)\n", Action_Name(a.type), pos.x, pos.y);
            return r;
        }
        if (!MazePos_Equal(pos, s_planner.pos) || heading != s_planner.heading) {
            if (!quiet) printf("planner lost track: sim (%u,%u) planner (%u,%u)\n",
                               pos.x, pos.y, s_planner.pos.x, s_planner.pos.y);
            return r;
        }
        if (s_planner.phase == SEARCH_PHASE_TO_GOAL) r.moves_to_goal++;
        else r.moves_to_start++;

        if (verbose) {
            printf("\n#%d %s -> (%u,%u)\n", step, Action_Name(a.type), pos.x, pos.y);
            MazePrint_Map(&s_map, algo == SEARCH_ALGO_DIJKSTRA ? &s_planner.work.solver : NULL,
                          &pos, heading, s_goals, s_goal_count);
        }
    }
    if (s_planner.phase != SEARCH_PHASE_DONE) {
        if (!quiet) printf("search did not finish in %d steps\n", STEP_LIMIT);
        return r;
    }

    // --- 分かった壁だけで最短経路 ---
    Dijkstra_Compute(&s_solver, &s_map, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
    r.cost_found = Dijkstra_Cost(&s_solver, kSimStart, DIR_NORTH);
    if (!quiet) {
        printf("\n=== explored map (cost to goal, known walls only) ===\n");
        MazePrint_Map(&s_map, &s_solver, NULL, DIR_NORTH, s_goals, s_goal_count);
    }
    if (!Dijkstra_BuildRoute(&s_solver, kSimStart, DIR_NORTH, true, &s_route)) {
        if (!quiet) printf("no route on the explored map\n");
        return r;
    }
    if (!quiet) {
        printf("\n=== fastest-run route ===\n");
        CommandList_Print(&s_route);
    }

    // --- その経路を本当の迷路で走らせる ---
    pos = kSimStart;
    heading = DIR_NORTH;
    for (uint16_t i = 0; i < s_route.count; i++) {
        if (!SimCore_Execute(truth, &pos, &heading, s_route.items[i])) {
            if (!quiet) printf("route CRASH at command %u\n", (unsigned)i);
            return r;
        }
    }
    if (!MazePos_InList(pos, s_goals, s_goal_count)) {
        if (!quiet) printf("route ended at (%u,%u), not a goal\n", pos.x, pos.y);
        return r;
    }

    // --- 迷路を全部知っていたときの最短と比べる ---
    Dijkstra_Compute(&s_solver, truth, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
    r.cost_best = Dijkstra_Cost(&s_solver, kSimStart, DIR_NORTH);
    r.ok = true;
    return r;
}

// "S,T90,T180,K" を読む
static bool ParseCost(const char *text, MazeCost *out) {
    unsigned v[4];
    if (sscanf(text, "%u,%u,%u,%u", &v[0], &v[1], &v[2], &v[3]) != 4) {
        fprintf(stderr, "cost must be S,T90,T180,K: %s\n", text);
        return false;
    }
    for (int i = 0; i < 4; i++) {
        if (v[i] > 1000u) {
            fprintf(stderr, "cost too large: %s\n", text);
            return false;
        }
    }
    if (v[0] == 0) {
        fprintf(stderr, "straight cost must be >= 1: %s\n", text);
        return false;
    }
    out->straight = (uint16_t)v[0];
    out->turn90 = (uint16_t)v[1];
    out->turn180 = (uint16_t)v[2];
    out->known_cell = (uint16_t)v[3];
    return true;
}

// 経路のコストが13bit(MazeNodeState.cost)に収まる設定か
static bool CheckCostFits(const char *label, const MazeCost *c) {
    if (MAZE_COST_FITS(c->straight, c->turn90, c->turn180, c->known_cell)) return true;
    fprintf(stderr, "%s cost %u,%u,%u,%u may exceed the 13-bit cost limit (%u)\n", label,
            (unsigned)c->straight, (unsigned)c->turn90, (unsigned)c->turn180,
            (unsigned)c->known_cell, (unsigned)MAZE_COST_INF);
    return false;
}

static void PrintSizes(void) {
    printf("sizeof (on this PC; the STM32 build should match except pointers)\n");
    printf("  WallMap        %5u\n", (unsigned)sizeof(WallMap));
    printf("  MazeNodeState  %5u\n", (unsigned)sizeof(MazeNodeState));
    printf("  MazeSolver     %5u\n", (unsigned)sizeof(MazeSolver));
    printf("  PriorityQueue  %5u  (one shared copy in dijkstra.c)\n", (unsigned)sizeof(PriorityQueue));
    printf("  StepMap        %5u  (+%u shared queue in step_map.c)\n", (unsigned)sizeof(StepMap),
           (unsigned)(MAZE_CELL_COUNT * sizeof(uint16_t)));
    printf("  SearchPlanner  %5u\n", (unsigned)sizeof(SearchPlanner));
    printf("  Action         %5u\n", (unsigned)sizeof(Action));
    printf("  CommandList    %5u\n", (unsigned)sizeof(CommandList));
}

static void PrintCost(const char *label, const MazeCost *c) {
    printf("%s %u,%u,%u,%u", label, (unsigned)c->straight, (unsigned)c->turn90,
           (unsigned)c->turn180, (unsigned)c->known_cell);
}

static void PrintResult(const Result *r) {
    printf("search: %d moves to goal, %d moves back\n", r->moves_to_goal, r->moves_to_start);
    printf("route cost: %u (best possible %u)%s\n", (unsigned)r->cost_found, (unsigned)r->cost_best,
           (r->cost_found == r->cost_best) ? "  = optimal" : "");
}

// まとめて試したときの集計
typedef struct {
    int count;
    int failed;
    int optimal;
    long sum_goal;
    long sum_back;
    double sum_ratio;
} BatchStats;

static void Batch_Add(BatchStats *b, const Result *r, const char *label) {
    b->count++;
    if (!r->ok) {
        printf("%s: FAILED\n", label);
        b->failed++;
        return;
    }
    b->sum_goal += r->moves_to_goal;
    b->sum_back += r->moves_to_start;
    b->sum_ratio += (double)r->cost_found / (double)r->cost_best;
    if (r->cost_found == r->cost_best) b->optimal++;
}

static void Batch_Print(const BatchStats *b, const SimConfig *cfg) {
    int ok = b->count - b->failed;
    printf("%s", cfg->algo == SEARCH_ALGO_ADACHI ? "adachi  " : "dijkstra");
    if (cfg->algo == SEARCH_ALGO_DIJKSTRA) {
        PrintCost(" goal", &cfg->goal_cost);
        PrintCost(" back", &cfg->back_cost);
    }
    printf(" | %d mazes, %d failed, optimal %3d", b->count, b->failed, b->optimal);
    if (ok > 0) {
        printf(", moves %.1f + %.1f = %.1f, cost ratio %.3f",
               (double)b->sum_goal / ok, (double)b->sum_back / ok,
               (double)(b->sum_goal + b->sum_back) / ok, b->sum_ratio / ok);
    }
    printf("\n");
}

int main(int argc, char **argv) {
    // 迷路ファイル(2つ以上ならまとめて試す)
    const char **files = calloc((size_t)argc, sizeof(*files));
    int file_count = 0;
    long seed = -1;
    long batch = 0;
    bool verbose = false;
    SimConfig cfg;
    cfg.algo = SEARCH_ALGO_DIJKSTRA;
    cfg.set_goal_cost = false;
    cfg.set_back_cost = false;
    // 既定は SearchPlanner_Init() と同じ値
    cfg.goal_cost = MazeCost_Default();
    cfg.back_cost = MazeCost_Default();
    cfg.back_cost.known_cell = MAZE_COST_KNOWN_CELL_RETURN;
    if (files == NULL) return 2;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--random") == 0 && i + 1 < argc) {
            seed = strtol(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "--batch") == 0 && i + 1 < argc) {
            batch = strtol(argv[++i], NULL, 10);
        } else if (strcmp(argv[i], "--algo") == 0 && i + 1 < argc) {
            i++;
            if (strcmp(argv[i], "adachi") == 0) cfg.algo = SEARCH_ALGO_ADACHI;
            else if (strcmp(argv[i], "dijkstra") == 0) cfg.algo = SEARCH_ALGO_DIJKSTRA;
            else { fprintf(stderr, "unknown algo: %s\n", argv[i]); return 2; }
        } else if (strcmp(argv[i], "--goal") == 0 && i + 1 < argc) {
            if (!ParseCost(argv[++i], &cfg.goal_cost)) return 2;
            cfg.set_goal_cost = true;
        } else if (strcmp(argv[i], "--back") == 0 && i + 1 < argc) {
            if (!ParseCost(argv[++i], &cfg.back_cost)) return 2;
            cfg.set_back_cost = true;
        } else if (strcmp(argv[i], "--known-back") == 0 && i + 1 < argc) {
            long k = strtol(argv[++i], NULL, 10);
            if (k < 0 || k > 1000) { fprintf(stderr, "bad --known-back: %ld\n", k); return 2; }
            cfg.back_cost.known_cell = (uint16_t)k;
            cfg.set_back_cost = true;
        } else if (strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "--sizes") == 0) {
            PrintSizes();
            return 0;
        } else if (argv[i][0] != '-') {
            files[file_count++] = argv[i];
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            return 2;
        }
    }
    if (!CheckCostFits("goal", &cfg.goal_cost) || !CheckCostFits("back", &cfg.back_cost)) return 2;

    if (batch > 0) {
        BatchStats b = { 0, 0, 0, 0, 0, 0.0 };
        for (long s = 1; s <= batch; s++) {
            char label[32];
            snprintf(label, sizeof(label), "seed %ld", s);
            MakeRandomMaze((uint32_t)s);
            Result r = RunOne(&s_truth, &cfg, false, true);
            Batch_Add(&b, &r, label);
        }
        Batch_Print(&b, &cfg);
        return b.failed == 0 ? 0 : 1;
    }

    if (file_count > 1) {
        BatchStats b = { 0, 0, 0, 0, 0, 0.0 };
        int unreadable = 0;
        for (int i = 0; i < file_count; i++) {
            if (!LoadMaze(files[i])) {
                unreadable++;
                continue;
            }
            Result r = RunOne(&s_truth, &cfg, false, true);
            Batch_Add(&b, &r, files[i]);
        }
        if (unreadable > 0) printf("%d files could not be read\n", unreadable);
        Batch_Print(&b, &cfg);
        return (b.failed == 0 && unreadable == 0) ? 0 : 1;
    }

    if (file_count == 1) {
        if (!LoadMaze(files[0])) return 2;
    } else {
        MakeRandomMaze((uint32_t)(seed >= 0 ? seed : 1));
    }

    printf("=== maze ===\n");
    MazePrint_Map(&s_truth, NULL, &kSimStart, DIR_NORTH, s_goals, s_goal_count);

    Result r = RunOne(&s_truth, &cfg, verbose, false);
    if (!r.ok) return 1;
    printf("\n");
    PrintResult(&r);
    return 0;
}
