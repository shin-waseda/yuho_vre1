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
//   --scope round|oneway|full 探索の行き先(往復・片道・全面。既定 round)
//   --search V,TV,ACCEL      探索の直進の速さ・スラロームの速さ・加速度[mm/s, mm/s, mm/s²](既定は params.h)
//   --pivot                  探索で曲がるとき、スラロームでなく超信地旋回(機体の TURN 2)
//   --band N                 最短走行の速度帯(params.h の FAST_BANDS の N 番目。既定 3)
//   --plant-log DIR          同じ迷路・同じ探索の設定で plant_sim を走らせ、機体と同じ形式のログを DIR に残す
//                            (迷路ファイル1つのときだけ。plant_bridge.h)
//   --plant-exe PATH         plant_sim の実行ファイル(既定: 環境変数 YUHO_PLANT_SIM、tools/maze_sim/plant_sim.local)
//   --plant-no-build         走らせる前に plant_sim をビルドし直さない
//   --plant-opt "..."        plant_sim にそのまま渡すオプション(例 "--no-crash-stop")
//   --verbose                1区画ごとに地図を表示する
//   --sizes                  構造体の大きさを表示して終わる
// 探索で見つけた経路の評価(最短走行のコスト)は、常に params.h のコストで計算する。
// 探索のコストを変えても、比べる物差しは変わらない。
//
// 迷路ファイルの形式 (micromouseonline/mazefiles の classic と同じ):
//   北(上)から 2*MAZE_SIZE+1 行。角は 'o' か '+'、横の壁は "---"、縦の壁は '|'。
//
// 流れ: 探索(ゴールへ行ってスタートへ戻る。全面は最短経路になりうる区画を回ってから戻る。片道はゴールまで) →
//       探索の時間を見積もる(search_time.c。機体の探索の動きを params.h の値でたどる) → 分かった壁だけで最短経路を計算 →
//       その経路を本当の迷路で走らせて、壁にぶつからずゴールに着くか確かめる →
//       最短走行の時間を見積もる(速度帯の速さで。コストが最短の経路を大回りに置き換えたもの、
//       走行時間が最短の経路(TimeDijkstra)、迷路を全部知っていたときの走行時間が最短の経路)。

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
#include "logic/maze/run_path.h"
#include "logic/maze/time_dijkstra.h"
#include "sim_core.h"
#include "search_time.h"
#include "plant_bridge.h"

#define STEP_LIMIT 5000 // 探索で指令をこれだけ実行しても終わらなければ打ち切る

// 大きな構造体は静的に置く
static WallMap s_truth;     // 本当の迷路(全部の壁が既知)
static WallMap s_map;       // 探索で分かった地図
static SearchPlanner s_planner;
static MazeSolver s_solver;
static CommandList s_route;
static RunList s_run;
static TimeSolver s_time_solver;
static CommandList s_run_route; // 最短走行の指令を区画の経路に戻したもの(壁の確認用)

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
    float time_small;     // 見つけた経路を小回りだけで走ったときの時間[s](負なら加速度が足りない)
    float time_large;     // 見つけた経路を大回りに置き換えて走ったときの時間[s](同上)
    float time_opt;       // 時間で選んだ経路(TimeDijkstra)の時間[s](負なら失敗)
    float time_opt_best;  // 迷路を全部知っていたときの、時間で選んだ経路の時間[s]
    float search_s;       // 探索の時間の見積もり[s](スタートの尻当てから、終わりの 180° と待ちまで)
    float goal_s;         // 往復・片道: ゴールに止まった時刻、全面: 最短経路が決まった時刻[s]
    int setpos;           // 尻当ての回数
    int known_runs;       // 既知の区間をまとめて走った回数
} Result;

static void PrintRunList(const char *title, const RunList *list, const float *times) {
    printf("\n=== %s ===\n", title);
    for (uint16_t i = 0; i < list->count; i++) {
        const RunCommand *c = &list->items[i];
        if (c->type == RUN_STRAIGHT) {
            printf("%3u: %-10s x%-3u/2 %6.3f s\n", (unsigned)i, RunType_Name((RunType)c->type),
                   (unsigned)c->halves, (double)times[i]);
        } else {
            printf("%3u: %-10s        %6.3f s\n", (unsigned)i, RunType_Name((RunType)c->type),
                   (double)times[i]);
        }
    }
}

// mapの分かっている壁だけで、時間が最短の最短走行の経路を求めて時間[s]を返す(失敗なら負)。
// 経路は本当の迷路(truth)でも走らせ、壁にぶつからずにゴールで止まるかを確かめる。
static RunProfile s_prof; // 最短走行の速さ(--band の速度帯。main で決める)

static float TimeOptimalRun(const WallMap *map, const WallMap *truth, bool print) {
    RunProfile prof = s_prof;
    TimeDijkstra_Compute(&s_time_solver, map, WALL_VIEW_KNOWN, &prof, s_goals, s_goal_count,
                         kSimStart, DIR_NORTH);
    if (!TimeDijkstra_BuildRun(&s_time_solver, &s_run)) {
        if (print) printf("time-optimal: no route\n");
        return -1.0f;
    }
    float total;
    float times[RUN_LIST_MAX];
    bool ok = RunList_EstimateTime(&s_run, &prof, &total, times, NULL);
    if (print) PrintRunList("fastest-run commands (time-optimal)", &s_run, times);

    // 計算の時間と見積もりの時間が合うか(直進がまとまると見積もりの方が短くなることはある)
    double planned = (double)TimeDijkstra_StartTime(&s_time_solver) * 1e-6;
    if (!ok || (double)total > planned + 1e-3) {
        printf("time-optimal: estimate mismatch (planned %.4f s, estimated %.4f s, feasible %d)\n",
               planned, (double)total, ok);
        return -1.0f;
    }

    // 区画の経路に戻して、本当の迷路で走らせる
    if (!RunList_ToRoute(&s_run, &s_run_route)) {
        printf("time-optimal: could not convert to a cell route\n");
        return -1.0f;
    }
    MazePos pos = kSimStart;
    Direction heading = DIR_NORTH;
    for (uint16_t i = 0; i < s_run_route.count; i++) {
        if (!SimCore_Execute(truth, &pos, &heading, s_run_route.items[i])) {
            printf("time-optimal: route CRASH at cell command %u\n", (unsigned)i);
            return -1.0f;
        }
    }
    if (!MazePos_InList(pos, s_goals, s_goal_count)) {
        printf("time-optimal: route ended at (%u,%u), not a goal\n", pos.x, pos.y);
        return -1.0f;
    }
    return total;
}

typedef struct {
    SearchAlgo algo;
    bool set_goal_cost;
    bool set_back_cost;
    MazeCost goal_cost;
    MazeCost back_cost;
    SearchTimeParams search; // 探索の速さと行き先(search_time.h)
} SimConfig;

static Result RunOne(const WallMap *truth, const SimConfig *cfg, bool verbose, bool quiet) {
    Result r = { false, 0, 0, MAZE_COST_INF, MAZE_COST_INF, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, 0, 0 };
    SearchAlgo algo = cfg->algo;

    // --- 探索 ---
    WallMap_Init(&s_map);
    SearchPlanner_Init(&s_planner, &s_map, algo, kSimStart, DIR_NORTH, s_goals, s_goal_count);
    if (cfg->set_goal_cost) s_planner.cost_to_goal = cfg->goal_cost;
    if (cfg->set_back_cost) s_planner.cost_to_start = cfg->back_cost;
    bool one_way = (cfg->search.scope == SEARCH_TIME_ONE_WAY);
    if (cfg->search.scope == SEARCH_TIME_FULL) SearchPlanner_StartFull(&s_planner);
    SearchTime st;
    SearchTime_Init(&st, &cfg->search);
    SearchTime_Start(&st);
    MazePos pos = kSimStart;
    Direction heading = DIR_NORTH;

    for (int step = 0; step < STEP_LIMIT; step++) {
        WallObservation obs = SimCore_Sense(truth, pos, heading);
        SearchPhase before = s_planner.phase;
        Action a = SearchPlanner_Step(&s_planner, obs);
        SearchTime_Step(&st, &s_planner, obs, a, before);
        if (!quiet && before == SEARCH_PHASE_FULL && s_planner.phase == SEARCH_PHASE_TO_START) {
            printf("shortest route decided at (%u,%u) after %d moves (%.1f s)\n", pos.x, pos.y, r.moves_to_goal,
                   (double)st.t_goal_s);
        }

        if (a.type == ACTION_STOP) {
            if (s_planner.phase == SEARCH_PHASE_FAILED) {
                if (!quiet) printf("search FAILED at (%u,%u)\n", pos.x, pos.y);
                return r;
            }
            if (s_planner.phase == SEARCH_PHASE_DONE) break;
            if (!quiet && before == SEARCH_PHASE_TO_GOAL) {
                printf("reached goal (%u,%u) after %d moves (%.1f s)\n", pos.x, pos.y, r.moves_to_goal,
                       (double)st.t_goal_s);
            }
            if (one_way) break; // 片道: ゴールで止まって終わる
            // 機体はゴールの真ん中で 180° 回ってから帰り始める(app/search_run の SearchLoop)。向きを合わせる
            heading = Dir_Opposite(heading);
            s_planner.heading = heading;
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
        if (s_planner.phase == SEARCH_PHASE_TO_GOAL || s_planner.phase == SEARCH_PHASE_FULL) r.moves_to_goal++;
        else r.moves_to_start++;

        if (verbose) {
            printf("\n#%d %s -> (%u,%u)\n", step, Action_Name(a.type), pos.x, pos.y);
            // プランナーは途中で計算を止めるので、表示する値は全部を計算し直す(s_solver は後でまた計算する)
            if (algo == SEARCH_ALGO_DIJKSTRA) SimCore_PlannerValues(&s_planner, &s_solver);
            MazePrint_Map(&s_map, algo == SEARCH_ALGO_DIJKSTRA ? &s_solver : NULL,
                          &pos, heading, s_goals, s_goal_count);
        }
    }
    bool finished = (s_planner.phase == SEARCH_PHASE_DONE) || (one_way && s_planner.phase == SEARCH_PHASE_TO_START);
    if (!finished) {
        if (!quiet) printf("search did not finish in %d steps\n", STEP_LIMIT);
        return r;
    }
    r.search_s = st.t_s;
    r.goal_s = st.t_goal_s;
    r.setpos = st.n_setpos;
    r.known_runs = st.n_known;
    if (!quiet) {
        printf("search time: %.1f s (straight %.1f, slalom %.1f, pivot %.1f, setpos %.1f x%u, wait %.1f, "
               "known runs %.1f x%u / %u cells)\n",
               (double)st.t_s, (double)st.t_straight, (double)st.t_slalom, (double)st.t_pivot, (double)st.t_setpos,
               st.n_setpos, (double)st.t_goal_wait, (double)st.t_known, st.n_known, st.n_known_cells);
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

    // --- 最短走行の指令に置き換えて、走行時間を見積もる ---
    RunProfile prof = s_prof;
    for (int large = 0; large <= 1; large++) {
        float total;
        float times[RUN_LIST_MAX];
        float *t = large ? &r.time_large : &r.time_small;
        if (!RunPath_FromRoute(&s_route, &prof, large != 0, &s_run)) {
            if (!quiet) printf("could not convert the route to run commands\n");
            continue;
        }
        bool ok = RunList_EstimateTime(&s_run, &prof, &total, times, NULL);
        *t = ok ? total : -1.0f;
        if (!quiet && large) {
            PrintRunList("fastest-run commands (route above, large turns)", &s_run, times);
            if (!ok) printf("(acceleration is not enough somewhere in this route)\n");
        }
    }

    // --- 時間で最短の経路を求める ---
    r.time_opt = TimeOptimalRun(&s_map, truth, !quiet);
    if (r.time_opt < 0.0f) return r;

    // --- 迷路を全部知っていたときの最短と比べる ---
    Dijkstra_Compute(&s_solver, truth, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
    r.cost_best = Dijkstra_Cost(&s_solver, kSimStart, DIR_NORTH);
    r.time_opt_best = TimeOptimalRun(truth, truth, false);
    if (r.time_opt_best < 0.0f) return r;
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
    printf("search time: %.1f s (goal / shortest route decided at %.1f s), setpos %d, known runs %d\n",
           (double)r->search_s, (double)r->goal_s, r->setpos, r->known_runs);
    printf("route cost: %u (best possible %u)%s\n", (unsigned)r->cost_found, (unsigned)r->cost_best,
           (r->cost_found == r->cost_best) ? "  = optimal" : "");
    printf("run time: %.3f s small turns only, %.3f s with large turns (negative = not feasible)\n",
           (double)r->time_small, (double)r->time_large);
    printf("time-optimal run: %.3f s (best possible %.3f s)%s\n", (double)r->time_opt,
           (double)r->time_opt_best, (r->time_opt <= r->time_opt_best + 1e-4f) ? "  = optimal" : "");
}

// まとめて試したときの集計
typedef struct {
    int count;
    int failed;
    int optimal;
    long sum_goal;
    long sum_back;
    double sum_ratio;
    int timed;          // 小回り・大回りとも時間が出た迷路の数
    double sum_small;
    double sum_large;
    double sum_opt;      // 時間で選んだ経路(timed の迷路だけ)
    double sum_opt_best;
    int opt_optimal;     // 時間で選んだ経路が、全部知っていたときと同じ時間だった迷路の数
    double sum_search;   // 探索の時間の見積もり
    double sum_goal_s;   // ゴールに止まった(全面: 最短経路が決まった)時刻
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
    b->sum_search += r->search_s;
    b->sum_goal_s += r->goal_s;
    if (r->cost_found == r->cost_best) b->optimal++;
    if (r->time_small > 0.0f && r->time_large > 0.0f) {
        b->timed++;
        b->sum_small += r->time_small;
        b->sum_large += r->time_large;
        b->sum_opt += r->time_opt;
        b->sum_opt_best += r->time_opt_best;
    }
    if (r->time_opt <= r->time_opt_best + 1e-4f) b->opt_optimal++;
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
        printf(", moves %.1f + %.1f = %.1f, search %.1f s (goal %.1f s), cost ratio %.3f",
               (double)b->sum_goal / ok, (double)b->sum_back / ok,
               (double)(b->sum_goal + b->sum_back) / ok, b->sum_search / ok, b->sum_goal_s / ok, b->sum_ratio / ok);
    }
    if (b->timed > 0) {
        printf(", run time %.3f s (small) %.3f s (large) %.3f s (time-opt, best %.3f s, optimal %d)",
               b->sum_small / b->timed, b->sum_large / b->timed,
               b->sum_opt / b->timed, b->sum_opt_best / b->timed, b->opt_optimal);
        if (b->timed < ok) printf(" [%d not feasible]", ok - b->timed);
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
    cfg.search = SearchTime_DefaultParams();
    int band = SIM_DEFAULT_BAND;
    const char *plant_dir = NULL;
    PlantOptions plant = { NULL, true, NULL, argv[0] };
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
        } else if (strcmp(argv[i], "--scope") == 0 && i + 1 < argc) {
            i++;
            if (strcmp(argv[i], "round") == 0) cfg.search.scope = SEARCH_TIME_ROUND;
            else if (strcmp(argv[i], "oneway") == 0) cfg.search.scope = SEARCH_TIME_ONE_WAY;
            else if (strcmp(argv[i], "full") == 0) cfg.search.scope = SEARCH_TIME_FULL;
            else { fprintf(stderr, "unknown scope: %s\n", argv[i]); return 2; }
        } else if (strcmp(argv[i], "--search") == 0 && i + 1 < argc) {
            float v, tv, acc;
            if (sscanf(argv[++i], "%f,%f,%f", &v, &tv, &acc) != 3 || v <= 0.0f || tv <= 0.0f || acc <= 0.0f) {
                fprintf(stderr, "--search must be V,TURN_V,ACCEL: %s\n", argv[i]);
                return 2;
            }
            cfg.search.v_mm_s = v;
            cfg.search.turn_v_mm_s = tv;
            cfg.search.accel_mm_s2 = acc;
        } else if (strcmp(argv[i], "--pivot") == 0) {
            cfg.search.slalom = false;
        } else if (strcmp(argv[i], "--band") == 0 && i + 1 < argc) {
            band = (int)strtol(argv[++i], NULL, 10);
            if (band < 1 || band > SimCore_FastBandCount()) {
                fprintf(stderr, "--band must be 1..%d\n", SimCore_FastBandCount());
                return 2;
            }
        } else if (strcmp(argv[i], "--plant-log") == 0 && i + 1 < argc) {
            plant_dir = argv[++i];
        } else if (strcmp(argv[i], "--plant-exe") == 0 && i + 1 < argc) {
            plant.exe = argv[++i];
        } else if (strcmp(argv[i], "--plant-no-build") == 0) {
            plant.build = false;
        } else if (strcmp(argv[i], "--plant-opt") == 0 && i + 1 < argc) {
            plant.extra = argv[++i];
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
    s_prof = SimCore_FastProfile(band, NULL);
    {
        static const char *const kScope[] = { "", "round", "oneway", "full" };
        printf("search: %s, v %.0f / slalom %.0f mm/s, accel %.0f, %s | fast run: band %d "
               "(v %.0f, accel %.0f / %.0f, small turn %.0f mm/s)\n",
               kScope[cfg.search.scope], (double)cfg.search.v_mm_s, (double)cfg.search.turn_v_mm_s,
               (double)cfg.search.accel_mm_s2, cfg.search.slalom ? "slalom" : "pivot", band, (double)s_prof.vmax,
               (double)s_prof.accel, (double)s_prof.decel, (double)s_prof.v_turn[RUN_SMALL90_R]);
    }

    if (plant_dir != NULL && (batch > 0 || file_count != 1)) {
        fprintf(stderr, "--plant-log needs exactly one maze file (plant_sim reads the same file)\n");
        return 2;
    }

    if (batch > 0) {
        BatchStats b = { 0 };
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
        BatchStats b = { 0 };
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
    if (r.ok) {
        printf("\n");
        PrintResult(&r);
    }

    if (plant_dir != NULL) {
        // plant_sim は機体のファームウェアそのものなので、ゴールは params.h の MAZE_GOALS、コストは params.h の値
        printf("\n=== plant_sim ===\n");
        bool same_goals = (s_goal_count == MAZE_GOAL_COUNT);
        for (uint8_t i = 0; same_goals && i < s_goal_count; i++) {
            same_goals = MazePos_InList(s_goals[i], kSimGoals, MAZE_GOAL_COUNT);
        }
        if (!same_goals) printf("warning: the maze file's goals differ from params.h MAZE_GOALS (plant_sim uses MAZE_GOALS)\n");
        if (cfg.set_goal_cost || cfg.set_back_cost) {
            printf("warning: --goal / --back are not used by plant_sim (the firmware uses params.h)\n");
        }
        float est = (r.search_s > 0.0f) ? r.search_s : 300.0f;
        if (!PlantLog_Run(&plant, files[0], &cfg.search, cfg.algo, est, plant_dir, NULL, 0)) return 1;
    }
    return r.ok ? 0 : 1;
}
