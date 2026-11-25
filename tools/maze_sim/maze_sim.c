// yuho の迷路 logic 層 (lib/mouse_core/Src/logic/maze/*.c, lib/mouse_core/Src/logic/command.c) を
// PC で動かして確かめるシミュレータ。ファームウェアと同じソースをそのままコンパイルする。
//
// ビルド:  sh tools/maze_sim/build.sh
// 使い方:
//   tools/maze_sim/build/maze_sim --random 1          乱数(シード1)で作った迷路で探索する
//   tools/maze_sim/build/maze_sim maze.txt            テキストの迷路ファイルで探索する
//   tools/maze_sim/build/maze_sim --batch 200         シード1〜200の迷路で続けて試し、結果だけ出す
//   tools/maze_sim/build/maze_sim ~/.mazesim/mazefiles/alljapan-*.txt  迷路ファイルが2つ以上なら、まとめて試す
//   (迷路は mouse maze sim のもの。大会の迷路は python tools/maze_sim/msim.py fetch で ~/.mazesim/mazefiles に取ってくる。
//    同梱の公式図の迷路は python tools/maze_sim/msim.py list --paths alljapan などでパスが出る)
// オプション:
//   --algo dijkstra|adachi   探索のアルゴリズム(既定 dijkstra)
//   --goal S,T90,T180,K      探索の行きのDijkstraのコスト(直進,90°旋回,180°旋回,既知区画の上乗せ)
//   --back S,T90,T180,K      探索の帰りのコスト(既定は params.h。K = MAZE_COST_KNOWN_CELL_RETURN)
//   --known-back K           帰りの既知区画の上乗せだけを変える
//   --scope round|oneway|full 探索の行き先(往復・片道・全面。既定 round)
//   --search V,TV,ACCEL      探索の速さ: 直進の最高速度・小回りの速さ・加速度[mm/s, mm/s, mm/s²](既定は params.h)
//   --run VMAX,ACCEL,SMALL90,LARGE90,LARGE180  最短走行の速さ(既定は params.h の RUN_*)
//   --full-diag              全面探索で確かめる最短経路を、斜めのコスト(--diag-cost)で選ぶ(遠回りの長い斜めも見に行く)
//   --diag-cost N            斜めの経路を選ぶときの、ジグザグを続ける1区画のコスト(直進の1区画を 10 とした値。既定は params.h の MAZE_COST_DIAG_CELL_X10)
//   --verbose                1区画ごとに地図を表示する
//   --sizes                  構造体の大きさを表示して終わる
// 探索で見つけた経路の評価(最短走行のコスト)は、常に params.h のコストで計算する。
// 探索のコストを変えても、比べる物差しは変わらない。
// 時間は計算上の速さの運動だけで数える(滑りは考えない。ideal_time.h。mouse maze sim の yuho と同じ)。
//
// 迷路ファイルの形式 (micromouseonline/mazefiles の classic と同じ):
//   北(上)から 2*MAZE_SIZE+1 行。角は 'o' か '+'、横の壁は "---"、縦の壁は '|'。
//
// 流れ: 探索(ゴールへ行ってスタートへ戻る。全面は最短経路になりうる区画を回ってから戻る。片道はゴールまで) →
//       探索の時間を数える(ideal_time.c。運動だけ) → 分かった壁だけで最短経路を計算 →
//       その経路を本当の迷路で走らせて、壁にぶつからずゴールに着くか確かめる →
//       最短走行の時間を数える(--run の速さで。コストが最短の経路を大回りに置き換えたもの、
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
#include "logic/maze/diag_route.h"
#include "sim_core.h"
#include "ideal_time.h"

// 探索の行き先
typedef enum { SCOPE_ROUND, SCOPE_ONE_WAY, SCOPE_FULL } Scope;

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
static DiagRouteSolver s_diag_solver; // 斜めのコストで区画の経路を選ぶ
static CommandList s_diag_route;
static uint16_t s_diag_cost = MAZE_COST_DIAG_CELL_X10; // --diag-cost
static bool s_full_diag = false;                       // --full-diag
static DiagRouteSolver s_full_diag_solver;

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
    float time_pivot;     // 見つけた経路を、止まって超信地旋回で走ったときの時間[s](回る時間は数えない)
    float time_small;     // 見つけた経路を小回りだけで走ったときの時間[s](負なら加速度が足りない)
    float time_large;     // 見つけた経路を大回りに置き換えて走ったときの時間[s](同上)
    float time_diag;      // 斜めのコストで選び直した経路を大回りと斜めに置き換えて走ったときの時間[s](同上。本当の迷路でぶつかったら -2)
    int diag_sections;    // その経路の斜めの区間の数
    float time_opt;       // 時間で選んだ経路(TimeDijkstra)の時間[s](負なら失敗)
    float time_opt_best;  // 迷路を全部知っていたときの、時間で選んだ経路の時間[s]
    float time_opt_diag;  // 時間で選んだ経路で斜めも使ったときの時間[s](負なら失敗)
    float time_opt_diag_best; // 迷路を全部知っていたときの、その時間[s]
    float search_s;       // 探索の時間[s](運動だけ。ideal_time.h)
    float goal_s;         // 往復・片道: ゴールに止まった時刻、全面: 最短経路が決まった時刻[s]
    int known_runs;       // 既知の区間をまとめて走った回数
    int known_cells;      // まとめて走った区画の数
} Result;

static void PrintRunList(const char *title, const RunList *list, const float *times) {
    printf("\n=== %s ===\n", title);
    for (uint16_t i = 0; i < list->count; i++) {
        const RunCommand *c = &list->items[i];
        if (c->type == RUN_STRAIGHT) {
            printf("%3u: %-10s x%-3u/2 %6.3f s\n", (unsigned)i, RunType_Name((RunType)c->type),
                   (unsigned)c->halves, (double)times[i]);
        } else if (c->type == RUN_DIAG) {
            printf("%3u: %-10s x%-5u %6.3f s\n", (unsigned)i, RunType_Name((RunType)c->type),
                   (unsigned)c->halves, (double)times[i]);
        } else {
            printf("%3u: %-10s        %6.3f s\n", (unsigned)i, RunType_Name((RunType)c->type),
                   (double)times[i]);
        }
    }
}

// 最短走行の指令の列を区画の経路に戻して本当の迷路(truth)で走らせ、壁にぶつからずにゴールで止まるかを確かめる
static bool RunOnTruth(const RunList *list, const WallMap *truth, const char *label) {
    if (!RunList_ToRoute(list, &s_run_route)) {
        printf("%s: could not convert to a cell route\n", label);
        return false;
    }
    MazePos pos = kSimStart;
    Direction heading = DIR_NORTH;
    for (uint16_t i = 0; i < s_run_route.count; i++) {
        if (!SimCore_Execute(truth, &pos, &heading, s_run_route.items[i])) {
            printf("%s: route CRASH at cell command %u\n", label, (unsigned)i);
            return false;
        }
    }
    if (!MazePos_InList(pos, s_goals, s_goal_count)) {
        printf("%s: route ended at (%u,%u), not a goal\n", label, pos.x, pos.y);
        return false;
    }
    return true;
}

// mapの分かっている壁だけで、時間が最短の最短走行の経路を求めて時間[s]を返す(失敗なら負)。
// 経路は本当の迷路(truth)でも走らせ、壁にぶつからずにゴールで止まるかを確かめる。
static RunProfile s_prof; // 最短走行の速さ(--run。main で決める)

static float TimeOptimalRun(const WallMap *map, const WallMap *truth, bool print, bool use_diag) {
    RunProfile prof = s_prof;
    TimeDijkstra_ComputeEx(&s_time_solver, map, WALL_VIEW_KNOWN, &prof, s_goals, s_goal_count,
                           kSimStart, DIR_NORTH, use_diag);
    if (!TimeDijkstra_BuildRun(&s_time_solver, &s_run)) {
        if (print) printf("time-optimal: no route\n");
        return -1.0f;
    }
    float total;
    float times[RUN_LIST_MAX];
    bool ok = RunList_EstimateTime(&s_run, &prof, &total, times, NULL);
    if (print) PrintRunList(use_diag ? "fastest-run commands (time-optimal with diagonal)" : "fastest-run commands (time-optimal)",
                            &s_run, times);

    // 計算の時間と見積もりの時間が合うか(直進がまとまると見積もりの方が短くなることはある)
    double planned = (double)TimeDijkstra_StartTime(&s_time_solver) * 1e-6;
    if (!ok || (double)total > planned + 1e-3) {
        printf("time-optimal: estimate mismatch (planned %.4f s, estimated %.4f s, feasible %d)\n",
               planned, (double)total, ok);
        return -1.0f;
    }

    if (!RunOnTruth(&s_run, truth, "time-optimal")) return -1.0f;
    return total;
}

typedef struct {
    SearchAlgo algo;
    bool set_goal_cost;
    bool set_back_cost;
    MazeCost goal_cost;
    MazeCost back_cost;
    Scope scope;             // 探索の行き先
    SearchSpeeds search;     // 探索の速さ(ideal_time.h)
} SimConfig;

static Result RunOne(const WallMap *truth, const SimConfig *cfg, bool verbose, bool quiet) {
    Result r = { false, 0, 0, MAZE_COST_INF, MAZE_COST_INF, -1.0f, -1.0f, -1.0f, -1.0f, 0, -1.0f, -1.0f, -1.0f, -1.0f,
                 -1.0f, -1.0f, 0, 0 };
    SearchAlgo algo = cfg->algo;

    // --- 探索 ---
    WallMap_Init(&s_map);
    SearchPlanner_Init(&s_planner, &s_map, algo, kSimStart, DIR_NORTH, s_goals, s_goal_count);
    if (cfg->set_goal_cost) s_planner.cost_to_goal = cfg->goal_cost;
    if (cfg->set_back_cost) s_planner.cost_to_start = cfg->back_cost;
    bool one_way = (cfg->scope == SCOPE_ONE_WAY);
    if (cfg->scope == SCOPE_FULL) {
        SearchPlanner_StartFull(&s_planner);
        if (s_full_diag) {
            s_planner.full_diag = &s_full_diag_solver;
            s_planner.full_diag_cell_x10 = s_diag_cost;
        }
    }
    // 往復は機体と同じく、ゴールで止まって 180° 回らず、ゴールの区画を全部回ってから戻る(app/search_run の RunSearch)
    s_planner.goal_tour = (cfg->scope == SCOPE_ROUND);
    SearchTime st;
    SearchTime_Init(&st, &cfg->search);
    MazePos pos = kSimStart;
    Direction heading = DIR_NORTH;
    static Direction known[MAZE_CELL_COUNT]; // 既知の区間で C1 から先に進む向き(SearchTime_KnownRun)

    for (int step = 0; step < STEP_LIMIT; step++) {
        WallObservation obs = SimCore_Sense(truth, pos, heading);
        SearchPhase before = s_planner.phase;
        Action a = SearchPlanner_Step(&s_planner, obs);
        // 機体と同じく、先の区画の壁が分かっていればまとめて走る(プランナーの位置は区間の終わりへ進む)
        uint16_t km = SearchTime_KnownRun(&st, &s_planner, a, before, s_planner.phase, known);
        if (km == 0u) SearchTime_Step(&st, a, before, s_planner.phase);
        if (!quiet && before == SEARCH_PHASE_FULL && s_planner.phase == SEARCH_PHASE_TO_START) {
            printf("shortest route decided at (%u,%u) after %d moves (%.1f s)\n", pos.x, pos.y, r.moves_to_goal,
                   (double)st.t_goal_s);
        }
        if (!quiet && before == SEARCH_PHASE_TO_GOAL && s_planner.phase == SEARCH_PHASE_GOAL_TOUR) {
            printf("reached goal (%u,%u) after %d moves (%.1f s), touring the goal cells\n", pos.x, pos.y,
                   r.moves_to_goal, (double)st.t_goal_s);
        }
        if (!quiet && before == SEARCH_PHASE_TO_GOAL &&
            (s_planner.phase == SEARCH_PHASE_FULL || (cfg->scope == SCOPE_FULL && s_planner.phase == SEARCH_PHASE_TO_START))) {
            printf("reached goal (%u,%u) after %d moves (%.1f s), starting the full search\n", pos.x, pos.y,
                   r.moves_to_goal, (double)st.t_s);
        }
        if (!quiet && before == SEARCH_PHASE_GOAL_TOUR && s_planner.phase == SEARCH_PHASE_TO_START) {
            printf("goal tour done at (%u,%u), back to start without stopping\n", pos.x, pos.y);
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
        // 既知の区間: 残りの区画も本当の迷路で進む(分かっている壁だけを通るので、ぶつかれば logic 層のバグ)
        for (uint16_t k = 1; k <= km; k++) {
            if (!SimCore_Execute(truth, &pos, &heading, Action_Move((int)known[k] - (int)heading))) {
                if (!quiet) printf("CRASH: known-section run at (%u,%u)\n", pos.x, pos.y);
                return r;
            }
        }
        if (!MazePos_Equal(pos, s_planner.pos) || heading != s_planner.heading) {
            if (!quiet) printf("planner lost track: sim (%u,%u) planner (%u,%u)\n",
                               pos.x, pos.y, s_planner.pos.x, s_planner.pos.y);
            return r;
        }
        int cells = 1 + (int)km;
        if (s_planner.phase == SEARCH_PHASE_TO_GOAL || s_planner.phase == SEARCH_PHASE_FULL) r.moves_to_goal += cells;
        else r.moves_to_start += cells;

        if (verbose) {
            if (km > 0u) printf("\n#%d KNOWN x%d -> (%u,%u)\n", step, cells, pos.x, pos.y);
            else printf("\n#%d %s -> (%u,%u)\n", step, Action_Name(a.type), pos.x, pos.y);
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
    r.known_runs = st.known_runs;
    r.known_cells = st.known_cells;
    if (!quiet) printf("search time: %.1f s\n", (double)st.t_s);

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

    // --- 同じ経路を、曲がる区画の真ん中で止まって超信地旋回する(時間は直進だけ) ---
    if (RunPath_FromRoutePivot(&s_route, &s_run)) {
        float total;
        r.time_pivot = RunList_EstimateTime(&s_run, &prof, &total, NULL, NULL) ? total : -1.0f;
    }

    // --- 斜めのコスト(--diag-cost)で区画の経路を選び直し、大回りと斜めに置き換える(本当の迷路でぶつからないかも確かめる) ---
    DiagRoute_Compute(&s_diag_solver, &s_map, WALL_VIEW_KNOWN, NULL, s_diag_cost, s_goals, s_goal_count);
    if (DiagRoute_BuildRoute(&s_diag_solver, kSimStart, DIR_NORTH, &s_diag_route) &&
        RunPath_FromRouteEx(&s_diag_route, &prof, true, true, &s_run)) {
        float total;
        float times[RUN_LIST_MAX];
        bool ok = RunList_EstimateTime(&s_run, &prof, &total, times, NULL);
        r.time_diag = ok ? total : -1.0f;
        for (uint16_t i = 0; i < s_run.count; i++) {
            RunType t = (RunType)s_run.items[i].type;
            if (t == RUN_D45IN_R || t == RUN_D45IN_L || t == RUN_D135IN_R || t == RUN_D135IN_L) r.diag_sections++;
        }
        if (!quiet) {
            printf("\n=== route chosen with diagonal cost %u ===\n", (unsigned)s_diag_cost);
            CommandList_Print(&s_diag_route);
            PrintRunList("fastest-run commands (route above, large turns + diagonal)", &s_run, times);
            if (!ok) printf("(acceleration is not enough somewhere in this route)\n");
        }
        if (!RunOnTruth(&s_run, truth, "diagonal")) r.time_diag = -2.0f;
    } else if (!quiet) {
        printf("could not make the diagonal route\n");
    }

    // --- 時間で最短の経路を求める ---
    r.time_opt = TimeOptimalRun(&s_map, truth, !quiet, false);
    r.time_opt_diag = TimeOptimalRun(&s_map, truth, !quiet, true);
    if (r.time_opt < 0.0f) return r;

    // --- 迷路を全部知っていたときの最短と比べる ---
    Dijkstra_Compute(&s_solver, truth, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
    r.cost_best = Dijkstra_Cost(&s_solver, kSimStart, DIR_NORTH);
    r.time_opt_best = TimeOptimalRun(truth, truth, false, false);
    r.time_opt_diag_best = TimeOptimalRun(truth, truth, false, true);
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
    printf("search time: %.1f s (goal / shortest route decided at %.1f s)\n", (double)r->search_s, (double)r->goal_s);
    printf("known-section runs: %d (%d cells)\n", r->known_runs, r->known_cells);
    printf("route cost: %u (best possible %u)%s\n", (unsigned)r->cost_found, (unsigned)r->cost_best,
           (r->cost_found == r->cost_best) ? "  = optimal" : "");
    printf("run time: %.3f s pivot, %.3f s small turns only, %.3f s with large turns, %.3f s with large + diagonal"
           " (diagonal cost %u, %d sections) (negative = not feasible, -2 = crash)\n",
           (double)r->time_pivot, (double)r->time_small, (double)r->time_large, (double)r->time_diag,
           (unsigned)s_diag_cost, r->diag_sections);
    printf("time-optimal run: %.3f s (best possible %.3f s)%s\n", (double)r->time_opt,
           (double)r->time_opt_best, (r->time_opt <= r->time_opt_best + 1e-4f) ? "  = optimal" : "");
    printf("time-optimal run with diagonal: %.3f s (best possible %.3f s)%s\n", (double)r->time_opt_diag,
           (double)r->time_opt_diag_best, (r->time_opt_diag <= r->time_opt_diag_best + 1e-4f) ? "  = optimal" : "");
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
    double sum_pivot;
    int diag_timed;      // timed の迷路のうち、斜めでも時間が出た数
    double sum_diag;     // 斜めの時間(diag_timed の迷路だけ)
    double sum_diag_large; // 同じ迷路の大回りの時間
    long sum_diag_sections;
    int diag_crash;      // 斜めの経路が本当の迷路でぶつかった数(0 のはず)
    double sum_opt;      // 時間で選んだ経路(timed の迷路だけ)
    double sum_opt_best;
    int opt_diag_timed;  // 時間で選んだ斜めありの経路の時間が出た迷路の数
    double sum_opt_diag;
    double sum_opt_diag_best;
    int opt_optimal;     // 時間で選んだ経路が、全部知っていたときと同じ時間だった迷路の数
    double sum_search;   // 探索の時間の見積もり
    double sum_goal_s;   // ゴールに止まった(全面: 最短経路が決まった)時刻
    long sum_known_cells; // 既知の区間でまとめて走った区画の数
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
    b->sum_known_cells += r->known_cells;
    if (r->cost_found == r->cost_best) b->optimal++;
    if (r->time_small > 0.0f && r->time_large > 0.0f) {
        b->timed++;
        b->sum_small += r->time_small;
        b->sum_large += r->time_large;
        b->sum_pivot += r->time_pivot;
        b->sum_opt += r->time_opt;
        b->sum_opt_best += r->time_opt_best;
        if (r->time_opt_diag > 0.0f && r->time_opt_diag_best > 0.0f) {
            b->opt_diag_timed++;
            b->sum_opt_diag += r->time_opt_diag;
            b->sum_opt_diag_best += r->time_opt_diag_best;
        }
        if (r->time_diag > 0.0f) {
            b->diag_timed++;
            b->sum_diag += r->time_diag;
            b->sum_diag_large += r->time_large;
            b->sum_diag_sections += r->diag_sections;
        }
    }
    if (r->time_diag == -2.0f) b->diag_crash++;
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
        printf(", known cells %.1f", (double)b->sum_known_cells / ok);
    }
    if (b->timed > 0) {
        printf(", run time %.3f s (pivot) %.3f s (small) %.3f s (large) %.3f s (time-opt, best %.3f s, optimal %d)",
               b->sum_pivot / b->timed, b->sum_small / b->timed, b->sum_large / b->timed,
               b->sum_opt / b->timed, b->sum_opt_best / b->timed, b->opt_optimal);
        if (b->timed < ok) printf(" [%d not feasible]", ok - b->timed);
        if (b->diag_timed > 0) {
            printf(", diagonal (cost %u) %.3f s vs large %.3f s on %d mazes (%.1f sections)", (unsigned)s_diag_cost,
                   b->sum_diag / b->diag_timed,
                   b->sum_diag_large / b->diag_timed, b->diag_timed,
                   (double)b->sum_diag_sections / b->diag_timed);
        }
    }
    if (b->opt_diag_timed > 0) {
        printf(", time-opt + diagonal %.3f s (best %.3f s) on %d mazes", b->sum_opt_diag / b->opt_diag_timed,
               b->sum_opt_diag_best / b->opt_diag_timed, b->opt_diag_timed);
    }
    if (b->diag_crash > 0) printf(" [diagonal CRASH %d]", b->diag_crash);
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
    cfg.scope = SCOPE_ROUND;
    cfg.search = SearchSpeeds_Default();
    s_prof = IdealRun_Default();
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
            if (strcmp(argv[i], "round") == 0) cfg.scope = SCOPE_ROUND;
            else if (strcmp(argv[i], "oneway") == 0) cfg.scope = SCOPE_ONE_WAY;
            else if (strcmp(argv[i], "full") == 0) cfg.scope = SCOPE_FULL;
            else { fprintf(stderr, "unknown scope: %s\n", argv[i]); return 2; }
        } else if (strcmp(argv[i], "--search") == 0 && i + 1 < argc) {
            if (!SearchSpeeds_Parse(argv[++i], &cfg.search)) {
                fprintf(stderr, "--search must be V,TURN_V,ACCEL (all > 0): %s\n", argv[i]);
                return 2;
            }
        } else if (strcmp(argv[i], "--run") == 0 && i + 1 < argc) {
            if (!IdealRun_Parse(argv[++i], &s_prof)) {
                fprintf(stderr, "--run must be VMAX,ACCEL,SMALL90,LARGE90,LARGE180 (all > 0): %s\n", argv[i]);
                return 2;
            }
        } else if (strcmp(argv[i], "--diag-cost") == 0 && i + 1 < argc) {
            long v = strtol(argv[++i], NULL, 10);
            if (v < 0 || v > 100) {
                fprintf(stderr, "--diag-cost must be 0..100: %s\n", argv[i]);
                return 2;
            }
            s_diag_cost = (uint16_t)v;
        } else if (strcmp(argv[i], "--full-diag") == 0) {
            s_full_diag = true;
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
    {
        static const char *const kScope[] = { "round", "oneway", "full" };
        printf("search: %s, v %.0f / turn %.0f mm/s, accel %.0f | fast run: v %.0f, accel %.0f, "
               "small %.0f / large90 %.0f / large180 %.0f mm/s\n",
               kScope[cfg.scope], (double)cfg.search.v, (double)cfg.search.turn_v, (double)cfg.search.accel,
               (double)s_prof.vmax, (double)s_prof.accel, (double)s_prof.v_turn[RUN_SMALL90_R],
               (double)s_prof.v_turn[RUN_LARGE90_R], (double)s_prof.v_turn[RUN_LARGE180_R]);
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

    return r.ok ? 0 : 1;
}
