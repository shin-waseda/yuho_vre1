// GUI (tools/maze_sim/gui.py) から ctypes で呼ぶための DLL の窓口。
// 探索の判断はファームウェアと同じ logic 層 (SearchPlanner) が行い、
// 本当の迷路での観測・移動は sim_core.c が行う。Python は描画と操作だけを受け持つ。
//
// 状態はこのファイルの static に1組だけ持つ(GUIは1つの迷路しか扱わない)。
// Python との受け渡しは整数とバイト列だけにして、構造体の並びに依存しないようにする。

#include <stddef.h>

#include "sim_core.h"
#include "search_time.h"
#include "plant_bridge.h"
#include "logic/maze/dijkstra.h"
#include "logic/maze/step_map.h"
#include "logic/maze/search_planner.h"
#include "logic/maze/run_path.h"
#include "logic/maze/time_dijkstra.h"

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
static TimeSolver s_time_solver; // 最短走行(時間)の計算用
static RunList s_run;

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
static MazeSolver s_value_solver; // 区画の値の表示用(プランナーは途中で計算を止めるので、全部を計算し直す)

// 探索の設定(行き先・速さ)と、その時間の見積もり(search_time.c)
static bool s_search_set = false;
static SearchTimeParams s_search;
static SearchTime s_st;
// 最短走行の速さ(FAST_BANDS の速度帯)
static int s_band = SIM_DEFAULT_BAND;
static RunProfile s_prof;
static bool s_prof_ready = false;

static const RunProfile *Prof(void) {
    if (!s_prof_ready) {
        s_prof = SimCore_FastProfile(s_band, NULL);
        s_prof_ready = true;
    }
    return &s_prof;
}

static const SearchTimeParams *Search(void) {
    if (!s_search_set) {
        s_search = SearchTime_DefaultParams();
        s_search_set = true;
    }
    return &s_search;
}

// 探索の設定。scope: 1 往復, 2 片道, 3 全面。速さが 0 以下なら params.h の値。次の sim_reset から使う。
SIM_EXPORT void sim_set_search(int scope, float v, float turn_v, float accel, int slalom) {
    SearchTimeParams p = SearchTime_DefaultParams();
    if (scope >= SEARCH_TIME_ROUND && scope <= SEARCH_TIME_FULL) p.scope = (SearchTimeScope)scope;
    if (v > 0.0f) p.v_mm_s = v;
    if (turn_v > 0.0f) p.turn_v_mm_s = turn_v;
    if (accel > 0.0f) p.accel_mm_s2 = accel;
    p.slalom = (slalom != 0);
    s_search = p;
    s_search_set = true;
}

// out[0] 直進の速さ, [1] スラロームの速さ, [2] 加速度, [3] スラローム(1/0), [4] 行き先
SIM_EXPORT void sim_search_params(float *out) {
    const SearchTimeParams *p = Search();
    out[0] = p->v_mm_s;
    out[1] = p->turn_v_mm_s;
    out[2] = p->accel_mm_s2;
    out[3] = p->slalom ? 1.0f : 0.0f;
    out[4] = (float)p->scope;
}

// 探索の時間の見積もり。out[0] ここまでの時間[s], [1] ゴールに止まった(全面: 最短経路が決まった)時刻[s](まだなら負),
// [2] 尻当ての回数, [3] 既知の区間をまとめて走った回数, [4] 終わったら 1
SIM_EXPORT void sim_search_time(float *out) {
    out[0] = s_st.t_s;
    out[1] = s_st.t_goal_s;
    out[2] = (float)s_st.n_setpos;
    out[3] = (float)s_st.n_known;
    out[4] = s_st.finished ? 1.0f : 0.0f;
}

// 今の迷路と探索の設定で、探索を最後まで(GUI の探索とは別に)たどって時間[s]を見積もる。終わらなければ負。
// Dijkstra の作業領域を使うので、sim_step と同時に(別のスレッドから)呼ばないこと
SIM_EXPORT float sim_search_estimate(void) {
    static WallMap map;
    static SearchPlanner sp;
    WallMap_Init(&map);
    SearchPlanner_Init(&sp, &map, s_algo, kSimStart, DIR_NORTH, s_goals, s_goal_count);
    sp.cost_to_goal = s_planner.cost_to_goal; // GUI の探索と同じコスト(sim_reset で決めたもの)
    sp.cost_to_start = s_planner.cost_to_start;
    if (Search()->scope == SEARCH_TIME_FULL) SearchPlanner_StartFull(&sp);
    SearchTime st;
    SearchTime_Init(&st, Search());
    SearchTime_Start(&st);
    MazePos pos = kSimStart;
    Direction heading = DIR_NORTH;
    for (int i = 0; i < 5000 && !st.finished; i++) {
        WallObservation obs = SimCore_Sense(&s_truth, pos, heading);
        SearchPhase before = sp.phase;
        Action a = SearchPlanner_Step(&sp, obs);
        SearchTime_Step(&st, &sp, obs, a, before);
        if (a.type == ACTION_STOP) {
            if (sp.phase != SEARCH_PHASE_TO_START || Search()->scope == SEARCH_TIME_ONE_WAY) break;
            heading = Dir_Opposite(heading); // ゴールで 180°(sim_step と同じ)
            sp.heading = heading;
            continue;
        }
        if (!SimCore_Execute(&s_truth, &pos, &heading, a)) return -1.0f;
    }
    return st.finished ? st.t_s : -1.0f;
}

// 今の迷路(maze_path に書き出したもの)と探索の設定で plant_sim を走らせ、機体と同じ形式のログを out_dir に残す
// (plant_bridge.h。maze_sim の --plant-log と同じ)。exe が NULL か空なら plant_sim.local などから探す。
// self は maze_sim のビルドのフォルダの中のパス(plant_sim.local と tools/get_log.py を探す基準)。
// est_s は探索の時間の見積もり(sim_search_estimate。打ち切りの時間を決める。0 以下なら 300 秒とみなす)。
// できたログのパスを log_out に書いて 1、失敗なら 0。時間がかかる(約 20 秒〜)ので、GUI は別のスレッドから呼ぶ
// (中で logic 層の計算はしないので、sim_step と同時に呼んでよい)。
SIM_EXPORT int sim_plant_log(const char *maze_path, const char *out_dir, const char *exe, int build, const char *extra,
                             const char *self, float est_s, char *log_out, int log_len) {
    PlantOptions o = { (exe != NULL && exe[0]) ? exe : NULL, build != 0, (extra != NULL && extra[0]) ? extra : NULL, self };
    SearchTimeParams p = *Search();
    float est = (est_s > 0.0f) ? est_s : 300.0f;
    return PlantLog_Run(&o, maze_path, &p, s_algo, est, out_dir, log_out, (size_t)(log_len > 0 ? log_len : 0)) ? 1 : 0;
}

SIM_EXPORT int sim_band_count(void) {
    return SimCore_FastBandCount();
}

// 最短走行の速度帯(1〜)を選ぶ。範囲の外なら端の帯。選んだ番号を返す
SIM_EXPORT int sim_set_band(int band) {
    if (band < 1) band = 1;
    if (band > SimCore_FastBandCount()) band = SimCore_FastBandCount();
    s_band = band;
    s_prof_ready = false;
    return s_band;
}

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
    if (Search()->scope == SEARCH_TIME_FULL) SearchPlanner_StartFull(&s_planner);
    SearchTime_Init(&s_st, Search());
    SearchTime_Start(&s_st);

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
    SearchPhase before = s_planner.phase;
    Action a = SearchPlanner_Step(&s_planner, obs);
    SearchTime_Step(&s_st, &s_planner, obs, a, before);
    s_has_values = true;
    // 指令を返したときだけプランナーは経路を計算している(止まったときは前の値のまま)
    if (s_algo == SEARCH_ALGO_DIJKSTRA && a.type != ACTION_STOP) {
        SimCore_PlannerValues(&s_planner, &s_value_solver);
    }
    *type = a.type;
    *cells = a.cells;

    if (a.type == ACTION_STOP) {
        switch (s_planner.phase) {
            case SEARCH_PHASE_DONE:     s_status = SIM_DONE; break;
            case SEARCH_PHASE_FAILED:   s_status = SIM_FAILED; break;
            case SEARCH_PHASE_TO_START: s_status = SIM_AT_GOAL; break;
            default:                    s_status = SIM_MOVED; break;
        }
        if (s_status == SIM_AT_GOAL) {
            if (Search()->scope == SEARCH_TIME_ONE_WAY) {
                s_status = SIM_DONE; // 片道: ゴールで止まって終わる
            } else {
                // 機体はゴールの真ん中で 180° 回ってから帰り始める(app/search_run の SearchLoop)。向きを合わせる
                s_heading = Dir_Opposite(s_heading);
                s_planner.heading = s_heading;
            }
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
    if (s_planner.phase == SEARCH_PHASE_TO_GOAL || s_planner.phase == SEARCH_PHASE_FULL) s_moves_to_goal++;
    else s_moves_back++;
    s_status = SIM_MOVED;
    return s_status;
}

SIM_EXPORT void sim_pose(uint8_t *x, uint8_t *y, uint8_t *dir) {
    *x = s_pos.x;
    *y = s_pos.y;
    *dir = (uint8_t)s_heading;
}

// 0: ゴールへ向かう 1: スタートへ戻る 2: 終わり 3: 失敗 4: 全面探索で最短経路になりうる区画を回っている(SearchPhaseと同じ)
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
        uint16_t c = Dijkstra_Cost(&s_value_solver, p, (Direction)d);
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

// ---- 最短走行(時間) ----

// sim_run_plan() の kind
enum {
    SIM_PLAN_TIME = 0,      // 探索で分かった壁だけで、走行時間が最短の経路(TimeDijkstra)
    SIM_PLAN_COST = 1,      // 探索で分かった壁だけで、コストが最短の経路を大回りに置き換えたもの
    SIM_PLAN_TIME_BEST = 2, // 迷路を全部知っていたときの、走行時間が最短の経路
};

// 最短走行の速さ(sim_set_band で選んだ速度帯。既定は SIM_DEFAULT_BAND)と区画の大きさ。
// out[0] 加速度, [1] 最高速度, [2] 小回り90°, [3] 大回り90°, [4] 大回り180°, [5] SECTION_MM, [6] 減速度, [7] 帯の番号
SIM_EXPORT void sim_run_profile(float *out) {
    RunProfile p = *Prof();
    out[0] = p.accel;
    out[1] = p.vmax;
    out[2] = p.v_turn[RUN_SMALL90_R];
    out[3] = p.v_turn[RUN_LARGE90_R];
    out[4] = p.v_turn[RUN_LARGE180_R];
    out[5] = SECTION_MM;
    out[6] = p.decel;
    out[7] = (float)s_band;
}

// 最短走行の指令の列を求め、types/halves/times(指令ごとの時間[s])に最大 max 個書いて個数を返す
// (行けなければ0)。total に合計の時間[s]、feasible に加速度が足りているか(1/0)を書く。
// 指令はスタートの区画の中心から、北向きに静止した状態で始まる。
SIM_EXPORT int sim_run_plan(int kind, uint8_t *types, uint8_t *halves, float *times, int max,
                            float *total, int *feasible) {
    RunProfile prof = *Prof();
    *total = 0.0f;
    *feasible = 0;

    if (kind == SIM_PLAN_COST) {
        Dijkstra_Compute(&s_solver, &s_map, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
        if (!Dijkstra_BuildRoute(&s_solver, kSimStart, DIR_NORTH, true, &s_route)) return 0;
        if (!RunPath_FromRoute(&s_route, &prof, true, &s_run)) return 0;
    } else {
        const WallMap *map = (kind == SIM_PLAN_TIME_BEST) ? &s_truth : &s_map;
        TimeDijkstra_Compute(&s_time_solver, map, WALL_VIEW_KNOWN, &prof, s_goals, s_goal_count,
                             kSimStart, DIR_NORTH);
        if (!TimeDijkstra_BuildRun(&s_time_solver, &s_run)) return 0;
    }

    static float t[RUN_LIST_MAX];
    *feasible = RunList_EstimateTime(&s_run, &prof, total, t, NULL) ? 1 : 0;
    int n = (s_run.count < max) ? s_run.count : max;
    for (int i = 0; i < n; i++) {
        types[i] = s_run.items[i].type;
        halves[i] = s_run.items[i].halves;
        times[i] = t[i];
    }
    return n;
}
