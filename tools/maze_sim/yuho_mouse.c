// yuho の迷路 logic 層を mouse maze sim(公開版の迷路シミュレータ)につなぐ窓口(msim.h)。
// 探索の判断はファームウェアと同じ logic 層 (SearchPlanner) が行い、
// 本当の迷路での観測・移動・衝突の判定は mouse maze sim(Python)が行う。
//
// maze_sim はアルゴリズムの確認だけ。時間は「計算上の速さ」の運動だけで数え、滑り・ログで合わせた補正・
// 尻当てや待ち・超信地旋回の時間は数えない(時間で選ぶ経路などの、時間に依存するアルゴリズムを確かめるため)。
//   - 最短走行: logic 層の RunProfile_Default() の形(理想の円弧、オフセットなし、減速度 = 加速度)に、
//               オプション run の速さを入れる。経路は時間 Dijkstra と「コストで最短 + 大回り」の2つ
//   - 探索    : オプション search の速さで、指令ごとの運動を足す
// 時間の数え方は ideal_time.h(C の CLI maze_sim.c と同じ)。
//
// 状態はこのファイルの static に1組だけ持つ(シミュレータは1つの迷路しか扱わない)。

#include <stdio.h>
#include <string.h>

#include "msim.h"
#include "sim_core.h"
#include "ideal_time.h"
#include "logic/maze/dijkstra.h"
#include "logic/maze/step_map.h"
#include "logic/maze/search_planner.h"
#include "logic/maze/run_path.h"
#include "logic/maze/time_dijkstra.h"

// 探索の行き先
typedef enum { SCOPE_ROUND, SCOPE_ONE_WAY, SCOPE_FULL } Scope;

static WallMap s_map;
static SearchPlanner s_planner;
static MazeSolver s_solver;       // 最短経路の計算用(プランナーの作業領域とは別)
static MazeSolver s_value_solver; // 区画の値の表示用(プランナーは途中で計算を止めるので、全部を計算し直す)
static CommandList s_route;
static TimeSolver s_time_solver;  // 最短走行(時間)の計算用
static RunList s_run;

static MazePos s_start;
static MazePos s_goals[MAZE_GOAL_MAX];
static uint8_t s_goal_count;

static SearchAlgo s_algo;
static Scope s_scope;
static SearchSpeeds s_speeds;    // 探索の速さ(オプション search)
static SearchTime s_st;
static RunProfile s_prof;         // 最短走行の速さ(オプション run)
static bool s_has_values;         // リセット後、プランナーが一度でも経路を計算したか
static bool s_decided_shown;      // 全面探索で最短経路が決まったことを出したか
static char s_note[128];          // 状態の行に出す出来事(LOST など)
static bool s_has_cost;           // 探索の後の経路のコスト(msim_results で計算する)
static uint16_t s_cost_found, s_cost_best;

// 既知の区間をまとめて走っている途中(SearchTime_KnownRun)。msim の1手は「向きを変えて何区画か進む」なので、
// 曲がりを含む区間は何手かに分けて返す。その間はプランナーを呼ばない(機体も区間の途中では壁を読まない)
static Direction s_known[MAZE_CELL_COUNT]; // 区画を移る向き(s_known[0] は C1 に入る向き)
static uint16_t s_known_m;                 // 最後の番号(0 ならまとめていない)
static uint16_t s_known_next;              // 次に返す番号
static Direction s_known_heading;          // シミュレータの機体の今の向き

// s_known[s_known_next..] から1手ぶん(同じ向きが続く間)を move に書く
static void NextKnownMove(MsimMove *move) {
    Direction d = s_known[s_known_next];
    int q = ((int)d - (int)s_known_heading) & 3;
    move->turn = (q == 3) ? -1 : q; // 右 1、左 -1、180° 2
    move->cells = 0;
    while (s_known_next <= s_known_m && s_known[s_known_next] == d) {
        move->cells++;
        s_known_next++;
    }
    s_known_heading = d;
    if (s_known_next > s_known_m) s_known_m = 0; // 終わり(プランナーの位置は区間の終わりの区画になっている)
}

// ------------------------------------------------------------
// オプション("名前=値" の行)
// ------------------------------------------------------------

static const char *FindOption(const char *options, const char *name, char *value, size_t len) {
    size_t n = strlen(name);
    for (const char *p = options; p != NULL && *p; ) {
        const char *end = strchr(p, '\n');
        size_t line = end ? (size_t)(end - p) : strlen(p);
        if (line > n && strncmp(p, name, n) == 0 && p[n] == '=') {
            size_t v = line - n - 1;
            if (v >= len) v = len - 1;
            memcpy(value, p + n + 1, v);
            value[v] = '\0';
            return value;
        }
        p = end ? end + 1 : NULL;
    }
    return NULL;
}

static bool ParseCost(const char *text, MazeCost *out) {
    unsigned v[4];
    if (sscanf(text, "%u,%u,%u,%u", &v[0], &v[1], &v[2], &v[3]) != 4 || v[0] == 0) return false;
    for (int i = 0; i < 4; i++) {
        if (v[i] > 1000u) return false;
    }
    MazeCost c = { (uint16_t)v[0], (uint16_t)v[1], (uint16_t)v[2], (uint16_t)v[3] };
    if (!MAZE_COST_FITS(c.straight, c.turn90, c.turn180, c.known_cell)) return false;
    *out = c;
    return true;
}

static MazeCost BackCostDefault(void) {
    MazeCost c = MazeCost_Default();
    c.known_cell = MAZE_COST_KNOWN_CELL_RETURN;
    return c;
}

// ------------------------------------------------------------
// 窓口(必ず作るもの)
// ------------------------------------------------------------

MSIM_EXPORT int msim_api_version(void) {
    return MSIM_API_VERSION;
}

MSIM_EXPORT int msim_reset(const MsimMaze *maze, const char *options, char *err, int err_len) {
    char v[96];
    if (maze->size != MAZE_SIZE) {
        snprintf(err, (size_t)err_len, "yuho is built for %dx%d mazes (params.h MAZE_SIZE)", MAZE_SIZE, MAZE_SIZE);
        return 0;
    }
    s_algo = (FindOption(options, "algo", v, sizeof(v)) && strcmp(v, "adachi") == 0) ? SEARCH_ALGO_ADACHI
                                                                                     : SEARCH_ALGO_DIJKSTRA;
    s_scope = SCOPE_ROUND;
    if (FindOption(options, "scope", v, sizeof(v))) {
        if (strcmp(v, "oneway") == 0) s_scope = SCOPE_ONE_WAY;
        else if (strcmp(v, "full") == 0) s_scope = SCOPE_FULL;
    }
    SearchSpeeds sp = SearchSpeeds_Default();
    if (FindOption(options, "search", v, sizeof(v)) && v[0] && !SearchSpeeds_Parse(v, &sp)) {
        snprintf(err, (size_t)err_len, "search must be V,TURN_V,ACCEL (all > 0): %s", v);
        return 0;
    }
    RunProfile prof = IdealRun_Default();
    if (FindOption(options, "run", v, sizeof(v)) && v[0] && !IdealRun_Parse(v, &prof)) {
        snprintf(err, (size_t)err_len, "run must be VMAX,ACCEL,SMALL90,LARGE90,LARGE180 (all > 0): %s", v);
        return 0;
    }
    MazeCost to_goal = MazeCost_Default();
    MazeCost to_start = BackCostDefault();
    if (FindOption(options, "goal_cost", v, sizeof(v)) && v[0] && !ParseCost(v, &to_goal)) {
        snprintf(err, (size_t)err_len, "goal_cost must be S,T90,T180,K (S >= 1, fits in 13 bits): %s", v);
        return 0;
    }
    if (FindOption(options, "back_cost", v, sizeof(v)) && v[0] && !ParseCost(v, &to_start)) {
        snprintf(err, (size_t)err_len, "back_cost must be S,T90,T180,K (S >= 1, fits in 13 bits): %s", v);
        return 0;
    }
    s_speeds = sp;
    s_prof = prof;

    s_start.x = (uint8_t)maze->start_x;
    s_start.y = (uint8_t)maze->start_y;
    s_goal_count = 0;
    for (int i = 0; i < maze->goal_count && s_goal_count < MAZE_GOAL_MAX; i++) {
        MazePos g = { (uint8_t)maze->goal_x[i], (uint8_t)maze->goal_y[i] };
        s_goals[s_goal_count++] = g;
    }

    WallMap_Init(&s_map);
    SearchPlanner_Init(&s_planner, &s_map, s_algo, s_start, (Direction)maze->start_dir, s_goals, s_goal_count);
    s_planner.cost_to_goal = to_goal;
    s_planner.cost_to_start = to_start;
    if (s_scope == SCOPE_FULL) SearchPlanner_StartFull(&s_planner);
    // 往復は機体と同じく、ゴールで止まって 180° 回らず、ゴールの区画を全部回ってから戻る(app/search_run の RunSearch)
    s_planner.goal_tour = (s_scope == SCOPE_ROUND);
    SearchTime_Init(&s_st, &s_speeds);
    s_known_m = 0;
    s_has_values = false;
    s_decided_shown = false;
    s_has_cost = false;
    s_note[0] = '\0';
    return 1;
}

MSIM_EXPORT void msim_step(const MsimSense *sense, MsimMove *move) {
    memset(move, 0, sizeof(*move));
    // 既知の区間の続き(プランナーは区間の終わりにいるので、位置は比べない)
    if (s_known_m > 0u) {
        move->status = MSIM_MOVE;
        NextKnownMove(move);
        move->returning = (s_planner.phase == SEARCH_PHASE_TO_START || s_planner.phase == SEARCH_PHASE_DONE ||
                           s_planner.phase == SEARCH_PHASE_GOAL_TOUR) ? 1 : 0;
        return;
    }
    // プランナーの位置とシミュレータの位置がずれたら logic 層のバグ
    if (sense->x != s_planner.pos.x || sense->y != s_planner.pos.y || sense->dir != (int)s_planner.heading) {
        snprintf(s_note, sizeof(s_note), "LOST: planner (%u,%u) %c, sim (%d,%d) %c", s_planner.pos.x,
                 s_planner.pos.y, "NESW"[s_planner.heading], sense->x, sense->y, "NESW"[sense->dir & 3]);
        move->status = MSIM_FAILED;
        return;
    }
    if (s_planner.phase == SEARCH_PHASE_DONE || s_planner.phase == SEARCH_PHASE_FAILED) {
        move->status = (s_planner.phase == SEARCH_PHASE_DONE) ? MSIM_DONE : MSIM_FAILED;
        return;
    }

    WallObservation obs = { sense->front != 0, sense->right != 0, sense->left != 0 };
    SearchPhase before = s_planner.phase;
    Direction h_before = (Direction)sense->dir;
    Action a = SearchPlanner_Step(&s_planner, obs);
    // 機体と同じく、先の区画の壁が分かっていればまとめて走る(プランナーの位置は区間の終わりへ進む)
    uint16_t km = SearchTime_KnownRun(&s_st, &s_planner, a, before, s_planner.phase, s_known);
    if (km == 0u) SearchTime_Step(&s_st, a, before, s_planner.phase);
    s_has_values = true;
    // 指令を返したときだけプランナーは経路を計算している(止まったときは前の値のまま)
    if (s_algo == SEARCH_ALGO_DIJKSTRA && a.type != ACTION_STOP) {
        SimCore_PlannerValues(&s_planner, &s_value_solver);
    }
    if (before == SEARCH_PHASE_FULL && s_planner.phase == SEARCH_PHASE_TO_START && !s_decided_shown) {
        snprintf(s_note, sizeof(s_note), "shortest route decided at (%u,%u)", s_planner.pos.x, s_planner.pos.y);
        s_decided_shown = true;
    }

    if (a.type == ACTION_STOP) {
        switch (s_planner.phase) {
            case SEARCH_PHASE_DONE:
                move->status = MSIM_DONE;
                break;
            case SEARCH_PHASE_FAILED:
                move->status = MSIM_FAILED;
                break;
            case SEARCH_PHASE_TO_START:
                if (s_scope == SCOPE_ONE_WAY) {
                    move->status = MSIM_DONE; // 片道: ゴールで止まって終わる
                } else {
                    // 機体はゴールの真ん中で 180° 回ってから帰り始める(app/search_run の SearchLoop)。向きを合わせる
                    s_planner.heading = Dir_Opposite(s_planner.heading);
                    move->status = MSIM_GOAL;
                    move->turn = 2;
                    move->returning = 1;
                }
                break;
            default:
                move->status = MSIM_MOVE; // 止まったが続ける(その場で次の Step)
                break;
        }
        return;
    }
    // 走ったままゴールに入った所(ゴールを回る・全面探索。機体はここで止まって地図を残し、真ん中から次の指令)。
    // ゴールに着いた合図を出しつつ、この手の動きもそのまま返す
    bool goal_moving = (before == SEARCH_PHASE_TO_GOAL && s_planner.phase != SEARCH_PHASE_TO_GOAL);
    move->status = goal_moving ? MSIM_GOAL : MSIM_MOVE;
    if (km > 0u) {
        // 既知の区間: 最初の1手(C1 へ入る向きから同じ向きが続く間)を返し、残りは次の呼び出しから
        s_known_m = km;
        s_known_next = 0;
        s_known_heading = h_before;
        NextKnownMove(move);
    } else {
        move->turn = Action_QuarterTurns(a.type); // 右 1、左 -1、180° 2(msim.h と同じ)
        move->cells = a.cells;
    }
    // ゴールを回っている間も、ゴールに着いた後なので「帰り」に数える
    move->returning = (s_planner.phase == SEARCH_PHASE_TO_START || s_planner.phase == SEARCH_PHASE_DONE ||
                       s_planner.phase == SEARCH_PHASE_GOAL_TOUR) ? 1 : 0;
}

// ------------------------------------------------------------
// 窓口(画面に出すもの)
// ------------------------------------------------------------

MSIM_EXPORT void msim_info(char *buf, int len) {
    snprintf(buf, (size_t)len, "name=yuho\nsizes=%d\n", MAZE_SIZE);
}

MSIM_EXPORT void msim_options(char *buf, int len) {
    MazeCost g = MazeCost_Default();
    MazeCost b = BackCostDefault();
    snprintf(buf, (size_t)len,
             "algo\tdijkstra\tdijkstra|adachi\t探索のアルゴリズム\ta\n"
             "scope\tround\tround|oneway|full\t探索の行き先(往復・片道・全面。機体の RUN → SEARCH の SCOPE)\to\n"
             "search\t%.0f,%.0f,%.0f\t\t探索の速さ V,TURN_V,ACCEL(直進の最高速度, 小回りの速さ [mm/s], 加速度 [mm/s^2])\t\n"
             "run\t%.0f,%.0f,%.0f,%.0f,%.0f\t\t最短走行の速さ VMAX,ACCEL,SMALL90,LARGE90,LARGE180 [mm/s, mm/s^2]\t\n"
             "goal_cost\t%u,%u,%u,%u\t\t行きの Dijkstra のコスト S,T90,T180,K(直進, 90°, 180°, 既知区画の上乗せ)\t\n"
             "back_cost\t%u,%u,%u,%u\t\t帰りのコスト S,T90,T180,K\t\n",
             (double)SEARCH_V_MM_S, (double)SLALOM_V_MM_S, (double)SEARCH_ACCEL_MM_S2,
             (double)RUN_VMAX_MM_S, (double)RUN_ACCEL_MM_S2, (double)RUN_V_SMALL90_MM_S,
             (double)RUN_V_LARGE90_MM_S, (double)RUN_V_LARGE180_MM_S,
             g.straight, g.turn90, g.turn180, g.known_cell, b.straight, b.turn90, b.turn180, b.known_cell);
}

// 最短走行の速さは探索に関係しないので、探索をやり直さずに変える
MSIM_EXPORT int msim_set_option(const char *name, const char *value) {
    if (strcmp(name, "run") == 0) return IdealRun_Parse(value, &s_prof) ? 1 : 0;
    return 0;
}

MSIM_EXPORT int msim_wall(int x, int y, int dir) {
    MazePos p = { (uint8_t)x, (uint8_t)y };
    if (!WallMap_IsKnown(&s_map, p, (Direction)dir)) return MSIM_WALL_UNKNOWN;
    return WallMap_HasWall(&s_map, p, (Direction)dir, WALL_VIEW_KNOWN) ? MSIM_WALL_EXISTS : MSIM_WALL_OPEN;
}

// プランナーが最後に計算した、その区画の値(Dijkstraはゴールまでの最小コスト、足立法は歩数)
MSIM_EXPORT int msim_cell_value(int x, int y) {
    if (!s_has_values) return -1;
    MazePos p = { (uint8_t)x, (uint8_t)y };
    if (s_algo == SEARCH_ALGO_ADACHI) {
        uint16_t v = s_planner.work.step.step[p.y][p.x];
        return (v == 0xFFFFu) ? -1 : (int)v;
    }
    uint16_t best = MAZE_COST_INF;
    for (int d = 0; d < 4; d++) {
        uint16_t c = Dijkstra_Cost(&s_value_solver, p, (Direction)d);
        if (c < best) best = c;
    }
    return (best == MAZE_COST_INF) ? -1 : (int)best;
}

MSIM_EXPORT void msim_status(char *buf, int len) {
    static const char *const kPhase[] = { "to goal", "to start", "done", "failed", "full search", "goal tour" };
    static const char *const kScope[] = { "round", "oneway", "full" };
    const char *target = (s_planner.phase == SEARCH_PHASE_TO_GOAL) ? "goal"
                       : (s_planner.phase == SEARCH_PHASE_FULL) ? "targets"
                       : (s_planner.phase == SEARCH_PHASE_GOAL_TOUR) ? "goal cells" : "start";
    const char *goal_label = (s_scope == SCOPE_FULL) ? "route decided" : "goal";
    int n = snprintf(buf, (size_t)len,
                     "phase  : %s\n"
                     "search : %s\n"
                     "numbers: %s to %s\n"
                     "time   : %5.1f s",
                     kPhase[s_planner.phase], kScope[s_scope],
                     s_algo == SEARCH_ALGO_ADACHI ? "steps" : "cost", target, (double)s_st.t_s);
    if (n < 0 || n >= len) return;
    if (s_st.t_goal_s >= 0.0f) n += snprintf(buf + n, (size_t)(len - n), " (%s %.1f s)", goal_label, (double)s_st.t_goal_s);
    if (n < 0 || n >= len) return;
    n += snprintf(buf + n, (size_t)(len - n), "\n");
    if (n < 0 || n >= len) return;
    n += snprintf(buf + n, (size_t)(len - n), "known  : %d runs, %d cells\n", s_st.known_runs, s_st.known_cells);
    if (n < 0 || n >= len) return;
    if (s_has_cost) {
        n += snprintf(buf + n, (size_t)(len - n), "route cost %u%s", s_cost_found,
                      s_cost_found == s_cost_best ? " = optimal\n" : "");
        if (n < 0 || n >= len) return;
        if (s_cost_found != s_cost_best) n += snprintf(buf + n, (size_t)(len - n), " (best %u)\n", s_cost_best);
        if (n < 0 || n >= len) return;
    }
    if (s_note[0]) snprintf(buf + n, (size_t)(len - n), "%s\n", s_note);
}

// 本当の迷路(MsimMaze の walls)を WallMap にする(全部の壁が既知)
static void TruthFromMaze(const MsimMaze *maze, WallMap *m) {
    WallMap_Init(m);
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            MazePos p = { (uint8_t)x, (uint8_t)y };
            unsigned char w = maze->walls[y * MAZE_SIZE + x];
            if (y + 1 < MAZE_SIZE) WallMap_SetWall(m, p, DIR_NORTH, (w & MSIM_BIT_NORTH) != 0);
            if (x + 1 < MAZE_SIZE) WallMap_SetWall(m, p, DIR_EAST, (w & MSIM_BIT_EAST) != 0);
        }
    }
}

MSIM_EXPORT void msim_results(const MsimMaze *truth, char *buf, int len) {
    static WallMap t;
    TruthFromMaze(truth, &t);
    // 経路の評価は、常に params.h の最短走行用コスト(探索のコストを変えても物差しは変わらない)
    Dijkstra_Compute(&s_solver, &t, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
    s_cost_best = Dijkstra_Cost(&s_solver, s_start, DIR_NORTH);
    Dijkstra_Compute(&s_solver, &s_map, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
    s_cost_found = Dijkstra_Cost(&s_solver, s_start, DIR_NORTH);
    s_has_cost = true;
    snprintf(buf, (size_t)len, "search_s=%.3f\ngoal_s=%.3f\nknown_cells=%d\ncost_optimal=%d\ncost_ratio=%.4f\n",
             (double)s_st.t_s, (double)s_st.t_goal_s, s_st.known_cells, s_cost_found == s_cost_best ? 1 : 0,
             (double)s_cost_found / (double)s_cost_best);
}

// ------------------------------------------------------------
// 最短走行
// ------------------------------------------------------------

// kind: 0 走行時間が最短の経路(TimeDijkstra)、1 コストが最短の経路を大回りに置き換えたもの
MSIM_EXPORT int msim_run_plan_count(void) {
    return 2;
}

MSIM_EXPORT int msim_run_plan(int kind, const MsimMaze *maze, MsimRun *out, int max, MsimRunInfo *info) {
    static WallMap truth;
    const WallMap *map = &s_map;
    if (maze != NULL) {
        TruthFromMaze(maze, &truth);
        map = &truth;
    }
    RunProfile prof = s_prof;
    memset(info, 0, sizeof(*info));
    if (kind == 1) {
        Dijkstra_Compute(&s_solver, map, WALL_VIEW_KNOWN, NULL, s_goals, s_goal_count);
        if (!Dijkstra_BuildRoute(&s_solver, s_start, DIR_NORTH, true, &s_route)) return 0;
        if (!RunPath_FromRoute(&s_route, &prof, true, &s_run)) return 0;
        snprintf(info->name, sizeof(info->name), "cost + large");
    } else {
        TimeDijkstra_Compute(&s_time_solver, map, WALL_VIEW_KNOWN, &prof, s_goals, s_goal_count, s_start, DIR_NORTH);
        if (!TimeDijkstra_BuildRun(&s_time_solver, &s_run)) return 0;
        snprintf(info->name, sizeof(info->name), "time-optimal");
    }
    static float t[RUN_LIST_MAX];
    float total = 0.0f;
    info->feasible = RunList_EstimateTime(&s_run, &prof, &total, t, NULL) ? 1 : 0;
    info->total_s = total;
    info->accel = prof.accel;
    info->vmax = prof.vmax;
    info->section_mm = SECTION_MM;
    int n = 0;
    for (uint16_t i = 0; i < s_run.count && n < max; i++) {
        const RunCommand *c = &s_run.items[i];
        if (c->type == RUN_STOP) break;
        out[n].type = c->type; // RunType と MSIM_RUN_* は同じ番号
        out[n].halves = c->halves;
        out[n].time_s = t[i];
        out[n].v = (c->type == RUN_STRAIGHT) ? 0.0f : prof.v_turn[c->type];
        n++;
    }
    return n;
}
