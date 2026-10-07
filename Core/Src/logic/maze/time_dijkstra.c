#include "logic/maze/time_dijkstra.h"

#include <stddef.h>

// 直進の長さの上限(半区画単位)。端から端 + スタートとゴールの半区画で 2 × MAZE_SIZE。
#define HALVES_MAX (2 * MAZE_SIZE + 1)
// 速度の添字。0 .. TIME_CLASS_COUNT-1 が旋回の種類、SPEED_STOP が静止(スタート・ゴール)。
#define SPEED_STOP  TIME_CLASS_COUNT
#define SPEED_COUNT (TIME_CLASS_COUNT + 1)

#define HEAP_NONE 0xFFFFu

// 作業領域(約14KB)。全てのTimeSolverで共有する。
static uint32_t s_straight_us[HALVES_MAX + 1][SPEED_COUNT][SPEED_COUNT]; // 直進の時間(行けなければTIME_INF)
static float s_speed[SPEED_COUNT];
static uint32_t s_turn_us[RUN_TYPE_COUNT]; // 旋回の曲線の部分の時間(オフセットは直進の側に数える)
static uint16_t s_heap[TIME_NODE_COUNT];     // ノード番号の二分ヒープ(時間の小さい順)
static uint16_t s_heap_pos[TIME_NODE_COUNT]; // ノード → ヒープ上の位置(入っていなければHEAP_NONE)
static uint16_t s_heap_size;
static const uint32_t *s_key; // ヒープの並べ替えに使う時間(TimeSolver.time_us)

_Static_assert(TIME_NODE_COUNT < HEAP_NONE, "node index must fit in uint16_t");

// ---- 優先度付きキュー(時間をキーにした二分ヒープ) ----

static void Heap_Place(uint16_t i, uint16_t node) {
    s_heap[i] = node;
    s_heap_pos[node] = i;
}

static void Heap_Up(uint16_t i) {
    uint16_t node = s_heap[i];
    while (i > 0) {
        uint16_t parent = (uint16_t)((i - 1u) / 2u);
        if (s_key[s_heap[parent]] <= s_key[node]) break;
        Heap_Place(i, s_heap[parent]);
        i = parent;
    }
    Heap_Place(i, node);
}

static void Heap_Down(uint16_t i) {
    uint16_t node = s_heap[i];
    for (;;) {
        uint16_t child = (uint16_t)(2u * i + 1u);
        if (child >= s_heap_size) break;
        if (child + 1u < s_heap_size && s_key[s_heap[child + 1u]] < s_key[s_heap[child]]) child++;
        if (s_key[node] <= s_key[s_heap[child]]) break;
        Heap_Place(i, s_heap[child]);
        i = child;
    }
    Heap_Place(i, node);
}

// 入っていなければ入れ、入っていれば(時間が減ったので)位置を直す
static void Heap_PushOrDecrease(uint16_t node) {
    if (s_heap_pos[node] == HEAP_NONE) {
        uint16_t i = s_heap_size++;
        Heap_Place(i, node);
        Heap_Up(i);
    } else {
        Heap_Up(s_heap_pos[node]);
    }
}

static uint16_t Heap_Pop(void) {
    uint16_t top = s_heap[0];
    s_heap_pos[top] = HEAP_NONE;
    s_heap_size--;
    if (s_heap_size > 0) {
        Heap_Place(0, s_heap[s_heap_size]);
        Heap_Down(0);
    }
    return top;
}

// ---- ノード ----

static inline uint16_t Node(MazePos p, Direction d, int cls) {
    return (uint16_t)(MazeNode_Index(p, d) * TIME_CLASS_COUNT + (uint16_t)cls);
}

static int TurnClass(RunType t) {
    switch (t) {
        case RUN_SMALL90_R:
        case RUN_SMALL90_L:  return TIME_CLASS_SMALL90;
        case RUN_LARGE90_R:
        case RUN_LARGE90_L:  return TIME_CLASS_LARGE90;
        default:             return TIME_CLASS_LARGE180;
    }
}

static uint32_t SecondsToUs(float t) {
    return (uint32_t)(t * 1.0e6f + 0.5f);
}

// 逆向きに広げるときの、今の計算の設定
typedef struct {
    TimeSolver *s;
    const WallMap *map;
    WallView view;
    const MazePos *goals;
    uint8_t goal_count;
} Ctx;

// 走行中の位置。center が false なら「区画 cell に向き dir で入る境目」、
// true なら「区画 cell の中心で向き dir」。小回りは境目から、大回りは中心から始まる。
typedef struct {
    MazePos cell;
    Direction dir;
    bool center;
} RunPos;

static bool IsGoal(const Ctx *c, MazePos p) {
    return MazePos_InList(p, c->goals, c->goal_count);
}

static bool ClassIsCenter(int k) {
    return k != TIME_CLASS_SMALL90; // 大回りは区画の中心から始まる
}

static void Relax(Ctx *c, uint16_t v, uint32_t t, uint8_t type, uint8_t halves, uint8_t next_class) {
    if (t >= c->s->time_us[v]) return;
    c->s->time_us[v] = t;
    c->s->next[v].type = type;
    c->s->next[v].halves = halves;
    c->s->next[v].next_class = next_class;
    Heap_PushOrDecrease(v);
}

// 位置を進行方向と逆へ半区画戻す。境目 → 手前の区画の中心 へ戻るときは、
// 壁がないこと・その区画がゴールでないこと(ゴールを通り抜けない)を確かめる。
static bool StepBack(const Ctx *c, RunPos *q) {
    if (q->center) {
        q->center = false; // 中心 → 同じ区画に入る境目
        return true;
    }
    MazePos p;
    if (!MazePos_Step(q->cell, Dir_Opposite(q->dir), &p)) return false;
    if (WallMap_HasWall(c->map, p, q->dir, c->view)) return false;
    if (IsGoal(c, p)) return false;
    q->cell = p;
    q->center = true;
    return true;
}

// 位置qを出口とする旋回typeの、入口の位置を求める(小回りは境目どうし、大回りは中心どうし)。
// 旋回が通る区画の間に壁がなく、ゴールの区画を通り抜けなければtrue。
static bool TurnEntry(const Ctx *c, RunPos q, RunType type, RunPos *entry) {
    bool large = RunType_IsLarge(type);
    if (q.center != large) return false;

    Direction d_in = Dir_Turn(q.dir, -RunType_QuarterTurns(type));
    Direction moves[RUN_TURN_MOVES_MAX];
    uint8_t n = RunType_TurnMoves(type, d_in, moves);
    if (large) n--; // 大回りは最後の区画の中心で終わる

    // 出口から逆にたどって、旋回の入口の区画を求める
    MazePos a = q.cell;
    for (int i = (int)n - 1; i >= 0; i--) {
        if (!MazePos_Step(a, Dir_Opposite(moves[i]), &a)) return false;
    }
    // 入口から前向きにたどって確かめる
    MazePos cur = a;
    for (uint8_t i = 0; i < n; i++) {
        if (IsGoal(c, cur)) return false;
        if (WallMap_HasWall(c->map, cur, moves[i], c->view)) return false;
        if (!MazePos_Step(cur, moves[i], &cur)) return false;
    }
    if (large && IsGoal(c, q.cell)) return false; // ゴールの区画の中心で曲がり終えない

    entry->cell = a;
    entry->dir = d_in;
    entry->center = large;
    return true;
}

// 位置qを出口とする全ての旋回について、その入口のノードへ
// 「旋回の時間 + tail_us[旋回の速度の種類]」を入れる(TIME_INF の種類は使わない)。
// halves と next_class は、旋回の後の直進の長さと、その後のノードの種類(TIME_NEXT_STOP 可)。
static void RelaxTurnsInto(Ctx *c, RunPos q, const uint32_t *tail_us, uint8_t halves, uint8_t next_class) {
    static const RunType kTurns[] = {
        RUN_SMALL90_R, RUN_SMALL90_L, RUN_LARGE90_R, RUN_LARGE90_L, RUN_LARGE180_R, RUN_LARGE180_L,
    };
    for (unsigned i = 0; i < sizeof(kTurns) / sizeof(kTurns[0]); i++) {
        RunType type = kTurns[i];
        int kt = TurnClass(type);
        if (tail_us[kt] == TIME_INF) continue;
        RunPos e;
        if (!TurnEntry(c, q, type, &e)) continue;
        Relax(c, Node(e.cell, e.dir, kt), s_turn_us[type] + tail_us[kt], (uint8_t)type, halves, next_class);
    }
}

// ゴールの区画gの中心(向きdで入って止まる)からまっすぐ戻った各位置について、
// そこを出口とする旋回の入口へ「旋回 + まっすぐ進んでゴールの中心で止まる」時間を入れる
static void SeedGoal(Ctx *c, MazePos g, Direction d) {
    RunPos q = { g, d, true };
    for (uint8_t k = 1; k <= HALVES_MAX; k++) {
        if (!StepBack(c, &q)) break;
        uint32_t tail[TIME_CLASS_COUNT];
        for (int kt = 0; kt < TIME_CLASS_COUNT; kt++) tail[kt] = s_straight_us[k][kt][SPEED_STOP];
        RelaxTurnsInto(c, q, tail, k, TIME_NEXT_STOP);
    }
}

// uに入ってくる辺(旋回 + 半区画0個以上の直進)をたどる
static void RelaxPreds(Ctx *c, uint16_t u, RunPos q, int k) {
    uint32_t t_u = c->s->time_us[u];
    for (uint8_t h = 0; h <= HALVES_MAX; h++) {
        if (h > 0 && !StepBack(c, &q)) break;
        uint32_t tail[TIME_CLASS_COUNT];
        for (int kt = 0; kt < TIME_CLASS_COUNT; kt++) {
            // h = 0 は旋回どうしが直接つながる場合(前後のオフセットの中で速度を変える)
            uint32_t st = s_straight_us[h][kt][k];
            tail[kt] = (st == TIME_INF) ? TIME_INF : t_u + st;
        }
        RelaxTurnsInto(c, q, tail, h, (uint8_t)k);
    }
}

void TimeDijkstra_Compute(TimeSolver *s, const WallMap *map, WallView view, const RunProfile *prof,
                          const MazePos *goals, uint8_t goal_count,
                          MazePos start, Direction heading) {
    RunProfile default_prof = RunProfile_Default();
    if (prof == NULL) prof = &default_prof;

    // 速度と、直進の時間の表
    s_speed[TIME_CLASS_SMALL90]  = prof->v_turn[RUN_SMALL90_R];
    s_speed[TIME_CLASS_LARGE90]  = prof->v_turn[RUN_LARGE90_R];
    s_speed[TIME_CLASS_LARGE180] = prof->v_turn[RUN_LARGE180_R];
    s_speed[SPEED_STOP] = 0.0f;
    // 速度の種類ごとの旋回のオフセット(直進の入りは前の旋回の後オフセット、出は次の旋回の前オフセット)
    float off_post[SPEED_COUNT] = {
        prof->turn_post[RUN_SMALL90_R], prof->turn_post[RUN_LARGE90_R], prof->turn_post[RUN_LARGE180_R], 0.0f,
    };
    float off_pre[SPEED_COUNT] = {
        prof->turn_pre[RUN_SMALL90_R], prof->turn_pre[RUN_LARGE90_R], prof->turn_pre[RUN_LARGE180_R], 0.0f,
    };
    for (int h = 0; h <= HALVES_MAX; h++) {
        for (int a = 0; a < SPEED_COUNT; a++) {
            for (int b = 0; b < SPEED_COUNT; b++) {
                float t;
                // h = 0 は旋回どうしが直接つながる場合(前後のオフセットだけで速度を変える)
                bool ok = RunProfile_LinkedStraightTime(prof, (float)h * SECTION_MM * 0.5f,
                                                        s_speed[a], off_post[a], s_speed[b], off_pre[b], &t);
                s_straight_us[h][a][b] = ok ? SecondsToUs(t) : TIME_INF;
            }
        }
    }
    for (int t = RUN_SMALL90_R; t < RUN_TYPE_COUNT; t++) {
        s_turn_us[t] = SecondsToUs(RunProfile_CurveTime(prof, (RunType)t));
    }

    // 初期化
    for (uint16_t i = 0; i < TIME_NODE_COUNT; i++) {
        s->time_us[i] = TIME_INF;
        s->next[i].type = 0xFF;
        s->next[i].halves = 0;
        s->next[i].next_class = 0;
        s_heap_pos[i] = HEAP_NONE;
    }
    s_heap_size = 0;
    s_key = s->time_us;

    Ctx c = { s, map, view, goals, goal_count };

    // ゴールの区画の中心で止まる直前の旋回
    for (uint8_t i = 0; i < goal_count; i++) {
        for (int d = 0; d < 4; d++) SeedGoal(&c, goals[i], (Direction)d);
    }

    // ゴール側から広げる
    while (s_heap_size > 0) {
        uint16_t u = Heap_Pop();
        uint16_t cell_dir = (uint16_t)(u / TIME_CLASS_COUNT);
        int k = (int)(u % TIME_CLASS_COUNT);
        RunPos q = { MazeNode_Pos(cell_dir), MazeNode_Dir(cell_dir), ClassIsCenter(k) };
        RelaxPreds(&c, u, q, k);
    }

    // スタート: 中心から向きheadingへまっすぐ半区画ずつ進み、最初の旋回を始める位置へ
    // (またはそのままゴールの中心へ)
    s->start = start;
    s->start_heading = heading;
    s->start_time_us = TIME_INF;
    s->start_halves = 0;
    s->start_class = 0;
    s->start_to_goal = false;
    if (IsGoal(&c, start)) {
        s->start_time_us = 0;
        s->start_to_goal = true;
        return;
    }
    RunPos q = { start, heading, true };
    for (uint8_t h = 1; h <= HALVES_MAX; h++) {
        // 半区画進む
        if (q.center) {
            MazePos p;
            if (WallMap_HasWall(map, q.cell, heading, view)) break;
            if (!MazePos_Step(q.cell, heading, &p)) break;
            q.cell = p;
            q.center = false;
        } else {
            q.center = true;
            if (IsGoal(&c, q.cell)) { // ゴールの中心で止まる
                uint32_t t = s_straight_us[h][SPEED_STOP][SPEED_STOP];
                if (t < s->start_time_us) {
                    s->start_time_us = t;
                    s->start_halves = h;
                    s->start_to_goal = true;
                }
                break;
            }
        }
        for (int k = 0; k < TIME_CLASS_COUNT; k++) {
            if (ClassIsCenter(k) != q.center) continue;
            uint32_t rest = s->time_us[Node(q.cell, heading, k)];
            uint32_t t = s_straight_us[h][SPEED_STOP][k];
            if (rest == TIME_INF || t == TIME_INF) continue;
            if (t + rest < s->start_time_us) {
                s->start_time_us = t + rest;
                s->start_halves = h;
                s->start_class = (uint8_t)k;
                s->start_to_goal = false;
            }
        }
    }
}

uint32_t TimeDijkstra_StartTime(const TimeSolver *s) {
    return s->start_time_us;
}

uint32_t TimeDijkstra_NodeTime(const TimeSolver *s, MazePos p, Direction d) {
    uint32_t best = TIME_INF;
    for (int k = 0; k < TIME_CLASS_COUNT; k++) {
        uint32_t t = s->time_us[Node(p, d, k)];
        if (t < best) best = t;
    }
    return best;
}

// 位置を半区画ずつ h 回進める(経路は計算で確かめてあるので、迷路の外には出ない)
static bool StepForward(RunPos *q, uint8_t h) {
    for (uint8_t i = 0; i < h; i++) {
        if (q->center) {
            if (!MazePos_Step(q->cell, q->dir, &q->cell)) return false;
            q->center = false;
        } else {
            q->center = true;
        }
    }
    return true;
}

bool TimeDijkstra_BuildRun(const TimeSolver *s, RunList *out) {
    RunList_Clear(out);
    if (s->start_time_us == TIME_INF) return false;

    RunCommand stop = { RUN_STOP, 0 };
    if (s->start_halves == 0) return RunList_Push(out, stop); // スタートがゴール

    RunCommand first = { RUN_STRAIGHT, s->start_halves };
    if (!RunList_Push(out, first)) return false;
    if (s->start_to_goal) return RunList_Push(out, stop);

    // 最初の直進の後の位置
    RunPos q = { s->start, s->start_heading, true };
    if (!StepForward(&q, s->start_halves)) return false;
    int k = s->start_class;

    // ノードの数より多く回ることはない(時間は辺ごとに必ず減る)
    for (uint16_t guard = 0; guard < TIME_NODE_COUNT; guard++) {
        TimeNext nx = s->next[Node(q.cell, q.dir, k)];
        if (nx.type == 0xFF) return false;

        // 旋回
        RunCommand turn = { nx.type, 0 };
        if (!RunList_Push(out, turn)) return false;
        bool large = RunType_IsLarge((RunType)nx.type);
        Direction moves[RUN_TURN_MOVES_MAX];
        uint8_t n = RunType_TurnMoves((RunType)nx.type, q.dir, moves);
        if (large) n--; // 大回りは最後の区画の中心で終わる
        for (uint8_t i = 0; i < n; i++) {
            if (!MazePos_Step(q.cell, moves[i], &q.cell)) return false;
        }
        q.dir = moves[n - 1];
        q.center = large;

        // その後の直進(0なら次の旋回へ直接つながる)
        if (nx.halves > 0) {
            RunCommand straight = { RUN_STRAIGHT, nx.halves };
            if (!RunList_Push(out, straight)) return false;
        }
        if (nx.next_class == TIME_NEXT_STOP) return RunList_Push(out, stop);
        if (!StepForward(&q, nx.halves)) return false;
        k = nx.next_class;
    }
    return false;
}
