#include "logic/maze/time_dijkstra.h"

#include <stddef.h>

#define HALVES_MAX TIME_HALVES_MAX
// 速度の添字。0 .. TIME_CLASS_COUNT-1 が旋回の種類、SPEED_STOP が静止(スタート・ゴール)。
#define SPEED_STOP  TIME_CLASS_COUNT
#define SPEED_COUNT TIME_SPEED_COUNT

#define HEAP_NONE 0xFFFFu
#define DIAG_MM   (SECTION_MM / 1.41421356f) // 斜めの1区間

_Static_assert(TIME_NODE_COUNT < HEAP_NONE, "node index must fit in uint16_t");

// ---- 優先度付きキュー(時間をキーにした二分ヒープ) ----

static void Heap_Place(TimeSolver *s, uint16_t i, uint16_t node) {
    s->heap[i] = node;
    s->heap_pos[node] = i;
}

static void Heap_Up(TimeSolver *s, uint16_t i) {
    uint16_t node = s->heap[i];
    while (i > 0) {
        uint16_t parent = (uint16_t)((i - 1u) / 2u);
        if (s->time_us[s->heap[parent]] <= s->time_us[node]) break;
        Heap_Place(s, i, s->heap[parent]);
        i = parent;
    }
    Heap_Place(s, i, node);
}

static void Heap_Down(TimeSolver *s, uint16_t i) {
    uint16_t node = s->heap[i];
    for (;;) {
        uint16_t child = (uint16_t)(2u * i + 1u);
        if (child >= s->heap_size) break;
        if (child + 1u < s->heap_size && s->time_us[s->heap[child + 1u]] < s->time_us[s->heap[child]]) child++;
        if (s->time_us[node] <= s->time_us[s->heap[child]]) break;
        Heap_Place(s, i, s->heap[child]);
        i = child;
    }
    Heap_Place(s, i, node);
}

// 入っていなければ入れ、入っていれば(時間が減ったので)位置を直す
static void Heap_PushOrDecrease(TimeSolver *s, uint16_t node) {
    if (s->heap_pos[node] == HEAP_NONE) {
        uint16_t i = s->heap_size++;
        Heap_Place(s, i, node);
        Heap_Up(s, i);
    } else {
        Heap_Up(s, s->heap_pos[node]);
    }
}

static uint16_t Heap_Pop(TimeSolver *s) {
    uint16_t top = s->heap[0];
    s->heap_pos[top] = HEAP_NONE;
    s->heap_size--;
    if (s->heap_size > 0) {
        Heap_Place(s, 0, s->heap[s->heap_size]);
        Heap_Down(s, 0);
    }
    return top;
}

// ---- 位置とノード ----

// 走行中の位置の種類
enum {
    POS_EDGE = 0, // 区画 cell に向き dir で入る境目
    POS_CENTER,   // 区画 cell の中心で向き dir
    POS_DIAG,     // 区画 cell に向き dir で入る境目を、ジグザグの続きの向き z(右 +1、左 −1)へ傾いて通る所
};

typedef struct {
    MazePos cell;
    Direction dir;
    uint8_t kind;
    int8_t z; // POS_DIAG のときだけ
} RunPos;

// 旋回の種類 k が始まる位置の種類
static uint8_t ClassKind(int k) {
    switch (k) {
        case TIME_CLASS_LARGE90:
        case TIME_CLASS_LARGE180: return POS_CENTER;
        case TIME_CLASS_SMALL90:
        case TIME_CLASS_D45IN:
        case TIME_CLASS_D135IN:   return POS_EDGE;
        default:                  return POS_DIAG;
    }
}

static uint16_t Node(RunPos q, int k) {
    uint16_t base = MazeNode_Index(q.cell, q.dir);
    if (k < TIME_CLASS_ORTH_COUNT) return (uint16_t)(base * TIME_CLASS_ORTH_COUNT + (uint16_t)k);
    uint16_t lean = (uint16_t)(base * 2u + ((q.z > 0) ? 0u : 1u));
    return (uint16_t)(TIME_ORTH_NODE_COUNT + lean * TIME_CLASS_DIAG_COUNT + (uint16_t)(k - TIME_CLASS_ORTH_COUNT));
}

static void NodeDecode(uint16_t u, RunPos *q, int *k) {
    uint16_t base;
    if (u < TIME_ORTH_NODE_COUNT) {
        base = (uint16_t)(u / TIME_CLASS_ORTH_COUNT);
        *k = (int)(u % TIME_CLASS_ORTH_COUNT);
        q->z = 0;
    } else {
        uint16_t r = (uint16_t)(u - TIME_ORTH_NODE_COUNT);
        *k = TIME_CLASS_ORTH_COUNT + (int)(r % TIME_CLASS_DIAG_COUNT);
        r = (uint16_t)(r / TIME_CLASS_DIAG_COUNT);
        q->z = (r % 2u == 0u) ? 1 : -1;
        base = (uint16_t)(r / 2u);
    }
    q->cell = MazeNode_Pos(base);
    q->dir = MazeNode_Dir(base);
    q->kind = ClassKind(*k);
}

// 旋回の種類(左右は区別しない)。旋回でなければ −1
static int TurnClass(RunType t) {
    switch (t) {
        case RUN_SMALL90_R:
        case RUN_SMALL90_L:  return TIME_CLASS_SMALL90;
        case RUN_LARGE90_R:
        case RUN_LARGE90_L:  return TIME_CLASS_LARGE90;
        case RUN_LARGE180_R:
        case RUN_LARGE180_L: return TIME_CLASS_LARGE180;
        case RUN_D45IN_R:
        case RUN_D45IN_L:    return TIME_CLASS_D45IN;
        case RUN_D135IN_R:
        case RUN_D135IN_L:   return TIME_CLASS_D135IN;
        case RUN_D45OUT_R:
        case RUN_D45OUT_L:   return TIME_CLASS_D45OUT;
        case RUN_D135OUT_R:
        case RUN_D135OUT_L:  return TIME_CLASS_D135OUT;
        case RUN_DV90_R:
        case RUN_DV90_L:     return TIME_CLASS_DV90;
        case RUN_DV90P_R:
        case RUN_DV90P_L:    return TIME_CLASS_DV90P;
        default:             return -1;
    }
}

// 旋回の種類ごとの代表(右)。速度・オフセットを引くのに使う
static const RunType kClassType[TIME_CLASS_COUNT] = {
    RUN_SMALL90_R, RUN_LARGE90_R, RUN_LARGE180_R, RUN_D45IN_R, RUN_D135IN_R,
    RUN_D45OUT_R, RUN_D135OUT_R, RUN_DV90_R, RUN_DV90P_R,
};

static uint8_t StartKind(RunType t) {
    return RunType_IsLarge(t) ? POS_CENTER : RunType_StartsDiag(t) ? POS_DIAG : POS_EDGE;
}

static uint8_t EndKind(RunType t) {
    return RunType_IsLarge(t) ? POS_CENTER : RunType_EndsDiag(t) ? POS_DIAG : POS_EDGE;
}

// 斜めの点から始まる旋回 t を始められる、その点のジグザグの続きの向き
// (出・柱を回る V90 は続きの向きへ、V90 は続きと逆の向きへ曲がる。run_path.h の表)
static int StartZ(RunType t) {
    int s = (RunType_TurnDeg(t) > 0.0f) ? 1 : -1;
    return (t == RUN_DV90_R || t == RUN_DV90_L) ? -s : s;
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

static bool IsGoal(const Ctx *c, MazePos p) {
    return MazePos_InList(p, c->goals, c->goal_count);
}

static void Relax(Ctx *c, uint16_t v, uint32_t t, uint8_t type, uint8_t halves, uint8_t next_class) {
    if (t >= c->s->time_us[v]) return;
    c->s->time_us[v] = t;
    c->s->next[v].type = type;
    c->s->next[v].halves = halves;
    c->s->next[v].next_class = next_class;
    Heap_PushOrDecrease(c->s, v);
}

// 直交の位置を進行方向と逆へ半区画戻す。境目 → 手前の区画の中心 へ戻るときは、
// 壁がないこと・その区画がゴールでないこと(ゴールを通り抜けない)を確かめる。
static bool StepBack(const Ctx *c, RunPos *q) {
    if (q->kind == POS_CENTER) {
        q->kind = POS_EDGE; // 中心 → 同じ区画に入る境目
        return true;
    }
    MazePos p;
    if (!MazePos_Step(q->cell, Dir_Opposite(q->dir), &p)) return false;
    if (WallMap_HasWall(c->map, p, q->dir, c->view)) return false;
    if (IsGoal(c, p)) return false;
    q->cell = p;
    q->kind = POS_CENTER;
    return true;
}

// 斜めの点を斜めの1区間前へ戻す。1つ手前の区画 p に向き Dir_Turn(dir, z) で入り、p を dir へ抜けて今の点に来た
// (p では続きの向きが −z)。p と今の区画の間に壁がないこと・p がゴールでないことを確かめる。
static bool StepBackDiag(const Ctx *c, RunPos *q) {
    MazePos p;
    if (!MazePos_Step(q->cell, Dir_Opposite(q->dir), &p)) return false;
    if (WallMap_HasWall(c->map, p, q->dir, c->view)) return false;
    if (IsGoal(c, p)) return false;
    q->cell = p;
    q->dir = Dir_Turn(q->dir, q->z);
    q->z = (int8_t)-q->z;
    return true;
}

// 位置qを出口とする旋回typeの、入口の位置を求める。
// 旋回が通る区画の間に壁がなく、ゴールの区画を通り抜けなければtrue。
static bool TurnEntry(const Ctx *c, RunPos q, RunType type, RunPos *entry) {
    if (EndKind(type) != q.kind) return false;
    if (q.kind == POS_DIAG && RunType_DiagNextTurn(type) != q.z) return false;
    bool large = RunType_IsLarge(type);

    // 最後に区画を移る向きが q.dir になる入口の向きを探す(旋回ごとに1つだけ)
    for (int din = 0; din < 4; din++) {
        Direction moves[RUN_TURN_MOVES_MAX];
        uint8_t n = RunType_TurnMoves(type, (Direction)din, moves);
        if (n == 0 || moves[n - 1] != q.dir) continue;
        if (large) n--; // 大回りは最後の区画の中心で終わる

        // 出口から逆にたどって、旋回の入口の区画を求める
        MazePos a = q.cell;
        bool ok = true;
        for (int i = (int)n - 1; i >= 0 && ok; i--) ok = MazePos_Step(a, Dir_Opposite(moves[i]), &a);
        if (!ok) return false;
        // 入口から前向きにたどって確かめる
        MazePos cur = a;
        for (uint8_t i = 0; i < n; i++) {
            if (IsGoal(c, cur)) return false;
            if (WallMap_HasWall(c->map, cur, moves[i], c->view)) return false;
            if (!MazePos_Step(cur, moves[i], &cur)) return false;
        }
        if (large && IsGoal(c, q.cell)) return false; // ゴールの区画の中心で曲がり終えない

        entry->cell = a;
        entry->dir = (Direction)din;
        entry->kind = StartKind(type);
        entry->z = (int8_t)((entry->kind == POS_DIAG) ? StartZ(type) : 0);
        return true;
    }
    return false;
}

// 位置qを出口とする全ての旋回について、その入口のノードへ
// 「旋回の時間 + tail_us[旋回の速度の種類]」を入れる(TIME_INF の種類は使わない)。
// halves と next_class は、旋回の後の直進の長さと、その後のノードの種類(TIME_NEXT_STOP 可)。
static void RelaxTurnsInto(Ctx *c, RunPos q, const uint32_t *tail_us, uint8_t halves, uint8_t next_class) {
    static const RunType kTurns[] = {
        RUN_SMALL90_R, RUN_SMALL90_L, RUN_LARGE90_R, RUN_LARGE90_L, RUN_LARGE180_R, RUN_LARGE180_L,
        // ここから斜め(use_diag のときだけ)
        RUN_D45IN_R, RUN_D45IN_L, RUN_D135IN_R, RUN_D135IN_L, RUN_D45OUT_R, RUN_D45OUT_L,
        RUN_D135OUT_R, RUN_D135OUT_L, RUN_DV90_R, RUN_DV90_L, RUN_DV90P_R, RUN_DV90P_L,
    };
    unsigned count = c->s->use_diag ? (unsigned)(sizeof(kTurns) / sizeof(kTurns[0])) : 6u;
    for (unsigned i = 0; i < count; i++) {
        RunType type = kTurns[i];
        int kt = TurnClass(type);
        if (tail_us[kt] == TIME_INF) continue;
        RunPos e;
        if (!TurnEntry(c, q, type, &e)) continue;
        Relax(c, Node(e, kt), c->s->turn_us[type] + tail_us[kt], (uint8_t)type, halves, next_class);
    }
}

// ゴールの区画gの中心(向きdで入って止まる)からまっすぐ戻った各位置について、
// そこを出口とする旋回の入口へ「旋回 + まっすぐ進んでゴールの中心で止まる」時間を入れる
static void SeedGoal(Ctx *c, MazePos g, Direction d) {
    RunPos q = { g, d, POS_CENTER, 0 };
    for (uint8_t k = 1; k <= HALVES_MAX; k++) {
        if (!StepBack(c, &q)) break;
        uint32_t tail[TIME_CLASS_COUNT];
        for (int kt = 0; kt < TIME_CLASS_COUNT; kt++) tail[kt] = c->s->straight_us[k][kt][SPEED_STOP];
        RelaxTurnsInto(c, q, tail, k, TIME_NEXT_STOP);
    }
}

// uに入ってくる辺(旋回 + 0個以上の直進。斜めの点なら斜めの直進)をたどる
static void RelaxPreds(Ctx *c, uint16_t u, RunPos q, int k) {
    uint32_t t_u = c->s->time_us[u];
    bool diag = (q.kind == POS_DIAG);
    for (uint8_t h = 0; h <= HALVES_MAX; h++) {
        if (h > 0 && !(diag ? StepBackDiag(c, &q) : StepBack(c, &q))) break;
        uint32_t tail[TIME_CLASS_COUNT];
        for (int kt = 0; kt < TIME_CLASS_COUNT; kt++) {
            // h = 0 は旋回どうしが直接つながる場合(前後のオフセットの中で速度を変える)
            uint32_t st = diag ? c->s->diag_us[h][kt][k] : c->s->straight_us[h][kt][k];
            tail[kt] = (st == TIME_INF) ? TIME_INF : t_u + st;
        }
        RelaxTurnsInto(c, q, tail, h, (uint8_t)k);
    }
}

void TimeDijkstra_Compute(TimeSolver *s, const WallMap *map, WallView view, const RunProfile *prof,
                          const MazePos *goals, uint8_t goal_count,
                          MazePos start, Direction heading) {
    TimeDijkstra_ComputeEx(s, map, view, prof, goals, goal_count, start, heading, false);
}

void TimeDijkstra_ComputeEx(TimeSolver *s, const WallMap *map, WallView view, const RunProfile *prof,
                            const MazePos *goals, uint8_t goal_count,
                            MazePos start, Direction heading, bool use_diag) {
    RunProfile default_prof = RunProfile_Default();
    if (prof == NULL) prof = &default_prof;
    s->use_diag = use_diag;

    // 速度と、直進の時間の表(直進は半区画、斜めは斜めの1区間ごと)
    float speed[SPEED_COUNT], off_post[SPEED_COUNT], off_pre[SPEED_COUNT];
    for (int k = 0; k < TIME_CLASS_COUNT; k++) {
        // 速度の種類ごとの旋回のオフセット(直進の入りは前の旋回の後オフセット、出は次の旋回の前オフセット)
        speed[k] = prof->v_turn[kClassType[k]];
        off_post[k] = prof->turn_post[kClassType[k]];
        off_pre[k] = prof->turn_pre[kClassType[k]];
    }
    speed[SPEED_STOP] = off_post[SPEED_STOP] = off_pre[SPEED_STOP] = 0.0f;
    for (int h = 0; h <= HALVES_MAX; h++) {
        for (int a = 0; a < SPEED_COUNT; a++) {
            for (int b = 0; b < SPEED_COUNT; b++) {
                float t;
                // h = 0 は旋回どうしが直接つながる場合(前後のオフセットだけで速度を変える)
                bool ok = RunProfile_LinkedStraightTime(prof, (float)h * SECTION_MM * 0.5f,
                                                        speed[a], off_post[a], speed[b], off_pre[b], &t);
                s->straight_us[h][a][b] = ok ? SecondsToUs(t) : TIME_INF;
                ok = RunProfile_LinkedStraightTime(prof, (float)h * DIAG_MM,
                                                   speed[a], off_post[a], speed[b], off_pre[b], &t);
                s->diag_us[h][a][b] = ok ? SecondsToUs(t) : TIME_INF;
            }
        }
    }
    for (int t = 0; t < RUN_TYPE_COUNT; t++) {
        s->turn_us[t] = SecondsToUs(RunProfile_CurveTime(prof, (RunType)t));
    }

    // 初期化
    for (uint16_t i = 0; i < TIME_NODE_COUNT; i++) {
        s->time_us[i] = TIME_INF;
        s->next[i].type = 0xFF;
        s->next[i].halves = 0;
        s->next[i].next_class = 0;
        s->heap_pos[i] = HEAP_NONE;
    }
    s->heap_size = 0;

    Ctx c = { s, map, view, goals, goal_count };

    // ゴールの区画の中心で止まる直前の旋回
    for (uint8_t i = 0; i < goal_count; i++) {
        for (int d = 0; d < 4; d++) SeedGoal(&c, goals[i], (Direction)d);
    }

    // ゴール側から広げる
    while (s->heap_size > 0) {
        uint16_t u = Heap_Pop(s);
        RunPos q;
        int k;
        NodeDecode(u, &q, &k);
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
    RunPos q = { start, heading, POS_CENTER, 0 };
    for (uint8_t h = 1; h <= HALVES_MAX; h++) {
        // 半区画進む
        if (q.kind == POS_CENTER) {
            MazePos p;
            if (WallMap_HasWall(map, q.cell, heading, view)) break;
            if (!MazePos_Step(q.cell, heading, &p)) break;
            q.cell = p;
            q.kind = POS_EDGE;
        } else {
            q.kind = POS_CENTER;
            if (IsGoal(&c, q.cell)) { // ゴールの中心で止まる
                uint32_t t = s->straight_us[h][SPEED_STOP][SPEED_STOP];
                if (t < s->start_time_us) {
                    s->start_time_us = t;
                    s->start_halves = h;
                    s->start_to_goal = true;
                }
                break;
            }
        }
        for (int k = 0; k < TIME_CLASS_ORTH_COUNT; k++) {
            if (ClassKind(k) != q.kind) continue;
            uint32_t rest = s->time_us[Node(q, k)];
            uint32_t t = s->straight_us[h][SPEED_STOP][k];
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
    RunPos q = { p, d, POS_EDGE, 0 };
    for (int k = 0; k < TIME_CLASS_ORTH_COUNT; k++) {
        uint32_t t = s->time_us[Node(q, k)];
        if (t < best) best = t;
    }
    return best;
}

// 位置を h 回進める(直交は半区画、斜めの点は斜めの1区間。経路は計算で確かめてあるので、迷路の外には出ない)
static bool StepForward(RunPos *q, uint8_t h) {
    for (uint8_t i = 0; i < h; i++) {
        if (q->kind == POS_DIAG) {
            Direction d = Dir_Turn(q->dir, q->z); // 入った区画を続きの向きへ抜ける
            if (!MazePos_Step(q->cell, d, &q->cell)) return false;
            q->dir = d;
            q->z = (int8_t)-q->z;
        } else if (q->kind == POS_CENTER) {
            if (!MazePos_Step(q->cell, q->dir, &q->cell)) return false;
            q->kind = POS_EDGE;
        } else {
            q->kind = POS_CENTER;
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
    RunPos q = { s->start, s->start_heading, POS_CENTER, 0 };
    if (!StepForward(&q, s->start_halves)) return false;
    int k = s->start_class;

    // ノードの数より多く回ることはない(時間は辺ごとに必ず減る)
    for (uint16_t guard = 0; guard < TIME_NODE_COUNT; guard++) {
        TimeNext nx = s->next[Node(q, k)];
        if (nx.type == 0xFF) return false;
        RunType type = (RunType)nx.type;

        // 旋回
        RunCommand turn = { nx.type, 0 };
        if (!RunList_Push(out, turn)) return false;
        bool large = RunType_IsLarge(type);
        Direction moves[RUN_TURN_MOVES_MAX];
        uint8_t n = RunType_TurnMoves(type, q.dir, moves);
        if (large) n--; // 大回りは最後の区画の中心で終わる
        for (uint8_t i = 0; i < n; i++) {
            if (!MazePos_Step(q.cell, moves[i], &q.cell)) return false;
        }
        q.dir = moves[n - 1];
        q.kind = EndKind(type);
        q.z = (int8_t)((q.kind == POS_DIAG) ? RunType_DiagNextTurn(type) : 0);

        // その後の直進(0なら次の旋回へ直接つながる)
        if (nx.halves > 0) {
            RunCommand straight = { (uint8_t)((q.kind == POS_DIAG) ? RUN_DIAG : RUN_STRAIGHT), nx.halves };
            if (!RunList_Push(out, straight)) return false;
        }
        if (nx.next_class == TIME_NEXT_STOP) return RunList_Push(out, stop);
        if (!StepForward(&q, nx.halves)) return false;
        k = nx.next_class;
    }
    return false;
}
