#include "logic/maze/diag_route.h"

#include <stddef.h>

#define DR_INF    0xFFFFu
#define HEAP_NONE 0xFFFFu
#define NEXT_HAS  0x04u

// 区画に入るときの曲がり
enum { LAST_NONE = 0, LAST_RIGHT, LAST_LEFT };

static inline uint16_t Node(MazePos p, Direction d, int last) {
    return (uint16_t)(MazeNode_Index(p, d) * 3u + (uint16_t)last);
}

// 向きを from から to へ変える曲がり(180° と曲がらないときは LAST_NONE)
static int LastOf(Direction from, Direction to) {
    switch (((int)to - (int)from) & 3) {
        case 1:  return LAST_RIGHT;
        case 3:  return LAST_LEFT;
        default: return LAST_NONE;
    }
}

// 入るときの曲がりが last の区画で、向きを from から to へ変えて抜けるのがジグザグの続きか
static bool IsZigzag(int last, Direction from, Direction to) {
    int q = ((int)to - (int)from) & 3;
    return (q == 1 && last == LAST_LEFT) || (q == 3 && last == LAST_RIGHT);
}

// 区画を、入るときの曲がりが last で、向きを from から to へ変えて抜けるときの、直進以外の上乗せ(10 倍)
static uint32_t TurnCost10(const MazeCost *c, int last, Direction from, Direction to) {
    (void)last;
    switch (((int)to - (int)from) & 3) {
        case 0:  return 0;
        case 2:  return 10u * c->turn180;
        default: return 10u * c->turn90;
    }
}

// ---- 二分ヒープ(コストをキーにする) ----

static void Heap_Place(DiagRouteSolver *s, uint16_t i, uint16_t node) {
    s->heap[i] = node;
    s->heap_pos[node] = i;
}

static void Heap_Up(DiagRouteSolver *s, uint16_t i) {
    uint16_t node = s->heap[i];
    while (i > 0) {
        uint16_t parent = (uint16_t)((i - 1u) / 2u);
        if (s->cost[s->heap[parent]] <= s->cost[node]) break;
        Heap_Place(s, i, s->heap[parent]);
        i = parent;
    }
    Heap_Place(s, i, node);
}

static void Heap_Down(DiagRouteSolver *s, uint16_t i) {
    uint16_t node = s->heap[i];
    for (;;) {
        uint16_t child = (uint16_t)(2u * i + 1u);
        if (child >= s->heap_size) break;
        if (child + 1u < s->heap_size && s->cost[s->heap[child + 1u]] < s->cost[s->heap[child]]) child++;
        if (s->cost[node] <= s->cost[s->heap[child]]) break;
        Heap_Place(s, i, s->heap[child]);
        i = child;
    }
    Heap_Place(s, i, node);
}

static void Heap_PushOrDecrease(DiagRouteSolver *s, uint16_t node) {
    if (s->heap_pos[node] == HEAP_NONE) {
        uint16_t i = s->heap_size++;
        Heap_Place(s, i, node);
        Heap_Up(s, i);
    } else {
        Heap_Up(s, s->heap_pos[node]);
    }
}

// ---- バケツのキュー(Dial の方法)。heap[] を次、heap_pos[] を前のノードに使う(先頭は BUCKET_HEAD、入っていなければ HEAP_NONE) ----
#define BUCKET_HEAD 0xFFFEu
#define BUCKET_NIL  0xFFFFu

static void Bucket_Insert(DiagRouteSolver *s, uint16_t node) {
    uint16_t b = (uint16_t)(s->cost[node] & (DIAG_ROUTE_BUCKETS - 1u));
    uint16_t h = s->bucket_head[b];
    s->heap[node] = h;
    s->heap_pos[node] = BUCKET_HEAD;
    if (h != BUCKET_NIL) s->heap_pos[h] = node;
    s->bucket_head[b] = node;
    s->heap_size++;
}

// 今のコストの箱から外す(コストを変える前に呼ぶ)
static void Bucket_Remove(DiagRouteSolver *s, uint16_t node) {
    uint16_t prev = s->heap_pos[node];
    uint16_t next = s->heap[node];
    if (prev == BUCKET_HEAD) s->bucket_head[s->cost[node] & (DIAG_ROUTE_BUCKETS - 1u)] = next;
    else s->heap[prev] = next;
    if (next != BUCKET_NIL) s->heap_pos[next] = prev;
    s->heap_pos[node] = HEAP_NONE;
    s->heap_size--;
}

// キューの中のコストは「今のコスト〜今のコスト + 1区画の上乗せの最大」に収まるので、箱は混ざらない
static uint16_t Bucket_Pop(DiagRouteSolver *s) {
    while (s->bucket_head[s->bucket_cur & (DIAG_ROUTE_BUCKETS - 1u)] == BUCKET_NIL) s->bucket_cur++;
    uint16_t node = s->bucket_head[s->bucket_cur & (DIAG_ROUTE_BUCKETS - 1u)];
    Bucket_Remove(s, node);
    return node;
}

// ノードのコストを c にして、キューに入れる(入っていれば位置を直す)
static void Q_Set(DiagRouteSolver *s, uint16_t node, uint16_t c) {
    if (s->use_bucket) {
        if (s->heap_pos[node] != HEAP_NONE) Bucket_Remove(s, node);
        s->cost[node] = c;
        Bucket_Insert(s, node);
    } else {
        s->cost[node] = c;
        Heap_PushOrDecrease(s, node);
    }
}

static uint16_t Heap_Pop(DiagRouteSolver *s) {
    uint16_t top = s->heap[0];
    s->heap_pos[top] = HEAP_NONE;
    s->heap_size--;
    if (s->heap_size > 0) {
        Heap_Place(s, 0, s->heap[s->heap_size]);
        Heap_Down(s, 0);
    }
    return top;
}

// stop が 0xFFFF でなければ、そのノードを取り出した(コストが決まった)所で止める
static void Compute(DiagRouteSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                    uint16_t diag_cell_x10, const MazePos *goals, uint8_t goal_count, uint16_t stop) {
    MazeCost default_cost = MazeCost_Default();
    if (cost == NULL) cost = &default_cost;

    // 1区画で増えるコストの最大(10 倍)が箱の数に収まれば、バケツのキュー
    uint32_t turn_max = (cost->turn90 > cost->turn180) ? cost->turn90 : cost->turn180;
    uint32_t step_max = 10u * ((uint32_t)cost->straight + turn_max);
    if (diag_cell_x10 > step_max) step_max = diag_cell_x10;
    step_max += 10u * (uint32_t)cost->known_cell;
    s->use_bucket = !s->heap_only && step_max < DIAG_ROUTE_BUCKETS;
    s->bucket_cur = 0;
    for (uint16_t b = 0; b < DIAG_ROUTE_BUCKETS; b++) s->bucket_head[b] = BUCKET_NIL;

    for (uint16_t i = 0; i < DIAG_ROUTE_NODE_COUNT; i++) {
        s->cost[i] = DR_INF;
        s->next[i] = 0;
        s->heap_pos[i] = HEAP_NONE;
    }
    s->heap_size = 0;

    // ゴールの区画は、どの向き・曲がりで入ってもコスト0
    for (uint8_t i = 0; i < goal_count; i++) {
        for (int d = 0; d < 4; d++) {
            for (int last = 0; last < 3; last++) {
                uint16_t n = Node(goals[i], (Direction)d, last);
                if (s->heap_pos[n] == HEAP_NONE) Q_Set(s, n, 0); // 同じゴールが2回あっても1回だけ
            }
        }
    }

    while (s->heap_size > 0) {
        uint16_t u = s->use_bucket ? Bucket_Pop(s) : Heap_Pop(s);
        if (u == stop) break; // 取り出したノードのコストと次の向きは、もう変わらない
        uint16_t cell_dir = (uint16_t)(u / 3u);
        int last = (int)(u % 3u);
        MazePos c = MazeNode_Pos(cell_dir);
        Direction d = MazeNode_Dir(cell_dir);

        // u に入ってくるのは、後ろの区画 p から向き d で進んできたとき
        MazePos p;
        if (!MazePos_Step(c, Dir_Opposite(d), &p)) continue;
        if (WallMap_HasWall(map, p, d, view)) continue;
        uint32_t known = (cost->known_cell > 0 && WallMap_IsCellKnown(map, c)) ? 10u * cost->known_cell : 0u;

        // p での向き pd は、u に入るときの曲がり last で決まる(曲がらないときは、まっすぐか 180°)
        Direction pds[2];
        int npd = 0;
        if (last == LAST_RIGHT) {
            pds[npd++] = Dir_Turn(d, -1);
        } else if (last == LAST_LEFT) {
            pds[npd++] = Dir_Turn(d, 1);
        } else {
            pds[npd++] = d;
            pds[npd++] = Dir_Opposite(d);
        }
        for (int k = 0; k < npd; k++) {
            for (int lp = 0; lp < 3; lp++) {
                // ジグザグの続きの区画は 直進 + 90° の代わりに diag_cell_x10
                bool zig = diag_cell_x10 > 0 && IsZigzag(lp, pds[k], d);
                uint32_t step = zig ? (uint32_t)diag_cell_x10 : 10u * cost->straight + TurnCost10(cost, lp, pds[k], d);
                uint32_t c_new = (uint32_t)s->cost[u] + step + known;
                uint16_t v = Node(p, pds[k], lp);
                if (c_new < s->cost[v]) { // 0xFFFF 以上になる値は入らない
                    s->next[v] = (uint8_t)(NEXT_HAS | (uint8_t)d);
                    Q_Set(s, v, (uint16_t)c_new);
                } else if (c_new == s->cost[v] && (s->next[v] & NEXT_HAS)) {
                    // 同じコスト: 曲がりの小さい向き、同じなら Direction の番号の小さい向き(dijkstra.c と同じ決まり)
                    Direction old = (Direction)(s->next[v] & 3u);
                    uint32_t rank_new = TurnCost10(cost, lp, pds[k], d) * 4u + (uint32_t)d;
                    uint32_t rank_old = TurnCost10(cost, lp, pds[k], old) * 4u + (uint32_t)old;
                    if (rank_new < rank_old) s->next[v] = (uint8_t)(NEXT_HAS | (uint8_t)d);
                }
            }
        }
    }
}

void DiagRoute_Compute(DiagRouteSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                       uint16_t diag_cell_x10, const MazePos *goals, uint8_t goal_count) {
    Compute(s, map, view, cost, diag_cell_x10, goals, goal_count, 0xFFFFu);
}

void DiagRoute_ComputeFrom(DiagRouteSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                           uint16_t diag_cell_x10, const MazePos *goals, uint8_t goal_count,
                           MazePos from, Direction heading) {
    Compute(s, map, view, cost, diag_cell_x10, goals, goal_count, Node(from, heading, LAST_NONE));
}

uint16_t DiagRoute_Cost(const DiagRouteSolver *s, MazePos p, Direction heading) {
    return s->cost[Node(p, heading, LAST_NONE)];
}

uint16_t DiagRoute_BuildDirs(const DiagRouteSolver *s, MazePos start, Direction heading, uint8_t *dirs, uint16_t max) {
    if (DiagRoute_Cost(s, start, heading) == DR_INF) return 0;
    MazePos p = start;
    int last = LAST_NONE;
    uint16_t n = 0;
    while (n < max) {
        uint8_t nx = s->next[Node(p, heading, last)];
        if (!(nx & NEXT_HAS)) break;
        Direction d = (Direction)(nx & 3u);
        if (!MazePos_Step(p, d, &p)) break; // 計算が正しければ起きない
        dirs[n++] = (uint8_t)d;
        last = LastOf(heading, d);
        heading = d;
    }
    return n;
}

bool DiagRoute_BuildRoute(const DiagRouteSolver *s, MazePos start, Direction heading, CommandList *out) {
    CommandList_Clear(out);
    if (DiagRoute_Cost(s, start, heading) == DR_INF) return false;

    // 次をたどるたびにコストは減る(straight >= 1、diag_cell_x10 >= 1)ので、ノード数より多く回ることはない
    MazePos p = start;
    int last = LAST_NONE;
    for (uint16_t guard = 0; guard < DIAG_ROUTE_NODE_COUNT; guard++) {
        uint8_t nx = s->next[Node(p, heading, last)];
        if (!(nx & NEXT_HAS)) break;
        Direction d = (Direction)(nx & 3u);
        if (!CommandList_Push(out, Action_Move((int)d - (int)heading), true)) return false;
        if (!MazePos_Step(p, d, &p)) return false; // 計算が正しければ起きない
        last = LastOf(heading, d);
        heading = d;
    }

    Action stop = { ACTION_STOP, 0 };
    return CommandList_Push(out, stop, false);
}
