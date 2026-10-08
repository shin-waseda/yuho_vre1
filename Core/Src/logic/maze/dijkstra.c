#include "logic/maze/dijkstra.h"

#include <stddef.h>
#include "logic/maze/priority_queue.h"
#include "logic/maze/bucket_queue.h"

// params.h の既定値が13bitのコストに収まることを確かめる
_Static_assert(MAZE_COST_FITS(MAZE_COST_STRAIGHT, MAZE_COST_TURN90, MAZE_COST_TURN180,
                              MAZE_COST_KNOWN_CELL_RETURN),
               "maze costs in params.h are too large for 13-bit MazeNodeState.cost");

// 1区画で増えるコストの最大(直進 + 既知の区画 + 曲がりの大きい方)
#define MAZE_COST_MAX_STEP(s, t90, t180, k) ((s) + (k) + (((t90) > (t180)) ? (t90) : (t180)))

// params.h の既定値なら、速いバケツのキューが使えることを確かめる
_Static_assert(MAZE_COST_MAX_STEP(MAZE_COST_STRAIGHT, MAZE_COST_TURN90, MAZE_COST_TURN180,
                                  MAZE_COST_KNOWN_CELL_RETURN) <= BQ_MAX_STEP,
               "maze costs in params.h are too large for the bucket queue (it falls back to the heap)");

// 計算用の作業領域(約5KB)。全てのMazeSolverで共有する。
// コストがバケツに収まればバケツのキュー(速い)、収まらなければヒープを使う(同時には使わないので重ねる)。
static union {
    PriorityQueue heap;
    BucketQueue bucket;
} s_q;
static bool s_use_bucket;

static void Q_Init(const MazeNodeState *state) {
    if (s_use_bucket) BQ_Init(&s_q.bucket, state);
    else PQ_Init(&s_q.heap, state);
}

static void Q_PushOrUpdate(uint16_t node) {
    if (s_use_bucket) BQ_PushOrUpdate(&s_q.bucket, node);
    else PQ_PushOrUpdate(&s_q.heap, node);
}

static bool Q_IsEmpty(void) {
    return s_use_bucket ? BQ_IsEmpty(&s_q.bucket) : PQ_IsEmpty(&s_q.heap);
}

static uint16_t Q_Pop(void) {
    return s_use_bucket ? BQ_Pop(&s_q.bucket) : PQ_Pop(&s_q.heap);
}

MazeCost MazeCost_Default(void) {
    MazeCost c = { MAZE_COST_STRAIGHT, MAZE_COST_TURN90, MAZE_COST_TURN180, 0 };
    return c;
}

// 向きをfromからtoへ変えるときに上乗せするコスト
static uint16_t TurnCost(const MazeCost *cost, Direction from, Direction to) {
    switch (((int)to - (int)from) & 3) {
        case 0:  return 0;
        case 2:  return cost->turn180;
        default: return cost->turn90;
    }
}

// 同じコストで行ける向きが2つあるとき、どちらを選ぶかの順位(小さい方を選ぶ)。
// 曲がりの小さい向き、同じならDirectionの番号の小さい向き。
// こう決めておくと、結果がキューから取り出す順によらない(ヒープでもバケツでも同じになる)。
static uint32_t TieRank(const MazeCost *cost, Direction now, Direction next) {
    return (uint32_t)TurnCost(cost, now, next) * 4u + (uint32_t)next;
}

// stop がMAZE_NODE_NONEでなければ、そのノードを取り出した(コストが決まった)所で止める
static void Compute(MazeSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                    const MazePos *goals, uint8_t goal_count, uint16_t stop) {
    MazeCost default_cost = MazeCost_Default();
    if (cost == NULL) cost = &default_cost;
    s_use_bucket = MAZE_COST_MAX_STEP((uint32_t)cost->straight, (uint32_t)cost->turn90,
                                      (uint32_t)cost->turn180, (uint32_t)cost->known_cell) <= BQ_MAX_STEP;

    MazeNodeState unreached;
    unreached.raw = 0;
    unreached.f.cost = MAZE_COST_INF;
    unreached.f.has_next = 0;
    for (uint16_t i = 0; i < MAZE_NODE_COUNT; i++) {
        s->node[i] = unreached;
    }
    Q_Init(s->node);

    // ゴールの区画は、どの向きで入ってもコスト0
    for (uint8_t i = 0; i < goal_count; i++) {
        for (int d = 0; d < 4; d++) {
            uint16_t n = MazeNode_Index(goals[i], (Direction)d);
            s->node[n].f.cost = 0;
            Q_PushOrUpdate(n);
        }
    }

    while (!Q_IsEmpty()) {
        uint16_t u = Q_Pop();
        // 取り出したノードのコストと次の向きは、もう変わらない
        // (そこへ来る辺はどれもコストが1以上なので、残りのノードから同じコストで来ることはない)
        if (u == stop) break;
        MazePos c = MazeNode_Pos(u);
        Direction d = MazeNode_Dir(u);

        // uに入ってくるのは、後ろの区画pから向きdで進んできたとき
        MazePos p;
        if (!MazePos_Step(c, Dir_Opposite(d), &p)) continue;
        if (WallMap_HasWall(map, p, d, view)) continue;

        uint32_t enter = cost->straight;
        if (cost->known_cell > 0 && WallMap_IsCellKnown(map, c)) {
            enter += cost->known_cell;
        }

        // pにどの向きでいても、dへ向きを変えて1区画進めばuになる
        for (int pd = 0; pd < 4; pd++) {
            uint32_t c_new = (uint32_t)s->node[u].f.cost + enter + TurnCost(cost, (Direction)pd, d);
            uint16_t v = MazeNode_Index(p, (Direction)pd);
            if (c_new < s->node[v].f.cost) { // INF以上になる値は入らない(13bitに収まる)
                s->node[v].f.cost = (uint16_t)c_new;
                s->node[v].f.next_dir = (uint16_t)d;
                s->node[v].f.has_next = 1;
                Q_PushOrUpdate(v);
            } else if (c_new == s->node[v].f.cost && s->node[v].f.has_next &&
                       TieRank(cost, (Direction)pd, d) <
                           TieRank(cost, (Direction)pd, (Direction)s->node[v].f.next_dir)) {
                s->node[v].f.next_dir = (uint16_t)d; // 同じコスト: 決まりで選ぶ
            }
        }
    }
}

void Dijkstra_Compute(MazeSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                      const MazePos *goals, uint8_t goal_count) {
    Compute(s, map, view, cost, goals, goal_count, MAZE_NODE_NONE);
}

void Dijkstra_ComputeFrom(MazeSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                          const MazePos *goals, uint8_t goal_count, MazePos from, Direction heading) {
    Compute(s, map, view, cost, goals, goal_count, MazeNode_Index(from, heading));
}

uint16_t Dijkstra_Cost(const MazeSolver *s, MazePos p, Direction heading) {
    return s->node[MazeNode_Index(p, heading)].f.cost;
}

bool Dijkstra_NextDir(const MazeSolver *s, MazePos p, Direction heading, Direction *next) {
    MazeNodeState st = s->node[MazeNode_Index(p, heading)];
    if (!st.f.has_next) return false;
    *next = (Direction)st.f.next_dir;
    return true;
}

bool Dijkstra_BuildRoute(const MazeSolver *s, MazePos start, Direction heading,
                         bool merge_forward, CommandList *out) {
    CommandList_Clear(out);
    if (Dijkstra_Cost(s, start, heading) == MAZE_COST_INF) return false;

    // 次の状態をたどるたびにコストは減る(straight >= 1 のとき)ので、
    // ノード数より多く回ることはない。念のため回数でも打ち切る。
    MazePos p = start;
    Direction d;
    for (uint16_t guard = 0; guard < MAZE_NODE_COUNT && Dijkstra_NextDir(s, p, heading, &d); guard++) {
        Action a = Action_Move((int)d - (int)heading);
        if (!CommandList_Push(out, a, merge_forward)) return false;
        if (!MazePos_Step(p, d, &p)) return false; // 計算が正しければ起きない
        heading = d;
    }

    Action stop = { ACTION_STOP, 0 };
    return CommandList_Push(out, stop, false);
}
