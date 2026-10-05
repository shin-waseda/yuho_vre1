#include "logic/maze/dijkstra.h"

#include <stddef.h>
#include "logic/maze/priority_queue.h"

// params.h の既定値が13bitのコストに収まることを確かめる
_Static_assert(MAZE_COST_FITS(MAZE_COST_STRAIGHT, MAZE_COST_TURN90, MAZE_COST_TURN180,
                              MAZE_COST_KNOWN_CELL_RETURN),
               "maze costs in params.h are too large for 13-bit MazeNodeState.cost");

// 計算用の作業領域(4KB)。全てのMazeSolverで共有する。
static PriorityQueue s_pq;

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

void Dijkstra_Compute(MazeSolver *s, const WallMap *map, WallView view, const MazeCost *cost,
                      const MazePos *goals, uint8_t goal_count) {
    MazeCost default_cost = MazeCost_Default();
    if (cost == NULL) cost = &default_cost;

    MazeNodeState unreached;
    unreached.raw = 0;
    unreached.f.cost = MAZE_COST_INF;
    unreached.f.has_next = 0;
    for (uint16_t i = 0; i < MAZE_NODE_COUNT; i++) {
        s->node[i] = unreached;
    }
    PQ_Init(&s_pq, s->node);

    // ゴールの区画は、どの向きで入ってもコスト0
    for (uint8_t i = 0; i < goal_count; i++) {
        for (int d = 0; d < 4; d++) {
            uint16_t n = MazeNode_Index(goals[i], (Direction)d);
            s->node[n].f.cost = 0;
            PQ_PushOrUpdate(&s_pq, n);
        }
    }

    while (!PQ_IsEmpty(&s_pq)) {
        uint16_t u = PQ_Pop(&s_pq);
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
                PQ_PushOrUpdate(&s_pq, v);
            }
        }
    }
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
