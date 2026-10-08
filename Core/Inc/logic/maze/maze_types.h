#ifndef INC_MAZE_TYPES_H_
#define INC_MAZE_TYPES_H_


#include "global.h"
#include "params.h"

// 迷路の座標と向き。logic/maze の全ファイルが使う。
// 座標: 区画(x, y)。スタート区画が(0, 0)、xが東、yが北へ増える。
// 向き: 北0 → 東1 → 南2 → 西3(時計回りに+1)。BlueEyesと同じ。
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3,
} Direction;

typedef struct {
    uint8_t x;
    uint8_t y;
} MazePos;

#define MAZE_CELL_COUNT (MAZE_SIZE * MAZE_SIZE)

// (区画, 向き)の組を1つの番号にしたもの。Dijkstraのノード番号に使う。
#define MAZE_NODE_COUNT (MAZE_CELL_COUNT * 4)
#define MAZE_NODE_NONE  0xFFFFu

// Dijkstraの1ノードの状態(2バイト)。
// 次の状態は「next_dirの向きの隣の区画に、向きnext_dirで入った状態」と決まるので、
// ノード番号(10bit)ではなく向き(2bit)だけを持てばよい。
#define MAZE_COST_BITS 13
#define MAZE_COST_INF  ((1u << MAZE_COST_BITS) - 1u) // 8191: 行けない

typedef union {
    uint16_t raw; // まとめて初期化するとき用
    struct {
        uint16_t cost     : MAZE_COST_BITS; // ゴールまでの最小コスト
        uint16_t next_dir : 2;              // ゴールへ向かう次の向き(Direction)
        uint16_t has_next : 1;              // next_dirが有効(ゴール・行けないノードは0)
    } f;
} MazeNodeState;

// dから時計回りにquarter_cw × 90°回った向き(負なら反時計回り)。
static inline Direction Dir_Turn(Direction d, int quarter_cw) {
    return (Direction)(((int)d + quarter_cw) & 3);
}

static inline Direction Dir_Opposite(Direction d) {
    return Dir_Turn(d, 2);
}

static inline bool MazePos_Equal(MazePos a, MazePos b) {
    return (a.x == b.x) && (a.y == b.y);
}

// pからdの向きに1区画進んだ区画をoutへ書く。迷路の外に出るならfalse(outは変えない)。
static inline bool MazePos_Step(MazePos p, Direction d, MazePos *out) {
    switch (d) {
        case DIR_NORTH: if (p.y + 1 >= MAZE_SIZE) return false; p.y++; break;
        case DIR_EAST:  if (p.x + 1 >= MAZE_SIZE) return false; p.x++; break;
        case DIR_SOUTH: if (p.y == 0) return false; p.y--; break;
        case DIR_WEST:  if (p.x == 0) return false; p.x--; break;
    }
    *out = p;
    return true;
}

static inline bool MazePos_InList(MazePos p, const MazePos *list, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        if (MazePos_Equal(p, list[i])) return true;
    }
    return false;
}

static inline uint16_t MazeNode_Index(MazePos p, Direction d) {
    return (uint16_t)(((uint16_t)p.y * MAZE_SIZE + p.x) * 4u + (uint16_t)d);
}

static inline MazePos MazeNode_Pos(uint16_t node) {
    uint16_t cell = node / 4u;
    MazePos p = { (uint8_t)(cell % MAZE_SIZE), (uint8_t)(cell / MAZE_SIZE) };
    return p;
}

static inline Direction MazeNode_Dir(uint16_t node) {
    return (Direction)(node & 3u);
}

#endif
