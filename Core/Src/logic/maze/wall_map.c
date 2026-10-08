#include "logic/maze/wall_map.h"

// dの向きの壁のビット(下位4bit側)。上位4bit側はこれを4bit左へずらしたもの。
#define WALL_BIT(d) ((uint8_t)(0x08u >> (d)))

static void SetCellBits(uint8_t *cell, Direction d, bool exists) {
    uint8_t both = (uint8_t)(WALL_BIT(d) | (WALL_BIT(d) << 4));
    if (exists) {
        *cell |= both;
    } else {
        *cell &= (uint8_t)~both;
    }
}

void WallMap_Init(WallMap *map) {
    for (uint8_t y = 0; y < MAZE_SIZE; y++) {
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
            map->cell[y][x] = 0xF0; // 4方向とも未知
        }
    }
    for (uint8_t i = 0; i < MAZE_SIZE; i++) {
        SetCellBits(&map->cell[i][0], DIR_WEST, true);
        SetCellBits(&map->cell[i][MAZE_SIZE - 1], DIR_EAST, true);
        SetCellBits(&map->cell[0][i], DIR_SOUTH, true);
        SetCellBits(&map->cell[MAZE_SIZE - 1][i], DIR_NORTH, true);
    }
}

void WallMap_SetWall(WallMap *map, MazePos p, Direction d, bool exists) {
    MazePos n;
    bool inside = MazePos_Step(p, d, &n);
    if (!inside) {
        exists = true; // 外周の壁は消さない
    }
    SetCellBits(&map->cell[p.y][p.x], d, exists);
    if (inside) {
        SetCellBits(&map->cell[n.y][n.x], Dir_Opposite(d), exists);
    }
}

bool WallMap_HasWall(const WallMap *map, MazePos p, Direction d, WallView view) {
    uint8_t bit = WALL_BIT(d);
    if (view == WALL_VIEW_KNOWN) {
        bit = (uint8_t)(bit << 4);
    }
    return (map->cell[p.y][p.x] & bit) != 0;
}

bool WallMap_IsKnown(const WallMap *map, MazePos p, Direction d) {
    uint8_t c = map->cell[p.y][p.x];
    bool search_view = (c & WALL_BIT(d)) != 0;
    bool known_view = (c & (WALL_BIT(d) << 4)) != 0;
    return search_view == known_view;
}

bool WallMap_IsCellKnown(const WallMap *map, MazePos p) {
    // 未知の壁は「下位0・上位1」。そうなっているビットが1つもなければ全部既知
    uint8_t c = map->cell[p.y][p.x];
    uint8_t unknown = (uint8_t)((c >> 4) & (uint8_t)~c & 0x0Fu);
    return unknown == 0;
}

void WallMap_Observe(WallMap *map, MazePos p, Direction heading, WallObservation obs) {
    WallMap_SetWall(map, p, heading, obs.front);
    WallMap_SetWall(map, p, Dir_Turn(heading, 1), obs.right);
    WallMap_SetWall(map, p, Dir_Turn(heading, -1), obs.left);
}
