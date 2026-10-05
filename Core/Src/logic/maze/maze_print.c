#include "logic/maze/maze_print.h"

#include <stdio.h>

// pのdの向き(北か南)の横壁 / 東か西の縦壁を表す文字
static const char *HorizontalWall(const WallMap *map, MazePos p, Direction d) {
    if (!WallMap_IsKnown(map, p, d)) return " . ";
    return WallMap_HasWall(map, p, d, WALL_VIEW_KNOWN) ? "---" : "   ";
}

static char VerticalWall(const WallMap *map, MazePos p, Direction d) {
    if (!WallMap_IsKnown(map, p, d)) return ':';
    return WallMap_HasWall(map, p, d, WALL_VIEW_KNOWN) ? '|' : ' ';
}

static void PrintCell(MazePos p, const MazeSolver *solver, const MazePos *robot,
                      Direction heading, const MazePos *goals, uint8_t goal_count) {
    static const char *const kArrow[4] = { " ^ ", " > ", " v ", " < " };

    if (robot != NULL && MazePos_Equal(p, *robot)) {
        printf("%s", kArrow[heading]);
        return;
    }
    if (MazePos_InList(p, goals, goal_count)) {
        printf(" G ");
        return;
    }
    if (solver == NULL) {
        printf("   ");
        return;
    }

    uint16_t best = MAZE_COST_INF;
    for (int d = 0; d < 4; d++) {
        uint16_t c = Dijkstra_Cost(solver, p, (Direction)d);
        if (c < best) best = c;
    }
    if (best == MAZE_COST_INF) {
        printf(" * ");
    } else if (best >= 1000) {
        printf("###");
    } else {
        printf("%3u", (unsigned)best);
    }
}

void MazePrint_Map(const WallMap *map, const MazeSolver *solver,
                   const MazePos *robot, Direction heading,
                   const MazePos *goals, uint8_t goal_count) {
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        // 北側の壁
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
            MazePos p = { x, (uint8_t)y };
            printf("+%s", HorizontalWall(map, p, DIR_NORTH));
        }
        printf("+\r\n");

        // 西側の壁と区画の中身
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
            MazePos p = { x, (uint8_t)y };
            printf("%c", VerticalWall(map, p, DIR_WEST));
            PrintCell(p, solver, robot, heading, goals, goal_count);
        }
        MazePos east = { MAZE_SIZE - 1, (uint8_t)y };
        printf("%c\r\n", VerticalWall(map, east, DIR_EAST));
    }

    // 南端の壁
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        MazePos p = { x, 0 };
        printf("+%s", HorizontalWall(map, p, DIR_SOUTH));
    }
    printf("+\r\n");
}
