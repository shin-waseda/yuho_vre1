#include "logic/maze/step_map.h"

// 計算用のキュー。各区画は1回しか入らないので、MAZE_CELL_COUNTで足りる
static uint16_t s_queue[MAZE_CELL_COUNT];

void StepMap_Compute(StepMap *s, const WallMap *map, WallView view,
                     const MazePos *goals, uint8_t goal_count) {
    for (uint8_t y = 0; y < MAZE_SIZE; y++) {
        for (uint8_t x = 0; x < MAZE_SIZE; x++) {
            s->step[y][x] = STEP_MAP_INF;
        }
    }

    uint16_t head = 0;
    uint16_t tail = 0;
    for (uint8_t i = 0; i < goal_count; i++) {
        MazePos g = goals[i];
        if (s->step[g.y][g.x] == 0) continue; // 同じゴールが2回書かれていても1回だけ入れる
        s->step[g.y][g.x] = 0;
        s_queue[tail++] = (uint16_t)(g.y * MAZE_SIZE + g.x);
    }

    while (head < tail) {
        uint16_t cell = s_queue[head++];
        MazePos p = { (uint8_t)(cell % MAZE_SIZE), (uint8_t)(cell / MAZE_SIZE) };
        uint16_t next_step = (uint16_t)(s->step[p.y][p.x] + 1u);

        for (int d = 0; d < 4; d++) {
            if (WallMap_HasWall(map, p, (Direction)d, view)) continue;
            MazePos n;
            if (!MazePos_Step(p, (Direction)d, &n)) continue;
            if (s->step[n.y][n.x] != STEP_MAP_INF) continue;
            s->step[n.y][n.x] = next_step;
            s_queue[tail++] = (uint16_t)(n.y * MAZE_SIZE + n.x);
        }
    }
}

bool StepMap_NextDir(const StepMap *s, const WallMap *map, WallView view,
                     MazePos p, Direction heading, Direction *next) {
    uint16_t best = s->step[p.y][p.x];
    if (best == STEP_MAP_INF || best == 0) return false;

    static const int kOrder[4] = { 0, 1, -1, 2 }; // 前 → 右 → 左 → 後ろ
    bool found = false;
    for (int i = 0; i < 4; i++) {
        Direction d = Dir_Turn(heading, kOrder[i]);
        if (WallMap_HasWall(map, p, d, view)) continue;
        MazePos n;
        if (!MazePos_Step(p, d, &n)) continue;
        if (s->step[n.y][n.x] < best) {
            best = s->step[n.y][n.x];
            *next = d;
            found = true;
        }
    }
    return found;
}
