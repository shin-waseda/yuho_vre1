#include "logic/maze/search_planner.h"

static const Action kStop = { ACTION_STOP, 0 };

void SearchPlanner_Init(SearchPlanner *sp, WallMap *map, SearchAlgo algo,
                        MazePos start, Direction heading,
                        const MazePos *goals, uint8_t goal_count) {
    sp->map = map;
    sp->algo = algo;
    sp->phase = SEARCH_PHASE_TO_GOAL;
    sp->start = start;
    if (goal_count > MAZE_GOAL_MAX) goal_count = MAZE_GOAL_MAX;
    for (uint8_t i = 0; i < goal_count; i++) {
        sp->goals[i] = goals[i];
    }
    sp->goal_count = goal_count;
    sp->pos = start;
    sp->heading = heading;
    sp->cost_to_goal = MazeCost_Default();
    sp->cost_to_start = MazeCost_Default();
    sp->cost_to_start.known_cell = MAZE_COST_KNOWN_CELL_RETURN;
}

// 目的地(targets)へ向かうとき、次に進む向きを地図から計算する
static bool NextDir(SearchPlanner *sp, const MazePos *targets, uint8_t count,
                    const MazeCost *cost, Direction *next) {
    if (sp->algo == SEARCH_ALGO_ADACHI) {
        StepMap_Compute(&sp->work.step, sp->map, WALL_VIEW_SEARCH, targets, count);
        return StepMap_NextDir(&sp->work.step, sp->map, WALL_VIEW_SEARCH, sp->pos, sp->heading, next);
    }
    // 今のノードのコストが決まったら止める(境界で計算する時間を短くするため。選ぶ向きは全部計算したときと同じ)
    Dijkstra_ComputeFrom(&sp->work.solver, sp->map, WALL_VIEW_SEARCH, cost, targets, count,
                         sp->pos, sp->heading);
    return Dijkstra_NextDir(&sp->work.solver, sp->pos, sp->heading, next);
}

Action SearchPlanner_Step(SearchPlanner *sp, WallObservation obs) {
    if (sp->phase == SEARCH_PHASE_DONE || sp->phase == SEARCH_PHASE_FAILED) return kStop;

    WallMap_Observe(sp->map, sp->pos, sp->heading, obs);

    // 目的地に着いたら止まる(ゴールなら、次の呼び出しから戻り始める)
    if (sp->phase == SEARCH_PHASE_TO_GOAL && MazePos_InList(sp->pos, sp->goals, sp->goal_count)) {
        sp->phase = SEARCH_PHASE_TO_START;
        return kStop;
    }
    if (sp->phase == SEARCH_PHASE_TO_START && MazePos_Equal(sp->pos, sp->start)) {
        sp->phase = SEARCH_PHASE_DONE;
        return kStop;
    }

    Direction next;
    bool ok = (sp->phase == SEARCH_PHASE_TO_GOAL)
                  ? NextDir(sp, sp->goals, sp->goal_count, &sp->cost_to_goal, &next)
                  : NextDir(sp, &sp->start, 1, &sp->cost_to_start, &next);
    if (!ok) {
        sp->phase = SEARCH_PHASE_FAILED;
        return kStop;
    }

    Action a = Action_Move((int)next - (int)sp->heading);
    sp->heading = next;
    // 未知の壁は「ない」とみなした地図で選んだ向きなので、地図の上では必ず進める
    MazePos_Step(sp->pos, next, &sp->pos);
    return a;
}
