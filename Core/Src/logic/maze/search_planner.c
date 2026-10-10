#include "logic/maze/search_planner.h"

#include <stddef.h>

static const Action kStop = { ACTION_STOP, 0 };

void SearchPlanner_Init(SearchPlanner *sp, WallMap *map, SearchAlgo algo,
                        MazePos start, Direction heading,
                        const MazePos *goals, uint8_t goal_count) {
    sp->map = map;
    sp->algo = algo;
    sp->phase = SEARCH_PHASE_TO_GOAL;
    sp->start = start;
    sp->start_heading = heading;
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
    sp->full_route_len = 0;
    sp->full_route_valid = false;
    sp->full_target_count = 0;
}

void SearchPlanner_StartFull(SearchPlanner *sp) {
    sp->phase = SEARCH_PHASE_FULL;
    sp->full_route_len = 0;
    sp->full_route_valid = false;
    sp->full_target_count = 0;
}

// ---- 全面探索 ----

// 未知の壁は「ない」とみなした地図で、スタートからゴールまでの最短経路を求め、進む向きを full_route に書く。
// コストは最短走行(PrepareFastRoute)と同じ MazeCost_Default()。ゴールへ行けなければ false。
// 計算にはプランナーの作業領域を使う(この後の NextDir で上書きするので、足立法でも使える)。
static bool FullComputeRoute(SearchPlanner *sp) {
    MazeSolver *s = &sp->work.solver;
    Dijkstra_ComputeFrom(s, sp->map, WALL_VIEW_SEARCH, NULL, sp->goals, sp->goal_count,
                         sp->start, sp->start_heading);
    MazePos p = sp->start;
    Direction h = sp->start_heading;
    uint16_t n = 0;
    Direction d;
    while (n < MAZE_CELL_COUNT && Dijkstra_NextDir(s, p, h, &d)) {
        sp->full_route[n++] = (uint8_t)d;
        MazePos_Step(p, d, &p);
        h = d;
    }
    sp->full_route_len = n;
    return MazePos_InList(p, sp->goals, sp->goal_count);
}

// full_route が横切る壁のどれかが見つかった(探索の地図で「ある」になった)か
static bool FullRouteBlocked(const SearchPlanner *sp) {
    MazePos p = sp->start;
    for (uint16_t i = 0; i < sp->full_route_len; i++) {
        Direction d = (Direction)sp->full_route[i];
        if (WallMap_HasWall(sp->map, p, d, WALL_VIEW_SEARCH)) return true;
        MazePos_Step(p, d, &p);
    }
    return false;
}

// まだ見ていない区画 p を行き先に加える(今いる区画・同じ区画は加えない)
static void FullAddTarget(SearchPlanner *sp, MazePos p, uint8_t *added) {
    uint16_t i = (uint16_t)p.y * MAZE_SIZE + p.x;
    if (added[i / 8u] & (uint8_t)(1u << (i % 8u))) return;
    if (MazePos_Equal(p, sp->pos) || WallMap_IsCellKnown(sp->map, p)) return;
    if (sp->full_target_count >= SEARCH_FULL_TARGET_MAX) return;
    added[i / 8u] |= (uint8_t)(1u << (i % 8u));
    sp->full_targets[sp->full_target_count++] = p;
}

// 最短経路の候補を(必要なら計算し直して)確かめ、その経路が横切る未知の壁の両側の区画を行き先にする。
// 行き先がなくなったら(full_target_count == 0)、最短経路が決まった。ゴールへ行けなければ false。
// 今いる区画は行き先にしない(壁は見たばかり。後ろの壁が未知でも、その壁は後ろの区画を行き先にすれば分かる)。
static bool FullUpdateTargets(SearchPlanner *sp) {
    if (!sp->full_route_valid || FullRouteBlocked(sp)) {
        if (!FullComputeRoute(sp)) return false;
        sp->full_route_valid = true;
    }
    uint8_t added[(MAZE_CELL_COUNT + 7u) / 8u] = { 0 };
    sp->full_target_count = 0;
    MazePos p = sp->start;
    for (uint16_t i = 0; i < sp->full_route_len; i++) {
        Direction d = (Direction)sp->full_route[i];
        MazePos n;
        MazePos_Step(p, d, &n);
        if (!WallMap_IsKnown(sp->map, p, d)) {
            FullAddTarget(sp, p, added);
            FullAddTarget(sp, n, added);
        }
        p = n;
    }
    return true;
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

    // 全面探索: 行き先がなくなったら(最短経路が決まったら)、止まらずにスタートへ戻り始める
    if (sp->phase == SEARCH_PHASE_FULL) {
        if (!FullUpdateTargets(sp)) {
            sp->phase = SEARCH_PHASE_FAILED;
            return kStop;
        }
        if (sp->full_target_count == 0) sp->phase = SEARCH_PHASE_TO_START;
    }

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
    bool ok;
    if (sp->phase == SEARCH_PHASE_TO_GOAL) {
        ok = NextDir(sp, sp->goals, sp->goal_count, &sp->cost_to_goal, &next);
    } else if (sp->phase == SEARCH_PHASE_FULL) {
        ok = NextDir(sp, sp->full_targets, sp->full_target_count, &sp->cost_to_goal, &next);
    } else {
        ok = NextDir(sp, &sp->start, 1, &sp->cost_to_start, &next);
    }
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

static bool IsTarget(const SearchPlanner *sp, MazePos p) {
    if (sp->phase == SEARCH_PHASE_TO_GOAL) return MazePos_InList(p, sp->goals, sp->goal_count);
    if (sp->phase == SEARCH_PHASE_FULL) return MazePos_InList(p, sp->full_targets, sp->full_target_count);
    return MazePos_Equal(p, sp->start);
}

uint16_t SearchPlanner_KnownRun(const SearchPlanner *sp, uint16_t min_moves, Direction *moves) {
    if (sp->algo != SEARCH_ALGO_DIJKSTRA) return 0;
    if (sp->phase != SEARCH_PHASE_TO_GOAL && sp->phase != SEARCH_PHASE_TO_START &&
        sp->phase != SEARCH_PHASE_FULL) return 0;
    MazePos c = sp->pos;
    Direction h = sp->heading;
    uint16_t m = 0;
    moves[0] = h;
    while (m < MAZE_CELL_COUNT - 1u) {
        if (IsTarget(sp, c) || !WallMap_IsCellKnown(sp->map, c)) break;
        Direction d;
        if (!Dijkstra_NextDir(&sp->work.solver, c, h, &d)) break;
        if (d == Dir_Opposite(h)) break; // その場で向きを変える所はまとめない
        MazePos nc;
        if (!MazePos_Step(c, d, &nc)) break;
        c = nc;
        h = d;
        moves[++m] = d;
    }
    while (m >= 2u && !(moves[m] == moves[m - 1u] && moves[m - 1u] == moves[m - 2u])) m--;
    if (m < min_moves || m < 2u) return 0;
    return m;
}
