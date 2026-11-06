// maze_sim の時間(計算上の速さの運動だけ)。ideal_time.h を参照
#include "ideal_time.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define HALF_MM (SECTION_MM * 0.5f)
#define PI_F 3.14159265f

SearchSpeeds SearchSpeeds_Default(void) {
    SearchSpeeds s = { SEARCH_V_MM_S, SLALOM_V_MM_S, SEARCH_ACCEL_MM_S2 };
    return s;
}

bool SearchSpeeds_Parse(const char *text, SearchSpeeds *out) {
    SearchSpeeds s;
    if (sscanf(text, "%f,%f,%f", &s.v, &s.turn_v, &s.accel) != 3) return false;
    if (s.v <= 0.0f || s.turn_v <= 0.0f || s.accel <= 0.0f) return false;
    *out = s;
    return true;
}

RunProfile IdealRun_Default(void) {
    return RunProfile_Default(); // 理想の円弧の長さ、オフセット 0、減速度 = 加速度
}

bool IdealRun_Parse(const char *text, RunProfile *out) {
    float vmax, accel, small, l90, l180;
    if (sscanf(text, "%f,%f,%f,%f,%f", &vmax, &accel, &small, &l90, &l180) != 5) return false;
    if (vmax <= 0.0f || accel <= 0.0f || small <= 0.0f || l90 <= 0.0f || l180 <= 0.0f) return false;
    RunProfile p = RunProfile_Default();
    p.vmax = vmax;
    p.accel = accel;
    p.decel = accel;
    p.v_turn[RUN_SMALL90_R] = p.v_turn[RUN_SMALL90_L] = small;
    p.v_turn[RUN_LARGE90_R] = p.v_turn[RUN_LARGE90_L] = l90;
    p.v_turn[RUN_LARGE180_R] = p.v_turn[RUN_LARGE180_L] = l180;
    *out = p;
    return true;
}

// dist [mm] を v_in で入って v_out で出る台形加速の時間。加速度が足りなければ一様に変える
static float StraightTime(float dist, float v_in, float v_out, float vmax, float accel) {
    if (dist <= 0.0f) return 0.0f;
    if (fabsf(v_out * v_out - v_in * v_in) > 2.0f * accel * dist) return 2.0f * dist / (v_in + v_out);
    float vp = sqrtf(accel * dist + 0.5f * (v_in * v_in + v_out * v_out));
    if (vp > vmax) vp = vmax;
    if (vp < v_in) vp = v_in;
    if (vp < v_out) vp = v_out;
    float d_acc = (vp * vp - v_in * v_in) / (2.0f * accel);
    float d_dec = (vp * vp - v_out * v_out) / (2.0f * accel);
    return (vp - v_in) / accel + (vp - v_out) / accel + (dist - d_acc - d_dec) / vp;
}

void SearchTime_Init(SearchTime *st, const SearchSpeeds *sp) {
    memset(st, 0, sizeof(*st));
    st->sp = *sp;
    st->t_goal_s = -1.0f;
    st->at_center = true;
}

// ゴールに着いた・最短経路が決まった時刻を残す(段階が変わった Step の時間を足した後に呼ぶ)
static void MarkGoal(SearchTime *st, SearchPhase before, SearchPhase after) {
    bool goal = (before == SEARCH_PHASE_TO_GOAL && after == SEARCH_PHASE_TO_START);
    bool decided = (before == SEARCH_PHASE_FULL && after == SEARCH_PHASE_TO_START);
    if ((goal || decided) && st->t_goal_s < 0.0f) st->t_goal_s = st->t_s;
}

// ---- 既知の区間 ----

static RunList s_known_list;

uint16_t SearchTime_KnownRun(SearchTime *st, SearchPlanner *sp, Action a, SearchPhase before, SearchPhase after,
                             Direction *moves) {
    // TryKnownRun と同じ条件: 境目を走っている所で直進を返されたとき(真ん中にいるとき・走ったままゴールに入った所
    // (真ん中で止まる)ではまとめない)。Dijkstra でなければ SearchPlanner_KnownRun が 0 を返す
    if (!SEARCH_KNOWN_FAST_ENABLE || a.type != ACTION_FORWARD || st->at_center) return 0;
    if (before == SEARCH_PHASE_TO_GOAL && after != SEARCH_PHASE_TO_GOAL) return 0;

    uint16_t m = SearchPlanner_KnownRun(sp, SEARCH_KNOWN_MIN_MOVES, moves);
    if (m == 0u) return 0;

    // 走り方: 機体の ComputeSlalomOffsets と同じ速さ(直進の最高速度は探索の直進と小回りの速い方、加速度は
    // SEARCH_KNOWN_ACCEL_MM_S2、小回りは探索の小回りの速さ)を、理想の円弧の形(RunProfile_Default)に入れる
    const float tv = st->sp.turn_v;
    RunProfile prof = RunProfile_Default();
    prof.vmax = (st->sp.v > tv) ? st->sp.v : tv;
    prof.accel = SEARCH_KNOWN_ACCEL_MM_S2;
    prof.decel = SEARCH_KNOWN_ACCEL_MM_S2;
    prof.v_turn[RUN_SMALL90_R] = prof.v_turn[RUN_SMALL90_L] = tv;
    prof.v_turn[RUN_LARGE90_R] = prof.v_turn[RUN_LARGE90_L] = FAST_LARGE90_V_MM_S;
    prof.v_turn[RUN_LARGE180_R] = prof.v_turn[RUN_LARGE180_L] = FAST_LARGE180_V_MM_S;
    bool use_large = SEARCH_KNOWN_LARGE && st->sp.v >= FAST_LARGE90_V_MM_S && st->sp.v >= FAST_LARGE180_V_MM_S;
    if (!RunPath_FromKnownRun(moves, m, &prof, use_large, &s_known_list)) return 0;

    // 列の先頭の半区画(1つ手前の区画 P の真ん中 → C0 の入口の境界)は走り済みなので、時間から除く
    RunList list = s_known_list;
    if (list.count > 0 && list.items[0].type == RUN_STRAIGHT) {
        if (list.items[0].halves > 1u) {
            list.items[0].halves--;
        } else {
            memmove(&list.items[0], &list.items[1], sizeof(list.items[0]) * (size_t)(list.count - 1u));
            list.count--;
        }
    }
    float t = 0.0f;
    RunList_EstimateTimeEnds(&list, &prof, tv, tv, &t, NULL, NULL); // 加速度が足りなくても、一様に変えた時間を使う

    // 区間の終わりの区画 E と、そこに入る向き(Step で sp->pos は C1 になっている)
    MazePos e = sp->pos;
    for (uint16_t k = 1; k <= m; k++) MazePos_Step(e, moves[k], &e);
    sp->pos = e;
    sp->heading = moves[m];

    st->t_s += t;
    st->at_center = false;
    st->known_runs++;
    st->known_cells += (int)m + 1;
    MarkGoal(st, before, after);
    return m;
}

// ---- 1手ずつ ----

void SearchTime_Step(SearchTime *st, Action a, SearchPhase before, SearchPhase after) {
    const float tv = st->sp.turn_v;
    const float vmax = st->sp.v;
    const float acc = st->sp.accel;
    float t = 0.0f;
    // 走ったままゴールに入った所(ゴールを回る・全面探索): 機体は最初のゴールの区画の真ん中まで半区画で止まり
    // (地図を flash に残す。その時間は数えない)、真ん中から次の指令を始める
    bool goal_moving = (before == SEARCH_PHASE_TO_GOAL && after != SEARCH_PHASE_TO_GOAL && a.type != ACTION_STOP);
    if (goal_moving) {
        if (!st->at_center) t += StraightTime(HALF_MM, tv, 0.0f, vmax, acc);
        st->at_center = true;
        // ゴールの時刻は往復だけ(全面探索は最短経路が決まった時刻)
        if (after == SEARCH_PHASE_GOAL_TOUR && st->t_goal_s < 0.0f) st->t_goal_s = st->t_s + t;
    }
    switch (a.type) {
        case ACTION_STOP:
            if (!st->at_center) t += StraightTime(HALF_MM, tv, 0.0f, vmax, acc);
            st->at_center = true;
            break;
        case ACTION_FORWARD: {
            float dist = (st->at_center ? HALF_MM : SECTION_MM) + SECTION_MM * (float)(a.cells - 1);
            t += StraightTime(dist, st->at_center ? 0.0f : tv, tv, vmax, acc);
            st->at_center = false;
            break;
        }
        case ACTION_TURN_RIGHT:
        case ACTION_TURN_LEFT:
            if (st->at_center) t += StraightTime(HALF_MM, 0.0f, tv, vmax, acc);
            else t += (PI_F * 0.5f * HALF_MM) / tv;
            st->at_center = false;
            break;
        case ACTION_TURN_BACK:
            if (!st->at_center) t += StraightTime(HALF_MM, tv, 0.0f, vmax, acc);
            t += StraightTime(HALF_MM, 0.0f, tv, vmax, acc);
            st->at_center = false;
            break;
        default:
            break;
    }
    st->t_s += t;
    MarkGoal(st, before, after);
}
