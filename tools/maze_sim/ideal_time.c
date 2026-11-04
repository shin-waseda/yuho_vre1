// maze_sim の時間(計算上の速さの運動だけ)。ideal_time.h を参照
#include "ideal_time.h"

#include <math.h>
#include <stdio.h>

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
    st->sp = *sp;
    st->t_s = 0.0f;
    st->t_goal_s = -1.0f;
    st->at_center = true;
}

void SearchTime_Step(SearchTime *st, Action a, SearchPhase before, SearchPhase after) {
    const float tv = st->sp.turn_v;
    const float vmax = st->sp.v;
    const float acc = st->sp.accel;
    float t = 0.0f;
    switch (a.type) {
        case ACTION_STOP:
            if (!st->at_center) t = StraightTime(HALF_MM, tv, 0.0f, vmax, acc);
            st->at_center = true;
            break;
        case ACTION_FORWARD: {
            float dist = (st->at_center ? HALF_MM : SECTION_MM) + SECTION_MM * (float)(a.cells - 1);
            t = StraightTime(dist, st->at_center ? 0.0f : tv, tv, vmax, acc);
            st->at_center = false;
            break;
        }
        case ACTION_TURN_RIGHT:
        case ACTION_TURN_LEFT:
            if (st->at_center) t = StraightTime(HALF_MM, 0.0f, tv, vmax, acc);
            else t = (PI_F * 0.5f * HALF_MM) / tv;
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
    bool goal = (before == SEARCH_PHASE_TO_GOAL && after == SEARCH_PHASE_TO_START);
    bool decided = (before == SEARCH_PHASE_FULL && after == SEARCH_PHASE_TO_START);
    if ((goal || decided) && st->t_goal_s < 0.0f) st->t_goal_s = st->t_s;
}
