#include "logic/control/velocity_profile.h"

// 減速度を指定値の何倍まで許すか(離散化の誤差を吸収する余裕)
#define PROFILE_DECEL_MARGIN 1.5f

void VelocityProfile_Start(VelocityProfile *p, float distance_mm, float v_start,
                           float v_max, float v_end, float accel) {
    VelocityProfile_StartAD(p, distance_mm, v_start, v_max, v_end, accel, accel);
}

void VelocityProfile_StartAD(VelocityProfile *p, float distance_mm, float v_start,
                             float v_max, float v_end, float accel, float decel) {
    p->distance_mm = distance_mm;
    p->v_max = v_max;
    p->v_end = (v_end < v_max) ? v_end : v_max;
    p->accel = accel;
    p->decel = decel;
    p->pos_mm = 0.0f;
    p->v = v_start;
    p->a = 0.0f;
    p->decelerating = false;
    p->done = (distance_mm <= 0.0f) || (accel <= 0.0f) || (decel <= 0.0f);
}

void VelocityProfile_Step(VelocityProfile *p, float dt) {
    if (p->done) {
        p->v = p->v_end;
        p->a = 0.0f;
        return;
    }

    float remaining = p->distance_mm - p->pos_mm;

    // 減速開始の判定: 今の速度からv_endまで減速するのに要る距離に、
    // 1tick先の位置が入ったら減速に入る(以後は加速に戻らない)。
    if (!p->decelerating) {
        float decel_dist = 0.0f;
        if (p->v > p->v_end) {
            decel_dist = (p->v * p->v - p->v_end * p->v_end) / (2.0f * p->decel);
        }
        if (remaining - p->v * dt <= decel_dist) {
            p->decelerating = true;
        }
    }

    float a_cmd;
    if (p->decelerating) {
        // 残りの距離でちょうどv_endになる減速度を毎tick計算し直す。
        // 固定の減速度だと、離散化の誤差で終点に速度が残ったまま着いてしまう。
        if (remaining > 1e-6f && p->v > p->v_end) {
            a_cmd = -(p->v * p->v - p->v_end * p->v_end) / (2.0f * remaining);
            if (a_cmd < -PROFILE_DECEL_MARGIN * p->decel) a_cmd = -PROFILE_DECEL_MARGIN * p->decel;
        } else {
            a_cmd = 0.0f;
        }
    } else if (p->v < p->v_max) {
        a_cmd = p->accel;  // 加速
    } else if (p->v > p->v_max) {
        a_cmd = -p->decel; // 上限より速く始めた(走りながら遅い指令に切り替えた): すぐ上限まで減速する
    } else {
        a_cmd = 0.0f;      // 等速
    }

    float v_next = p->v + a_cmd * dt;
    if (a_cmd > 0.0f && v_next > p->v_max) v_next = p->v_max;
    if (a_cmd < 0.0f && !p->decelerating && v_next < p->v_max) v_next = p->v_max;
    if (a_cmd < 0.0f && v_next < p->v_end) v_next = p->v_end;
    if (v_next < 0.0f) v_next = 0.0f;

    p->pos_mm += 0.5f * (p->v + v_next) * dt;
    p->a = (v_next - p->v) / dt; // クランプ後に実際にかけた加速度
    p->v = v_next;

    // 終了判定: 距離を進み終えた、または減速でv_endまで下がった
    // (v_end=0なら止まった)。終点の誤差はmm未満の想定。
    bool reached = p->pos_mm >= p->distance_mm - 1e-3f;
    bool slowed = p->decelerating && p->v <= p->v_end && (p->v_end > 0.0f || p->v <= 0.0f);
    if (reached || slowed) {
        p->done = true;
        p->v = p->v_end;
        p->a = 0.0f;
    }
}
