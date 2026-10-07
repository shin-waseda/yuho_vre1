#include "logic/control/slalom.h"

#include <math.h>
#include "logic/control/velocity_profile.h"

#define DEG_TO_RAD (3.14159265f / 180.0f)

SlalomShape Slalom_ComputeShape(const SlalomParams *p) {
    SlalomShape s = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    VelocityProfile prof;
    VelocityProfile_Start(&prof, p->angle_deg, 0.0f, p->omega_dps, 0.0f, p->alpha_dps2);

    // ISR と同じく、1tick ごとに角速度を進め、その tick の向きで v × dt だけ進む
    float theta = 0.0f; // 曲がった角度[deg](曲がる側を正)
    uint32_t ticks = 0;
    while (!prof.done && ticks < 100000u) {
        VelocityProfile_Step(&prof, CONTROL_DT_S);
        theta += prof.v * CONTROL_DT_S;
        float th = theta * DEG_TO_RAD;
        s.forward_mm += p->v_mm_s * cosf(th) * CONTROL_DT_S;
        s.side_mm += p->v_mm_s * sinf(th) * CONTROL_DT_S;
        if (s.forward_mm > s.forward_max_mm) s.forward_max_mm = s.forward_mm;
        ticks++;
    }
    s.time_s = (float)ticks * CONTROL_DT_S;
    s.length_mm = p->v_mm_s * s.time_s;
    return s;
}

void Slalom_ScaleToSpeed(SlalomParams *p, float v_mm_s) {
    if (p->v_mm_s <= 0.0f) return;
    float k = v_mm_s / p->v_mm_s;
    p->omega_dps *= k;
    p->alpha_dps2 *= k * k;
    p->v_mm_s = v_mm_s;
}

void Slalom_Turn90Offsets(const SlalomShape *s, float span_mm, float *pre_mm, float *post_mm) {
    *pre_mm = span_mm - s->forward_mm;
    *post_mm = span_mm - s->side_mm;
}

void Slalom_SolveOmegaForSide(SlalomParams *p, float side_mm) {
    // 最高角速度が大きいほど小さく回る(横に移る量が減る)ので、二分法で求める
    float lo = 30.0f, hi = 3000.0f;
    for (int i = 0; i < 30; i++) {
        p->omega_dps = 0.5f * (lo + hi);
        SlalomShape s = Slalom_ComputeShape(p);
        if (s.side_mm > side_mm) {
            lo = p->omega_dps;
        } else {
            hi = p->omega_dps;
        }
    }
    p->omega_dps = 0.5f * (lo + hi);
}

void Slalom_Turn180Offsets(const SlalomShape *s, float cell_mm, float *pre_mm, float *post_mm) {
    *pre_mm = cell_mm - s->forward_max_mm;
    // 曲がり終わりは forward_mm の高さにいて、向きが逆なので、後ろのオフセットで出発の高さまで戻る
    *post_mm = *pre_mm + s->forward_mm;
}
