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

// ---- スリップを含めた 90° の旋回 ----
// 左に曲がるとして、入る点 (0, 0)・入る向き +y(90°)、出る点 (−span, span)・出る向き −x とする
// (tools/matlab の yc.turn_kinds / yc.simulate_turn / yc.exit_errors と同じ置き方)。外は +y。

typedef struct {
    float x, y;    // 位置[mm]
    float th_deg;  // 機体の向き[deg]
    float beta;    // スリップアングル[rad]
    float v;       // 並進の速さ[mm/s]
    float K, C;
} SlipSim;

static void SlipTick(SlipSim *s, float om_dps, float h) {
    float target = s->K * (s->v * 1e-3f) * (om_dps * DEG_TO_RAD);
    if (s->C <= 0.0f) {
        s->beta = target;
    } else {
        s->beta += (1.0f - expf(-h / s->C)) * (target - s->beta);
    }
    s->th_deg += om_dps * h;
    float a = s->th_deg * DEG_TO_RAD - s->beta;
    s->x += s->v * h * cosf(a);
    s->y += s->v * h * sinf(a);
}

// 端の誤差が出ないよう、最後の1歩は残りの距離だけ進む
static void SlipStraight(SlipSim *s, float dist_mm) {
    if (dist_mm < 0.0f) dist_mm = 0.0f;
    float step = s->v * CONTROL_DT_S;
    uint32_t n = (uint32_t)(dist_mm / step);
    for (uint32_t i = 0; i < n; i++) {
        SlipTick(s, 0.0f, CONTROL_DT_S);
    }
    float rest = dist_mm - (float)n * step;
    if (rest > 1e-6f) SlipTick(s, 0.0f, rest / s->v);
}

SlalomSlipError Slalom_SlipError90(const SlalomParams *p, float span_mm, float pre_mm, float post_mm,
                                   float extra_mm, float K, float C) {
    SlalomSlipError e = { 0.0f, 0.0f, 0.0f };
    if (p->v_mm_s <= 0.0f) return e;
    SlipSim s = { 0.0f, 0.0f, 90.0f, 0.0f, p->v_mm_s, K, C };

    SlipStraight(&s, pre_mm);
    VelocityProfile prof;
    VelocityProfile_Start(&prof, 90.0f, 0.0f, p->omega_dps, 0.0f, p->alpha_dps2);
    uint32_t ticks = 0;
    while (!prof.done && ticks < 100000u) {
        VelocityProfile_Step(&prof, CONTROL_DT_S);
        SlipTick(&s, prof.v, CONTROL_DT_S);
        ticks++;
    }
    SlipStraight(&s, post_mm);
    e.along_mm = -(s.x - (-span_mm)); // 出る向きは −x
    e.lat_post_mm = s.y - span_mm;    // 外は +y
    SlipStraight(&s, extra_mm);
    e.lat_final_mm = s.y - span_mm;
    return e;
}

SlalomOffsets Slalom_SmallTurnOffsets(const SlalomParams *p) {
    SlalomShape sh = Slalom_ComputeShape(p);
    float pre0, post0;
    const float half = SECTION_MM * 0.5f; // 小回りは境界の真ん中 → 隣の境界の真ん中
    Slalom_Turn90Offsets(&sh, half, &pre0, &post0);

    // スリップ: 出口のずれを打ち消すように直す(前を伸ばすと出口の線が外へずれ、後ろを伸ばすと出口が先へ進む)
    float pre_adj = 0.0f, post_adj = 0.0f;
    for (int i = 0; i < 3; i++) {
        SlalomSlipError e = Slalom_SlipError90(p, half, pre0 + pre_adj, post0 + post_adj,
                                               SECTION_MM, SLALOM_SLIP_K, SLALOM_SLIP_C_S);
        pre_adj -= e.lat_final_mm;
        post_adj -= e.along_mm;
    }
    // スリップ以外の遅れ(速さによらず一定。300〜600mm/s のログで確かめた)
    post_adj += SLALOM_EXTRA_LAG_MM;
    // 手で足す分
    pre_adj += SLALOM_PRE_ADJ_MM;
    post_adj += SLALOM_POST_ADJ_MM;

    SlalomOffsets o = { pre0 + pre_adj, post0 + post_adj, pre_adj, post_adj };
    return o;
}

float Slalom_FrontRefSum(float pre_adj_mm) {
    return SLALOM_FRONT_REF_SUM_AT_PRE0 + SLALOM_FRONT_SUM_PER_MM * pre_adj_mm;
}
