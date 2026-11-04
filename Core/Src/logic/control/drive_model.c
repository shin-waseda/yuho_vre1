#include "logic/control/drive_model.h"

#include <math.h>

// κ0(v) の表．その間は直線でつなぎ，表の外は端の値にする．
static const float s_kappa0_v[MODEL_KAPPA0_N] = MODEL_KAPPA0_V_TABLE;
static const float s_kappa0[MODEL_KAPPA0_N] = MODEL_KAPPA0_TABLE;

static float Kappa0(float v) {
    if (v <= s_kappa0_v[0]) return s_kappa0[0];
    for (int i = 1; i < MODEL_KAPPA0_N; i++) {
        if (v < s_kappa0_v[i]) {
            float t = (v - s_kappa0_v[i - 1]) / (s_kappa0_v[i] - s_kappa0_v[i - 1]);
            return s_kappa0[i - 1] + t * (s_kappa0[i] - s_kappa0[i - 1]);
        }
    }
    return s_kappa0[MODEL_KAPPA0_N - 1];
}

static float SignOf(float x) {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

void DriveModel_Reset(DriveModelFF *m) {
    m->f_ff = 0.0f;
}

float DriveModel_FF_Vs(float v, float a_ref, float omega) {
    // 目標 0 のときは V0 をかけない(止まっている所に摩擦ぶんの電圧をかけない)
    return MODEL_A * v + MODEL_V0 * SignOf(v) + MODEL_B * a_ref + MODEL_C * v * omega * omega;
}

float DriveModel_FF_Vd(DriveModelFF *m, float v, float omega, float alpha, float dt) {
    if (v >= MODEL_TURN_V_MIN) {
        // 並進しながら曲がる(スラローム・直進)．h へ σ/v の時定数で近づける
        float kappa = omega / v;
        float amp = (kappa >= 0.0f) ? MODEL_A_PLUS : MODEL_A_MINUS;
        float h = amp * asinhf(kappa / Kappa0(v));
        float k = v * dt / MODEL_SIGMA_M;
        if (k > 1.0f) k = 1.0f;
        m->f_ff += k * (h - m->f_ff);
        return MODEL_B_OMEGA * alpha + MODEL_D + m->f_ff;
    }

    m->f_ff = 0.0f;
    if (omega == 0.0f && alpha == 0.0f) return 0.0f; // 止まっている・後ろへ下がる(D もかけない)

    // 超信地旋回．車輪の逆起電力の分(a·車輪の速さ)と，タイヤがこする摩擦を足す
    float fric = 0.0f;
    if (omega > 0.0f) fric = PIVOT_VD_FRIC_CCW;
    if (omega < 0.0f) fric = -PIVOT_VD_FRIC_CW;
    float wheel_v = omega * (TREAD_WIDTH_MM * 0.001f) * 0.5f;
    return MODEL_B_OMEGA * alpha + MODEL_D + fric + MODEL_A * wheel_v;
}
