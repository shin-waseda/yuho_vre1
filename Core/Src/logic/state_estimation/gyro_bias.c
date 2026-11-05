#include "logic/state_estimation/gyro_bias.h"

#include <math.h>

#define WINDOW_TICKS ((uint32_t)(GYRO_STILL_WINDOW_MS / (CONTROL_DT_S * 1000.0f) + 0.5f))

static void ClearWindow(GyroBiasEst *e) {
    e->n = 0u;
    e->sum_g = 0.0f;
    e->sum_g2 = 0.0f;
    for (int i = 0; i < 3; i++) {
        e->sum_a[i] = 0.0f;
        e->sum_a2[i] = 0.0f;
    }
    e->enc_l = 0;
    e->enc_r = 0;
    e->all_still = true;
}

// 平均と標準偏差(足し込みから。負になったら 0)
static float StdOf(float sum, float sum2, uint32_t n) {
    float m = sum / (float)n;
    float var = sum2 / (float)n - m * m;
    return (var > 0.0f) ? sqrtf(var) : 0.0f;
}

void GyroBias_Init(GyroBiasEst *e) {
    ClearWindow(e);
    e->quiet_run = 0u;
    e->run_start_offset = 0.0f;
}

GyroBiasStep GyroBias_Update(GyroBiasEst *e, float gyro_raw, const float acc[3], int16_t enc_l, int16_t enc_r,
                             bool cmd_still, float offset) {
    GyroBiasStep out = { offset, false, 0u, 0.0f };

    e->n++;
    e->sum_g += gyro_raw;
    e->sum_g2 += gyro_raw * gyro_raw;
    for (int i = 0; i < 3; i++) {
        e->sum_a[i] += acc[i];
        e->sum_a2[i] += acc[i] * acc[i];
    }
    e->enc_l += (enc_l >= 0) ? enc_l : -enc_l;
    e->enc_r += (enc_r >= 0) ? enc_r : -enc_r;
    if (!cmd_still) e->all_still = false;
    if (e->n < WINDOW_TICKS) return out;

    // ---- 窓が終わった: 止まっていたかを見る ----
    float mean_g = e->sum_g / (float)e->n;
    float std_g_dps = StdOf(e->sum_g, e->sum_g2, e->n) / GYRO_SENSITIVITY_LSB_PER_DPS;
    float std_ax = StdOf(e->sum_a[0], e->sum_a2[0], e->n);
    float std_ay = StdOf(e->sum_a[1], e->sum_a2[1], e->n);
    float std_az = StdOf(e->sum_a[2], e->sum_a2[2], e->n);
    float diff_dps = fabsf(mean_g - offset) / GYRO_SENSITIVITY_LSB_PER_DPS;
    bool quiet = e->all_still
              && e->enc_l <= GYRO_STILL_ENC_MAX_COUNTS && e->enc_r <= GYRO_STILL_ENC_MAX_COUNTS
              && std_g_dps < GYRO_STILL_GYRO_STD_DPS
              && std_ax < GYRO_STILL_ACC_XY_STD && std_ay < GYRO_STILL_ACC_XY_STD && std_az < GYRO_STILL_ACC_Z_STD
              && diff_dps < GYRO_BIAS_MAX_DIFF_DPS; // 離れすぎていたら、ゼロ点ではなく本当に回っている
    ClearWindow(e);

    if (quiet) {
        if (e->quiet_run == 0u) e->run_start_offset = offset;
        e->quiet_run++;
        if (e->quiet_run > GYRO_STILL_SKIP_WINDOWS) {
            out.offset = offset + GYRO_BIAS_GAIN * (mean_g - offset);
        }
    } else {
        if (e->quiet_run > GYRO_STILL_SKIP_WINDOWS) {
            out.run_ended = true;
            out.run_windows = e->quiet_run - GYRO_STILL_SKIP_WINDOWS;
            out.run_change = offset - e->run_start_offset;
        }
        e->quiet_run = 0u;
    }
    return out;
}
