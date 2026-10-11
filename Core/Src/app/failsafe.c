#include "app/failsafe.h"

#include <math.h>
#include "main.h"
#include "interface/led.h"

// 発動原因と、発動時の値(電圧/速度偏差/角速度)。ISRが書き、メインが読む。
static volatile FailSafeCause s_cause = FAILSAFE_NONE;
static volatile float s_trip_value = 0.0f;

// 以下はISR(FailSafe_Update)からのみ触る
static float s_vbat_filtered = 0.0f;
static bool s_vbat_initialized = false;
static uint16_t s_low_voltage_ms = 0;
static uint16_t s_vel_err_ms = 0;
static uint16_t s_gyro_ms = 0;

// 低電圧で発動させるか(メインが書き、ISRが読む。AUTO の間だけ false にする)
static volatile bool s_low_voltage_enabled = true;

void FailSafe_SetLowVoltageEnabled(bool enabled) {
    s_low_voltage_enabled = enabled;
}

void FailSafe_Init(void) {
    s_cause = FAILSAFE_NONE;
    s_trip_value = 0.0f;
    s_vbat_initialized = false;
    s_low_voltage_ms = 0;
    s_vel_err_ms = 0;
    s_gyro_ms = 0;
}

void FailSafe_Trip(FailSafeCause cause, float value) {
    if (s_cause != FAILSAFE_NONE) return; // 最初の原因を残す
    s_trip_value = value;
    s_cause = cause;
}

// 条件が成立している間カウントを進め、limit_msに達したらtrueを返す。
static bool Persist(uint16_t *counter_ms, bool cond, uint16_t limit_ms) {
    if (!cond) {
        *counter_ms = 0;
        return false;
    }
    if (*counter_ms < limit_ms) (*counter_ms)++;
    return *counter_ms >= limit_ms;
}

void FailSafe_Update(const FailSafeInput *in) {
    // --- バッテリー電圧: IIRで平滑化してから判定 ---
    if (!s_vbat_initialized) {
        s_vbat_filtered = in->battery_v;
        s_vbat_initialized = true;
    } else {
        s_vbat_filtered += BATTERY_IIR_ALPHA * (in->battery_v - s_vbat_filtered);
    }

    if (s_cause != FAILSAFE_NONE) return; // ラッチ済み

    bool low_voltage = s_low_voltage_enabled && (s_vbat_filtered < FAILSAFE_LOW_VOLTAGE_V);
    if (Persist(&s_low_voltage_ms, low_voltage, FAILSAFE_LOW_VOLTAGE_MS)) {
        FailSafe_Trip(FAILSAFE_LOW_VOLTAGE, s_vbat_filtered);
        return;
    }

    // --- 速度偏差: 制御中のみ。左右の大きい方で判定 ---
    float err_l = fabsf(in->target.left_mm_s  - in->actual.left_mm_s);
    float err_r = fabsf(in->target.right_mm_s - in->actual.right_mm_s);
    float err = (err_l > err_r) ? err_l : err_r;
    bool vel_diverge = in->control_enabled && (err > FAILSAFE_VEL_ERR_MM_S);
    if (Persist(&s_vel_err_ms, vel_diverge, FAILSAFE_VEL_ERR_MS)) {
        FailSafe_Trip(FAILSAFE_VELOCITY_DIVERGE, err);
        return;
    }

    // --- 角速度: 制御の有無に関係なく監視 ---
    bool gyro_diverge = fabsf(in->gyro_z_dps) > FAILSAFE_GYRO_DPS;
    if (Persist(&s_gyro_ms, gyro_diverge, FAILSAFE_GYRO_MS)) {
        FailSafe_Trip(FAILSAFE_GYRO_DIVERGE, in->gyro_z_dps);
        return;
    }
}

bool FailSafe_IsTripped(void) {
    return s_cause != FAILSAFE_NONE;
}

FailSafeCause FailSafe_GetCause(void) {
    return s_cause;
}

const char *FailSafe_CauseName(FailSafeCause cause) {
    switch (cause) {
        case FAILSAFE_NONE:             return "NONE";
        case FAILSAFE_LOW_VOLTAGE:      return "LOW_VOLTAGE";
        case FAILSAFE_VELOCITY_DIVERGE: return "VELOCITY_DIVERGE";
        case FAILSAFE_GYRO_DIVERGE:     return "GYRO_DIVERGE";
        default:                        return "UNKNOWN";
    }
}

float FailSafe_GetFilteredVoltage(void) {
    return s_vbat_filtered;
}

// 発動原因を、マイコン直結のLEDの点滅で示す(点灯 ⇔ 全消灯)。
// シフトレジスタ(595)を通さないので、595側が壊れていても表示できる。
#define FAILSAFE_BLINK_MS 250

static uint8_t CauseLeds(FailSafeCause cause) {
    switch (cause) {
        case FAILSAFE_LOW_VOLTAGE:      return LED_REAR_RIGHT;                              // LED_5
        case FAILSAFE_VELOCITY_DIVERGE: return LED_FRONT_RIGHT | LED_LEFT;                  // LED_1,2
        case FAILSAFE_GYRO_DIVERGE:     return LED_FRONT_RIGHT | LED_LEFT | LED_FRONT_LEFT; // LED_1〜3
        default:                        return LED_DIRECT_ALL;
    }
}

void FailSafe_Halt(void) {
    FailSafeCause cause = s_cause;
    uint8_t pattern = CauseLeds(cause);

    // 直前のモード表示などが残っていると紛らわしいので、シフトレジスタのLEDは消しておく
    LED_SetShiftPattern(0x0000u);

    while (1) {
        printf("FAILSAFE: %s (value=%.2f, vbat=%.2f V)\r\n",
               FailSafe_CauseName(cause), s_trip_value, s_vbat_filtered);

        for (int i = 0; i < 4; i++) {
            LED_SetDirectPattern(pattern);
            HAL_Delay(FAILSAFE_BLINK_MS);
            LED_SetDirectPattern(0x00u);
            HAL_Delay(FAILSAFE_BLINK_MS);
        }
    }
}
