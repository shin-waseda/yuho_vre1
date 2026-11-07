#include "app/gyro_scale_test.h"

#include <math.h>
#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/log_event.h"

// 回り方
#define GYRO_SCALE_TURNS       10       // 左へ・右へそれぞれ何回回るか(360° × これ)
#define GYRO_SCALE_OMEGA_DPS   360.0f   // 最高角速度[deg/s]
#define GYRO_SCALE_ALPHA_DPS2  1800.0f  // 角加速度[deg/s^2]
#define GYRO_SCALE_SLOPE_DEG   5.0f     // 1° あたりの FL − FR を測るために回る角度(左右に)

// 測り方
#define GYRO_SCALE_SETTLE_MS   500      // 止まってから測り始めるまでの待ち(合わせ込みの後に揺れが収まるまで)
#define GYRO_SCALE_AVG_MS      300      // FL − FR を平均する時間
#define GYRO_SCALE_START_DELAY_MS 1000  // 手を離してから測り始めるまでの待ち

// 1回の旋回の打ち切り[ms](台形: 角度/ω + ω/α に、合わせ込みの時間と余裕を足す)
static uint32_t TurnTimeoutMs(float angle_deg) {
    float t = fabsf(angle_deg) / GYRO_SCALE_OMEGA_DPS + GYRO_SCALE_OMEGA_DPS / GYRO_SCALE_ALPHA_DPS2;
    return (uint32_t)(t * 1000.0f) + PIVOT_SETTLE_TIMEOUT_MS + 1000u;
}

static uint32_t LogMs(void) {
    uint32_t big = TurnTimeoutMs(360.0f * GYRO_SCALE_TURNS);
    uint32_t small = TurnTimeoutMs(2.0f * GYRO_SCALE_SLOPE_DEG);
    return 6u * (GYRO_SCALE_SETTLE_MS + GYRO_SCALE_AVG_MS) + 2u * big + 3u * small;
}

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("gyro_scale");
    Logger_SetFileName("turns");
    Logger_AddField("omega_ref", &d->target_omega_dps);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("fb_d", &d->fb_d);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("vbat", &d->vbat);
    Logger_AddEventFields();
    Logger_SetDuration(LogMs());
}

// フェイルセーフが働いていたら、原因をログに残し、SD に保存してから FailSafe_Halt() へ入る(戻らない)。
// 回る時間が長い(20 秒あまり)ので、途中で止まっても何が起きたかを残す
static void CheckFailSafe(void) {
    if (!FailSafe_IsTripped()) return;
    FailSafeCause cause = FailSafe_GetCause();
    Logger_Event(LOG_EV_FAILSAFE, (float)cause, FailSafe_GetFilteredVoltage(), 0.0f, 0.0f, 0.0f);
    HAL_Delay(100); // イベントが記録の行に入るのを待つ(約 40ms に1行に間引いて記録しているので)
    Logger_Stop();
    App_ControlLoop_SetEnabled(false);
    printf("FAILSAFE: %s (vbat=%.2f V) during the test\r\n", FailSafe_CauseName(cause), FailSafe_GetFilteredVoltage());
    ModeUI_SaveLogToSD();
    FailSafe_Halt();
}

static void DelayWatching(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        CheckFailSafe();
        HAL_Delay(1);
    }
}

// 旋回して、終わるまで待つ。打ち切ったら false
static bool TurnAndWait(float angle_deg) {
    App_StartPivot(angle_deg, GYRO_SCALE_OMEGA_DPS, GYRO_SCALE_ALPHA_DPS2);
    uint32_t limit = TurnTimeoutMs(angle_deg);
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        CheckFailSafe();
        if (HAL_GetTick() - t0 > limit) {
            printf("timeout\r\n");
            return false;
        }
        HAL_Delay(1);
    }
    return true;
}

typedef struct {
    float diff;  // FL − FR の平均
    float sum;   // FL + FR の平均
    float angle; // ジャイロの角度の平均[deg]
} Measure;

// 止まったまま、揺れが収まるのを待ってから FL − FR とジャイロの角度を平均する
static Measure MeasureStill(uint8_t phase) {
    DelayWatching(GYRO_SCALE_SETTLE_MS);
    float sd = 0.0f, ss = 0.0f, sa = 0.0f;
    uint32_t n = 0;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < GYRO_SCALE_AVG_MS) {
        CheckFailSafe();
        float fl = (float)ad_fl;
        float fr = (float)ad_fr;
        sd += fl - fr;
        ss += fl + fr;
        sa += App_GetGyroAngle_deg();
        n++;
        HAL_Delay(1);
    }
    Measure m = { sd / (float)n, ss / (float)n, sa / (float)n };
    Logger_Event(LOG_EV_GYRO_SCALE, (float)phase, m.diff, m.sum, m.angle, 0.0f);
    printf("  phase %u: FL-FR %+7.1f, FL+FR %6.1f, gyro %+9.2f deg\r\n", (unsigned)phase, m.diff, m.sum, m.angle);
    return m;
}

// 何回も回った前(a)と後(b)から、本当に回った角度[deg]を出す。壁で見た向きの変わり(差 / 傾き)に、
// ジャイロの角度に一番近くなるよう 360° の倍数を足す
static float RealTurn(Measure a, Measure b, float slope_per_deg) {
    float gyro = b.angle - a.angle;
    float wall = (b.diff - a.diff) / slope_per_deg;
    float k = roundf((gyro - wall) / 360.0f);
    return wall + 360.0f * k;
}

// 測って回る手順。打ち切ったら false(結果は出さない)
static bool Sequence(void) {
    // 1° あたりの FL − FR の変わり方を、少し左右に回って測る(ジャイロの倍率が 1% ずれていても、傾きが 1% ずれるだけ)
    Measure m0 = MeasureStill(0u);
    if (!TurnAndWait(+GYRO_SCALE_SLOPE_DEG)) return false;
    Measure mp = MeasureStill(1u);
    if (!TurnAndWait(-2.0f * GYRO_SCALE_SLOPE_DEG)) return false;
    Measure mm = MeasureStill(2u);
    if (!TurnAndWait(+GYRO_SCALE_SLOPE_DEG)) return false;
    Measure m3 = MeasureStill(3u);
    float slope = (mp.diff - mm.diff) / (mp.angle - mm.angle);
    printf("  slope: %.2f /deg (front-align uses %.2f), back at start: FL-FR %+.1f -> %+.1f\r\n",
           slope, FRONT_ALIGN_DIFF_PER_DEG, m0.diff, m3.diff);
    if (fabsf(slope) < 1.0f) {
        printf("  slope too small (no front wall?)\r\n");
        return false;
    }

    // 左へ何回も回る → 右へ同じだけ回って戻る
    if (!TurnAndWait(+360.0f * GYRO_SCALE_TURNS)) return false;
    Measure m4 = MeasureStill(4u);
    if (!TurnAndWait(-360.0f * GYRO_SCALE_TURNS)) return false;
    Measure m5 = MeasureStill(5u);

    {
        float g_ccw = m4.angle - m3.angle, r_ccw = RealTurn(m3, m4, slope);
        float g_cw = m5.angle - m4.angle, r_cw = RealTurn(m4, m5, slope);
        float k_ccw = r_ccw / g_ccw; // 本当の角度 / ジャイロの角度(ジャイロの dps にかける倍率)
        float k_cw = r_cw / g_cw;
        printf("CCW: gyro %+.2f deg, real %+.2f deg -> scale %.5f (%+.3f %%)\r\n",
               g_ccw, r_ccw, k_ccw, (k_ccw - 1.0f) * 100.0f);
        printf("CW : gyro %+.2f deg, real %+.2f deg -> scale %.5f (%+.3f %%)\r\n",
               g_cw, r_cw, k_cw, (k_cw - 1.0f) * 100.0f);
        // ジャイロの dps = 生値 / 感度 なので、本当の感度 = 今の感度 / 倍率
        printf("GYRO_SENSITIVITY_LSB_PER_DPS: now %.3f -> CCW %.3f, CW %.3f, mean %.3f\r\n",
               GYRO_SENSITIVITY_LSB_PER_DPS, GYRO_SENSITIVITY_LSB_PER_DPS / k_ccw,
               GYRO_SENSITIVITY_LSB_PER_DPS / k_cw, GYRO_SENSITIVITY_LSB_PER_DPS * 2.0f / (k_ccw + k_cw));
        // 回っている間に位置がずれると FL − FR も変わる(向きのずれと区別できない)。FL+FR の変わりで前後のずれの目安を出す
        printf("position check: FL+FR %.0f -> %.0f -> %.0f (about %+.1f, %+.1f mm toward the wall)\r\n",
               m3.sum, m4.sum, m5.sum, (m4.sum - m3.sum) / FRONT_ALIGN_SUM_PER_MM,
               (m5.sum - m3.sum) / FRONT_ALIGN_SUM_PER_MM);
    }
    return true;
}

static void RunOnce(void) {
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);
    Logger_Start();

    Sequence();
    HAL_Delay(100); // 最後のイベントが記録の行に入るのを待つ(約 40ms に1行に間引いて記録しているので)

    App_SetTargetVelocity(0.0f); // タイムアウト時もここで止める
    Logger_Stop();
    App_ControlLoop_SetEnabled(false);
}

void GyroScaleTest_Run(void) {
    printf("GYRO SCALE TEST: %d turns each way, omega=%.0f dps, alpha=%.0f dps^2, slope +-%.0f deg\r\n",
           GYRO_SCALE_TURNS, GYRO_SCALE_OMEGA_DPS, GYRO_SCALE_ALPHA_DPS2, GYRO_SCALE_SLOPE_DEG);
    printf("place at a cell center, facing a front wall squarely (no side walls if possible)\r\n");
    printf("GYRO_SENSITIVITY_LSB_PER_DPS=%.3f, PIVOT_SETTLE_KI=%.0f\r\n", GYRO_SENSITIVITY_LSB_PER_DPS, PIVOT_SETTLE_KI);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    SetupLogger();

    while (1) {
        printf("hold hand over front-left sensor to RUN\r\n");
        ModeUI_WaitHandStart();
        HAL_Delay(GYRO_SCALE_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
        printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
        App_ResetGyroAngle(); // 置いた向きを 0° にする

        RunOnce();
        printf("done: %lu samples x %lu fields (recordable %lu ms)\r\n",
               (unsigned long)Logger_SampleCount(), (unsigned long)Logger_FieldCount(),
               (unsigned long)Logger_RecordableMs());
        SdSaveResult saved = ModeUI_SaveLogToSD(); // SDがあれば自動で保存

        printf("press button to DUMP\r\n");
        if (saved == SD_SAVE_FAILED) {
            ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR); // 保存の失敗を左後ろのLEDの点滅で知らせながら待つ
        } else {
            ModeUI_WaitClick();
        }
        Logger_Dump();
    }
}
