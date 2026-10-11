#include "app/sensor_spin.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "interface/led.h"
#include "interface/sdcard.h"

#define SENSOR_SPIN_START_DELAY_MS 1000  // 手を離してから回り出すまでの待ち
#define SENSOR_SPIN_HOLD_MS        500   // 回る前・左右の間・回った後に止まって記録する時間

// 1回の旋回の見積もり[ms](台形: 角度/ω + ω/α)の2倍を打ち切りの時間にする
static uint32_t TimeoutMs(void) {
    float t = SENSOR_SPIN_ANGLE_DEG / SENSOR_SPIN_OMEGA_DPS + SENSOR_SPIN_OMEGA_DPS / SENSOR_SPIN_ALPHA_DPS2;
    return (uint32_t)(t * 2000.0f);
}

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("sensor");
    Logger_SetFileName("spin");
    Logger_AddField("angle", &d->angle_deg); // スタートの向きが 0、左回りが正
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddField("dist", &d->dist_mm); // 前後のずれ(真ん中から動いていないか)
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("pwm_l", &d->pwm_l);
    Logger_AddField("pwm_r", &d->pwm_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_SetDecimation(SENSOR_SPIN_LOG_DECIMATION);
}

// 流したファイルを閉じてから止まる
static void HaltOnFailSafe(void) {
    App_ControlLoop_SetEnabled(false);
    Logger_StreamEnd(NULL);
    FailSafe_Halt();
}

// ms だけ待つ(その間 SD へ流す)。FailSafe なら戻らない
static void WaitStreaming(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        if (FailSafe_IsTripped()) HaltOnFailSafe();
        Logger_StreamPoll();
        HAL_Delay(1);
    }
}

// 回って、終わるまで待つ(その間 SD へ流す)。打ち切ったら false
static bool SpinAndWait(float angle_deg) {
    App_StartPivot(angle_deg, SENSOR_SPIN_OMEGA_DPS, SENSOR_SPIN_ALPHA_DPS2);
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        if (FailSafe_IsTripped()) HaltOnFailSafe();
        if (HAL_GetTick() - t0 > TimeoutMs()) {
            printf("timeout\r\n");
            return false;
        }
        Logger_StreamPoll();
        HAL_Delay(1);
    }
    return true;
}

void SensorSpin_Run(void) {
    printf("SENSOR SPIN: +%.0f deg then -%.0f deg, omega=%.0f dps, alpha=%.0f dps^2, log every %u ms\r\n",
           SENSOR_SPIN_ANGLE_DEG, SENSOR_SPIN_ANGLE_DEG, SENSOR_SPIN_OMEGA_DPS, SENSOR_SPIN_ALPHA_DPS2,
           (unsigned)SENSOR_SPIN_LOG_DECIMATION);
    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    while (1) {
        printf("put at the center of a cell (write down the walls and the heading). hand: START\r\n");
        LED_SetDirectPattern(0x00u);
        ModeUI_WaitHandStart();
        HAL_Delay(SENSOR_SPIN_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
        printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
        App_ResetGyroAngle(); // スタートの向きを 0° にする

        SetupLogger();
        char path[SDCARD_PATH_MAX];
        if (!Logger_StreamBegin(path, sizeof(path))) {
            printf("SD: cannot open a log file\r\n");
            ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR);
            continue;
        }
        printf("log: %s\r\n", path);

        App_SetPositionHold(true); // 回っている間も区画の真ん中に留まる
        App_SetTargetVelocity(0.0f);
        App_ControlLoop_SetEnabled(true);
        Logger_Start();

        WaitStreaming(SENSOR_SPIN_HOLD_MS);
        bool ok = SpinAndWait(+SENSOR_SPIN_ANGLE_DEG);
        if (ok) {
            WaitStreaming(SENSOR_SPIN_HOLD_MS);
            float angle1 = App_GetGyroAngle_deg();
            ok = SpinAndWait(-SENSOR_SPIN_ANGLE_DEG);
            WaitStreaming(SENSOR_SPIN_HOLD_MS);
            printf("gyro angle: after left %+.1f deg, after right %+.1f deg\r\n", angle1, App_GetGyroAngle_deg());
        }

        App_SetTargetVelocity(0.0f); // 打ち切ったときもここで止める
        App_ControlLoop_SetEnabled(false);
        uint32_t dropped = 0;
        bool saved = Logger_StreamEnd(&dropped);
        printf("log %s (dropped %lu rows)%s\r\n", saved ? "saved" : "save failed", (unsigned long)dropped,
               ok ? "" : ", spin timed out");
        if (saved) {
            LED_SetDirectPattern(LED_DIRECT_ALL); // 保存できた合図(ModeUI_SaveLogToSD と同じ)
            HAL_Delay(500);
            LED_SetDirectPattern(0x00u);
        } else {
            ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR);
        }
    }
}
