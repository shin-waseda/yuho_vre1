#include "app/straight_test.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"

// 走行条件: 区画数ぶんを 最高速度・加速度 で走って止まる。
#define STRAIGHT_TEST_SECTIONS   15
#define STRAIGHT_TEST_V_MAX      300.0f  // [mm/s]
#define STRAIGHT_TEST_ACCEL      2000.0f // [mm/s^2]

#define STRAIGHT_TEST_PRE_MS     200   // 走り出す前に止まったまま記録する時間
#define STRAIGHT_TEST_POST_MS    500   // 止まった後も記録する時間

// 走行時間の見積もり[ms](台形: 距離/v_max + v_max/accel。三角形になる短距離では長めに出る)。
// 15区画/300mm/s/2000mm/s^2 で約9.15s。
#define STRAIGHT_TEST_DISTANCE_MM (STRAIGHT_TEST_SECTIONS * SECTION_MM)
#define STRAIGHT_TEST_MOTION_MS \
    ((uint32_t)((STRAIGHT_TEST_DISTANCE_MM / STRAIGHT_TEST_V_MAX \
                 + STRAIGHT_TEST_V_MAX / STRAIGHT_TEST_ACCEL) * 1000.0f))
// これを過ぎても終わらなければ打ち切る(見積もりの1.5倍)
#define STRAIGHT_TEST_TIMEOUT_MS (STRAIGHT_TEST_MOTION_MS * 3u / 2u)
// 記録の長さ(前後の待ち + 走行 + 余裕0.5s)。Loggerがこれに収まるよう間引く。
#define STRAIGHT_TEST_LOG_MS \
    (STRAIGHT_TEST_PRE_MS + STRAIGHT_TEST_MOTION_MS + STRAIGHT_TEST_POST_MS + 500u)

// ボタンを離してから走り出すまでの待ち[ms](手を離す時間)。
#define STRAIGHT_TEST_START_DELAY_MS 1000

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("straight");
    Logger_SetFileName("trapezoid");
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("target_acc", &d->target_acc);
    Logger_AddField("pos_ref", &d->pos_ref);
    Logger_AddField("x_mm", &d->x_mm);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ang_corr", &d->ang_corr_dps);
    Logger_AddField("ff_l", &d->ff_l);
    Logger_AddField("ff_r", &d->ff_r);
    Logger_AddField("i_l", &d->i_l);
    Logger_AddField("i_r", &d->i_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_SetDuration(STRAIGHT_TEST_LOG_MS);
}

// FailSafe発動時はFailSafe_Halt()へ入る(戻らない)。
static void DelayWatching(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        if (FailSafe_IsTripped()) {
            Logger_Stop();
            FailSafe_Halt();
        }
        HAL_Delay(1);
    }
}

static void RunOnce(void) {
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);
    Logger_Start();

    DelayWatching(STRAIGHT_TEST_PRE_MS);

    App_StartStraight(STRAIGHT_TEST_DISTANCE_MM, STRAIGHT_TEST_V_MAX, 0.0f, STRAIGHT_TEST_ACCEL);

    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        if (FailSafe_IsTripped()) {
            Logger_Stop();
            FailSafe_Halt();
        }
        if (HAL_GetTick() - t0 > STRAIGHT_TEST_TIMEOUT_MS) {
            printf("timeout\r\n");
            break;
        }
        HAL_Delay(1);
    }

    App_SetTargetVelocity(0.0f); // タイムアウト時もここで止める
    DelayWatching(STRAIGHT_TEST_POST_MS);

    Logger_Stop();
    App_ControlLoop_SetEnabled(false);
}

void StraightTest_Run(void) {
    printf("STRAIGHT TEST: %d sections (%.0f mm), v_max=%.0f mm/s, accel=%.0f mm/s^2\r\n",
           STRAIGHT_TEST_SECTIONS, STRAIGHT_TEST_DISTANCE_MM,
           STRAIGHT_TEST_V_MAX, STRAIGHT_TEST_ACCEL);
    printf("motion ~%lu ms, timeout %lu ms, log %lu ms\r\n",
           (unsigned long)STRAIGHT_TEST_MOTION_MS, (unsigned long)STRAIGHT_TEST_TIMEOUT_MS,
           (unsigned long)STRAIGHT_TEST_LOG_MS);
    printf("KP=%.4f KI=%.4f [V]\r\n", VELOCITY_KP, VELOCITY_KI);
    printf("ANGULAR: %s KP=%.2f KI=%.2f\r\n", ANGULAR_CONTROL_ENABLE ? "ON" : "OFF",
           ANGULAR_KP, ANGULAR_KI);
    printf("FF L: fric=%.3f gain=%.5f acc=%.6f / R: fric=%.3f gain=%.5f acc=%.6f [V]\r\n",
           VELOCITY_FF_FRIC_L, VELOCITY_FF_GAIN_L, VELOCITY_FF_ACC_L,
           VELOCITY_FF_FRIC_R, VELOCITY_FF_GAIN_R, VELOCITY_FF_ACC_R);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    SetupLogger();

    while (1) {
        printf("press button to RUN\r\n");
        ModeUI_WaitClick();
        HAL_Delay(STRAIGHT_TEST_START_DELAY_MS);

        RunOnce();
        printf("done: %lu samples x %lu fields (recordable %lu ms)\r\n",
               (unsigned long)Logger_SampleCount(), (unsigned long)Logger_FieldCount(),
               (unsigned long)Logger_RecordableMs());
        ModeUI_SaveLogToSD(); // SDがあれば自動で保存(UARTの線なしでも残る)

        printf("press button to DUMP\r\n");
        ModeUI_WaitClick();
        Logger_Dump();
    }
}
