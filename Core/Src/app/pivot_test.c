#include "app/pivot_test.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"

// 旋回条件(未調整の控えめな初期値)
#define PIVOT_TEST_ANGLE_DEG   90.0f
#define PIVOT_TEST_OMEGA_DPS   180.0f   // 最高角速度[deg/s]
#define PIVOT_TEST_ALPHA_DPS2  1800.0f  // 角加速度[deg/s^2]

#define PIVOT_TEST_PRE_MS      200   // 回り出す前に止まったまま記録する時間
#define PIVOT_TEST_HOLD_MS     500   // 左に回った後、右に戻る前に止まっている時間
#define PIVOT_TEST_POST_MS     500   // 戻った後も記録する時間
#define PIVOT_TEST_TIMEOUT_MS  3000  // 1回の旋回がこれを過ぎても終わらなければ打ち切る

// 1回の旋回時間の見積もり[ms](台形: 角度/ω + ω/α)
#define PIVOT_TEST_TURN_MS \
    ((uint32_t)((PIVOT_TEST_ANGLE_DEG / PIVOT_TEST_OMEGA_DPS \
                 + PIVOT_TEST_OMEGA_DPS / PIVOT_TEST_ALPHA_DPS2) * 1000.0f))
#define PIVOT_TEST_LOG_MS \
    (PIVOT_TEST_PRE_MS + 2u * PIVOT_TEST_TURN_MS + PIVOT_TEST_HOLD_MS + PIVOT_TEST_POST_MS + 500u)

// ボタンを離してから回り出すまでの待ち[ms](手を離す時間)。
#define PIVOT_TEST_START_DELAY_MS 1000

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("pivot");
    Logger_SetFileName("turn90");
    Logger_AddField("omega_ref", &d->target_omega_dps);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->pos_ref); // プロファイルの進んだ角度(符号なし)
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ang_corr", &d->ang_corr_dps);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("pwm_l", &d->pwm_l);
    Logger_AddField("pwm_r", &d->pwm_r);
    Logger_AddField("ff_l", &d->ff_l);
    Logger_AddField("ff_r", &d->ff_r);
    Logger_AddField("i_l", &d->i_l);
    Logger_AddField("i_r", &d->i_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_SetDuration(PIVOT_TEST_LOG_MS);
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

// 旋回して、終わるまで待つ。タイムアウトならfalse。
static bool TurnAndWait(float angle_deg) {
    App_StartPivot(angle_deg, PIVOT_TEST_OMEGA_DPS, PIVOT_TEST_ALPHA_DPS2);
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        if (FailSafe_IsTripped()) {
            Logger_Stop();
            FailSafe_Halt();
        }
        if (HAL_GetTick() - t0 > PIVOT_TEST_TIMEOUT_MS) {
            printf("timeout\r\n");
            return false;
        }
        HAL_Delay(1);
    }
    return true;
}

static void RunOnce(void) {
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);
    Logger_Start();

    float angle0 = App_GetGyroAngle_deg();
    DelayWatching(PIVOT_TEST_PRE_MS);

    if (TurnAndWait(+PIVOT_TEST_ANGLE_DEG)) {
        DelayWatching(PIVOT_TEST_HOLD_MS);
        float angle1 = App_GetGyroAngle_deg();
        if (TurnAndWait(-PIVOT_TEST_ANGLE_DEG)) {
            DelayWatching(PIVOT_TEST_POST_MS);
        }
        float angle2 = App_GetGyroAngle_deg();
        printf("gyro angle: after left %+.1f deg, after right %+.1f deg (target %+.0f / 0)\r\n",
               angle1 - angle0, angle2 - angle0, PIVOT_TEST_ANGLE_DEG);
    }

    App_SetTargetVelocity(0.0f); // タイムアウト時もここで止める
    Logger_Stop();
    App_ControlLoop_SetEnabled(false);
}

void PivotTest_Run(void) {
    printf("PIVOT TEST: +%.0f deg then -%.0f deg, omega=%.0f dps, alpha=%.0f dps^2\r\n",
           PIVOT_TEST_ANGLE_DEG, PIVOT_TEST_ANGLE_DEG, PIVOT_TEST_OMEGA_DPS, PIVOT_TEST_ALPHA_DPS2);
    printf("ANGULAR: %s KP=%.2f KI=%.2f, GYRO_Z_SIGN=%.0f\r\n",
           ANGULAR_CONTROL_ENABLE ? "ON" : "OFF", ANGULAR_KP, ANGULAR_KI, GYRO_Z_SIGN);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    SetupLogger();

    while (1) {
        printf("press button to RUN\r\n");
        ModeUI_WaitClick();
        HAL_Delay(PIVOT_TEST_START_DELAY_MS);

        RunOnce();
        printf("done: %lu samples x %lu fields (recordable %lu ms)\r\n",
               (unsigned long)Logger_SampleCount(), (unsigned long)Logger_FieldCount(),
               (unsigned long)Logger_RecordableMs());
        ModeUI_SaveLogToSD(); // SDがあれば自動で保存

        printf("press button to DUMP\r\n");
        ModeUI_WaitClick();
        Logger_Dump();
    }
}
