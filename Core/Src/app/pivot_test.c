#include "app/pivot_test.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"

// 旋回条件(未調整の控えめな初期値)
#define PIVOT_TEST_ANGLE_DEG   90.0f
#define PIVOT_TEST_OMEGA_DPS   180.0f   // 最高角速度[deg/s](選ぶときに最初に出す値)
#define PIVOT_TEST_ALPHA_DPS2  1800.0f  // 角加速度[deg/s^2]

#define PIVOT_TEST_PRE_MS      200   // 回り出す前に止まったまま記録する時間
#define PIVOT_TEST_HOLD_MS     500   // 左に回った後、右に戻る前に止まっている時間
#define PIVOT_TEST_POST_MS     500   // 戻った後も記録する時間
#define PIVOT_TEST_TIMEOUT_MS  3000  // 1回の旋回がこれを過ぎても終わらなければ打ち切る

// 最高角速度は走る前に選ぶ(SPEED_SELECT_PIVOT_OMEGA_DPS。PIVOT_TEST_OMEGA_DPS に一番近いものから始める)
static float s_omega_dps = PIVOT_TEST_OMEGA_DPS;
static const float kOmegas[] = SPEED_SELECT_PIVOT_OMEGA_DPS;

// 1回の旋回時間の見積もり[ms](台形: 角度/ω + ω/α。三角形になるときは長めに出る)
static uint32_t TurnMs(void) {
    return (uint32_t)((PIVOT_TEST_ANGLE_DEG / s_omega_dps + s_omega_dps / PIVOT_TEST_ALPHA_DPS2) * 1000.0f);
}
static uint32_t LogMs(void) {
    return PIVOT_TEST_PRE_MS + 2u * TurnMs() + PIVOT_TEST_HOLD_MS + PIVOT_TEST_POST_MS + 500u;
}

// 手を離してから回り出すまでの待ち[ms](手を離す時間)。
#define PIVOT_TEST_START_DELAY_MS 1000

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("pivot");
    Logger_SetFileName("turn90");
    Logger_AddField("omega_ref", &d->target_omega_dps);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->angle_ref_deg); // 目標の向き(angle と同じ基準)
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ang_corr", &d->ang_corr_dps);
    // 前後のずれ。旋回では目標の距離が0のままなので、そのまま位置の誤差になる
    // (位置の補正 = −POSITION_KP × dist。列の上限のため pos_corr は記録しない)
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("vl_ref", &d->vl_ref);
    Logger_AddField("vr_ref", &d->vr_ref);
    Logger_AddField("pwm_l", &d->pwm_l);
    Logger_AddField("pwm_r", &d->pwm_r);
    Logger_AddField("ff_l", &d->ff_l);
    Logger_AddField("ff_r", &d->ff_r);
    Logger_AddField("i_l", &d->i_l);
    Logger_AddField("i_r", &d->i_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_SetDuration(LogMs());
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
    App_StartPivot(angle_deg, s_omega_dps, PIVOT_TEST_ALPHA_DPS2);
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
    // 最高角速度を選ぶ(記録の長さもこれで決まるので、ログの設定より前に選ぶ)
    s_omega_dps = ModeUI_SelectValue("OMEGA", "dps", kOmegas, (uint8_t)(sizeof(kOmegas) / sizeof(kOmegas[0])),
                                     PIVOT_TEST_OMEGA_DPS);
    printf("PIVOT TEST: +%.0f deg then -%.0f deg, omega=%.0f dps, alpha=%.0f dps^2\r\n",
           PIVOT_TEST_ANGLE_DEG, PIVOT_TEST_ANGLE_DEG, s_omega_dps, PIVOT_TEST_ALPHA_DPS2);
    printf("ANGULAR: %s ANGULAR_KP=%.2f ANGLE_KP=%.2f, GYRO_Z_SIGN=%.0f\r\n",
           ANGULAR_CONTROL_ENABLE ? "ON" : "OFF", ANGULAR_KP, ANGLE_KP, GYRO_Z_SIGN);
    printf("POSITION: %s POSITION_KP=%.2f\r\n",
           POSITION_CONTROL_ENABLE ? "ON" : "OFF", POSITION_KP);
    // 書き込んだプログラムの設定をログと突き合わせられるように出しておく
    printf("TREAD=%.2f mm, PIVOT_FF_FRIC CCW L=%.2f R=%.2f / CW L=%.2f R=%.2f V\r\n",
           TREAD_WIDTH_MM, PIVOT_FF_FRIC_CCW_L, PIVOT_FF_FRIC_CCW_R,
           PIVOT_FF_FRIC_CW_L, PIVOT_FF_FRIC_CW_R);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    SetupLogger();

    while (1) {
        printf("hold hand over front-left sensor to RUN\r\n");
        ModeUI_WaitHandStart();
        HAL_Delay(PIVOT_TEST_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
        // 機体が止まっている間に、ジャイロのゼロ点を測り直す(起動時の補正からずれていることがある)
        printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));

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
