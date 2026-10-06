#include "app/party_mode.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"

// 手を離してから向きを保ち始めるまでの待ち[ms](手を離す時間)。
#define PARTY_START_DELAY_MS 1000

// 記録する長さ[ms]。ボタンで止めるまで続くので長さが決まらない。RAMに収まるよう、
// 始めからこの時間ぶんだけ(間引いて)記録し、過ぎた後も向きは保ち続ける。
// 15列・15sで約19msごとの記録になる。
#define PARTY_LOG_MS 15000

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("party");
    Logger_SetFileName("hold");
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ang_corr", &d->ang_corr_dps);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("vl_ref", &d->vl_ref);
    Logger_AddField("vr_ref", &d->vr_ref);
    Logger_AddField("pwm_l", &d->pwm_l);
    Logger_AddField("pwm_r", &d->pwm_r);
    Logger_AddField("i_l", &d->i_l);
    Logger_AddField("i_r", &d->i_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_SetDuration(PARTY_LOG_MS);
}

void PartyMode_Run(void) {
    printf("PARTY: keep heading with gyro (position hold OFF)\r\n");
    printf("ANGULAR: %s ANGULAR_KP=%.2f ANGLE_KP=%.2f, limit %.0f dps\r\n",
           ANGULAR_CONTROL_ENABLE ? "ON" : "OFF", ANGULAR_KP, ANGLE_KP,
           ANGULAR_CORR_LIMIT_RAD_S * 180.0f / 3.14159265f);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    App_SetPositionHold(false);
    SetupLogger();

    while (1) {
        printf("hold hand over front-left sensor to START\r\n");
        ModeUI_WaitHandStart();
        HAL_Delay(PARTY_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
        // 機体が止まっている間に、ジャイロのゼロ点を測り直す(向きがゆっくり回っていかないように)
        printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));

        // 有効にした時点の向きを保つ(目標の角速度は0のまま)
        App_SetTargetVelocity(0.0f);
        App_ControlLoop_SetEnabled(true);
        Logger_Start(); // 記録が一杯になると自動で止まる(向きは保ち続ける)
        printf("holding heading (log first %lu ms). press button to STOP\r\n",
               (unsigned long)PARTY_LOG_MS);
        ModeUI_WaitClick(); // 待っている間にフェイルセーフが発動したら止まる(戻らない)
        Logger_Stop();
        App_ControlLoop_SetEnabled(false);
        printf("stopped: %lu samples x %lu fields (recordable %lu ms)\r\n",
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
