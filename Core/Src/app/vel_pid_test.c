#include "app/vel_pid_test.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"

// 1回の試験の目標速度: 0 (PRE) → HIGH (STEP) → 0 (POST)。
// 立ち上がり・定常・立ち下がり(停止時の積分リセット)を1回で見る。
#define VEL_PID_TEST_TARGET_HIGH_MM_S 200.0f
#define VEL_PID_TEST_PRE_MS           200
#define VEL_PID_TEST_STEP_MS          1000
#define VEL_PID_TEST_POST_MS          800
#define VEL_PID_TEST_TOTAL_MS (VEL_PID_TEST_PRE_MS + VEL_PID_TEST_STEP_MS + VEL_PID_TEST_POST_MS)

// ボタンを離してから走り出すまでの待ち[ms](手を離す時間)。
#define VEL_PID_TEST_START_DELAY_MS 1000

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("vel_pid");
    Logger_SetFileName("step");
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("pwm_l", &d->pwm_l);
    Logger_AddField("pwm_r", &d->pwm_r);
    Logger_AddField("ff_l", &d->ff_l);
    Logger_AddField("ff_r", &d->ff_r);
    Logger_AddField("i_l", &d->i_l);
    Logger_AddField("i_r", &d->i_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_SetDuration(VEL_PID_TEST_TOTAL_MS);
}

static void RunOnce(void) {
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);
    Logger_Start();

    uint32_t t0 = HAL_GetTick();
    while (1) {
        if (FailSafe_IsTripped()) {
            Logger_Stop(); // 発動直前までのログは残す(Haltからは戻らないので送れないが、原因はUARTに出る)
            FailSafe_Halt();
        }

        uint32_t t = HAL_GetTick() - t0;
        if (t >= VEL_PID_TEST_TOTAL_MS) break;

        bool step = (t >= VEL_PID_TEST_PRE_MS) && (t < VEL_PID_TEST_PRE_MS + VEL_PID_TEST_STEP_MS);
        App_SetTargetVelocity(step ? VEL_PID_TEST_TARGET_HIGH_MM_S : 0.0f);
        HAL_Delay(1);
    }

    Logger_Stop();
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(false);
}

void VelPIDTest_Run(void) {
    printf("VEL PID TEST: 0 -> %.1f mm/s (%d ms) -> 0, total %d ms\r\n",
           VEL_PID_TEST_TARGET_HIGH_MM_S, VEL_PID_TEST_STEP_MS, VEL_PID_TEST_TOTAL_MS);
    printf("KP=%.4f KI=%.4f KD=%.4f [V]\r\n", VELOCITY_KP, VELOCITY_KI, VELOCITY_KD);
    printf("FF L: fric=%.3f gain=%.5f / R: fric=%.3f gain=%.5f [V]\r\n",
           VELOCITY_FF_FRIC_L, VELOCITY_FF_GAIN_L, VELOCITY_FF_FRIC_R, VELOCITY_FF_GAIN_R);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    SetupLogger();

    while (1) {
        printf("press button to RUN\r\n");
        ModeUI_WaitClick();
        HAL_Delay(VEL_PID_TEST_START_DELAY_MS);

        RunOnce();
        printf("done: %lu samples x %lu fields (recordable %lu ms)\r\n",
               (unsigned long)Logger_SampleCount(), (unsigned long)Logger_FieldCount(),
               (unsigned long)Logger_RecordableMs());
        SdSaveResult saved = ModeUI_SaveLogToSD(); // SDがあれば自動で保存(UARTの線なしでも残る)

        printf("press button to DUMP\r\n");
        if (saved == SD_SAVE_FAILED) {
            ModeUI_WaitClickBlinking(); // 保存の失敗を直結LEDの点滅で知らせながら待つ
        } else {
            ModeUI_WaitClick();
        }
        Logger_Dump();
    }
}
