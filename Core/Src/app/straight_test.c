#include "app/straight_test.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"

// 走行条件: 区画数ぶんを 最高速度・加速度 で走って止まる。
#define STRAIGHT_TEST_SECTIONS   6   // 走らせる場所の広さに合わせる(6区画 = 1080mm)
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

// 手を離してから走り出すまでの待ち[ms](手を離す時間)。
#define STRAIGHT_TEST_START_DELAY_MS 1000

// 1: 壁センサーの値を記録する(壁のある通路を走らせて、境界での値や区画の中での変わり方を見る)。
//    列の上限があるので、FF・I 項・車輪の目標の代わりに壁センサーの4列を入れる。ファイル名は wall_run。
// 0: 制御の調整用(FF・I 項・車輪の目標を記録する)。ファイル名は trapezoid。
#define STRAIGHT_TEST_LOG_WALL 1

static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("straight");
#if STRAIGHT_TEST_LOG_WALL
    Logger_SetFileName("wall_run");
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("pos_ref", &d->pos_ref);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ang_corr", &d->ang_corr_dps);
    Logger_AddField("wall_ofs", &d->wall_offset_deg);
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddField("vbat", &d->vbat);
#else
    Logger_SetFileName("trapezoid");
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("pos_ref", &d->pos_ref); // 目標の距離(dist と同じ基準)
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("pos_corr", &d->pos_corr);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("vl_ref", &d->vl_ref);
    Logger_AddField("vr_ref", &d->vr_ref);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ang_corr", &d->ang_corr_dps);
    Logger_AddField("ff_l", &d->ff_l);
    Logger_AddField("ff_r", &d->ff_r);
    Logger_AddField("i_l", &d->i_l);
    Logger_AddField("i_r", &d->i_r);
    Logger_AddField("vbat", &d->vbat);
#endif
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
    printf("ANGULAR: %s ANGULAR_KP=%.2f ANGLE_KP=%.2f\r\n",
           ANGULAR_CONTROL_ENABLE ? "ON" : "OFF", ANGULAR_KP, ANGLE_KP);
    printf("POSITION: %s POSITION_KP=%.2f\r\n",
           POSITION_CONTROL_ENABLE ? "ON" : "OFF", POSITION_KP);
    printf("WALL: %s WALL_KP_DEG=%.3f max %.1f deg REF L=%d R=%d\r\n",
           WALL_CONTROL_ENABLE ? "ON" : "OFF", WALL_KP_DEG, WALL_OFFSET_MAX_DEG, WALL_REF_L, WALL_REF_R);
    printf("FF L: fric=%.3f gain=%.5f acc=%.6f / R: fric=%.3f gain=%.5f acc=%.6f [V]\r\n",
           VELOCITY_FF_FRIC_L, VELOCITY_FF_GAIN_L, VELOCITY_FF_ACC_L,
           VELOCITY_FF_FRIC_R, VELOCITY_FF_GAIN_R, VELOCITY_FF_ACC_R);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    SetupLogger();

    while (1) {
        printf("hold hand over front-left sensor to RUN\r\n");
        ModeUI_WaitHandStart();
        HAL_Delay(STRAIGHT_TEST_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
        // 機体が止まっている間に、ジャイロのゼロ点を測り直す(起動時の補正からずれていることがある)
        printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));

        RunOnce();
        printf("done: %lu samples x %lu fields (recordable %lu ms)\r\n",
               (unsigned long)Logger_SampleCount(), (unsigned long)Logger_FieldCount(),
               (unsigned long)Logger_RecordableMs());
        SdSaveResult saved = ModeUI_SaveLogToSD(); // SDがあれば自動で保存(UARTの線なしでも残る)

        printf("press button to DUMP\r\n");
        if (saved == SD_SAVE_FAILED) {
            ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR); // 保存の失敗を左後ろのLEDの点滅で知らせながら待つ
        } else {
            ModeUI_WaitClick();
        }
        Logger_Dump();
    }
}
