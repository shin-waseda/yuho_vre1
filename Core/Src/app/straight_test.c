#include "app/straight_test.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/search_run.h" // MazeRun_StartSequence(連続の試験の位置合わせ)
#include "interface/led.h"

// 走行条件: 区画数ぶんを 最高速度・加速度 で走って止まる。
#define STRAIGHT_TEST_SECTIONS   6   // 走らせる場所の広さに合わせる(6区画 = 1080mm)
#define STRAIGHT_TEST_V_MAX      300.0f  // [mm/s] 選ぶときに最初に出す速さ
#define STRAIGHT_TEST_ACCEL      2000.0f // [mm/s^2] 選ぶときの既定(1番から始まるので、表の1番目が最初に出る)

#define STRAIGHT_TEST_PRE_MS     200   // 走り出す前に止まったまま記録する時間
#define STRAIGHT_TEST_POST_MS    500   // 止まった後も記録する時間

// 走行時間の見積もり[ms](台形: 距離/v_max + v_max/accel。三角形になる短距離では長めに出る)。
// v_max は走る前に選ぶ(SPEED_SELECT_STRAIGHT_V_MM_S)。6区画/300mm/s/2000mm/s^2 で約3.75s。
#define STRAIGHT_TEST_DISTANCE_MM (STRAIGHT_TEST_SECTIONS * SECTION_MM)
static float s_v_max = STRAIGHT_TEST_V_MAX;
static const float kSpeeds[] = SPEED_SELECT_STRAIGHT_V_MM_S;
static float s_accel = STRAIGHT_TEST_ACCEL;
static const float kAccels[] = SPEED_SELECT_ACCEL_MM_S2;

static uint32_t MotionMs(void) {
    return (uint32_t)((STRAIGHT_TEST_DISTANCE_MM / s_v_max + s_v_max / s_accel) * 1000.0f);
}
// これを過ぎても終わらなければ打ち切る(見積もりの1.5倍)
static uint32_t TimeoutMs(void) {
    return MotionMs() * 3u / 2u;
}
// 記録の長さ(前後の待ち + 走行 + 余裕0.5s)。Loggerがこれに収まるよう間引く。
static uint32_t LogMs(void) {
    return STRAIGHT_TEST_PRE_MS + MotionMs() + STRAIGHT_TEST_POST_MS + 500u;
}

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
    Logger_SetDuration(LogMs());
}

// FailSafe発動時はFailSafe_Halt()へ入る(戻らない)。
static void DelayWatching(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        if (FailSafe_IsTripped()) {
            Logger_Stop();
            ModeUI_SaveLogToSD(); // 止まったときの様子も残す(限界を後で見るため)
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

    App_StartStraight(STRAIGHT_TEST_DISTANCE_MM, s_v_max, 0.0f, s_accel);

    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        if (FailSafe_IsTripped()) {
            Logger_Stop();
            ModeUI_SaveLogToSD(); // 止まったときの様子も残す(限界を後で見るため)
            FailSafe_Halt();
        }
        if (HAL_GetTick() - t0 > TimeoutMs()) {
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
    // 最高速度を選ぶ(記録の長さもこの速さで決まるので、ログの設定より前に選ぶ)
    s_v_max = ModeUI_SelectValue("SPEED", "mm/s", kSpeeds, (uint8_t)(sizeof(kSpeeds) / sizeof(kSpeeds[0])),
                                 STRAIGHT_TEST_V_MAX);
    s_accel = ModeUI_SelectValue("ACCEL", "mm/s2", kAccels, (uint8_t)(sizeof(kAccels) / sizeof(kAccels[0])),
                                 STRAIGHT_TEST_ACCEL);
    printf("STRAIGHT TEST: %d sections (%.0f mm), v_max=%.0f mm/s, accel=%.0f mm/s^2\r\n",
           STRAIGHT_TEST_SECTIONS, STRAIGHT_TEST_DISTANCE_MM,
           s_v_max, s_accel);
    printf("motion ~%lu ms, timeout %lu ms, log %lu ms\r\n",
           (unsigned long)MotionMs(), (unsigned long)TimeoutMs(),
           (unsigned long)LogMs());
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

// ---- 直進の連続の試験(速さと加速度を上げながら、行って戻る)----
// 両側と両端に壁のある通路(STRAIGHT_TEST_SECTIONS + 1 区画)の端の区画に、通路の向きに置いて手かざしで始める。
// 毎回、尻当て(start_sequence)で真ん中にそろえてから STRAIGHT_TEST_SECTIONS 区画走って止まり、180° 回って次へ
// (行きと帰りを交互にくり返すので置き直さない)。加速度ごとに、選んだ速さの範囲を順に上げる。各組み合わせで
// STRAIGHT_SWEEP_REPEAT 回(2 なら行きと帰り)。1回ごとに SD に sweep_NNNN で保存する。
// 限界を見るため、PWM・FF・I 項・車輪の目標・前の壁センサー(止まった所で前の壁までの距離が分かる)を記録する。

static void SetupSweepLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("straight");
    Logger_SetFileName("sweep");
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("target_acc", &d->target_acc);
    Logger_AddField("pos_ref", &d->pos_ref);
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
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("wall_ofs", &d->wall_offset_deg);
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddField("vbat", &d->vbat);
    Logger_SetDuration(LogMs());
}

// 止まっている所で 180° 回り、尻当てで真ん中・通路の向きにそろえる(通路の端の区画で使う)。打ち切ったら false
static bool TurnAroundAndAlign(void) {
    App_SetPositionHold(true);
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);
    DelayWatching(100);
    App_StartPivot(180.0f, SEARCH_TURN_OMEGA_DPS, SEARCH_TURN_ALPHA_DPS2);
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        DelayWatching(1);
        if (HAL_GetTick() - t0 > 3000u) return false;
    }
    DelayWatching(100);
    bool ok = MazeRun_StartSequence();
    App_ControlLoop_SetEnabled(false);
    return ok;
}

static void SweepHalt(const char *why, uint8_t no) {
    App_ControlLoop_SetEnabled(false);
    printf("STRAIGHT SWEEP stopped (%s) at #%u\r\n", why, no);
    while (1) {
        LED_SetShiftPattern((uint16_t)((1u << (no > 15u ? 15u : no)) - 1u));
        DelayWatching(500);
        LED_SetShiftPattern(0x0000u);
        DelayWatching(300);
    }
}

// 選んだ値の番号を表から探す
static uint8_t IndexOf(const float *values, uint8_t count, float v) {
    for (uint8_t i = 0; i < count; i++) {
        if (values[i] == v) return i;
    }
    return 0;
}

void StraightSweep_Run(void) {
    const uint8_t nv = (uint8_t)(sizeof(kSpeeds) / sizeof(kSpeeds[0]));
    const uint8_t na = (uint8_t)(sizeof(kAccels) / sizeof(kAccels[0]));
    uint8_t v0 = IndexOf(kSpeeds, nv, ModeUI_SelectValue("SPEED FROM", "mm/s", kSpeeds, nv, kSpeeds[0]));
    uint8_t v1 = IndexOf(kSpeeds, nv, ModeUI_SelectValue("SPEED TO", "mm/s", kSpeeds, nv, kSpeeds[nv - 1u]));
    uint8_t a0 = IndexOf(kAccels, na, ModeUI_SelectValue("ACCEL FROM", "mm/s2", kAccels, na, kAccels[0]));
    uint8_t a1 = IndexOf(kAccels, na, ModeUI_SelectValue("ACCEL TO", "mm/s2", kAccels, na, kAccels[na - 1u]));
    if (v1 < v0) v1 = v0;
    if (a1 < a0) a1 = a0;
    printf("STRAIGHT SWEEP: %d sections, speed %.0f..%.0f mm/s, accel %.0f..%.0f mm/s^2, x%u each\r\n",
           STRAIGHT_TEST_SECTIONS, kSpeeds[v0], kSpeeds[v1], kAccels[a0], kAccels[a1], STRAIGHT_SWEEP_REPEAT);
    printf("put at an end cell of a corridor (%d cells, walls on both sides and both ends), facing along it. hand: START\r\n",
           STRAIGHT_TEST_SECTIONS + 1);
    if (FailSafe_IsTripped()) FailSafe_Halt();

    LED_SetShiftPattern(0x0000u);
    ModeUI_WaitHandStart();
    HAL_Delay(STRAIGHT_TEST_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
    printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));

    // 最初の位置合わせ(左と後ろに壁があること)
    App_SetPositionHold(true);
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);
    if (!MazeRun_StartSequence()) SweepHalt("start sequence", 0);
    App_ControlLoop_SetEnabled(false);

    uint8_t no = 0;
    bool first = true;
    for (uint8_t ai = a0; ai <= a1; ai++) {
        for (uint8_t vi = v0; vi <= v1; vi++) {
            for (uint8_t rep = 0; rep < STRAIGHT_SWEEP_REPEAT; rep++) {
                no++;
                if (!first) {
                    DelayWatching(STRAIGHT_SWEEP_PAUSE_MS);
                    if (!TurnAroundAndAlign()) SweepHalt("turn around", no);
                }
                first = false;
                s_accel = kAccels[ai];
                s_v_max = kSpeeds[vi];
                float vbat = FailSafe_GetFilteredVoltage();
                printf("---- #%u: v %.0f mm/s, accel %.0f mm/s^2: vbat %.2f V\r\n", no, s_v_max, s_accel, vbat);
                if (vbat < LONG_LOG_MIN_VBAT_V) SweepHalt("low battery", no);

                SetupSweepLogger(); // 記録の長さは速さと加速度で決まるので毎回
                printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
                RunOnce();
                if (ModeUI_SaveLogToSD() == SD_SAVE_FAILED) SweepHalt("SD save failed", no);
            }
        }
    }
    printf("STRAIGHT SWEEP: all done\r\n");
    SweepHalt("all done", no);
}
