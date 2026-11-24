#include "app/lat_ruler_test.h"

#include <math.h>
#include <stdio.h>
#include "interface/timer.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/log_event.h"
#include "app/search_run.h" // MazeRun_StartSequence(後ろの壁から真ん中へ進む)

// 寄る量。機体の後ろの角は 4 輪の真ん中から約 51mm なので、区画の真ん中から 25mm 寄った所で回っても、壁(真ん中から 84mm)まで
// 約 8mm 残る(計算。寸法は plant_sim の外形)。30mm では約 3mm しか残らないのでやめた
#define LAT_RULER_MAX_MM   25.0f
#define LAT_RULER_STEP_MM  5.0f
#define LAT_RULER_STEP_V   100.0f  // 寄るときの速さ[mm/s](FW1 の 5mm ずつと同じ)
#define LAT_RULER_STEP_ACCEL 1000.0f
#define LAT_RULER_SETTLE_MS 300    // 止まってから測り始めるまで[ms]
#define LAT_RULER_RECORD_MS 500    // 1 点で測る長さ[ms](1kHz で約 500 回読む)
#define LAT_RULER_START_DELAY_MS 1000 // 手を離してから動き出すまで[ms](この中でジャイロのゼロ点を測り直す)
#define LAT_RULER_TIMEOUT_MS 3000  // 1 つの動きがこれを過ぎても終わらなければ打ち切る

// フェイルセーフが働いていたら、記録を SD に保存してから FailSafe_Halt() へ入る(戻らない)
static void CheckFailSafe(void) {
    if (!FailSafe_IsTripped()) return;
    Logger_Stop();
    App_ControlLoop_SetEnabled(false);
    printf("FAILSAFE: %s (vbat=%.2f V)\r\n", FailSafe_CauseName(FailSafe_GetCause()), FailSafe_GetFilteredVoltage());
    ModeUI_SaveLogToSD();
    FailSafe_Halt();
}

static void DelayWatching(uint32_t ms) {
    uint32_t t0 = Timer_GetMs();
    while (Timer_GetMs() - t0 < ms) {
        CheckFailSafe();
        Timer_DelayMs(1);
    }
}

static bool WaitMotionDone(void) {
    uint32_t t0 = Timer_GetMs();
    while (!App_IsMotionDone()) {
        CheckFailSafe();
        if (Timer_GetMs() - t0 > LAT_RULER_TIMEOUT_MS) {
            printf("timeout\r\n");
            return false;
        }
    }
    return true;
}

static bool Pivot(float angle_deg) {
    App_StartPivot(angle_deg, SEARCH_TURN_OMEGA_DPS, SEARCH_TURN_ALPHA_DPS2);
    return WaitMotionDone();
}

static bool Move(float dist_mm) {
    App_StartStraightAD(dist_mm, LAT_RULER_STEP_V, 0.0f, LAT_RULER_STEP_ACCEL, LAT_RULER_STEP_ACCEL);
    return WaitMotionDone();
}

typedef struct {
    float l, r, fl, fr;
} SensorMean;

// 止まって、壁センサーを 1kHz で読んで平均する
static SensorMean Measure(void) {
    DelayWatching(LAT_RULER_SETTLE_MS);
    float s_l = 0, s_r = 0, s_fl = 0, s_fr = 0;
    uint32_t n = 0;
    uint32_t t0 = Timer_GetMs();
    while (Timer_GetMs() - t0 < LAT_RULER_RECORD_MS) {
        CheckFailSafe();
        s_l += (float)ad_l;
        s_r += (float)ad_r;
        s_fl += (float)ad_fl;
        s_fr += (float)ad_fr;
        n++;
        Timer_DelayMs(1);
    }
    SensorMean m = { s_l / (float)n, s_r / (float)n, s_fl / (float)n, s_fr / (float)n };
    return m;
}

static uint32_t s_index = 0;

// 北を向いて止まっている所で L・R を読んで残す。offset_mm は区画の真ん中からの横のずれ(東が +)。
// facing は直前に向いていた壁(0 なし、1 西、2 東)、front はそのとき前のセンサーで読んだ値
static void RecordPoint(float offset_mm, int facing, SensorMean front) {
    const ControlDebug *d = App_GetControlDebug();
    SensorMean m = Measure();
    float angle_err = d->angle_deg - d->angle_ref_deg;
    s_index++;
    Logger_Event(LOG_EV_LAT_RULER, (float)s_index, offset_mm, m.l, m.r, (float)facing);
    Logger_Event(LOG_EV_LAT_RULER2, (float)s_index, front.fl, front.fr, angle_err, d->vbat);
    printf("%2lu offset %+5.1f mm (east +)  L %.1f R %.1f  L-R %+.1f | facing %s wall: FL %.1f FR %.1f FL+FR %.1f | "
           "angle err %+.2f deg\r\n",
           (unsigned long)s_index, offset_mm, m.l, m.r, m.l - m.r,
           (facing == 1) ? "west" : (facing == 2) ? "east" : "no", front.fl, front.fr, front.fl + front.fr, angle_err);
}

// 片側を測る。北を向いて区画の真ん中に止まっている所から始め、同じ所に北を向いて戻る。
// to_wall_deg: 寄る側と逆の壁を向く回り方(+90 で西を向く → 下がると東へ寄る、−90 で東を向く → 西へ寄る)
static bool RunSide(float to_wall_deg) {
    const int facing = (to_wall_deg > 0.0f) ? 1 : 2;
    const float side = (to_wall_deg > 0.0f) ? 1.0f : -1.0f; // 寄る向き(東が +)
    const uint32_t n = (uint32_t)(LAT_RULER_MAX_MM / LAT_RULER_STEP_MM + 0.5f);
    if (!Pivot(to_wall_deg)) return false;
    for (uint32_t k = 1; k <= n; k++) {
        if (!Move(-LAT_RULER_STEP_MM)) return false; // 向いている壁から下がる
        SensorMean front = Measure();                // 壁を向いたまま、前のセンサーで壁までの遠さを残す
        if (!Pivot(-to_wall_deg)) return false;      // 北を向く
        RecordPoint(side * LAT_RULER_STEP_MM * (float)k, facing, front);
        if (!Pivot(to_wall_deg)) return false;       // また壁を向く
    }
    if (!Move(LAT_RULER_STEP_MM * (float)n)) return false; // 真ん中へ戻る
    return Pivot(-to_wall_deg);                            // 北を向く
}

void LatRulerTest_Run(void) {
    const ControlDebug *d = App_GetControlDebug();
    const uint32_t n_side = (uint32_t)(LAT_RULER_MAX_MM / LAT_RULER_STEP_MM + 0.5f);
    const uint32_t n_points = 2u * n_side + 2u; // 真ん中(始め)+ 東側 + 西側 + 真ん中(終わり)
    printf("LAT RULER: offset 0, +%.0f..+%.0f (east), -%.0f..-%.0f (west), 0 mm every %.0f mm (%lu points), %u ms each\r\n",
           LAT_RULER_STEP_MM, LAT_RULER_MAX_MM, LAT_RULER_STEP_MM, LAT_RULER_MAX_MM, LAT_RULER_STEP_MM,
           (unsigned long)n_points, (unsigned)LAT_RULER_RECORD_MS);
    printf("place in a cell with walls on the left and right (S is fine), facing north, with the tail on the back wall\r\n");

    Logger_Init();
    Logger_SetDirName("lat_ruler");
    Logger_SetFileName("lat");
    Logger_AddField("target_dist", &d->pos_ref);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddEventFields(); // 1 点ごとの平均は LOG_EV_LAT_RULER / LAT_RULER2 に残す
    // 1 点 約 5 秒(回る 2 回・測る 2 回・寄る)× 点の数(間引いて全体を残す。細かい値はイベントと UART)
    Logger_SetDuration(n_points * 5000u + 5000u);

    if (FailSafe_IsTripped()) FailSafe_Halt();
    printf("hand over front-left sensor to RUN\r\n");
    ModeUI_WaitHandStart();
    Timer_DelayMs(LAT_RULER_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
    printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
    App_ResetGyroAngle(); // 置いた向きを 0° にする
    App_SetTargetVelocity(0.0f);
    App_SetPositionHold(true);
    App_SetWallControl(false); // 壁では直さない(横の位置を動かして読むため)
    App_ControlLoop_SetEnabled(true);
    Logger_Start();

    s_index = 0;
    const SensorMean none = { 0.0f, 0.0f, 0.0f, 0.0f };
    bool ok = MazeRun_StartSequence(); // 後ろの壁に付けて置いた所から、区画の真ん中へ
    if (ok) RecordPoint(0.0f, 0, none);
    ok = ok && RunSide(90.0f);   // 西を向いて下がる = 東へ寄る
    ok = ok && RunSide(-90.0f);  // 東を向いて下がる = 西へ寄る
    if (ok) RecordPoint(0.0f, 0, none); // 始めと同じ所。値の違いが、途中で動いた分の目安になる

    DelayWatching(100); // 最後のイベントが記録の行に入るのを待つ(間引いて記録しているので)
    Logger_Stop();
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(false);
    printf("LAT RULER %s\r\n", ok ? "done" : "stopped (timeout)");
    if (ModeUI_SaveLogToSD() == SD_SAVE_FAILED) printf("SD save FAILED\r\n");

    printf("finished. reset to run again\r\n");
    while (1) {
        Timer_DelayMs(1000);
    }
}
