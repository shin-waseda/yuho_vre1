#include "app/front_wall_test.h"

#include <math.h>
#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/log_event.h"

// ---- 位置(FW1・FW2 共通)----
// 距離は「置いた所(S の後ろの壁に付けた所)からの目標の距離」で数える(FW1 と FW2 で同じ約束)。
// 壁を読む所: 置いた所 → スタートの区画の真ん中 SEARCH_START_FRONT_MM → 境界まで半区画 → N1 を 1 区画 → 次の境界の
// SEARCH_WALL_READ_BEFORE_MM 手前(探索と同じ約束)
#define FW_READ_MM (SEARCH_START_FRONT_MM + 0.5f * SECTION_MM + SECTION_MM - SEARCH_WALL_READ_BEFORE_MM)
// 計画の d(4輪の真ん中から前壁の面まで)と、置いた所からの距離 s の対応: d = FW_D_AT_READ_MM + (FW_READ_MM − s)。
// 186mm は計画が寸法から出した値で、確かめていない(ログには s も残すので、あとで直せる)
#define FW_D_AT_READ_MM 186.0f
#define FW_S_OF_D(d) (FW_READ_MM + FW_D_AT_READ_MM - (d))

#define FW_ACCEL        3000.0f // 加速度・減速度[mm/s²](探索と同じ)
#define FW_START_DELAY_MS 1000  // 手を離してから動き出すまで[ms](この中でジャイロのゼロ点を測り直す)
#define FW_TIMEOUT_MS   3000    // 1つの動きがこれを過ぎても終わらなければ打ち切る

// ---- FW1(止めて測る。自動で 5mm ずつ動く)----
#define FW1_D_NEAR_MM   156.0f  // 一番近い d
#define FW1_D_FAR_MM    216.0f  // 一番遠い d
#define FW1_STEP_MM     5.0f
#define FW1_MOVE_V      300.0f  // 最初の点まで進む速さ[mm/s]
#define FW1_STEP_V      100.0f  // 5mm ずつ動く速さ[mm/s]
#define FW1_STEP_ACCEL  1000.0f
#define FW1_SETTLE_MS   300     // 止まってから測り始めるまで[ms]
#define FW1_RECORD_MS   500     // 1 点で測る長さ[ms](1kHz で約 500 回読む)

// ---- FW2(走って測る。自動で後ろの壁へ戻る)----
#define FW2_V_MAX       600.0f  // 直進の速さ[mm/s](探索の直進と同じ)
#define FW2_B_HOLD_MM   20.0f   // B: 壁を読む所からこれだけ先まで、同じ速さのまま通る[mm]
#define FW2_RUNS_EACH   3       // 1 つの走り方を何本ずつ走るか
#define FW2_SETTLE_MS   200     // 止まってから記録を止めるまで[ms]
#define FW2_RETURN_V    300.0f  // 後ろへ戻る速さ[mm/s]
#define FW2_RETURN_GAP_MM 20.0f // 置いた所のこれだけ手前まで下がってから押し当てる[mm]
#define FW2_PUSH_V      100.0f  // 後ろの壁へ押し当てる速さ[mm/s](前の尻当てと同じ)
#define FW2_PUSH_MS     600     // 押し当てる時間[ms](20mm を下がり、残りは押し当てる)
#define FW2_PUSH_SETTLE_MS 300  // 押し当てた後、モーターを止めて待つ時間[ms]

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
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        CheckFailSafe();
        HAL_Delay(1);
    }
}

static bool WaitMotionDone(void) {
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        CheckFailSafe();
        if (HAL_GetTick() - t0 > FW_TIMEOUT_MS) {
            printf("timeout\r\n");
            return false;
        }
    }
    return true;
}

// 置いた所(後ろの壁に付けた所)で止まっている状態から: ジャイロのゼロ点を測り直し、向きを 0 にして、
// 横壁追従だけ(横のずれの FB は切る)で制御を有効にする。ここが目標の距離 0
static void BeginAtStart(void) {
    printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
    App_ResetGyroAngle();
    App_SetTargetVelocity(0.0f);
    App_SetPositionHold(true);
    App_SetLateralControl(false); // 横は横壁追従だけで決める(計画: A と同じ選び方)
    App_SetWallControl(true);
    App_ControlLoop_SetEnabled(true);
}

static void EndControl(void) {
    App_SetTargetVelocity(0.0f);
    App_SetWallControl(false);
    App_SetLateralControl(true);
    App_ControlLoop_SetEnabled(false);
}

static void ReportSaved(SdSaveResult saved) {
    if (saved == SD_SAVE_FAILED) printf("SD save FAILED\r\n");
}

// ================= FW1 =================

static void Fw1_Run(void) {
    const ControlDebug *d = App_GetControlDebug();
    const uint32_t n_side = (uint32_t)((FW1_D_FAR_MM - FW1_D_NEAR_MM) / FW1_STEP_MM + 0.5f) + 1u; // 片道の点の数
    printf("FW1: d %.0f -> %.0f -> %.0f mm every %.0f mm (%lu points), %u ms each. "
           "place with the tail on the back wall of S\r\n",
           FW1_D_FAR_MM, FW1_D_NEAR_MM, FW1_D_FAR_MM, FW1_STEP_MM, (unsigned long)(2u * n_side), (unsigned)FW1_RECORD_MS);
    printf("s (from placed) = %.1f + %.1f - d  (read point s=%.1f is d=%.0f)\r\n",
           FW_READ_MM, FW_D_AT_READ_MM, FW_READ_MM, FW_D_AT_READ_MM);

    Logger_Init();
    Logger_SetDirName("front_wall");
    Logger_SetFileName("fw1");
    Logger_AddField("target_dist", &d->pos_ref);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddEventFields(); // 1 点ごとの平均は LOG_EV_FW1_POINT / POINT2 に残す
    // 1 点 約 0.9 秒 × 点の数 + 最初の点まで(間引いて全体を残す。細かい値はイベントと UART)
    Logger_SetDuration(2u * n_side * (FW1_SETTLE_MS + FW1_RECORD_MS + 150u) + 3000u);

    if (FailSafe_IsTripped()) FailSafe_Halt();
    printf("hand over front-left sensor to RUN\r\n");
    ModeUI_WaitHandStart();
    HAL_Delay(FW_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
    BeginAtStart();
    Logger_Start();

    // 一番遠い点(d = FW1_D_FAR_MM)まで進む
    App_StartStraightAD(FW_S_OF_D(FW1_D_FAR_MM), FW1_MOVE_V, 0.0f, FW_ACCEL, FW_ACCEL);
    bool ok = WaitMotionDone();

    for (uint32_t k = 0; ok && k < 2u * n_side; k++) {
        // 行き: 遠い → 近い(k = 0 は最初の点)。帰り: 近い → 遠い(折り返しの点も測り直す)
        uint32_t j = (k < n_side) ? k : (2u * n_side - 1u - k);
        float d_nom = FW1_D_FAR_MM - FW1_STEP_MM * (float)j;
        if (k > 0u && k != n_side) {
            float step = (k < n_side) ? FW1_STEP_MM : -FW1_STEP_MM; // 前へ(壁へ近づく)/ 後ろへ
            App_StartStraightAD(step, FW1_STEP_V, 0.0f, FW1_STEP_ACCEL, FW1_STEP_ACCEL);
            if (!WaitMotionDone()) { ok = false; break; }
        }
        DelayWatching(FW1_SETTLE_MS);

        // 1kHz で読んで平均と FL+FR のばらつきを出す
        float s_fl = 0, s_fr = 0, s_l = 0, s_r = 0, s_sum2 = 0, s_dist = 0;
        uint32_t n = 0;
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < FW1_RECORD_MS) {
            CheckFailSafe();
            float fl = (float)ad_fl, fr = (float)ad_fr;
            s_fl += fl;
            s_fr += fr;
            s_l += (float)ad_l;
            s_r += (float)ad_r;
            s_sum2 += (fl + fr) * (fl + fr);
            s_dist += d->dist_mm;
            n++;
            HAL_Delay(1);
        }
        float m_fl = s_fl / (float)n, m_fr = s_fr / (float)n, m_sum = m_fl + m_fr;
        float var = s_sum2 / (float)n - m_sum * m_sum;
        float sd = (var > 0.0f) ? sqrtf(var) : 0.0f;
        float dist = s_dist / (float)n;
        Logger_Event(LOG_EV_FW1_POINT, (float)(k + 1u), d_nom, dist, m_fl, m_fr);
        Logger_Event(LOG_EV_FW1_POINT2, (float)(k + 1u), s_l / (float)n, s_r / (float)n, sd, d->vbat);
        printf("%2lu d %.0f  s_enc %.1f  FL %.1f FR %.1f  FL+FR %.1f (sd %.1f)  FL-FR %+.1f  L %.1f R %.1f\r\n",
               (unsigned long)(k + 1u), d_nom, dist, m_fl, m_fr, m_sum, sd, m_fl - m_fr, s_l / (float)n, s_r / (float)n);
    }

    DelayWatching(100); // 最後のイベントが記録の行に入るのを待つ(間引いて記録しているので)
    Logger_Stop();
    EndControl();
    printf("FW1 %s\r\n", ok ? "done" : "stopped (timeout)");
    ReportSaved(ModeUI_SaveLogToSD());
}

// ================= FW2 =================

typedef struct {
    bool type_b;    // false: A(壁を読む所で減速)、true: B(同じ速さで通る)
    float v_pass;   // 壁を読む所の速さ
    const char *file;
} Fw2Kind;

static const Fw2Kind kFw2Kinds[] = {
    { false, 400.0f, "fw2a_400" },
    { false, 500.0f, "fw2a_500" },
    { false, 600.0f, "fw2a_600" },
    { true, 600.0f, "fw2b_600" },
    { true, 500.0f, "fw2b_500" }, // 探索の小回りの速さ(search_0037〜0043 は 500)。2026-11-06 に足した
};

static void Fw2_SetupLogger(const Fw2Kind *k) {
    const ControlDebug *d = App_GetControlDebug();
    Logger_Init();
    Logger_SetDirName("front_wall");
    Logger_SetFileName(k->file);
    // 1kHz で約 1 秒(time を入れて 12 列 → 1024 行)。走りは 0.8 秒ほど
    Logger_AddField("dist", &d->dist_mm);              // エンコーダの道のり(左右の平均)[mm]
    Logger_AddField("target", &d->target_mm_s);        // v_ref
    Logger_AddField("target_acc", &d->target_acc);     // a_ref
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_r", &d->ad_r);
    if (k->type_b) { // 前傾を見る(計画: gyro_y・acc_x は B だけでもよい)
        Logger_AddField("gyro_y", &d->gyro_y_dps);
        Logger_AddField("acc_x", &d->acc_x);
    } else {
        Logger_AddField("gyro_z", &d->gyro_z_dps);
        Logger_AddField("angle", &d->angle_deg);
    }
    Logger_SetDecimation(1);
}

typedef struct {
    float target_mm, dist_mm, v_mm_s;
    uint16_t l, fl, fr, r;
    bool got;
} ReadPoint;

// 置いた所で制御が有効になっている状態から 1 本走り、壁を読む所を通って止まる(制御は有効のまま)
static bool Fw2_RunOnce(const Fw2Kind *k, ReadPoint *rp) {
    const ControlDebug *d = App_GetControlDebug();
    rp->got = false;

    float first_mm = k->type_b ? FW_READ_MM + FW2_B_HOLD_MM : FW_READ_MM;
    // B は通る速さまで上げてそのまま通る(探索で小回りの手前を走るのと同じ形)。A は 600 まで上げて減速する
    float v_max = k->type_b ? k->v_pass : FW2_V_MAX;
    App_StartStraightAD(first_mm, v_max, k->v_pass, FW_ACCEL, FW_ACCEL);
    uint32_t t0 = HAL_GetTick();
    while (1) {
        CheckFailSafe();
        // A は直進の終わりがちょうど壁を読む所で、目標の距離が届くのと終わりの印が同じ tick に立つ。
        // 終わりを先に見て抜けると値を取りこぼすので、終わった回にも取る(2026-11-06)
        bool done = App_IsMotionDone();
        if (!rp->got && (done || App_GetTargetDistance() >= FW_READ_MM)) { // 探索が壁を読むのと同じ所の値を取る
            rp->target_mm = App_GetTargetDistance();
            rp->dist_mm = d->dist_mm;
            rp->v_mm_s = 0.5f * (d->vl + d->vr);
            rp->l = ad_l;
            rp->fl = ad_fl;
            rp->fr = ad_fr;
            rp->r = ad_r;
            rp->got = true;
        }
        if (done) break;
        if (HAL_GetTick() - t0 > FW_TIMEOUT_MS) {
            printf("timeout\r\n");
            return false;
        }
    }
    // 壁を読む所の後で、同じ減速度で止まる
    float stop_mm = k->v_pass * k->v_pass / (2.0f * FW_ACCEL) + 1.0f;
    App_StartStraightAD(stop_mm, k->v_pass, 0.0f, FW_ACCEL, FW_ACCEL);
    bool ok = WaitMotionDone();
    DelayWatching(FW2_SETTLE_MS);
    return ok && rp->got;
}

// 止まった所から下がって、後ろの壁に押し当てて置き直す(前の尻当てと同じやり方)。最後は制御を無効にして止まる
static bool Fw2_ReturnToStart(void) {
    App_SetWallControl(false);
    float back_mm = App_GetTargetDistance() - FW2_RETURN_GAP_MM;
    App_StartStraightAD(-back_mm, FW2_RETURN_V, 0.0f, FW_ACCEL, FW_ACCEL);
    if (!WaitMotionDone()) return false;
    App_SetPositionHold(false); // 押し当てる間は位置を保たない(壁に当たって止まっても、目標の距離が先へ行かない)
    App_SetTargetVelocity(-FW2_PUSH_V);
    DelayWatching(FW2_PUSH_MS);
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(false);
    App_SetPositionHold(true);
    DelayWatching(FW2_PUSH_SETTLE_MS);
    return true;
}

static void Fw2_Run(void) {
    const uint32_t n_kinds = (uint32_t)(sizeof(kFw2Kinds) / sizeof(kFw2Kinds[0]));
    printf("FW2: A 400/500/600 and B 600/500, %d runs each, returning to the back wall automatically\r\n", FW2_RUNS_EACH);
    printf("read point s=%.1f mm from the placed position (start front %.0f + half %.0f + %.0f - %.0f). "
           "wall following ON, lateral FB OFF. place with the tail on the back wall of S\r\n",
           FW_READ_MM, SEARCH_START_FRONT_MM, 0.5f * SECTION_MM, SECTION_MM, SEARCH_WALL_READ_BEFORE_MM);
    if (FailSafe_IsTripped()) FailSafe_Halt();
    printf("hand over front-left sensor to RUN\r\n");
    ModeUI_WaitHandStart();
    HAL_Delay(FW_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間

    uint32_t run = 0;
    for (uint32_t i = 0; i < n_kinds; i++) {
        const Fw2Kind *k = &kFw2Kinds[i];
        for (uint32_t r = 0; r < (uint32_t)FW2_RUNS_EACH; r++) {
            run++;
            Fw2_SetupLogger(k);
            BeginAtStart(); // 1 本目は手で置いた所、2 本目からは押し当てた所
            Logger_Start();
            ReadPoint rp;
            bool ok = Fw2_RunOnce(k, &rp);
            Logger_Stop();
            if (rp.got) {
                printf("#%lu %s: read target %.1f enc %.1f v %.0f | FL %u FR %u FL+FR %u FL-FR %+d | L %u R %u\r\n",
                       (unsigned long)run, k->file, rp.target_mm, rp.dist_mm, rp.v_mm_s, rp.fl, rp.fr,
                       (unsigned)(rp.fl + rp.fr), (int)rp.fl - (int)rp.fr, rp.l, rp.r);
            } else {
                printf("#%lu %s: read point not reached\r\n", (unsigned long)run, k->file);
            }
            // 後ろの壁へ戻ってモーターを止めてから保存する(モーターを回したまま保存すると、保存の後に下がらずに
            // 止まっていた。原因は未確認。2026-11-06)。戻れなかったときも、モーターを止めてから保存する
            bool back = ok && Fw2_ReturnToStart();
            if (!back) EndControl();
            ReportSaved(ModeUI_SaveLogToSD());
            if (!back) {
                printf("FW2 stopped at #%lu (%s)\r\n", (unsigned long)run,
                       !rp.got ? "read point not reached" : "timeout");
                return;
            }
        }
    }
    EndControl();
    printf("FW2 done (%lu runs)\r\n", (unsigned long)run);
}

void FrontWallTest_Run(void) {
    static const float kTwo[] = { 1.0f, 2.0f };
    bool run = (ModeUI_SelectValue("FW(1 STATIC 2 RUN)", "", kTwo, 2u, 1.0f) == 2.0f);
    if (run) {
        Fw2_Run();
    } else {
        Fw1_Run();
    }
    printf("finished. reset to run again\r\n");
    while (1) {
        HAL_Delay(1000);
    }
}
