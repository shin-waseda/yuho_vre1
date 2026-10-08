#include "app/slalom_test.h"

#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/search_run.h"
#include "logic/control/slalom.h"

#define SLALOM_TEST_ACCEL_MM_S2  SEARCH_ACCEL_MM_S2 // 直進の加速度・減速度(探索と同じ)
#define SLALOM_TEST_PRE_MS       200   // 走り出す前に止まったまま記録する時間
#define SLALOM_TEST_POST_MS      500   // 止まった後も記録する時間
#define SLALOM_TEST_TIMEOUT_MS   3000  // 1つの動きがこれを過ぎても終わらなければ打ち切る
#define SLALOM_TEST_LOG_MS       4000  // 記録の長さ(走る時間は約 1.5 秒 + 前後の待ち。尻当ての間は記録しない)
#define SLALOM_TEST_START_DELAY_MS 1000 // 手を離してから走り出すまでの待ち(ジャイロのゼロ点を測る時間を含む)

#define HALF_SECTION_MM (SECTION_MM * 0.5f)


static void SetupLogger(void) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("slalom");
    Logger_SetFileName("turn");
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("omega_ref", &d->target_omega_dps);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("pos_ref", &d->pos_ref);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("vl_ref", &d->vl_ref);
    Logger_AddField("vr_ref", &d->vr_ref);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("ff_l", &d->ff_l);
    Logger_AddField("ff_r", &d->ff_r);
    Logger_AddField("i_l", &d->i_l);
    Logger_AddField("i_r", &d->i_r);
    Logger_AddField("ad_l", &d->ad_l); // 曲がった後の横のずれを壁で見る
    Logger_AddField("ad_r", &d->ad_r);
    Logger_SetDuration(SLALOM_TEST_LOG_MS);
}

static void CheckFailSafe(void) {
    if (FailSafe_IsTripped()) {
        Logger_Stop();
        App_ControlLoop_SetEnabled(false);
        ModeUI_SaveLogToSD(); // 止まったときの様子も残す(どこが限界だったかを後で見るため)
        FailSafe_Halt();
    }
}

static void DelayWatching(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        CheckFailSafe();
        HAL_Delay(1);
    }
}

// 目標の距離が target_mm に届くまで待つ(曲がり始める位置を遅らせないよう HAL_Delay しない)
static bool WaitTargetDistance(float target_mm) {
    uint32_t t0 = HAL_GetTick();
    while (App_GetTargetDistance() < target_mm) {
        CheckFailSafe();
        if (HAL_GetTick() - t0 > SLALOM_TEST_TIMEOUT_MS) return false;
    }
    return true;
}

static bool WaitMotionDone(void) {
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        CheckFailSafe();
        if (HAL_GetTick() - t0 > SLALOM_TEST_TIMEOUT_MS) return false;
    }
    return true;
}

// 目標の距離が target_mm になる所まで、v_max まで加速して直進する(v_end の速さで着く)
static void StraightTo(float target_mm, float v_max, float v_end) {
    float d = target_mm - App_GetTargetDistance();
    if (d < 1.0f) d = 1.0f;
    App_StartStraight(d, v_max, v_end, SLALOM_TEST_ACCEL_MM_S2);
}

// ---- 旋回の種類 ----
// 小回り 90°: 真ん中 → 半区画 + 1区画で入口の境界 → 曲がる → 1区画 + 半区画で止まる(縦3・横2区画)
// 大回り 90°: 真ん中 → 1区画で次の区画の中心 → 曲がる(斜め隣の中心へ) → 1区画で止まる(縦3・横3区画)
// 大回り 180°: 真ん中 → 1区画で次の区画の中心 → U ターン(隣の列の中心へ) → 1区画で止まる(縦3・横2区画)
typedef struct {
    const char *name;     // 表示とファイル名(向きの r / l を後ろに付ける)
    float v_mm_s, omega_dps, alpha_dps2, angle_deg;
    float pre_mm, post_mm;
    float approach_mm;    // 真ん中から旋回の始まり(前のオフセットの前)まで
    float exit_mm;        // 旋回の終わり(後ろのオフセットの後)から止まる所まで
} TestTurn;

#define TEST_TURN_COUNT 3
// 小回り(s90)の速さ。走る前に選ぶ(SPEED_SELECT_SLALOM_TEST_V_MM_S。探索より上まで選べる)。大回りは FAST_LARGE* のまま
static float s_s90_v = SLALOM_V_MM_S;
static const float kSpeeds[] = SPEED_SELECT_SLALOM_TEST_V_MM_S;
static TestTurn s_turns[TEST_TURN_COUNT];

static void ComputeTurns(void) {
    float pre, post;
    SlalomShape sh;

    SlalomParams s90 = { SLALOM_V_MM_S, SLALOM_OMEGA_DPS, SLALOM_ALPHA_DPS2, 90.0f };
    Slalom_ScaleToSpeed(&s90, s_s90_v); // 選んだ速さでも同じ形で曲がる(探索と同じ)
    SlalomOffsets so = Slalom_SmallTurnOffsets(&s90); // 前後のオフセットは探索・最短走行と同じモデルで計算する
    s_turns[0] = (TestTurn){ "s90", s90.v_mm_s, s90.omega_dps, s90.alpha_dps2, 90.0f,
                             so.pre_mm, so.post_mm,
                             HALF_SECTION_MM + SECTION_MM, SECTION_MM + HALF_SECTION_MM };

    SlalomParams l90 = { FAST_LARGE90_V_MM_S, FAST_LARGE90_OMEGA_DPS, FAST_LARGE90_ALPHA_DPS2, 90.0f };
    sh = Slalom_ComputeShape(&l90);
    Slalom_Turn90Offsets(&sh, SECTION_MM, &pre, &post);
    s_turns[1] = (TestTurn){ "l90", l90.v_mm_s, l90.omega_dps, l90.alpha_dps2, 90.0f,
                             pre + FAST_LARGE90_PRE_ADJ_MM, post + FAST_LARGE90_POST_ADJ_MM,
                             SECTION_MM, SECTION_MM };

    SlalomParams l180 = { FAST_LARGE180_V_MM_S, 0.0f, FAST_LARGE180_ALPHA_DPS2, 180.0f };
    Slalom_SolveOmegaForSide(&l180, SECTION_MM);
    sh = Slalom_ComputeShape(&l180);
    Slalom_Turn180Offsets(&sh, SECTION_MM, &pre, &post);
    s_turns[2] = (TestTurn){ "l180", l180.v_mm_s, l180.omega_dps, l180.alpha_dps2, 180.0f,
                             pre + FAST_LARGE180_PRE_ADJ_MM, post + FAST_LARGE180_POST_ADJ_MM,
                             SECTION_MM, SECTION_MM };

    for (int i = 0; i < TEST_TURN_COUNT; i++) {
        const TestTurn *t = &s_turns[i];
        printf("%-4s: v=%.0f omega=%.0f alpha=%.0f (lateral %.0f mm/s^2) -> pre %.1f post %.1f mm%s\r\n",
               t->name, t->v_mm_s, t->omega_dps, t->alpha_dps2,
               t->v_mm_s * t->omega_dps * 3.14159265f / 180.0f, t->pre_mm, t->post_mm,
               (t->pre_mm < 0.0f || t->post_mm < 0.0f) ? "  (WARNING: negative offset)" : "");
    }
}

// 1回走る。right: 右(時計回り)に曲がる。打ち切ったら false。
static bool RunOnce(const TestTurn *t, bool right) {
    App_SetPositionHold(true);
    App_SetWallPush(false);
    App_SetWallControl(false); // 曲がった後のずれをそのまま見るため、壁の制御は使わない
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);

    // 尻当てで区画の真ん中・置いた向きにそろえる(手で置いたときの横のずれ・向きのずれを消す)
    if (!MazeRun_StartSequence()) return false;
    float base = App_GetTargetDistance(); // 区画の真ん中の目標の距離
    Logger_Start(); // 曲がるところを細かく残すため、記録は尻当ての後から
    DelayWatching(SLALOM_TEST_PRE_MS);

    // 旋回の始まりまで加速して、前のオフセットまで旋回の速さで進む
    float ref = base + t->approach_mm + t->pre_mm;
    StraightTo(ref, t->v_mm_s, t->v_mm_s);
    if (!WaitTargetDistance(ref)) return false;

    // 曲がる(並進の速さは保ったまま)。曲がっている間の道のりも目標の距離に足されていく
    App_StartSlalom(right ? -t->angle_deg : t->angle_deg, t->omega_dps, t->alpha_dps2);
    if (!WaitMotionDone()) return false;

    // 後ろのオフセット + 出口の直進で止まる
    ref = App_GetTargetDistance() + t->post_mm + t->exit_mm;
    StraightTo(ref, t->v_mm_s, 0.0f);
    if (!WaitMotionDone()) return false;

    DelayWatching(SLALOM_TEST_POST_MS);
    Logger_Stop();
    App_ControlLoop_SetEnabled(false);
    return true;
}

void SlalomTest_Run(void) {
    // 小回り(s90)の速さを選ぶ(角速度・角加速度は形が変わらないように合わせる)
    s_s90_v = ModeUI_SelectValue("SPEED(s90)", "mm/s", kSpeeds, (uint8_t)(sizeof(kSpeeds) / sizeof(kSpeeds[0])),
                                 SLALOM_V_MM_S);
    printf("SLALOM TEST (wall control off)\r\n");
    ComputeTurns();
    printf("ANGULAR_KP=%.2f ANGLE_KP=%.2f, POSITION_KP=%.2f, VELOCITY_KP=%.4f\r\n",
           ANGULAR_KP, ANGLE_KP, POSITION_KP, VELOCITY_KP);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    SetupLogger();
    // 選べる組み合わせ: 旋回の種類 × 右・左(番号 = 種類 × 2 + 左なら 1)
    static const char *const kFileNames[TEST_TURN_COUNT * 2] = {
        "s90r", "s90l", "l90r", "l90l", "l180r", "l180l",
    };
    uint8_t sel = 0;

    while (1) {
        // 旋回を選ぶ: ボタンのクリックで切り替え、手かざしで走り出す(シフトレジスタの LED に番号を出す)
        printf("put in a cell with walls on the left and behind (facing north). turn: %s  (click: change, hand: RUN)\r\n",
               kFileNames[sel]);
        LED_SetShiftPattern((uint16_t)(1u << sel));
        while (ModeUI_WaitHandStartOrClick()) {
            sel = (uint8_t)((sel + 1u) % (TEST_TURN_COUNT * 2u));
            printf("turn: %s\r\n", kFileNames[sel]);
            LED_SetShiftPattern((uint16_t)(1u << sel));
        }
        const TestTurn *t = &s_turns[sel / 2u];
        bool right = (sel % 2u) == 0u;
        Logger_SetFileName(kFileNames[sel]);

        HAL_Delay(SLALOM_TEST_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
        printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
        App_ResetGyroAngle(); // 走り出しの向きを 0° にする

        bool ok = RunOnce(t, right);
        if (!ok) {
            App_SetTargetVelocity(0.0f);
            DelayWatching(300);
            Logger_Stop();
            App_ControlLoop_SetEnabled(false);
            printf("timeout\r\n");
        }
        printf("done: %lu samples x %lu fields (recordable %lu ms)\r\n",
               (unsigned long)Logger_SampleCount(), (unsigned long)Logger_FieldCount(),
               (unsigned long)Logger_RecordableMs());
        SdSaveResult saved = ModeUI_SaveLogToSD(); // SDがあれば自動で保存

        // SD に保存できたら、UART へは送らずにすぐ次を選べるようにする(ログを貯めやすくするため)
        if (saved != SD_SAVE_OK) {
            printf("press button to DUMP\r\n");
            if (saved == SD_SAVE_FAILED) {
                ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR); // 保存の失敗を左後ろのLEDの点滅で知らせながら待つ
            } else {
                ModeUI_WaitClick();
            }
            Logger_Dump();
        }
    }
}

// ---- 小回りの連続の試験(速さを上げながら、右で行って左で戻る)----
// A(西と南に壁)から北向きに出て右の試験 → B(東と南に壁)の真ん中で東向きに止まる → 180° 回る →
// 左の試験 → A の真ん中で南向きに止まる → 180° 回る、をくり返す(置き直さずに右と左のログが同じ数ずつ取れる)。
// 1つの速さで SLALOM_SWEEP_REPEAT 往復し、選んだ始めの速さから終わりの速さまで、選べる速さの順に上げていく。
// 打ち切った・電池が下がったときは、その時の速さの番号を LED の棒グラフで点滅させて止まる。

// 区画の真ん中で 180° 回る(探索の超信地旋回と同じ速さ)。打ち切ったら false
static bool TurnAround(void) {
    App_SetPositionHold(true);
    App_SetWallPush(false);
    App_SetWallControl(false);
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);
    DelayWatching(100);
    App_StartPivot(180.0f, SEARCH_TURN_OMEGA_DPS, SEARCH_TURN_ALPHA_DPS2);
    bool ok = WaitMotionDone();
    DelayWatching(100);
    App_ControlLoop_SetEnabled(false);
    return ok;
}

// 選んでいた速さの番号(1〜)を棒グラフで点滅させ続ける(戻らない)
static void SweepHalt(const char *why, uint8_t speed_no) {
    App_ControlLoop_SetEnabled(false);
    printf("SLALOM SWEEP stopped (%s) at speed #%u\r\n", why, speed_no);
    while (1) {
        LED_SetShiftPattern((uint16_t)((1u << speed_no) - 1u));
        for (int i = 0; i < 50; i++) {
            if (FailSafe_IsTripped()) FailSafe_Halt();
            HAL_Delay(10);
        }
        LED_SetShiftPattern(0x0000u);
        HAL_Delay(300);
    }
}

void SlalomSweep_Run(void) {
    const uint8_t count = (uint8_t)(sizeof(kSpeeds) / sizeof(kSpeeds[0]));
    float v_from = ModeUI_SelectValue("FROM(s90)", "mm/s", kSpeeds, count, kSpeeds[0]);
    float v_to = ModeUI_SelectValue("TO(s90)", "mm/s", kSpeeds, count, kSpeeds[count - 1u]);
    uint8_t i_from = 0, i_to = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (kSpeeds[i] == v_from) i_from = i;
        if (kSpeeds[i] == v_to) i_to = i;
    }
    if (i_to < i_from) i_to = i_from;
    printf("SLALOM SWEEP: s90 %.0f -> %.0f mm/s, %u round trips each (wall control off)\r\n",
           kSpeeds[i_from], kSpeeds[i_to], SLALOM_SWEEP_REPEAT);
    printf("put in cell A (walls west and south) facing north. B (2 east, 2 north) needs walls east and south. hand: START\r\n");
    if (FailSafe_IsTripped()) FailSafe_Halt();

    SetupLogger();
    LED_SetShiftPattern(0x0000u);
    while (ModeUI_WaitHandStartOrClick()) {
        // 最初の1回だけ手かざしで始める(クリックは無視する)
    }
    HAL_Delay(SLALOM_TEST_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間

    bool first = true;
    for (uint8_t si = i_from; si <= i_to; si++) {
        s_s90_v = kSpeeds[si];
        ComputeTurns();
        for (uint8_t rep = 0; rep < SLALOM_SWEEP_REPEAT; rep++) {
            for (uint8_t dir = 0; dir < 2u; dir++) { // 0: 右(A → B)、1: 左(B → A)
                bool right = (dir == 0u);
                if (!first) {
                    DelayWatching(SLALOM_SWEEP_PAUSE_MS);
                    if (!TurnAround()) SweepHalt("turn around timeout", (uint8_t)(si + 1u));
                }
                first = false;
                float vbat = FailSafe_GetFilteredVoltage();
                printf("---- s90 %s %.0f mm/s (round %u/%u): vbat %.2f V\r\n", right ? "R" : "L", s_s90_v,
                       rep + 1u, SLALOM_SWEEP_REPEAT, vbat);
                if (vbat < LONG_LOG_MIN_VBAT_V) SweepHalt("low battery", (uint8_t)(si + 1u));

                Logger_SetFileName(right ? "s90r" : "s90l");
                printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
                App_ResetGyroAngle();
                bool ok = RunOnce(&s_turns[0], right);
                if (!ok) {
                    App_SetTargetVelocity(0.0f);
                    DelayWatching(300);
                    Logger_Stop();
                    App_ControlLoop_SetEnabled(false);
                }
                SdSaveResult saved = ModeUI_SaveLogToSD();
                if (!ok) SweepHalt("timeout", (uint8_t)(si + 1u));
                if (saved == SD_SAVE_FAILED) SweepHalt("SD save failed", (uint8_t)(si + 1u));
            }
        }
    }
    printf("SLALOM SWEEP: all done\r\n");
    SweepHalt("all done", (uint8_t)(i_to + 1u));
}
