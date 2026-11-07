#include "app/slip_turn_test.h"

#include <math.h>
#include <stdio.h>
#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/log_event.h"
#include "logic/control/slalom.h"

// ---- 位置 ----
// 距離は「置いた所(S の後ろの壁に付けた所)からの目標の距離」で数える(探索・FW と同じ約束)。
#define SLIP_B0_MM       (SEARCH_START_FRONT_MM + 0.5f * SECTION_MM) // S と (0,1) の境界
#define SLIP_STOP_MM     (SECTION_MM - SEARCH_WALL_READ_BEFORE_MM)   // 最後の境界から止まる所まで(次の境界の 12mm 手前)

// ---- 走り ----
#define SLIP_RUNS_EACH   3       // 1 つの速さで T1・T2 を何本ずつ走るか(交互に走る)
#define SLIP_ACCEL       3000.0f // 加速度・減速度[mm/s²](探索と同じ)
#define SLIP_START_DELAY_MS 1000 // 手を離してから動き出すまで[ms](この中でジャイロのゼロ点を測り直す)
#define SLIP_TIMEOUT_MS  3000    // 1つの動きがこれを過ぎても終わらなければ打ち切る
#define SLIP_SETTLE_MS   300     // 止まってから測り始めるまで[ms]
#define SLIP_RECORD_MS   500     // 止まった所で前壁を読む長さ[ms](1kHz)

// 途中で止まったときに点滅させる直結 LED(ボタンを押すまで。止まったことを見分けるため)
#define SLIP_LED_STOPPED (LED_REAR_LEFT | LED_REAR_RIGHT)

static const float kSlipSpeeds[] = { 300.0f, 400.0f, 500.0f, 600.0f };
static const float kStraightSpeeds[] = { 300.0f, 500.0f }; // まっすぐの走り(E・N)の速さ

// 走りの種類(ファイル名 slip<種類>_<速さ> と LOG_EV_SLIP_RUN の kind)
enum {
    SLIP_T1 = 1, // S → (0,1) で右 → (1,1) の東の端
    SLIP_T2 = 2, // S → (0,1) で右 → (1,1) で左 → (1,2) の北の端
    SLIP_E = 3,  // (0,1) で東向き(西の外周の壁に付ける)→ まっすぐ (1,1) の東の端(T1 と同じ止まる所)
    SLIP_N = 4,  // (1,0) で北向き(南の外周の壁に付ける)→ まっすぐ (1,2) の北の端(T2 と同じ止まる所)
    SLIP_P1 = 5, // S → (0,1) の真ん中で止まって右へ超信地旋回 90° → (1,1) の東の端(T1 の小回りを超信地旋回にしたもの)
    SLIP_P2 = 6, // S → (0,1) の真ん中で右へ超信地旋回 90° → (1,1) で左へ小回り → (1,2) の北の端(T2 の 1 回目を超信地旋回に)
    // ---- 左右反対(LR の組。2026-11-06)----
    SLIP_T1L = 7,  // (2,0) で北向き(南の外周の壁に付ける)→ (2,1) で左 → (1,1) の西の端(前壁は西の外周)
    SLIP_P1L = 8,  // (2,0) → (2,1) の真ん中で左へ超信地旋回 90° → (1,1) の西の端
    SLIP_EW = 9,   // (2,1) で西向き((2,1) の東の壁に付ける)→ まっすぐ (1,1) の西の端(T1L と同じ止まる所)
    SLIP_T2L = 10, // (2,0) → (2,1) で左 → (1,1) で右 → (1,2) の北の端(T2 と同じ止まる所)
    SLIP_P2L = 11, // (2,0) → (2,1) の真ん中で左へ超信地旋回 → (1,1) で右へ小回り → (1,2) の北の端
    // 最短走行の ADJ CHECK の比べ相手(2026-11-07)。N と同じ動きで、ゴールの列(例: ゴール (2,2) なら (2,0))に置く
    SLIP_NG = 12,  // (2,0) で北向き(南の外周の壁に付ける)→ まっすぐ (2,2) の北の端(ゴール (2,2) の ADJ CHECK と同じ止まる所)
};
static const float kPivotSpeeds[] = { 300.0f, 500.0f }; // 超信地旋回の走り(T1・P1・T2・P2)の直進・小回りの速さ
#define SLIP_LR_V 300.0f // LR の組の速さ(速さによらないずれを見るので 1 つだけ)

typedef struct {
    float omega_dps, alpha_dps2, pre_mm, post_mm;
} TurnShape;

// true の間は、小回りの手で足す調整(SLALOM_PRE/POST_ADJ_MM)を引いた形で走る(LR の組。回る中心のずれの式を、
// 調整の入っていない小回りと超信地旋回で比べるため)
static bool s_raw_slalom = false;

// 探索と同じ形の小回りを、速さ v に合わせて作る(前後のオフセットも探索と同じ計算)
static TurnShape ShapeFor(float v) {
    SlalomParams p = {
        .v_mm_s = SLALOM_V_MM_S,
        .omega_dps = SLALOM_OMEGA_DPS,
        .alpha_dps2 = SLALOM_ALPHA_DPS2,
        .angle_deg = 90.0f,
    };
    Slalom_ScaleToSpeed(&p, v);
    SlalomOffsets o = Slalom_SmallTurnOffsets(&p);
    TurnShape s = { p.omega_dps, p.alpha_dps2, o.pre_mm, o.post_mm };
    if (s_raw_slalom) {
        s.pre_mm -= SLALOM_PRE_ADJ_MM;
        s.post_mm -= SLALOM_POST_ADJ_MM;
    }
    return s;
}

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
        if (HAL_GetTick() - t0 > SLIP_TIMEOUT_MS) {
            printf("timeout (motion)\r\n");
            return false;
        }
    }
    return true;
}

// 目標の距離が target_mm に届くまで待つ(曲がり始めが遅れないよう HAL_Delay しない)
static bool WaitTargetDistance(float target_mm) {
    uint32_t t0 = HAL_GetTick();
    while (App_GetTargetDistance() < target_mm) {
        CheckFailSafe();
        if (HAL_GetTick() - t0 > SLIP_TIMEOUT_MS) {
            printf("timeout (distance)\r\n");
            return false;
        }
    }
    return true;
}

// 今の速さのまま(または止まった所から)、目標の距離が target_mm になる所まで直進する
static void StraightTo(float target_mm, float v_max, float v_end) {
    float d = target_mm - App_GetTargetDistance();
    if (d < 1.0f) d = 1.0f;
    App_StartStraight(d, v_max, v_end, SLIP_ACCEL);
}

// 境界(*ref_mm)から小回りで曲がり、曲がった先の境界を *ref_mm に入れる(探索の SlalomTurn と同じ数え方)。
// 曲がり始めまでは速さ v のまま進む。曲がった後の直進は呼び出し側が始める
static bool Turn(float *ref_mm, bool right, float v, const TurnShape *s) {
    float start = *ref_mm + s->pre_mm;
    StraightTo(start, v, v);
    if (!WaitTargetDistance(start)) return false;
    App_StartSlalom(right ? -90.0f : 90.0f, s->omega_dps, s->alpha_dps2);
    if (!WaitMotionDone()) return false;
    *ref_mm = App_GetTargetDistance() + s->post_mm;
    return true;
}

// 止まったまま、目標の位置に対する2次元の前後のずれが小さくなるのを待つ(探索の AlongSync と同じ。壁は使わない)
static void AlongSync(void) {
    DelayWatching(SEARCH_TURN_WAIT_MS);
    uint32_t t0 = HAL_GetTick();
    while (fabsf(App_GetPositionError()) > ALONG_SYNC_TOL_MM) {
        CheckFailSafe();
        if (HAL_GetTick() - t0 > ALONG_SYNC_TIMEOUT_MS) break; // 小さくならなくても次へ進む
        HAL_Delay(1);
    }
}

// 区画の真ん中で止まっている所から、90° 超信地旋回する(探索の 180° の半分と同じ: 前後を合わせる → 90° → 前後を合わせる)
static bool Pivot90(bool right) {
    AlongSync();
    DelayWatching(SEARCH_TURN_WAIT_MS);
    App_StartPivot(right ? -90.0f : 90.0f, SEARCH_TURN_OMEGA_DPS, SEARCH_TURN_ALPHA_DPS2);
    if (!WaitMotionDone()) return false;
    DelayWatching(SEARCH_TURN_WAIT_MS);
    AlongSync();
    return true;
}

// 置いた所(後ろの壁に付けた所)で止まっている状態から: ジャイロのゼロ点を測り直し、向きを 0 にして、
// 横壁追従は切り、横のずれの FB は入れて制御を有効にする。ここが目標の距離 0
static void BeginAtStart(void) {
    printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
    App_ResetGyroAngle();
    App_SetTargetVelocity(0.0f);
    App_SetPositionHold(true);
    App_SetWallControl(false);
    App_SetLateralControl(true);
    App_ControlLoop_SetEnabled(true);
}

static void EndControl(void) {
    App_SetTargetVelocity(0.0f);
    App_SetPositionHold(true);
    App_ControlLoop_SetEnabled(false);
}

typedef struct {
    float stop_mm, dist_mm, fl, fr, l, r, sum_sd;
} StopMeasure;

static char s_file_name[16];

static void SetupLogger(int kind, float v) {
    const ControlDebug *d = App_GetControlDebug();
    snprintf(s_file_name, sizeof(s_file_name), "slip%d_%03d", kind, (int)v);
    Logger_Init();
    Logger_SetDirName("slip");
    Logger_SetFileName(s_file_name);
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("pos_ref", &d->pos_ref);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("gyro_z", &d->gyro_z_dps);
    Logger_AddField("est_x", &d->est_x_mm);
    Logger_AddField("est_y", &d->est_y_mm);
    Logger_AddField("lat_err", &d->lat_err_mm);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_AddEventFields(); // 止まった所で読んだ平均は LOG_EV_SLIP_POINT / POINT2 に残す
    // 走り(T2・P2 で約 700mm、T1・P1 で約 450mm、N で 478mm、E で 298mm)+ 止まってから読み終わるまで + 余り。
    // 超信地旋回は止まる・前後を合わせる・回るの分を足す。長いほど間引かれる
    float path_mm = (kind == SLIP_T2 || kind == SLIP_P2 || kind == SLIP_T2L || kind == SLIP_P2L) ? 700.0f
                  : (kind == SLIP_E || kind == SLIP_EW) ? 300.0f : (kind == SLIP_N || kind == SLIP_NG) ? 480.0f : 450.0f;
    uint32_t pivot_ms = (kind == SLIP_P1 || kind == SLIP_P2 || kind == SLIP_P1L || kind == SLIP_P2L) ? 2000u : 0u;
    Logger_SetDuration((uint32_t)(path_mm / v * 1000.0f) + pivot_ms + SLIP_SETTLE_MS + SLIP_RECORD_MS + 700u);
}

// 置いた所で制御が有効になっている状態から 1 本走り、止まって前壁を読む(制御は有効のまま)
static bool RunOnce(int kind, float v, StopMeasure *m) {
    const ControlDebug *d = App_GetControlDebug();
    TurnShape s = ShapeFor(v);
    Logger_Event(LOG_EV_SLIP_RUN, (float)kind, v, s.pre_mm, s.post_mm, 0.0f);

    // 右の組(T1・T2・P1・P2)は S から (0,1) で右、左の組(T1L・T2L・P1L・P2L)は (2,0) から (2,1) で左に曲がる。
    // 2 回目(T2・P2 は左、T2L・P2L は右)はどちらも (1,1) で曲がる
    bool left_set = (kind == SLIP_T1L || kind == SLIP_T2L || kind == SLIP_P1L || kind == SLIP_P2L);
    bool second = (kind == SLIP_T2 || kind == SLIP_P2 || kind == SLIP_T2L || kind == SLIP_P2L);
    float ref = SLIP_B0_MM; // 置いた区画と次の区画の境界
    if (kind == SLIP_T1 || kind == SLIP_T2 || kind == SLIP_T1L || kind == SLIP_T2L) {
        if (!Turn(&ref, !left_set, v, &s)) return false;              // 1 回目
        if (second && !Turn(&ref, left_set, v, &s)) return false;     // 2 回目(反対の向き)
    } else if (kind == SLIP_N || kind == SLIP_NG) {
        ref += SECTION_MM; // (1,0) から (1,1) を通って (1,2) へ(E・EW は隣の区画へ入るだけ)
    } else if (kind == SLIP_P1 || kind == SLIP_P2 || kind == SLIP_P1L || kind == SLIP_P2L) {
        StraightTo(ref + 0.5f * SECTION_MM, v, 0.0f); // 曲がる区画の真ん中で止まる
        if (!WaitMotionDone()) return false;
        if (!Pivot90(!left_set)) return false;
        ref = App_GetTargetDistance() + 0.5f * SECTION_MM; // 次の区画との境界
        if (second && !Turn(&ref, left_set, v, &s)) return false; // (1,1) で反対の向きへ小回り
    }
    m->stop_mm = ref + SLIP_STOP_MM;
    StraightTo(m->stop_mm, v, 0.0f);
    if (!WaitMotionDone()) return false;
    DelayWatching(SLIP_SETTLE_MS);

    // 1kHz で読んで平均と FL+FR のばらつきを出す(FW1 と同じ)
    float s_fl = 0, s_fr = 0, s_l = 0, s_r = 0, s_sum2 = 0, s_dist = 0;
    uint32_t n = 0;
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < SLIP_RECORD_MS) {
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
    m->fl = s_fl / (float)n;
    m->fr = s_fr / (float)n;
    m->l = s_l / (float)n;
    m->r = s_r / (float)n;
    m->dist_mm = s_dist / (float)n;
    float sum = m->fl + m->fr;
    float var = s_sum2 / (float)n - sum * sum;
    m->sum_sd = (var > 0.0f) ? sqrtf(var) : 0.0f;
    Logger_Event(LOG_EV_SLIP_POINT, (float)kind, v, m->stop_mm, m->fl, m->fr);
    Logger_Event(LOG_EV_SLIP_POINT2, m->dist_mm, m->l, m->r, m->sum_sd, d->vbat);
    DelayWatching(50); // 最後のイベントが記録の行に入るのを待つ(間引いて記録しているので)
    return true;
}

static const char *KindName(int kind) {
    switch (kind) {
        case SLIP_T1: return "T1";
        case SLIP_T2: return "T2";
        case SLIP_E: return "E";
        case SLIP_N: return "N";
        case SLIP_NG: return "NG";
        case SLIP_P1: return "P1";
        case SLIP_P2: return "P2";
        case SLIP_T1L: return "T1L";
        case SLIP_P1L: return "P1L";
        case SLIP_EW: return "EW";
        case SLIP_T2L: return "T2L";
        case SLIP_P2L: return "P2L";
        default: return "?";
    }
}

// 置く所の番号(手かざしを待つ間に LED で見せる): 1 S、2 (2,0)、3 (0,1) 東向き、4 (2,1) 西向き、5 (1,0)
static int KindPlaceNo(int kind) {
    switch (kind) {
        case SLIP_T1L:
        case SLIP_P1L:
        case SLIP_T2L:
        case SLIP_P2L: return 2;
        case SLIP_E: return 3;
        case SLIP_EW: return 4;
        case SLIP_N: return 5;
        case SLIP_NG: return 2; // (2,0) で北向き(左の組と同じ所)
        default: return 1;
    }
}

// 手かざしを待つ。待つ間、シフトレジスタの LED を置く所の番号 × 3 個つける(1 番 3 個 … 5 番 15 個)。
// 置く所が前の 1 本から変わったときは、それと直結の LED 全部を 0.15 秒ごとに点滅させる(置き直しを見落とさないため)。
// 手をかざしたら消して、あとは ModeUI_WaitHandStart と同じ(かざし続けて離すまで待つ)
static void WaitHandShowingPlace(int place, bool changed) {
    uint16_t bar = (uint16_t)((1u << (3u * (uint32_t)place)) - 1u);
    uint32_t t0 = HAL_GetTick();
    int shown = -1;
    while (ad_fl <= SENSOR_START_THRESHOLD) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        int on = (!changed || ((HAL_GetTick() - t0) / 150u) % 2u == 0u) ? 1 : 0;
        if (on != shown) { // 変わったときだけ書く(シフトレジスタを毎回送らない)
            LED_SetShiftPattern(on ? bar : 0u);
            if (changed) LED_SetDirectPattern(on ? LED_DIRECT_ALL : 0u);
            shown = on;
        }
        HAL_Delay(1);
    }
    LED_SetShiftPattern(0u);
    LED_SetDirectPattern(0u);
    ModeUI_WaitHandStart();
}

// 置く所(UART に出す)
static const char *KindPlace(int kind) {
    switch (kind) {
        case SLIP_E: return "(0,1) facing east, tail on the west wall";
        case SLIP_N: return "(1,0) facing north, tail on the south wall";
        case SLIP_NG: return "(2,0) facing north, tail on the south wall (the column of the goal (2,2))";
        case SLIP_T1L:
        case SLIP_P1L:
        case SLIP_T2L:
        case SLIP_P2L: return "(2,0) facing north, tail on the south wall";
        case SLIP_EW: return "(2,1) facing west, tail on the east wall of (2,1)";
        default: return "S (0,0) facing north, tail on the south wall";
    }
}

// 1 本走る: 手で置いて手をかざすのを待ち、走って止まり、前壁を読んで保存する。続けてよければ true
// (1 本ごとに手で置き直す。自動で戻って押し当てると、フェイルセーフが働いたり、向きと横の位置がそろわなかったりしたので、
// やめた。2026-11-06)
static bool RunOneByHand(uint32_t run, int kind, float v) {
    float vbat = FailSafe_GetFilteredVoltage();
    if (vbat < LONG_LOG_MIN_VBAT_V) {
        printf("SLIP stopped before #%lu: battery %.2f V\r\n", (unsigned long)run, vbat);
        ModeUI_WaitClickBlinking(SLIP_LED_STOPPED);
        return false;
    }
    if (FailSafe_IsTripped()) FailSafe_Halt();
    static int s_last_place = 0; // 前の 1 本の置く所(0 はまだ走っていない)
    int place = KindPlaceNo(kind);
    bool changed = (place != s_last_place);
    s_last_place = place;
    printf("#%lu %s v %.0f: %splace at %s (place %d), then hand over front-left sensor to RUN\r\n", (unsigned long)run,
           KindName(kind), v, changed ? "NEW PLACE! " : "", KindPlace(kind), place);
    WaitHandShowingPlace(place, changed);
    HAL_Delay(SLIP_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
    SetupLogger(kind, v);
    BeginAtStart();
    Logger_Start();
    StopMeasure m;
    bool ok = RunOnce(kind, v, &m);
    Logger_Stop();
    EndControl(); // モーターを止めてから保存する(FW2 と同じ)
    if (ok) {
        printf("#%lu %s v %.0f: stop %.1f enc %.1f | FL %.1f FR %.1f FL+FR %.1f (sd %.1f) FL-FR %+.1f | "
               "L %.1f R %.1f\r\n",
               (unsigned long)run, KindName(kind), v, m.stop_mm, m.dist_mm, m.fl, m.fr, m.fl + m.fr, m.sum_sd,
               m.fl - m.fr, m.l, m.r);
    }
    if (ModeUI_SaveLogToSD() == SD_SAVE_FAILED) printf("SD save FAILED\r\n");
    if (!ok) {
        printf("SLIP stopped at #%lu (run)\r\n", (unsigned long)run);
        ModeUI_WaitClickBlinking(SLIP_LED_STOPPED);
    }
    return ok;
}

void SlipTurnTest_Run(void) {
    static const float kSix[] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f };
    int set = (int)ModeUI_SelectValue("SLIP(1 TURN 2 STRAIGHT 3 PIVOT 4 LR 5 LR REST 6 REF NG)", "", kSix, 6u, 1.0f);
    printf("walls: south of S, east of (2,1), north of (1,3). keep open: east of (0,1), east and north of (1,1), "
           "(1,2)-(1,3). place by hand before every run\r\n");
    uint32_t run = 0;

    if (set == 6) {
        // 最短走行の ADJ CHECK の比べ相手(2026-11-07): ゴール (2,2) の列の一番下 (2,0) から、まっすぐ北へ (2,2) の北の端まで。
        // ADJ CHECK と同じ迷路のまま走る(迷路は ADJ CHECK の迷路で、(2,0)〜(2,2) がまっすぐ空いていること)
        printf("SLIP REF NG: straight north from (2,0) to the north end of (2,2), v %.0f, %d runs\r\n", SLIP_LR_V,
               SLIP_RUNS_EACH);
        for (uint32_t k = 0; k < (uint32_t)SLIP_RUNS_EACH; k++) {
            if (!RunOneByHand(++run, SLIP_NG, SLIP_LR_V)) return;
        }
    } else if (set == 5) {
        // LR の残り(2026-11-07): 1 回目の LR は 19 本で止まった(S の 12 本と、(2,0) の T1L・P1L・T2L が 2 本、P2L が 1 本)。
        // 各 3 本になるように (2,0) の足りない分を走り、続けて E・EW・N を 3 本ずつ走る(同じ迷路のまま)
        static const int kRest[] = { SLIP_T1L, SLIP_P1L, SLIP_T2L, SLIP_P2L, SLIP_P2L,
                                     SLIP_E, SLIP_E, SLIP_E, SLIP_EW, SLIP_EW, SLIP_EW, SLIP_N, SLIP_N, SLIP_N };
        const uint32_t n_rest = (uint32_t)(sizeof(kRest) / sizeof(kRest[0]));
        s_raw_slalom = true;
        printf("SLIP LR REST: (2,0) T1L P1L T2L P2L P2L, then E x3, EW x3, N x3, v %.0f\r\n", SLIP_LR_V);
        printf("walls: also keep open (2,0)-(2,1), (2,1)-(1,1), (1,1)-(0,1)\r\n");
        for (uint32_t j = 0; j < n_rest; j++) {
            if (!RunOneByHand(++run, kRest[j], SLIP_LR_V)) return;
        }
    } else if (set == 4) {
        // 左右と回る中心のずれ: 右の組・左の組の小回りと超信地旋回、同じ止まる所のまっすぐの走りを、同じ回の中で順に走る。
        // 小回りは手で足す調整を引いた形で走る(調整の入っていない小回りと超信地旋回を、回る中心のずれの式で比べるため)。
        // 置き直しを減らすため、置く所ごとにまとめる(2026-11-07): S で T1 P1 T2 P2 を SLIP_RUNS_EACH 回 →
        // (2,0) で T1L P1L T2L P2L を同じように → E・EW・N を SLIP_RUNS_EACH 本ずつ。比べる組(T1 と P1 など)は同じ所で交互に走る
        static const int kGroups[5][4] = {
            { SLIP_T1, SLIP_P1, SLIP_T2, SLIP_P2 },
            { SLIP_T1L, SLIP_P1L, SLIP_T2L, SLIP_P2L },
            { SLIP_E, 0, 0, 0 },
            { SLIP_EW, 0, 0, 0 },
            { SLIP_N, 0, 0, 0 },
        };
        s_raw_slalom = true;
        printf("SLIP LR: right / left, slalom (no manual adjust) and pivot, with straight E / EW / N, v %.0f, "
               "grouped by place\r\n", SLIP_LR_V);
        printf("walls: also keep open (2,0)-(2,1), (2,1)-(1,1), (1,1)-(0,1)\r\n");
        for (uint32_t g = 0; g < 5u; g++) {
            for (uint32_t k = 0; k < (uint32_t)SLIP_RUNS_EACH; k++) {
                for (uint32_t j = 0; j < 4u && kGroups[g][j] != 0; j++) {
                    if (!RunOneByHand(++run, kGroups[g][j], SLIP_LR_V)) return;
                }
            }
        }
    } else if (set == 3) {
        // 超信地旋回: 小回りの走り(T1・T2)と、その 1 回目を超信地旋回にした走り(P1・P2)を同じ回の中で交互に走る。
        // どれも S に同じように置くので、P1 − T1 で超信地旋回の前後のずれ、P2 − T2 で横のずれが出る(小回りのずれは調整済み)
        const uint32_t n_speeds = (uint32_t)(sizeof(kPivotSpeeds) / sizeof(kPivotSpeeds[0]));
        printf("SLIP PIVOT: T1, P1, T2, P2 in turn, speeds 300 / 500, %d runs each\r\n", SLIP_RUNS_EACH);
        static const int kKinds[] = { SLIP_T1, SLIP_P1, SLIP_T2, SLIP_P2 };
        for (uint32_t i = 0; i < n_speeds; i++) {
            for (uint32_t k = 0; k < (uint32_t)SLIP_RUNS_EACH; k++) {
                for (uint32_t j = 0; j < 4u; j++) {
                    if (!RunOneByHand(++run, kKinds[j], kPivotSpeeds[i])) return;
                }
            }
        }
    } else if (set == 2) {
        // まっすぐの走り: T1・T2 と同じ止まる所の壁を、曲がらずに読む(曲がった走りとの差で、曲がって生まれたずれを出す)。
        // 置く所が同じものをまとめる: E を速さごとに SLIP_RUNS_EACH 本、続けて N を同じように
        const uint32_t n_speeds = (uint32_t)(sizeof(kStraightSpeeds) / sizeof(kStraightSpeeds[0]));
        printf("SLIP STRAIGHT: E then N, speeds 300 / 500, %d runs each\r\n", SLIP_RUNS_EACH);
        static const int kKinds[] = { SLIP_E, SLIP_N };
        for (uint32_t j = 0; j < 2u; j++) {
            for (uint32_t i = 0; i < n_speeds; i++) {
                for (uint32_t k = 0; k < (uint32_t)SLIP_RUNS_EACH; k++) {
                    if (!RunOneByHand(++run, kKinds[j], kStraightSpeeds[i])) return;
                }
            }
        }
    } else {
        const uint32_t n_speeds = (uint32_t)(sizeof(kSlipSpeeds) / sizeof(kSlipSpeeds[0]));
        printf("SLIP TURN: T1 (right) / T2 (right, left), speeds 300-600, %d runs each\r\n", SLIP_RUNS_EACH);
        for (uint32_t i = 0; i < n_speeds; i++) {
            TurnShape s = ShapeFor(kSlipSpeeds[i]);
            printf("v %.0f: omega %.0f alpha %.0f pre %.1f post %.1f\r\n", kSlipSpeeds[i], s.omega_dps, s.alpha_dps2,
                   s.pre_mm, s.post_mm);
        }
        for (uint32_t i = 0; i < n_speeds; i++) {
            for (uint32_t k = 0; k < 2u * (uint32_t)SLIP_RUNS_EACH; k++) {
                int kind = (k % 2u == 0u) ? SLIP_T1 : SLIP_T2; // T1 と T2 を交互に(電池の変わり方を両方にそろえる)
                if (!RunOneByHand(++run, kind, kSlipSpeeds[i])) return;
            }
        }
    }
    printf("SLIP done (%lu runs)\r\n", (unsigned long)run);
}
