#include "app/slip_turn_test.h"

#include <math.h>
#include <stdio.h>
#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "app/log_event.h"
#include "interface/button.h"
#include "interface/sound.h"
#include "logic/control/slalom.h"

// ---- 位置 ----
// 距離は「置いた所(S の後ろの壁に付けた所)からの目標の距離」で数える(探索・FW と同じ約束)。
#define SLIP_B0_MM       (SEARCH_START_FRONT_MM + 0.5f * SECTION_MM) // S と (0,1) の境界
#define SLIP_STOP_MM     (SECTION_MM - SEARCH_WALL_READ_BEFORE_MM)   // 最後の境界から止まる所まで(次の境界の 12mm 手前)

// ---- 走り ----
#define SLIP_RUNS_EACH   3       // 1 つの速さで T1・T2 を何本ずつ走るか(交互に走る)
#define SLIP_ACCEL       3000.0f // 加速度・減速度[mm/s²](探索と同じ)
// CAMERA の加速度・減速度[mm/s²](最短走行の FAST_DECEL_MAX_MM_S2 と同じ)。3000 では大回り 90° の 1200 に
// 曲がり始め(置いた所から 231mm)までに届かない(要る距離 240mm。5000 なら 144mm)
#define SLIP_CAMERA_ACCEL 5000.0f
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
    // ---- 2025 学生クラシック迷路の階段(STAIR。2026-11-07)----
    // 階段の走りは止まらずに、小回りの曲がり始めの直前に前壁を読む(ログの ad_fl・ad_fr)。比べ相手は、同じ前壁にまっすぐ入り、
    // 階段の走りが曲がり始める所で止まって読む(調整を引いた前のオフセットの所)
    SLIP_ST = 13,  // S から: (0,1) 右・(1,1) 右・(1,0) 左 → 東へ 6 → (8,0) 左 → 北へ 5 → (8,6) 右・(9,6) 左・…・(13,11) 右
                   // (階段は 11 回)→ (14,11) の真ん中で止まる(地図の最短走行の道 S → (8,6) に階段をつないだもの)
    SLIP_RE1 = 14, // (8,6) で東向き(西の壁に付ける)→ (9,6) の曲がり始めの所で止まる
    SLIP_RE2 = 15, // (9,7) で東向き → (10,7)
    SLIP_RE3 = 16, // (10,8) で東向き → (11,8)
    SLIP_RE4 = 17, // (11,9) で東向き → (12,9)
    SLIP_RE5 = 18, // (12,10) で東向き → (13,10)
    SLIP_RN1 = 19, // (9,6) で北向き(南の壁に付ける)→ (9,7) の曲がり始めの所で止まる
    SLIP_RN2 = 20, // (12,9) で北向き → (12,10)
    SLIP_RN3 = 21, // (13,10) で北向き → (13,11)
    // ---- 4×4 の大回り 90°(LARGE90。2026-11-08)----
    // 0 列目・3 列目の (x,0)〜(x,2) と 2 段目 (0,2)〜(3,2) を空け、(3,2) の東に壁を置く。止まる所は前壁の 186mm 手前
    SLIP_L90L = 22,  // (3,0) で北向き(南の外周に付ける)→ (3,1) の真ん中から大回り左 → (2,2) の真ん中 → 西へ
                     // → (1,2) の西の端(西の外周の手前)
    SLIP_L90LR = 23, // (3,2) で西向き((3,2) の東の壁に付ける)→ まっすぐ L90L と同じ止まる所
    SLIP_L90R = 24,  // S で北向き → (0,1) の真ん中から大回り右 → (1,2) の真ん中 → 東へ → (2,2) の東の端((3,2) の東の壁の手前)
    SLIP_L90RR = 25, // (0,2) で東向き(西の外周に付ける)→ まっすぐ L90R と同じ止まる所
    // ---- 4×4 の大回り 180°(LARGE180。2026-11-08)----
    // 4 列とも (x,0)〜(x,3) を空け、3 段目の (0,3)〜(1,3) と (2,3)〜(3,3) を空ける(0 列目と 1 列目、2 列目と 3 列目の間の壁は
    // あってもよい)。止まる所は南の外周の 186mm 手前((x,1) の南の端)
    SLIP_L180R = 26,  // S で北向き → (0,2) の真ん中から大回り右 180° → (1,2) の真ん中 → 南へ → (1,1) の南の端
    SLIP_L180RR = 27, // (1,3) で南向き(北の外周に付ける)→ まっすぐ L180R と同じ止まる所
    SLIP_L180L = 28,  // (3,0) で北向き(南の外周に付ける)→ (3,2) の真ん中から大回り左 180° → (2,2) の真ん中 → 南へ → (2,1) の南の端
    SLIP_L180LR = 29, // (2,3) で南向き(北の外周に付ける)→ まっすぐ L180L と同じ止まる所
};
static const float kPivotSpeeds[] = { 300.0f, 500.0f }; // 超信地旋回の走り(T1・P1・T2・P2)の直進・小回りの速さ
#define SLIP_ST_V        500.0f // 階段の速さ(探索の小回りと同じ。走りながら前壁を読む片寄りがほぼ 0 の速さ: FW2 の B 500)
#define SLIP_ST_TURNS    11     // 階段の小回りの数(右から始めて交互)
#define SLIP_ST_RUNS     6      // 階段を何本走るか(調整なし・ありを交互に)
#define SLIP_ST_STOP_MM  (0.5f * SECTION_MM) // 最後の小回りの後、(14,11) の真ん中で止まる(東に壁がある)
#define SLIP_TURN_READ_MM 15.0f  // STAIR: 曲がり始めのこれだけ手前から曲がり始めまで前壁を読む

// STAIR のときだけ 0 以上(小回りごとに前壁を読んで LOG_EV_SLIP_TURN に残す。値は次の小回りの番号)
static int s_turn_log_index = -1;
#define SLIP_ST_REF_RUNS 2      // 比べ相手を 1 か所あたり何本走るか
#define SLIP_LR_V 300.0f // LR の組の速さ(速さによらないずれを見るので 1 つだけ)
// 大回り 90° の速さ(遅い順に 1 段ずつ上げる。壁に当たるなどで止まったら、そこで終わる)。最短走行の目標は 1000(2026-11-08)
static const float kLarge90Speeds[] = { 600.0f, 700.0f, 800.0f, 900.0f, 1000.0f };
#define SLIP_L90_RUNS     3      // 1 つの速さで左・右を何本ずつ走るか(置く所ごとにまとめる)
#define SLIP_L90_REF_V    600.0f // 比べ相手(まっすぐ)の速さ
#define SLIP_L90_REF_RUNS 3      // 比べ相手を 1 か所あたり何本走るか
// 大回り 180° の速さ(LARGE90 と同じく遅い順に 1 段ずつ。本数・比べ相手の速さと本数も LARGE90 と同じ)。最短走行の目標は 1000
// 2026-11-17: 1000 は横加速度 1.15G なので消した(モデルは 1G 未満でだけ扱う)
static const float kLarge180Speeds[] = { 600.0f, 700.0f, 800.0f, 900.0f };

typedef struct {
    float omega_dps, alpha_dps2, pre_mm, post_mm;
} TurnShape;

// true の間は、小回りの手で足す調整(SLALOM_PRE/POST_ADJ_MM)を引いた形で走る(LR の組。回る中心のずれの式を、
// 調整の入っていない小回りと超信地旋回で比べるため)
static bool s_raw_slalom = false;

// 止まる所をこれだけ先へずらす(RULER: 前壁の物差しの傾きを、止まる所を変えて確かめる。ふつうは 0。LOG_EV_SLIP_RUN の e に出す)
static float s_stop_extra_mm = 0.0f;
static const float kRulerExtraMm[] = { -10.0f, 0.0f, 10.0f, 20.0f };
#define SLIP_RULER_ROUNDS 2 // 置く所ごとに kRulerExtraMm を何回まわすか

// CAMERA(2026-11-20): true の間は、止まって保存した後に直結の LED_1〜6 だけを点けて、ボタンが押されるまで待つ(真上から写真を撮る)
static bool s_camera = false;
static float s_accel = SLIP_ACCEL; // 直進の加速度・減速度(CAMERA の間だけ SLIP_CAMERA_ACCEL)
// CAMERA の速さ(50 刻み、横加速度 1G 未満まで。tools/camera/analyze_slip_camera.m の並びと合わせる)。1 つの速さで 1 本ずつ
// (K の誤差は本数の平方根でしか減らないので、同じ速さをくり返すより速さを増やす)。横加速度: 小回り 750 で 0.90G、
// 大回り 90° 1200 で 0.98G、大回り 180° 900 で 0.93G
static const float kCamTurnSpeeds[] = { 300.0f, 350.0f, 400.0f, 450.0f, 500.0f, 550.0f, 600.0f, 650.0f, 700.0f, 750.0f };
static const float kCamLarge90Speeds[] = { 500.0f, 550.0f, 600.0f, 650.0f, 700.0f, 750.0f, 800.0f, 850.0f, 900.0f, 950.0f,
                                           1000.0f, 1050.0f, 1100.0f, 1150.0f, 1200.0f };
static const float kCamLarge180Speeds[] = { 500.0f, 550.0f, 600.0f, 650.0f, 700.0f, 750.0f, 800.0f, 850.0f, 900.0f };

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

// 最短走行と同じ形の大回り 90°(FAST_LARGE90_*)を、速さ v に合わせて作る(角速度は速さに比例、角加速度は速さの2乗に比例)。
// 前後のオフセットは区画の真ん中から曲がり始めまで・曲がり終わりから斜め隣の区画の真ん中まで(run_path と同じ計算)
static TurnShape LargeShapeFor(float v) {
    SlalomParams p = {
        .v_mm_s = FAST_LARGE90_V_MM_S,
        .omega_dps = FAST_LARGE90_OMEGA_DPS,
        .alpha_dps2 = FAST_LARGE90_ALPHA_DPS2,
        .angle_deg = 90.0f,
    };
    Slalom_ScaleToSpeed(&p, v);
    SlalomOffsets o = Slalom_Large90Offsets(&p); // 最短走行と同じ(スリップのモデル + FAST_LARGE90_*_ADJ_MM)
    TurnShape s = { p.omega_dps, p.alpha_dps2, o.pre_mm, o.post_mm };
    return s;
}

// 最短走行と同じ形の大回り 180°(FAST_LARGE180_*。横にちょうど 1 区画移る角速度を求めてから)を、速さ v に合わせて作る。
// 前のオフセットは出発した区画の真ん中から曲がり始めまで、後ろのオフセットは曲がり終わりから隣の列の区画の真ん中まで(run_path と同じ)
static TurnShape Large180ShapeFor(float v) {
    SlalomParams p = {
        .v_mm_s = FAST_LARGE180_V_MM_S,
        .omega_dps = 0.0f,
        .alpha_dps2 = FAST_LARGE180_ALPHA_DPS2,
        .angle_deg = 180.0f,
    };
    Slalom_SolveOmegaForSide(&p, SECTION_MM);
    Slalom_ScaleToSpeed(&p, v);
    SlalomOffsets o = Slalom_Large180Offsets(&p); // 最短走行と同じ(スリップのモデル + FAST_LARGE180_*_ADJ_MM)
    TurnShape s = { p.omega_dps, p.alpha_dps2, o.pre_mm, o.post_mm };
    return s;
}

static bool IsLarge90Turn(int kind) { return kind == SLIP_L90L || kind == SLIP_L90R; }
static bool IsLarge90(int kind) { return kind >= SLIP_L90L && kind <= SLIP_L90RR; }
static bool IsLarge180Turn(int kind) { return kind == SLIP_L180R || kind == SLIP_L180L; }
static bool IsLarge180(int kind) { return kind >= SLIP_L180R && kind <= SLIP_L180LR; }

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
    App_StartStraight(d, v_max, v_end, s_accel);
}

// 境界(*ref_mm)から小回りで曲がり、曲がった先の境界を *ref_mm に入れる(探索の SlalomTurn と同じ数え方)。
// 曲がり始めまでは速さ v のまま進む。曲がった後の直進は呼び出し側が始める
static bool Turn(float *ref_mm, bool right, float v, const TurnShape *s) {
    float start = *ref_mm + s->pre_mm;
    StraightTo(start, v, v);
    if (s_turn_log_index >= 0) {
        // STAIR: 曲がり始めの SLIP_TURN_READ_MM 手前から曲がり始めまで、前壁を読んで平均する(止まらずに読む)
        if (!WaitTargetDistance(start - SLIP_TURN_READ_MM)) return false;
        float s_sum = 0.0f, s_diff = 0.0f, s_before = 0.0f;
        uint32_t n = 0;
        uint32_t t0 = HAL_GetTick();
        while (App_GetTargetDistance() < start) {
            CheckFailSafe();
            s_sum += (float)ad_fl + (float)ad_fr;
            s_diff += (float)ad_fl - (float)ad_fr;
            s_before += start - App_GetTargetDistance();
            n++;
            if (HAL_GetTick() - t0 > SLIP_TIMEOUT_MS) return false;
            HAL_Delay(1); // 1kHz で読む(壁センサーの値は制御の割り込みで 1ms ごとに変わる)
        }
        if (n == 0u) n = 1u;
        Logger_Event(LOG_EV_SLIP_TURN, (float)s_turn_log_index, s_sum / (float)n, s_diff / (float)n,
                     s_before / (float)n, s_raw_slalom ? 0.0f : 1.0f);
        s_turn_log_index++;
    } else if (!WaitTargetDistance(start)) {
        return false;
    }
    App_StartSlalom(right ? -90.0f : 90.0f, s->omega_dps, s->alpha_dps2);
    if (!WaitMotionDone()) return false;
    *ref_mm = App_GetTargetDistance() + s->post_mm;
    return true;
}

// 区画の真ん中(*center_mm)から大回り(angle_deg = 90 は斜め隣、180 は隣の列へ)で曲がり、出た区画の真ん中を *center_mm に入れる。
// 曲がり始めまでは速さ v のまま進む。曲がった後の直進は呼び出し側が始める
static bool LargeTurn(float *center_mm, bool right, float angle_deg, float v, const TurnShape *s) {
    float start = *center_mm + s->pre_mm;
    StraightTo(start, v, v);
    if (!WaitTargetDistance(start)) return false;
    App_StartSlalom(right ? -angle_deg : angle_deg, s->omega_dps, s->alpha_dps2);
    if (!WaitMotionDone()) return false;
    *center_mm = App_GetTargetDistance() + s->post_mm;
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
                  : (kind == SLIP_E || kind == SLIP_EW) ? 300.0f : (kind == SLIP_N || kind == SLIP_NG) ? 480.0f
                  : (kind == SLIP_ST) ? 5000.0f : (kind >= SLIP_RE1 && kind <= SLIP_RN3) ? 200.0f
                  : IsLarge90Turn(kind) ? 900.0f : IsLarge90(kind) ? 550.0f
                  : IsLarge180Turn(kind) ? 1100.0f : IsLarge180(kind) ? 550.0f : 450.0f;
    uint32_t pivot_ms = (kind == SLIP_P1 || kind == SLIP_P2 || kind == SLIP_P1L || kind == SLIP_P2L) ? 2000u : 0u;
    Logger_SetDuration((uint32_t)(path_mm / v * 1000.0f) + pivot_ms + SLIP_SETTLE_MS + SLIP_RECORD_MS + 700u);
}

// 置いた所で制御が有効になっている状態から 1 本走り、止まって前壁を読む(制御は有効のまま)
static bool RunOnce(int kind, float v, StopMeasure *m) {
    const ControlDebug *d = App_GetControlDebug();
    TurnShape s = IsLarge90(kind) ? LargeShapeFor(v) : IsLarge180(kind) ? Large180ShapeFor(v) : ShapeFor(v);
    Logger_Event(LOG_EV_SLIP_RUN, (float)kind, v, s.pre_mm, s.post_mm, s_stop_extra_mm);

    // 右の組(T1・T2・P1・P2)は S から (0,1) で右、左の組(T1L・T2L・P1L・P2L)は (2,0) から (2,1) で左に曲がる。
    // 2 回目(T2・P2 は左、T2L・P2L は右)はどちらも (1,1) で曲がる
    bool left_set = (kind == SLIP_T1L || kind == SLIP_T2L || kind == SLIP_P1L || kind == SLIP_P2L);
    bool second = (kind == SLIP_T2 || kind == SLIP_P2 || kind == SLIP_T2L || kind == SLIP_P2L);
    float ref = SLIP_B0_MM; // 置いた区画と次の区画の境界
    float stop_after = SLIP_STOP_MM; // 最後の境界から止まる所まで(ふつうは次の境界の 12mm 手前)
    if (IsLarge90(kind)) {
        // 大回り 90°: 置いた区画の真ん中は SEARCH_START_FRONT_MM 先。曲がる走りはその次の区画の真ん中から曲がり、斜め隣の区画の
        // 真ん中へ出る。比べ相手は置いた区画の次の区画の真ん中を通る。そこから半区画先の境界の SLIP_STOP_MM 先で止まる(前壁の 186mm 手前)
        float center = SEARCH_START_FRONT_MM + SECTION_MM; // (0,1)・(3,1)、比べ相手は (1,2)・(2,2) の真ん中
        if (IsLarge90Turn(kind) && !LargeTurn(&center, kind == SLIP_L90R, 90.0f, v, &s)) return false; // → (1,2)・(2,2) の真ん中
        ref = center + 0.5f * SECTION_MM; // 止まる所の手前の境界
    } else if (IsLarge180(kind)) {
        // 大回り 180°: 曲がる走りは置いた区画から 2 区画先((0,2)・(3,2))の真ん中から曲がり、隣の列の同じ段((1,2)・(2,2))の真ん中へ
        // 南向きに出る。比べ相手は (1,3)・(2,3) に置き、次の区画 (1,2)・(2,2) の真ん中を通る。そこから LARGE90 と同じく止まる
        float center = SEARCH_START_FRONT_MM + SECTION_MM;
        if (IsLarge180Turn(kind)) {
            center += SECTION_MM;
            if (!LargeTurn(&center, kind == SLIP_L180R, 180.0f, v, &s)) return false;
        }
        ref = center + 0.5f * SECTION_MM;
    } else if (kind == SLIP_ST) {
        // S から階段の入口まで(search_0092 の地図の最短走行の道): (0,1) 右・(1,1) 右・(1,0) 左 → (2,0)〜(7,0) を東へ →
        // (8,0) 左 → (8,1)〜(8,5) を北へ。ここまでの小回りも前壁を読む(番号 0〜3)
        s_turn_log_index = 0;
        bool ok = Turn(&ref, true, v, &s) && Turn(&ref, true, v, &s) && Turn(&ref, false, v, &s);
        if (ok) {
            ref += 6.0f * SECTION_MM;
            ok = Turn(&ref, false, v, &s);
        }
        if (ok) ref += 5.0f * SECTION_MM;
        // 階段: (8,6) 右・(9,6) 左・…・(13,11) 右(番号 4〜14)
        for (int k = 0; ok && k < SLIP_ST_TURNS; k++) {
            ok = Turn(&ref, (k % 2) == 0, v, &s);
        }
        s_turn_log_index = -1;
        if (!ok) return false;
        stop_after = SLIP_ST_STOP_MM;
    } else if (kind >= SLIP_RE1 && kind <= SLIP_RN3) {
        stop_after = s.pre_mm; // 隣の区画で、階段の走りが曲がり始める所
    } else if (kind == SLIP_T1 || kind == SLIP_T2 || kind == SLIP_T1L || kind == SLIP_T2L) {
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
    m->stop_mm = ref + stop_after + s_stop_extra_mm;
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
        case SLIP_ST: return "ST";
        case SLIP_RE1: return "RE1";
        case SLIP_RE2: return "RE2";
        case SLIP_RE3: return "RE3";
        case SLIP_RE4: return "RE4";
        case SLIP_RE5: return "RE5";
        case SLIP_RN1: return "RN1";
        case SLIP_RN2: return "RN2";
        case SLIP_RN3: return "RN3";
        case SLIP_L90L: return "L90L";
        case SLIP_L90LR: return "L90LR";
        case SLIP_L90R: return "L90R";
        case SLIP_L90RR: return "L90RR";
        case SLIP_L180R: return "L180R";
        case SLIP_L180RR: return "L180RR";
        case SLIP_L180L: return "L180L";
        case SLIP_L180LR: return "L180LR";
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
        case SLIP_ST: return 1;
        case SLIP_L90R: return 1;  // S
        case SLIP_L90L: return 2;  // (3,0) 北向き
        case SLIP_L90RR: return 3; // (0,2) 東向き
        case SLIP_L90LR: return 4; // (3,2) 西向き
        case SLIP_L180R: return 1;   // S
        case SLIP_L180L: return 2;   // (3,0) 北向き
        case SLIP_L180RR: return 3;  // (1,3) 南向き
        case SLIP_L180LR: return 4;  // (2,3) 南向き
        default:
            // 階段の比べ相手は 8 か所あるので、LED の番号は 2〜5 を順に使う(置く所が変わったかは置く所の文字で見る)
            if (kind >= SLIP_RE1 && kind <= SLIP_RN3) return 2 + (kind - SLIP_RE1) % 4;
            return 1;
    }
}

// 手かざしを待つ。待つ間、シフトレジスタの LED を置く所の番号 × 3 個つける(1 番 3 個 … 5 番 15 個)。
// 置く所が前の 1 本から変わったときは、それと直結の LED 全部を 0.15 秒ごとに点滅させる(置き直しを見落とさないため)。
// 手をかざしたら消して、あとは ModeUI_WaitHandStart と同じ(かざし続けて離すまで待つ)
// CAMERA のときは置いた所も撮るので、シフトレジスタの LED は消し、直結の LED_1〜6 を点けたままにする
// (置く所が変わったときだけ、初めの 2 秒は点滅させる)。手をかざす前に撮る
static void WaitHandShowingPlace(int place, bool changed) {
    uint16_t bar = s_camera ? 0u : (uint16_t)((1u << (3u * (uint32_t)place)) - 1u);
    uint32_t t0 = HAL_GetTick();
    int shown = -1;
    while (ad_fl <= SENSOR_START_THRESHOLD) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        bool blink = changed && (!s_camera || HAL_GetTick() - t0 < 2000u);
        int on = (!blink || ((HAL_GetTick() - t0) / 150u) % 2u == 0u) ? 1 : 0;
        if (on != shown) { // 変わったときだけ書く(シフトレジスタを毎回送らない)
            LED_SetShiftPattern(on ? bar : 0u);
            if (changed || s_camera) LED_SetDirectPattern(on ? LED_DIRECT_ALL : 0u);
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
        case SLIP_ST: return "S (0,0) facing north, tail on the south wall (2025 students classic)";
        case SLIP_RE1: return "(8,6) facing east, tail on the west wall";
        case SLIP_RE2: return "(9,7) facing east, tail on the west wall";
        case SLIP_RE3: return "(10,8) facing east, tail on the west wall";
        case SLIP_RE4: return "(11,9) facing east, tail on the west wall";
        case SLIP_RE5: return "(12,10) facing east, tail on the west wall";
        case SLIP_RN1: return "(9,6) facing north, tail on the south wall";
        case SLIP_RN2: return "(12,9) facing north, tail on the south wall";
        case SLIP_RN3: return "(13,10) facing north, tail on the south wall";
        case SLIP_L90L: return "(3,0) facing north, tail on the south wall";
        case SLIP_L90LR: return "(3,2) facing west, tail on the east wall of (3,2)";
        case SLIP_L90R: return "S (0,0) facing north, tail on the south wall";
        case SLIP_L90RR: return "(0,2) facing east, tail on the west wall";
        case SLIP_L180R: return "S (0,0) facing north, tail on the south wall";
        case SLIP_L180RR: return "(1,3) facing south, tail on the north wall";
        case SLIP_L180L: return "(3,0) facing north, tail on the south wall";
        case SLIP_L180LR: return "(2,3) facing south, tail on the north wall";
        default: return "S (0,0) facing north, tail on the south wall";
    }
}

// ---- 続きから走る(2026-11-17)----
// セットを選んだ後に「何本目から走るか」を 2 回に分けて選ぶ(FROM x10 = 十の位 0〜9、FROM x1 = 一の位 0〜9。
// 回さずに 2 回押すと 00 = 最初から)。それより前の本数は走らずに飛ばす(番号と置く所の並びは最初から走ったときと同じ)。
// 途中で止まったら(電池・走りの失敗)、止まった本の番号を、起動音の電圧と同じ音階で 2 音(十の位・一の位。0 = ド、1 = レ、…)
// 3 秒ごとに鳴らし、直結の LED を点滅させてボタンを待つ。その本はまだ取れていないので、次は同じ番号を選ぶ
// (値を選ぶときも、番号 n の音は n の数字の音なので、同じ音になるまで回せばよい)。
static uint32_t s_from_run = 1u;

static void SelectFromRun(void) {
    static const float kDigits[] = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f };
    uint32_t tens = (uint32_t)ModeUI_SelectValue("FROM x10", "", kDigits, 10u, 0.0f);
    uint32_t ones = (uint32_t)ModeUI_SelectValue("FROM x1", "", kDigits, 10u, 0.0f);
    s_from_run = 10u * tens + ones;
    if (s_from_run < 1u) s_from_run = 1u;
    printf("start from #%lu\r\n", (unsigned long)s_from_run);
}

// 止まった本の番号を音と LED で知らせて、ボタンが押されるまで待つ
static void WaitClickTellingRun(uint32_t run) {
    static SoundNote seq[4]; // 鳴らしている間も読まれるので static
    static const uint16_t kScale[] = { NOTE_C7, NOTE_D7, NOTE_E7, NOTE_F7, NOTE_G7, NOTE_A7, NOTE_B7,
                                       NOTE_C8, NOTE_D8, NOTE_E8 };
    uint32_t n = (run > 99u) ? 99u : run;
    seq[0] = (SoundNote){ kScale[n / 10u], 200u };
    seq[1] = (SoundNote){ NOTE_REST, 100u };
    seq[2] = (SoundNote){ kScale[n % 10u], 200u };
    seq[3] = (SoundNote){ 0u, 0u };
    printf("next time choose FROM x10 = %lu, FROM x1 = %lu\r\n", (unsigned long)(n / 10u), (unsigned long)(n % 10u));
    uint32_t t0 = HAL_GetTick(), last_sound = t0 - 3000u;
    while (!Button_IsPressed()) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        uint32_t now = HAL_GetTick();
        if (now - last_sound >= 3000u) {
            Sound_Play(seq);
            last_sound = now;
        }
        LED_SetDirectPattern((((now - t0) / 100u) % 2u == 0u) ? SLIP_LED_STOPPED : 0u);
        HAL_Delay(1);
    }
    LED_SetDirectPattern(0u);
    while (Button_IsPressed()) HAL_Delay(10);
    HAL_Delay(20);
}

// CAMERA: 止まった機体の直結の LED_1〜6 だけを点け(カメラの校正の CAMERA_LED と同じ)、写真を撮ってボタンが押されるまで待つ
static void WaitPhoto(uint32_t run) {
    printf("#%lu: take a photo from above, then click\r\n", (unsigned long)run);
    LED_SetShiftPattern(0u);
    LED_SetDirectPattern(LED_DIRECT_ALL);
    while (!Button_IsPressed()) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        HAL_Delay(1);
    }
    LED_SetDirectPattern(0u);
    while (Button_IsPressed()) HAL_Delay(10);
    HAL_Delay(20);
}

// 1 本走る: 手で置いて手をかざすのを待ち、走って止まり、前壁を読んで保存する。続けてよければ true
// (1 本ごとに手で置き直す。自動で戻って押し当てると、フェイルセーフが働いたり、向きと横の位置がそろわなかったりしたので、
// やめた。2026-11-06)。run が選んだ始めの番号より前なら走らずに true
static bool RunOneByHand(uint32_t run, int kind, float v) {
    if (run < s_from_run) {
        printf("#%lu %s v %.0f: skipped\r\n", (unsigned long)run, KindName(kind), v);
        return true;
    }
    float vbat = FailSafe_GetFilteredVoltage();
    if (vbat < LONG_LOG_MIN_VBAT_V) {
        printf("SLIP stopped before #%lu: battery %.2f V\r\n", (unsigned long)run, vbat);
        WaitClickTellingRun(run);
        return false;
    }
    if (FailSafe_IsTripped()) FailSafe_Halt();
    static const char *s_last_place = NULL; // 前の 1 本の置く所(NULL はまだ走っていない)
    int place = KindPlaceNo(kind);
    bool changed = (KindPlace(kind) != s_last_place);
    s_last_place = KindPlace(kind);
    printf("#%lu %s v %.0f: %splace at %s (place %d), then hand over front-left sensor to RUN\r\n", (unsigned long)run,
           KindName(kind), v, changed ? "NEW PLACE! " : "", KindPlace(kind), place);
    if (s_camera) printf("#%lu: take a photo of the start from above BEFORE the hand\r\n", (unsigned long)run);
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
    if (ok && s_camera) WaitPhoto(run);
    if (!ok) {
        printf("SLIP stopped at #%lu (run)\r\n", (unsigned long)run);
        WaitClickTellingRun(run);
    }
    return ok;
}

void SlipTurnTest_Run(void) {
    static const float kSets[] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f,
                                   9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f };
    int set = (int)ModeUI_SelectValue("SLIP(1 TURN 2 STRAIGHT 3 PIVOT 4 LR 5 LR REST 6 REF NG 7 CHECK REST 8 STAIR REF "
                                      "9 LARGE90 10 LARGE90 REF 11 LARGE CHECK 12 LARGE180 13 LARGE180 REF "
                                      "14 CAMERA 15 RULER)", "", kSets, 15u, 1.0f);
    SelectFromRun();
    uint32_t run = 0;
    if (set <= 6) {
        printf("walls: south of S, east of (2,1), north of (1,3). keep open: east of (0,1), east and north of (1,1), "
               "(1,2)-(1,3). place by hand before every run\r\n");
    }

    if (set == 9) {
        // 4×4 の大回り 90°(2026-11-08): 遅い順に、速さごとに右(L90R、S)を SLIP_L90_RUNS 本 → 左(L90L、(3,0))を同じだけ。
        // 置く所ごとにまとめる。途中で止まったら(壁に当たってフェイルセーフなど)そこで終わる
        const uint32_t n_speeds = (uint32_t)(sizeof(kLarge90Speeds) / sizeof(kLarge90Speeds[0]));
        printf("SLIP LARGE90 (4x4): speeds 600-1000, R from S x%d then L from (3,0) x%d at each speed\r\n",
               SLIP_L90_RUNS, SLIP_L90_RUNS);
        for (uint32_t i = 0; i < n_speeds; i++) {
            TurnShape s = LargeShapeFor(kLarge90Speeds[i]);
            printf("v %.0f: omega %.0f alpha %.0f pre %.1f post %.1f\r\n", kLarge90Speeds[i], s.omega_dps, s.alpha_dps2,
                   s.pre_mm, s.post_mm);
        }
        static const int kSides[] = { SLIP_L90R, SLIP_L90L };
        for (uint32_t i = 0; i < n_speeds; i++) {
            for (uint32_t j = 0; j < 2u; j++) {
                for (uint32_t k = 0; k < (uint32_t)SLIP_L90_RUNS; k++) {
                    if (!RunOneByHand(++run, kSides[j], kLarge90Speeds[i])) return;
                }
            }
        }
    } else if (set == 12) {
        // 4×4 の大回り 180°(2026-11-08): LARGE90 と同じく、遅い順に速さごとに右(L180R、S)→ 左(L180L、(3,0))を
        // SLIP_L90_RUNS 本ずつ。途中で止まったらそこで終わる
        const uint32_t n_speeds = (uint32_t)(sizeof(kLarge180Speeds) / sizeof(kLarge180Speeds[0]));
        printf("SLIP LARGE180 (4x4): speeds 600-1000, R from S x%d then L from (3,0) x%d at each speed\r\n",
               SLIP_L90_RUNS, SLIP_L90_RUNS);
        for (uint32_t i = 0; i < n_speeds; i++) {
            TurnShape s = Large180ShapeFor(kLarge180Speeds[i]);
            printf("v %.0f: omega %.0f alpha %.0f pre %.1f post %.1f\r\n", kLarge180Speeds[i], s.omega_dps, s.alpha_dps2,
                   s.pre_mm, s.post_mm);
        }
        static const int kSides[] = { SLIP_L180R, SLIP_L180L };
        for (uint32_t i = 0; i < n_speeds; i++) {
            for (uint32_t j = 0; j < 2u; j++) {
                for (uint32_t k = 0; k < (uint32_t)SLIP_L90_RUNS; k++) {
                    if (!RunOneByHand(++run, kSides[j], kLarge180Speeds[i])) return;
                }
            }
        }
    } else if (set == 15) {
        // 前壁の物差しの傾き(2026-11-08): LARGE180 の比べ相手と同じ置き方で、止まる所を kRulerExtraMm だけずらして読む。
        // 調整(後ろ +19mm)で止まる所を 19mm 先にしたのに、読みは 12〜17mm しか動かなかったので、この壁での傾きを直接測る
        const uint32_t n_extra = (uint32_t)(sizeof(kRulerExtraMm) / sizeof(kRulerExtraMm[0]));
        printf("SLIP RULER (4x4): L180RR then L180LR, stop shifted by -10 / 0 / +10 / +20 mm, %d rounds, v %.0f\r\n",
               SLIP_RULER_ROUNDS, SLIP_L90_REF_V);
        static const int kRefs[] = { SLIP_L180RR, SLIP_L180LR };
        for (uint32_t j = 0; j < 2u; j++) {
            for (uint32_t k = 0; k < (uint32_t)SLIP_RULER_ROUNDS * n_extra; k++) {
                s_stop_extra_mm = kRulerExtraMm[k % n_extra];
                printf("stop shift %+.0f mm\r\n", s_stop_extra_mm);
                bool ok = RunOneByHand(++run, kRefs[j], SLIP_L90_REF_V);
                s_stop_extra_mm = 0.0f;
                if (!ok) return;
            }
        }
    } else if (set == 14) {
        // CAMERA(2026-11-20。LARGE180 REST は取り終えたので置き換えた): スリップのモデルを切って走り(App_SetSlipModel)、
        // 止まった所を真上から撮って K を出す。機体は滑りを知らずに走るので、止まった所のずれがそのまま滑りの分になる。
        // 1 TURN(小回り 90°: T1・T1L を 300〜750)/ 2 LARGE90(L90R・L90L を 500〜1200)/ 3 LARGE180(L180R・L180L を 500〜900)。
        // 50 刻みの速さを 1 本ずつ。置く所ごとにまとめ、右を遅い順に全部 → 左を遅い順に全部(2026-11-20 に、速さごとに 3 本から変えた)
        static const float kTypes[] = { 1.0f, 2.0f, 3.0f };
        int type = (int)ModeUI_SelectValue("CAMERA(1 TURN 2 LARGE90 3 LARGE180)", "", kTypes, 3u, 1.0f);
        const float *speeds = (type == 2) ? kCamLarge90Speeds : (type == 3) ? kCamLarge180Speeds : kCamTurnSpeeds;
        uint32_t n_speeds = (type == 2) ? (uint32_t)(sizeof(kCamLarge90Speeds) / sizeof(kCamLarge90Speeds[0]))
                          : (type == 3) ? (uint32_t)(sizeof(kCamLarge180Speeds) / sizeof(kCamLarge180Speeds[0]))
                          : (uint32_t)(sizeof(kCamTurnSpeeds) / sizeof(kCamTurnSpeeds[0]));
        int kinds[2] = { SLIP_T1, SLIP_T1L };
        if (type == 2) { kinds[0] = SLIP_L90R; kinds[1] = SLIP_L90L; }
        if (type == 3) { kinds[0] = SLIP_L180R; kinds[1] = SLIP_L180L; }
        printf("SLIP CAMERA %s: slip model OFF, %s at %.0f-%.0f then %s, one run per speed, photos at start and stop\r\n",
               (type == 2) ? "LARGE90" : (type == 3) ? "LARGE180" : "TURN", KindName(kinds[0]), speeds[0],
               speeds[n_speeds - 1u], KindName(kinds[1]));
        if (type == 1) printf("walls: also keep open (2,0)-(2,1), (2,1)-(1,1), (1,1)-(0,1)\r\n");
        App_SetSlipModel(false);
        s_camera = true;
        s_accel = SLIP_CAMERA_ACCEL;
        bool ok = true;
        for (uint32_t j = 0; ok && j < 2u; j++) {
            for (uint32_t i = 0; ok && i < n_speeds; i++) {
                ok = RunOneByHand(++run, kinds[j], speeds[i]);
            }
        }
        s_camera = false;
        s_accel = SLIP_ACCEL;
        App_SetSlipModel(true);
        if (!ok) return;
    } else if (set == 13) {
        // 大回り 180° の比べ相手: 曲がった後の列の一番上に南向きに置き、まっすぐ同じ所で止まる
        printf("SLIP LARGE180 REF (4x4): L180RR x%d then L180LR x%d, v %.0f\r\n", SLIP_L90_REF_RUNS,
               SLIP_L90_REF_RUNS, SLIP_L90_REF_V);
        static const int kRefs[] = { SLIP_L180RR, SLIP_L180LR };
        for (uint32_t j = 0; j < 2u; j++) {
            for (uint32_t k = 0; k < (uint32_t)SLIP_L90_REF_RUNS; k++) {
                if (!RunOneByHand(++run, kRefs[j], SLIP_L90_REF_V)) return;
            }
        }
    } else if (set == 11) {
        // LARGE CHECK(2026-11-08。前の LARGE90 REST は取り終えたので置き換えた): スリップのモデル(SLALOM_SLIP_K)を大回りに
        // 入れた後の確かめ。600 と 1000 で、S から L90R・L180R、(3,0) から L90L・L180L を SLIP_L90_RUNS 本ずつ。
        // 迷路は LARGE90 と LARGE180 の両方が走れる形(4 列とも (x,0)〜(x,3) と 2 段目 (0,2)〜(3,2) と (0,3)〜(1,3)・(2,3)〜(3,3) を空け、
        // (3,2) の東と 3 段目の北に壁)。比べ相手は前に取った LARGE90 REF・LARGE180 REF(RULER)を使う
        // 2026-11-17: 速い方は 1G 未満にした(大回り 90° は 1000 のまま 0.68G、大回り 180° は 1000 → 900 で 0.93G)
        static const float kCheckSpeeds[2][2] = { { 600.0f, 600.0f }, { 1000.0f, 900.0f } }; // [遅い・速い][L90・L180]
        static const int kChecks[2][2] = { { SLIP_L90R, SLIP_L180R }, { SLIP_L90L, SLIP_L180L } };
        printf("SLIP LARGE CHECK (4x4): v 600 / fast (L90 1000, L180 900), from S: L90R L180R, from (3,0): L90L L180L, x%d each\r\n",
               SLIP_L90_RUNS);
        for (uint32_t i = 0; i < 2u; i++) {
            for (uint32_t side = 0; side < 2u; side++) {
                for (uint32_t j = 0; j < 2u; j++) {
                    for (uint32_t k = 0; k < (uint32_t)SLIP_L90_RUNS; k++) {
                        if (!RunOneByHand(++run, kChecks[side][j], kCheckSpeeds[i][j])) return;
                    }
                }
            }
        }
    } else if (set == 10) {
        // 大回り 90° の比べ相手: 曲がった後の道に、曲がらずにまっすぐ入って同じ所で止まる
        printf("SLIP LARGE90 REF (4x4): L90RR x%d then L90LR x%d, v %.0f\r\n", SLIP_L90_REF_RUNS,
               SLIP_L90_REF_RUNS, SLIP_L90_REF_V);
        static const int kRefs[] = { SLIP_L90RR, SLIP_L90LR };
        for (uint32_t j = 0; j < 2u; j++) {
            for (uint32_t k = 0; k < (uint32_t)SLIP_L90_REF_RUNS; k++) {
                if (!RunOneByHand(++run, kRefs[j], SLIP_L90_REF_V)) return;
            }
        }
    } else if (set == 7) {
        // LARGE CHECK の残り(2026-11-08。前の STAIR と置き換えた。迷路が変わって STAIR の道は使えない): 1 回目は 600 の右
        // (L90R・L180R)まで取れた。600 の左((3,0) から L90L・L180L)→ 1000 の右(S)→ 1000 の左を SLIP_L90_RUNS 本ずつ
        static const struct { int kind; float v; } kRest[] = {
            { SLIP_L90L, 600.0f }, { SLIP_L180L, 600.0f },
            { SLIP_L90R, 1000.0f }, { SLIP_L180R, 900.0f }, // 大回り 180° は 1G 未満の 900 にした(2026-11-17)
            { SLIP_L90L, 1000.0f }, { SLIP_L180L, 900.0f },
        };
        printf("SLIP LARGE CHECK REST (4x4): 600 L90L L180L from (3,0), L90R 1000 L180R 900 from S, L90L 1000 L180L 900, x%d each\r\n",
               SLIP_L90_RUNS);
        for (uint32_t j = 0; j < (uint32_t)(sizeof(kRest) / sizeof(kRest[0])); j++) {
            for (uint32_t k = 0; k < (uint32_t)SLIP_L90_RUNS; k++) {
                if (!RunOneByHand(++run, kRest[j].kind, kRest[j].v)) return;
            }
        }
    } else if (set == 8) {
        // 階段の比べ相手: 同じ前壁へまっすぐ入り、階段の走りが曲がり始める所(調整なしの前のオフセット)で止まって読む
        static const int kRefs[] = { SLIP_RE1, SLIP_RE2, SLIP_RE3, SLIP_RE4, SLIP_RE5, SLIP_RN1, SLIP_RN2, SLIP_RN3 };
        s_raw_slalom = true;
        printf("SLIP STAIR REF (2025 students classic): 8 places, %d runs each, v %.0f\r\n", SLIP_ST_REF_RUNS, SLIP_ST_V);
        for (uint32_t j = 0; j < (uint32_t)(sizeof(kRefs) / sizeof(kRefs[0])); j++) {
            for (uint32_t k = 0; k < (uint32_t)SLIP_ST_REF_RUNS; k++) {
                if (!RunOneByHand(++run, kRefs[j], SLIP_ST_V)) return;
            }
        }
    } else if (set == 6) {
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
