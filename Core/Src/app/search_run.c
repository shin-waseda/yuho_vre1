#include "app/search_run.h"

#include <math.h>
#include <string.h>
#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "interface/sdcard.h"
#include "interface/flash.h"
#include "interface/fault_diag.h"
#include "logic/wall_sense.h"
#include "logic/maze/search_planner.h"
#include "logic/maze/maze_print.h"
#include "logic/maze/run_path.h"
#include "logic/control/slalom.h"
#include "app/run_log.h"

// 探索走行。流れは BlueEyes の searchB_dijkstra と同じ:
//   start_sequence(右90° → 尻当て → 左90° → 尻当て)でスタート区画の真ん中に合わせる
//   → スタート区画の壁を地図に書いて経路を計算 → 半区画加速して最初の境界へ
//   → 境界ごとに: 壁を読んで地図を更新し、経路を計算し直し、次の動きをする
//       直進: 止まらずに次の境界まで
//       左右: 真ん中で止まる → 超信地旋回 → 半区画加速
//       180°: 真ん中で止まる → (後ろと横に壁があれば尻当てを2回) → 半区画加速
//   → ゴール・スタートでは真ん中で止まって 180°(尻当てあり)
// 違い: 地図の更新と経路の計算は SearchPlanner_Step() でまとめて行う。BlueEyes は計算の間に
// 13mm(CALC_OFFSET_DIST)等速で進む区間を取っているが、ここでは計算の間に進んだ分を次の動きの
// 距離から差し引く(目標の距離の絶対値で区画の境界・真ん中を決めている)ので、働きは同じ。

// 手を離してから走り出すまでの待ち[ms](この間にジャイロのゼロ点を測り直す)。
#define SEARCH_START_DELAY_MS 1000

#define HALF_SECTION_MM (SECTION_MM * 0.5f)

// ---- 区画ごとの記録 ----
// 1区画で1つ。行き帰りで同じ区画を何度も通るので、区画数の2倍あれば足りる。12バイト × 512。
#define SEARCH_EVENT_MAX (MAZE_CELL_COUNT * 2)

typedef struct {
    MazePos pos;         // Step に渡した時点の区画
    uint8_t heading;     // その時点の向き(Direction)
    uint8_t walls;       // 渡した壁(bit0 前, bit1 右, bit2 左)
    uint8_t action;      // 返ってきた指令(ActionType)
    uint8_t plan_ms;     // Step にかかった時間[ms](255で頭打ち)
    uint16_t l, fl, fr, r; // 壁を読んだときのセンサーの値(センサーで読んでいない区画は0)
} SearchEvent;

static SearchEvent s_events[SEARCH_EVENT_MAX];
static uint16_t s_event_count = 0;

static WallMap s_map;
static SearchPlanner s_planner;

static bool s_log_file = false;   // ログのファイルを開けたか
static bool s_log_failed = false; // 探索の途中でログの保存に失敗したか
static float s_gyro_offset = 0.0f; // 走る前に測り直したジャイロのゼロ点(ログに残す)
static float s_run_kind = 0.0f;    // 走り方の種類(LOG_EV_RUN_TYPE の kind と type)
static float s_run_type = 0.0f;
// 走る前に選んだ速さ(SearchRun_Run / FastRun_Run の最初に ModeUI_SelectValue で選ぶ)
static float s_search_v = SEARCH_V_MM_S; // 探索の直進の最高速度
static float s_search_turn_v = SEARCH_V_MM_S; // 探索の小回り(スラローム)の速さ。境界ではいつもこの速さで走る
static float s_search_accel = SEARCH_ACCEL_MM_S2; // 探索の直進の加速度・減速度(RUN の SEARCH で選ぶ)
// 探索の地図と行き先(RUN の SEARCH で選ぶ。ほかのモードは初期化・往復のまま)
typedef enum { SEARCH_MAP_NEW = 1, SEARCH_MAP_CONTINUE = 2 } SearchMapMode;           // 初期化 / flash の地図に重ねる
typedef enum { SEARCH_SCOPE_ROUND = 1, SEARCH_SCOPE_ONE_WAY = 2, SEARCH_SCOPE_FULL = 3 } SearchScope; // 往復 / 片道 / 全面
static SearchMapMode s_search_map = SEARCH_MAP_NEW;
static SearchScope s_search_scope = SEARCH_SCOPE_ROUND;
static SearchAlgo s_search_algo = SEARCH_ALGO_DIJKSTRA; // 今の探索のアルゴリズム(ログに残す用。RunSearch で入れる)
static float s_fast_v = FAST_V_MM_S;     // 最短走行の直進の最高速度
static float s_fast_small_v = FAST_SMALL_V_MM_S; // 最短走行の小回りの速さ
static float s_fast_accel = FAST_ACCEL_MM_S2;    // 最短走行の直進の加速度
static float s_fast_decel = 0.0f;                // 最短走行の直進の減速度(0 なら加速度と FAST_DECEL_MAX_MM_S2 の小さい方)

// 最短走行の直進の減速度(RunProfile の decel にも同じ値を入れる)
static float FastDecel(void) {
    if (s_fast_decel > 0.0f) return s_fast_decel;
    return (s_fast_accel < FAST_DECEL_MAX_MM_S2) ? s_fast_accel : FAST_DECEL_MAX_MM_S2;
}
static const float kSearchSpeeds[] = SPEED_SELECT_SEARCH_V_MM_S;
static const float kSearchTurnSpeeds[] = SPEED_SELECT_SEARCH_TURN_V_MM_S;
static const float kFastSpeeds[] = SPEED_SELECT_FAST_V_MM_S;
static const float kFastSmallSpeeds[] = SPEED_SELECT_FAST_SMALL_V_MM_S;
static const float kAccels[] = SPEED_SELECT_ACCEL_MM_S2; // 探索・最短走行で選ぶ加速度(RUN の SEARCH・FAST)
static const float kFastAccelForSpeed[] = FAST_ACCEL_FOR_SPEED_MM_S2; // kFastSpeeds と同じ並び
static const float kFastDecelForSpeed[] = FAST_DECEL_FOR_SPEED_MM_S2;
_Static_assert(sizeof(kFastAccelForSpeed) == sizeof(kFastSpeeds), "FAST_ACCEL_FOR_SPEED must match SPEED_SELECT_FAST_V");
_Static_assert(sizeof(kFastDecelForSpeed) == sizeof(kFastSpeeds), "FAST_DECEL_FOR_SPEED must match SPEED_SELECT_FAST_V");

// 最短走行の直進の最高速度 s_fast_v から、加速度・減速度を決める(FAST_RUN・FAST_SWEEP)。
// 表にない速さは、それ以下で一番近い速さの値(一番遅い速さより遅ければ最初の値)
static void SetFastAccelForSpeed(void) {
    uint8_t k = 0;
    for (uint8_t i = 0; i < (uint8_t)(sizeof(kFastSpeeds) / sizeof(kFastSpeeds[0])); i++) {
        if (kFastSpeeds[i] <= s_fast_v) k = i;
    }
    s_fast_accel = kFastAccelForSpeed[k];
    s_fast_decel = kFastDecelForSpeed[k];
}
static const float kLongSearchSpeeds[] = LONG_LOG_SEARCH_V_MM_S; // 長い走行の探索の直進の速さ
// 最短走行の経路の情報(LOG_EV_ROUTE に残す)
static float s_route_cost = 0.0f;
static float s_route_est = 0.0f;
static uint16_t s_route_count = 0;
static float s_route_est_by_type[3] = { 0.0f, 0.0f, 0.0f };

static const char *DirName(Direction d) {
    switch (d) {
        case DIR_NORTH: return "N";
        case DIR_EAST:  return "E";
        case DIR_SOUTH: return "S";
        case DIR_WEST:  return "W";
        default:        return "?";
    }
}

static const char *PhaseName(SearchPhase p) {
    switch (p) {
        case SEARCH_PHASE_TO_GOAL:  return "TO_GOAL";
        case SEARCH_PHASE_TO_START: return "TO_START";
        case SEARCH_PHASE_DONE:     return "DONE";
        case SEARCH_PHASE_FAILED:   return "FAILED";
        default:                    return "?";
    }
}

// ---- ログ ----

// 全部の値 + イベント(app/run_log)。区画ごとの判断や動きはイベント(app/log_event.h)で残す。
static void SetupLogger(const char *file_name) {
    RunLog_Setup("search", file_name, SEARCH_LOG_DECIMATION);
}

static void Ev(LogEventCode code, float a, float b, float c, float d, float e) {
    Logger_Event((uint16_t)code, a, b, c, d, e);
}

static void DelayWatching(uint32_t ms);

// LED を消すときのパターン。ログの保存に失敗していたら、SD の失敗を知らせる LED(左後ろ)を残す。
static uint8_t LedIdlePattern(void) {
    return s_log_failed ? MODE_UI_LED_SD_ERROR : 0x00u;
}

// 直結の LED を全部 SEARCH_LOG_SAVED_LIGHT_MS 点けて消す(SD へ保存できた合図。ModeUI_SaveLogToSD と同じ)。
static void ShowLogSaved(void) {
    LED_SetDirectPattern(LED_DIRECT_ALL);
    DelayWatching(SEARCH_LOG_SAVED_LIGHT_MS);
    LED_SetDirectPattern(LedIdlePattern());
}

// ログを SD へ流す(待ちのループの中で何度も呼ぶ。待たない)。
// 途中で書けなくなったら、SD の失敗を知らせる LED(左後ろ)を点けたままにする(走りは続ける)。
static void PollLog(void) {
    if (!s_log_file) return;
    Logger_StreamPoll();
    if (!s_log_failed && Logger_StreamFailed()) {
        s_log_failed = true;
        LED_SetDirectPattern(LedIdlePattern());
    }
}

// ---- 走行の待ち ----
// フェイルセーフが発動したら止まる(戻らない)。打ち切ったら false。
// どの待ちでも、ログを SD へ流す(PollLog)。

static void CheckFailSafe(void) {
    if (FailSafe_IsTripped()) {
        // 原因をログに残し、ここまでのログを閉じてから止まる(FailSafe_Halt からは戻らない)
        Ev(LOG_EV_FAILSAFE, (float)FailSafe_GetCause(), FailSafe_GetFilteredVoltage(), 0.0f, 0.0f, 0.0f);
        App_ControlLoop_SetEnabled(false);
        if (s_log_file) {
            HAL_Delay(SEARCH_LOG_DECIMATION * 2u); // イベントが記録の行に入るのを待つ
            Logger_StreamEnd(NULL);
            s_log_file = false;
        }
        FailSafe_Halt();
    }
}

// 目標の距離が target_mm に届くまで待つ(区画の境界で壁を読むため、遅れないよう HAL_Delay しない)。
static bool WaitTargetDistance(float target_mm) {
    uint32_t t0 = HAL_GetTick();
    while (App_GetTargetDistance() < target_mm) {
        CheckFailSafe();
        PollLog();
        if (HAL_GetTick() - t0 > SEARCH_STEP_TIMEOUT_MS) return false;
    }
    return true;
}

// ---- 壁切れ補正 ----
// 直進中に新しい壁切れがあれば、その位置から、目標の距離の基準(境界の位置)のずれを求める。
// boundary0 は、この直進で通る境界のうち最初のもの(目標の距離)。壁切れは、どれかの境界を
// WALL_EDGE_POS_L/R_MM 過ぎた所で起きるはずなので、一番近い境界からのずれを補正の量にする。
// 補正したら true を返し、*corr にその量を書く(+ なら機体は思っていたより後ろにいる → 先の基準を先へ)。
typedef struct {
    uint32_t seq;     // 見た壁切れの数(App_GetWallEdge)
    float boundary0;  // 最初の境界の目標の距離(補正するたびに一緒にずらす)
} EdgeCorr;

static void EdgeCorr_Begin(EdgeCorr *ec, float boundary0) {
    ec->seq = App_GetWallEdge(NULL, NULL);
    ec->boundary0 = boundary0;
}

static bool EdgeCorr_Check(EdgeCorr *ec, float *corr) {
    uint8_t side;
    float pos;
    uint32_t seq = App_GetWallEdge(&side, &pos);
    if (seq == ec->seq) return false;
    ec->seq = seq;
    // 壁が切れる位置は左右で違う(side: 0 左, 1 右)
    float edge_mm = (side == 1u) ? WALL_EDGE_POS_R_MM : WALL_EDGE_POS_L_MM;
    // 一番近い境界(boundary0 + 1区画 × k)を選ぶ
    float k = floorf((pos - edge_mm - ec->boundary0) / SECTION_MM + 0.5f);
    if (k < 0.0f) k = 0.0f;
    float expected = ec->boundary0 + k * SECTION_MM + edge_mm;
    float c = pos - expected;
    if (c > WALL_EDGE_WINDOW_MM || c < -WALL_EDGE_WINDOW_MM) return false; // 予想から外れすぎ(使わない)
    if (c > WALL_EDGE_MAX_CORR_MM) c = WALL_EDGE_MAX_CORR_MM;
    if (c < -WALL_EDGE_MAX_CORR_MM) c = -WALL_EDGE_MAX_CORR_MM;
    Ev(LOG_EV_EDGE_CORR, (float)side, WALL_EDGE_CORR_ENABLE ? c : 0.0f, expected, pos, 0.0f);
    if (!WALL_EDGE_CORR_ENABLE) return false;
    ec->boundary0 += c;
    *corr = c;
    return true;
}

static void StartStraightTo(float target_mm, float v_end);

// 目標の距離が *target − before に届くまで待つ。途中の壁切れで *target を直す(探索の1区画の直進用)。
// 直したら、*target − before(壁を読む所)で v_end になるようにプロファイルを今の速さから引き直す(区画の中で
// 加速・減速するため。最短走行の FastStraightTo と同じ)。before は壁を境界の手前で読むための分(SEARCH_WALL_READ_BEFORE_MM)。
static bool WaitTargetDistanceEdge(float *target, EdgeCorr *ec, float before, float v_end) {
    uint32_t t0 = HAL_GetTick();
    while (App_GetTargetDistance() < *target - before) {
        CheckFailSafe();
        PollLog();
        float c;
        if (EdgeCorr_Check(ec, &c)) {
            *target += c;
            StartStraightTo(*target - before, v_end);
        }
        if (HAL_GetTick() - t0 > SEARCH_STEP_TIMEOUT_MS) return false;
    }
    return true;
}

static bool WaitMotionDone(void) {
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        CheckFailSafe();
        PollLog();
        if (HAL_GetTick() - t0 > SEARCH_STEP_TIMEOUT_MS) return false;
        HAL_Delay(1);
    }
    return true;
}

static void DelayWatching(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        CheckFailSafe();
        PollLog();
        HAL_Delay(1);
    }
}

// ---- 動き ----
// ref_mm: 機体の基準点(今いる真ん中、または次に壁を読む境界)の目標の距離。
// 距離は目標の距離の絶対値で決めるので、壁を読む・経路を計算する間に進んだ分が積み重ならない。
// 尻当てで制御を有効にし直すと目標の距離は0に戻るので、ref_mm もそれに合わせる。

typedef struct {
    float ref_mm;
    bool at_center;
} DrivePos;

// 今の速さのまま(または止まった状態から)、目標の距離が target_mm になる所まで直進する。
static void StartStraightToV(float target_mm, float v_max, float v_end, float accel) {
    float d = target_mm - App_GetTargetDistance();
    if (d < 1.0f) d = 1.0f;
    App_StartStraight(d, v_max, v_end, accel);
}

// 探索の速さで直進する(最高速度は探索の直進の速さ。終わりの速さの方が速ければ、それに合わせる)
static void StartStraightTo(float target_mm, float v_end) {
    float v_max = (v_end > s_search_v) ? v_end : s_search_v;
    StartStraightToV(target_mm, v_max, v_end, s_search_accel);
}

// 小回りの速さのまま直進する(曲がる前後のオフセット・前壁補正の範囲)
static void StartTurnStraightTo(float target_mm) {
    StartStraightToV(target_mm, s_search_turn_v, s_search_turn_v, s_search_accel);
}

// 真ん中で止まる(既に真ん中にいれば何もしない。BlueEyes の half_sectionD)。追記の時間が来ていれば追記する。
static bool StopAtCenter(DrivePos *dp) {
    if (dp->at_center) return true;
    App_SetWallControl(false); // 半区画では壁の制御を使わない(BlueEyes の half_sectionD と同じ)
    dp->ref_mm += HALF_SECTION_MM;
    Ev(LOG_EV_STOP_CENTER, dp->ref_mm, 0.0f, 0.0f, 0.0f, 0.0f);
    StartStraightTo(dp->ref_mm, 0.0f);
    if (!WaitMotionDone()) return false;
    dp->at_center = true;
    return true;
}

static bool Pivot(float angle_deg) {
    DelayWatching(SEARCH_TURN_WAIT_MS);
    App_StartPivot(angle_deg, SEARCH_TURN_OMEGA_DPS, SEARCH_TURN_ALPHA_DPS2);
    if (!WaitMotionDone()) return false;
    DelayWatching(SEARCH_TURN_WAIT_MS);
    return true;
}

// 尻当て(BlueEyes の set_position)。真ん中から後ろの壁に押し当てて向きと位置をそろえ、真ん中へ戻る。
// 押し当てている間は向きと位置の補正を止め、両輪を同じ速さで押す(機体が壁にそろうのを打ち消さないため)。
// 押し当て終わったら制御を有効にし直し、その向き・位置を新しい基準にする(目標の距離は0になる)。
static bool SetPosition(DrivePos *dp) {
    Ev(LOG_EV_SETPOS, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    App_SetWallControl(false);
    App_SetWallPush(true);
    App_SetTargetVelocity(-SEARCH_SETPOS_BACK_V_MM_S);
    DelayWatching(SEARCH_SETPOS_BACK_MS); // 決めた時間だけ下がる(壁に当たった後も押し続ける)
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(false);
    App_SetWallPush(false);
    DelayWatching(SEARCH_SETPOS_SETTLE_MS);
    App_ControlLoop_SetEnabled(true); // ここ(壁に当たった所)が目標の距離0、今の向きが目標の向き

    dp->ref_mm = SEARCH_SETPOS_FRONT_MM;
    StartStraightTo(dp->ref_mm, 0.0f);
    if (!WaitMotionDone()) return false;
    dp->at_center = true;
    Ev(LOG_EV_SETPOS, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    return true;
}

// 180°向きを変える(BlueEyes の turn_back の真ん中の部分)。真ん中にいること。
// 前と右に壁があれば左90°→尻当てを2回、前と左に壁があれば右90°→尻当てを2回、なければその場で180°。
static bool TurnBack(DrivePos *dp, MazePos pos, Direction heading) {
    bool front = WallMap_HasWall(&s_map, pos, heading, WALL_VIEW_KNOWN);
    bool right = WallMap_HasWall(&s_map, pos, Dir_Turn(heading, 1), WALL_VIEW_KNOWN);
    bool left = WallMap_HasWall(&s_map, pos, Dir_Turn(heading, -1), WALL_VIEW_KNOWN);

    if (front && right) {
        return Pivot(90.0f) && SetPosition(dp) && Pivot(90.0f) && SetPosition(dp);
    }
    if (front && left) {
        return Pivot(-90.0f) && SetPosition(dp) && Pivot(-90.0f) && SetPosition(dp);
    }
    return Pivot(180.0f);
}

// BlueEyes の start_sequence。スタート区画(左右と後ろに壁がある)で、右90° → 尻当て → 左90° → 尻当て。
static bool StartSequence(DrivePos *dp) {
    return Pivot(-90.0f) && SetPosition(dp) && Pivot(90.0f) && SetPosition(dp);
}

// ---- 区画の真ん中で回る(RUN の SEARCH_SPIN。壁センサーのモデル用)----
static bool s_spin_cells = false;              // 初めて入った区画ごとに真ん中で回るか(SearchSpin_Run のときだけ)
static bool s_spun[MAZE_SIZE][MAZE_SIZE];      // もう回った区画

static bool SpinAndWait(float angle_deg) {
    App_StartPivot(angle_deg, SENSOR_SPIN_OMEGA_DPS, SENSOR_SPIN_ALPHA_DPS2);
    // 打ち切りは見積もり(台形: 角度/ω + ω/α)の2倍
    float t = fabsf(angle_deg) / SENSOR_SPIN_OMEGA_DPS + SENSOR_SPIN_OMEGA_DPS / SENSOR_SPIN_ALPHA_DPS2;
    uint32_t limit_ms = (uint32_t)(t * 2000.0f);
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        CheckFailSafe();
        PollLog();
        if (HAL_GetTick() - t0 > limit_ms) return false;
        HAL_Delay(1);
    }
    return true;
}

// pos の真ん中で止まり、左に SENSOR_SPIN_ANGLE_DEG 回ってから右に同じだけ回って元の向きに戻る。
// 区画の壁(4方向とも分かっていること)をイベントに残す。打ち切ったら false
static bool SpinAtCenter(DrivePos *dp, MazePos pos, Direction heading) {
    if (!StopAtCenter(dp)) return false;
    App_SetWallControl(false);
    uint8_t walls = 0;
    for (uint8_t d = 0; d < 4u; d++) {
        if (WallMap_HasWall(&s_map, pos, (Direction)d, WALL_VIEW_KNOWN)) walls |= (uint8_t)(1u << d);
    }
    Ev(LOG_EV_SENSOR_SPIN, (float)pos.x, (float)pos.y, (float)heading, (float)walls, 0.0f);
    DelayWatching(SEARCH_SPIN_HOLD_MS);
    if (!SpinAndWait(+SENSOR_SPIN_ANGLE_DEG)) return false;
    DelayWatching(SEARCH_SPIN_HOLD_MS);
    Ev(LOG_EV_SENSOR_SPIN, (float)pos.x, (float)pos.y, (float)heading, (float)walls, 1.0f);
    if (!SpinAndWait(-SENSOR_SPIN_ANGLE_DEG)) return false;
    Ev(LOG_EV_SENSOR_SPIN, (float)pos.x, (float)pos.y, (float)heading, (float)walls, 2.0f);
    DelayWatching(SEARCH_SPIN_HOLD_MS);
    return true;
}

static WallObservation ReadWalls(WallSensorValues *sv) {
    sv->l = ad_l;
    sv->fl = ad_fl;
    sv->fr = ad_fr;
    sv->r = ad_r;
    WallObservation obs = WallSense_Judge(*sv);

    // BlueEyes の get_wall と同じく、見えた壁を直結の LED に出す
    uint8_t leds = LedIdlePattern();
    if (obs.front) leds |= LED_FRONT_LEFT | LED_FRONT_RIGHT;
    if (obs.left) leds |= LED_LEFT;
    if (obs.right) leds |= LED_RIGHT;
    LED_SetDirectPattern(leds);
    return obs;
}

// 次の境界へ向かい、境界の SEARCH_WALL_READ_BEFORE_MM 手前で壁を読む(真ん中からなら半区画加速 =
// BlueEyes の half_sectionA、境界からなら1区画 = one_sectionU)。読んだ後も機体は同じ速さで境界へ進み続け、
// その間に次の動きを決める(dp->ref_mm は境界の位置のまま)。
static bool GoToNextBoundary(DrivePos *dp, uint8_t cells, WallObservation *obs, WallSensorValues *sv) {
    // 壁の制御は境界から境界までの1区画だけ(真ん中からの半区画加速では使わない)
    App_SetWallControl(!dp->at_center);
    bool from_boundary = !dp->at_center;
    float boundary0 = dp->ref_mm; // 境界から走るなら、今いる境界
    dp->ref_mm += dp->at_center ? HALF_SECTION_MM + SECTION_MM * (float)(cells - 1)
                                : SECTION_MM * (float)cells;
    // 壁を読む所(境界の SEARCH_WALL_READ_BEFORE_MM 手前)までに小回りの速さにしておく。曲がるかどうかはそこで決まり、
    // すぐ曲がり始めることがある(前壁補正)ので、そこからでは減速が間に合わない(2026-10-08 に、境界で小回りの速さに
    // なるようにしていたら、読む所ではまだ速く、300mm/s 用の角速度のまま 370mm/s で曲がって壁に寄った)。
    // 読んだ後は、次の指令が出るまで小回りの速さのまま進む。
    StartStraightTo(dp->ref_mm - SEARCH_WALL_READ_BEFORE_MM, s_search_turn_v);
    if (from_boundary) {
        // 境界から境界へ走る間は、壁切れで次の境界の位置を直す
        EdgeCorr ec;
        EdgeCorr_Begin(&ec, boundary0);
        if (!WaitTargetDistanceEdge(&dp->ref_mm, &ec, SEARCH_WALL_READ_BEFORE_MM, s_search_turn_v)) return false;
    } else {
        if (!WaitTargetDistance(dp->ref_mm - SEARCH_WALL_READ_BEFORE_MM)) return false;
    }
    *obs = ReadWalls(sv);
    dp->at_center = false;
    return true;
}

// ---- スラローム(小回り 90°)----
// 境界の手前で壁を読んで「曲がる」と決まったら、止まらずに 前のオフセット → 曲がる → 後ろのオフセット で
// 隣の区画の境界へ進み、その手前で壁を読む(SEARCH_WALL_READ_BEFORE_MM)。オフセットは SearchRun_Run の最初に計算する。
static float s_slalom_pre_mm = 0.0f;
static float s_slalom_post_mm = 0.0f;
static bool s_search_slalom = true; // 探索で曲がるとき、スラローム(true)か超信地旋回(false)か。走る前に選ぶ

static float s_slalom_omega_dps = SLALOM_OMEGA_DPS; // 選んだ速さに合わせた角速度・角加速度
static float s_slalom_alpha_dps2 = SLALOM_ALPHA_DPS2;
static float s_front_ref_sum = SLALOM_FRONT_REF_SUM_AT_PRE0; // 前壁補正の閾値(ComputeSlalomOffsets で計算する)
static RunProfile s_known_prof; // 探索の既知の区間をまとめて走るときの速さ(ComputeSlalomOffsets で計算する)
static RunProfile ComputeFastTurns(void);

static void ComputeSlalomOffsets(void) {
    SlalomParams p = {
        .v_mm_s = SLALOM_V_MM_S,
        .omega_dps = SLALOM_OMEGA_DPS,
        .alpha_dps2 = SLALOM_ALPHA_DPS2,
        .angle_deg = 90.0f,
    };
    Slalom_ScaleToSpeed(&p, s_search_turn_v); // 選んだ速さでも同じ形で曲がる
    s_slalom_omega_dps = p.omega_dps;
    s_slalom_alpha_dps2 = p.alpha_dps2;
    // 前後のオフセットは、スリップなどのずれのモデルから選んだ速さに合わせて計算する(params.h の SLALOM_SLIP_*)
    SlalomOffsets o = Slalom_SmallTurnOffsets(&p);
    s_slalom_pre_mm = o.pre_mm;
    s_slalom_post_mm = o.post_mm;
    s_front_ref_sum = Slalom_FrontRefSum(o.pre_adj_mm); // 曲がり始めが動いた分、前壁補正の閾値も動かす
    printf("slalom: v=%.0f omega=%.0f alpha=%.0f -> pre %.1f (adj %+.1f) post %.1f (adj %+.1f) mm, front ref %.0f\r\n",
           p.v_mm_s, p.omega_dps, p.alpha_dps2, s_slalom_pre_mm, o.pre_adj_mm, s_slalom_post_mm, o.post_adj_mm,
           s_front_ref_sum);

    // 既知の区間をまとめて走るとき(TryKnownRun)の設定。最短走行の部品を使うので、その変数に探索の値を入れる
    // (最短走行は走る前に自分の値を入れ直すので、探索・帰り道の始めにここで上書きしてよい)。
    // 直進の最高速度は、探索の直進の速さとスラロームの速さの速い方(終わりの速さより遅いと、その速さまで上げられない)
    s_fast_v = (s_search_v > s_search_turn_v) ? s_search_v : s_search_turn_v;
    s_fast_accel = SEARCH_KNOWN_ACCEL_MM_S2;
    s_fast_decel = 0.0f; // 減速度は加速度から決める(速度帯の最短走行の値を、帰り道の既知の区間に残さない)
    s_fast_small_v = s_search_turn_v;
    s_known_prof = ComputeFastTurns();
}

// 境界にいる(走っている)ときに呼ぶ。right: 右(時計回り)に曲がる。
// 前壁補正: 曲がる区画の奥に壁があれば、距離で決めた曲がり始めの位置(start)の前後
// SLALOM_FRONT_WINDOW_MM の間で、FL + FR が閾値(s_front_ref_sum)に届いた瞬間まで待つ。
// 届かなければ範囲の終わりまで待つ。前に壁がなければ start まで待つ。打ち切ったら false。
static bool WaitSlalomStart(float start, bool front_wall) {
    if (!SLALOM_FRONT_ENABLE || !front_wall) {
        StartTurnStraightTo(start);
        return WaitTargetDistance(start);
    }
    float lo = start - SLALOM_FRONT_WINDOW_MM;
    float hi = start + SLALOM_FRONT_WINDOW_MM;
    StartTurnStraightTo(hi); // 範囲の終わりまで同じ速さで進めておく
    if (!WaitTargetDistance(lo)) return false;
    uint32_t t0 = HAL_GetTick();
    bool by_sensor = false;
    uint32_t sum = 0;
    while (App_GetTargetDistance() < hi) {
        CheckFailSafe();
        PollLog();
        sum = (uint32_t)ad_fl + ad_fr;
        if ((float)sum >= s_front_ref_sum) {
            by_sensor = true;
            break;
        }
        if (HAL_GetTick() - t0 > SEARCH_STEP_TIMEOUT_MS) return false;
    }
    Ev(LOG_EV_FRONT_TRIG, App_GetTargetDistance() - start, (float)sum, by_sensor ? 1.0f : 0.0f, 0.0f, 0.0f);
    return true;
}

static bool SlalomTurn(DrivePos *dp, bool right, WallObservation *obs, WallSensorValues *sv) {
    App_SetWallControl(false); // 曲がる区画では壁の制御を使わない
    Ev(LOG_EV_TURN_KIND, 0.0f, right ? 1.0f : 0.0f, s_slalom_pre_mm, s_slalom_post_mm, 0.0f);
    float start = dp->ref_mm + s_slalom_pre_mm;
    if (!WaitSlalomStart(start, obs->front)) return false; // obs はこの(曲がる)区画の壁
    App_StartSlalom(right ? -90.0f : 90.0f, s_slalom_omega_dps, s_slalom_alpha_dps2);
    if (!WaitMotionDone()) return false;
    // 曲がっている間の道のりも目標の距離に足されているので、曲がり終わった所から後ろのオフセットぶん進む
    dp->ref_mm = App_GetTargetDistance() + s_slalom_post_mm;
    StartTurnStraightTo(dp->ref_mm);
    // 次の境界の SEARCH_WALL_READ_BEFORE_MM 手前で壁を読む(GoToNextBoundary と同じ。dp->ref_mm は境界の位置のまま)
    if (!WaitTargetDistance(dp->ref_mm - SEARCH_WALL_READ_BEFORE_MM)) return false;
    *obs = ReadWalls(sv);
    dp->at_center = false;
    return true;
}

// 真ん中で 180° 回った後の、今いる区画の壁(新しい向きから見た前・右・左)を地図から作る。
// 次の Step はこの壁を地図に書き込むので、センサーで見ていない向きの壁を壊さないようにする。
static WallObservation WallsFromMap(MazePos pos, Direction heading) {
    WallObservation obs = {
        .front = WallMap_HasWall(&s_map, pos, heading, WALL_VIEW_SEARCH),
        .right = WallMap_HasWall(&s_map, pos, Dir_Turn(heading, 1), WALL_VIEW_SEARCH),
        .left = WallMap_HasWall(&s_map, pos, Dir_Turn(heading, -1), WALL_VIEW_SEARCH),
    };
    return obs;
}

static void Record(MazePos pos, Direction heading, WallObservation obs, WallSensorValues sv,
                   Action act, uint32_t plan_ms) {
    if (s_event_count >= SEARCH_EVENT_MAX) return;
    SearchEvent *e = &s_events[s_event_count++];
    e->pos = pos;
    e->heading = (uint8_t)heading;
    e->walls = (uint8_t)((obs.front ? 1u : 0u) | (obs.right ? 2u : 0u) | (obs.left ? 4u : 0u));
    e->action = act.type;
    e->plan_ms = (uint8_t)((plan_ms > 255u) ? 255u : plan_ms);
    e->l = sv.l;
    e->fl = sv.fl;
    e->fr = sv.fr;
    e->r = sv.r;
    Ev(LOG_EV_STEP, (float)pos.x, (float)pos.y, (float)heading, (float)e->walls, (float)act.type);
    Ev(LOG_EV_STEP_INFO, (float)plan_ms, (float)sv.l, (float)sv.fl, (float)sv.fr, (float)sv.r);
}

static void PrintResult(const MazePos *goals, uint8_t goal_count) {
    printf("---- search log: %u cells ----\r\n", s_event_count);
    printf(" #   x y h  F R L  action   plan   L    FL   FR   R\r\n");
    for (uint16_t i = 0; i < s_event_count; i++) {
        const SearchEvent *e = &s_events[i];
        printf("%3u  %u %u %s  %c %c %c  %-10s %3ums %4u %4u %4u %4u\r\n",
               i, e->pos.x, e->pos.y, DirName((Direction)e->heading),
               (e->walls & 1u) ? 'W' : '.', (e->walls & 2u) ? 'W' : '.', (e->walls & 4u) ? 'W' : '.',
               Action_Name((ActionType)e->action), e->plan_ms, e->l, e->fl, e->fr, e->r);
    }
    printf("phase: %s\r\n", PhaseName(s_planner.phase));
    MazePrint_Map(&s_map, NULL, &s_planner.pos, s_planner.heading, goals, goal_count);
}

bool MazeRun_StartSequence(void) {
    DrivePos dp = { .ref_mm = App_GetTargetDistance(), .at_center = true };
    return StartSequence(&dp);
}

// 32bit の値の上位・下位 16bit(イベントの値は CSV で有効数字6桁なので分けて入れる)
static float Hi16(uint32_t v) { return (float)(v >> 16); }
static float Lo16(uint32_t v) { return (float)(v & 0xFFFFu); }

// 今の起動のリセットの原因と、その前の HardFault の記録をログに残す(止まった原因を後から調べるため)
static void LogBootInfo(void) {
    const FaultDiagInfo *b = FaultDiag_GetBootInfo();
    Ev(LOG_EV_BOOT, (float)b->reset_flags, b->had_fault ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f);
    if (b->had_fault) {
        Ev(LOG_EV_FAULT_PC, Hi16(b->pc), Lo16(b->pc), Hi16(b->lr), Lo16(b->lr), 0.0f);
        Ev(LOG_EV_FAULT_REG, Hi16(b->cfsr), Lo16(b->cfsr), Hi16(b->hfsr), Hi16(b->bfar), Lo16(b->bfar));
    }
}

// 長い走行のモード(LongLogRun_Run)で今走っているのが何番目か(ログの LOG_EV_LONG_RUN 用)。そのモードでなければ 0
static uint16_t s_long_index = 0;
static uint8_t s_long_part = 0;
static uint8_t s_long_step = 0;
static float s_long_v = 0.0f;
static float s_long_v_turn = 0.0f;
// 速度帯の最短走行(FastBands_Run)で今走っている速度帯(1〜)と、何本目か・全部で何本か。そのモードでなければ 0
static uint8_t s_band = 0;
static uint16_t s_band_no = 0;
static uint16_t s_band_total = 0;

// 走り出す前の準備(探索・最短走行・帰り道で共通): 制御を有効にし、記録を始める。今いる所(区画の真ん中)が基準になる。
static void StartRunBegin(DrivePos *dp) {
    App_SetPositionHold(true);
    App_SetWallPush(false);
    App_SetWallControl(false); // 使うのは境界から境界までの直進の間だけ
    Logger_Start(); // ここから記録(手かざしで始めた走りの始まり)
    Ev(LOG_EV_MODE, (float)ModeUI_CurrentMode(), 0.0f, 0.0f, 0.0f, 0.0f);
    Ev(LOG_EV_RUN_TYPE, s_run_type, s_run_kind, 0.0f, 0.0f, 0.0f);
    if (s_run_kind == 1.0f) { // 最短走行: 経路のコスト・見積もりの時間・指令の数
        Ev(LOG_EV_ROUTE, s_route_cost, s_route_est, (float)s_route_count, s_run_type, 0.0f);
    }
    Ev(LOG_EV_HAND_START, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Ev(LOG_EV_GYRO_RECAL, s_gyro_offset, 0.0f, 0.0f, 0.0f, 0.0f);
    LogBootInfo();
    if (s_run_kind == 1.0f) {
        Ev(LOG_EV_FAST_PARAMS, s_fast_v, s_fast_accel, s_fast_small_v, s_run_type, FastDecel());
    } else {
        Ev(LOG_EV_SEARCH_PARAMS, s_search_v, s_search_turn_v, s_search_accel, s_search_slalom ? 1.0f : 0.0f, 0.0f);
        Ev(LOG_EV_SEARCH_MODE, (float)s_search_map, (float)s_search_scope,
           (s_search_algo == SEARCH_ALGO_ADACHI) ? 2.0f : 1.0f, 0.0f, 0.0f);
    }
    if (s_long_index > 0) {
        Ev(LOG_EV_LONG_RUN, (float)s_long_index, (float)s_long_part, (float)s_long_step, s_long_v, s_long_v_turn);
    }
    if (s_band > 0) {
        Ev(LOG_EV_FAST_BAND, (float)s_band, (float)s_band_no, (float)s_band_total, 0.0f, 0.0f);
    }
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);

    dp->ref_mm = 0.0f;
    dp->at_center = true;
}

// 走り出す前の準備(探索・最短走行で共通): StartRunBegin の後、start_sequence で真ん中に合わせる。
static bool StartRun(DrivePos *dp) {
    StartRunBegin(dp);
    return StartSequence(dp);
}

static bool SearchLoop(const DrivePos *dp_start, WallObservation obs, WallSensorValues sv);
static bool TryKnownRun(DrivePos *dp, MazePos c0, Direction h0, WallObservation *obs, WallSensorValues *sv,
                        bool *used);

// 1回の探索(往復: スタート → ゴール → スタート、片道: スタート → ゴール)。打ち切ったら false。
// 地図は、初期化(SEARCH_MAP_NEW)か、flash に残した地図に重ねる(SEARCH_MAP_CONTINUE)。
static bool RunSearch(SearchAlgo algo, const MazePos *goals, uint8_t goal_count) {
    MazePos start = { MAZE_START_X, MAZE_START_Y };
    if (s_search_map == SEARCH_MAP_CONTINUE) {
        if (!Flash_ReadUserData(&s_map, sizeof(s_map))) {
            printf("no map in flash: start from an empty map\r\n");
            WallMap_Init(&s_map);
        }
    } else {
        WallMap_Init(&s_map);
    }
    SearchPlanner_Init(&s_planner, &s_map, algo, start, DIR_NORTH, goals, goal_count);
    s_search_algo = algo;
    s_event_count = 0;
    memset(s_spun, 0, sizeof(s_spun));
    if (s_search_scope == SEARCH_SCOPE_FULL) {
        // 全面探索(迷路の全部の区画を見る)はまだ作っていない。作るときはここから始める。
        // 段階を FAILED にして、FinishSearchRun が地図を flash に書かないようにする
        printf("FULL SEARCH: not implemented yet\r\n");
        s_planner.phase = SEARCH_PHASE_FAILED;
        return false;
    }

    DrivePos dp;
    if (!StartRun(&dp)) return false;

    // スタート区画は、決まりで左右に壁があり前は開いている
    // (BlueEyes はセンサーで読んで前の壁を消しているが、yuho の斜め前のセンサーは真ん中では
    //  隣の区画の入り口を見ているので、スタート区画の壁は決まりで入れる)
    WallObservation obs = { .front = false, .right = true, .left = true };
    WallSensorValues sv = { 0, 0, 0, 0 };
    return SearchLoop(&dp, obs, sv);
}

// 探索の繰り返し(壁を読む → 次の動きを決める → 動く)。s_planner の今の位置・向き・段階から始め、
// スタートに着いて段階が DONE になったら true(打ち切った・行けなかったら false)。
// dp_start: 今いる所(制御を有効にした後)、obs / sv: 今いる区画の壁(プランナーに最初に渡す)。
static bool SearchLoop(const DrivePos *dp_start, WallObservation obs, WallSensorValues sv) {
    DrivePos dp = *dp_start;
    while (1) {
        MazePos pos = s_planner.pos;
        Direction heading = s_planner.heading;
        uint32_t t0 = HAL_GetTick();
        SearchPhase phase_before = s_planner.phase;
        Action act = SearchPlanner_Step(&s_planner, obs);
        Record(pos, heading, obs, sv, act, HAL_GetTick() - t0);
        if (s_planner.phase != phase_before) Ev(LOG_EV_PHASE, (float)s_planner.phase, 0.0f, 0.0f, 0.0f, 0.0f);

        // SEARCH_SPIN: 初めて入った区画なら、真ん中で止まって回る(壁は今の Step で地図に入った)。
        // 回った後は真ん中から次の動きを始める(ゴールで回った後と同じ)
        if (s_spin_cells && !s_spun[pos.y][pos.x]) {
            s_spun[pos.y][pos.x] = true;
            if (!SpinAtCenter(&dp, pos, heading)) {
                Ev(LOG_EV_TIMEOUT, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                App_SetTargetVelocity(0.0f);
                DelayWatching(300);
                App_ControlLoop_SetEnabled(false);
                printf("search: spin timeout\r\n");
                return false;
            }
        }

        bool ok = true;
        switch ((ActionType)act.type) {
            case ACTION_FORWARD: {
                // 先の区画の壁が分かっていれば、まとめて走る(直線の加速・大回り)。使わなければ1区画進む
                bool used = false;
                ok = TryKnownRun(&dp, pos, heading, &obs, &sv, &used);
                if (ok && !used) ok = GoToNextBoundary(&dp, act.cells, &obs, &sv);
                break;
            }
            case ACTION_TURN_RIGHT:
            case ACTION_TURN_LEFT:
                if (s_search_slalom && !dp.at_center) {
                    // 走っている: 止まらずにスラロームで曲がる
                    ok = SlalomTurn(&dp, act.type == ACTION_TURN_RIGHT, &obs, &sv);
                } else {
                    // 超信地旋回を選んだとき、または真ん中で止まっている(ゴールで回った後など)とき:
                    // 真ん中で止まって超信地旋回する
                    Ev(LOG_EV_TURN_KIND, 3.0f, (act.type == ACTION_TURN_RIGHT) ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f);
                    ok = StopAtCenter(&dp)
                      && Pivot((act.type == ACTION_TURN_RIGHT) ? -90.0f : 90.0f)
                      && GoToNextBoundary(&dp, 1, &obs, &sv);
                }
                break;
            case ACTION_TURN_BACK:
                ok = StopAtCenter(&dp) && TurnBack(&dp, pos, heading)
                  && GoToNextBoundary(&dp, 1, &obs, &sv);
                break;
            case ACTION_STOP:
            default:
                // ゴール・スタートに着いた(または行けない): 真ん中で止まって 180°(BlueEyes の last_run)
                ok = StopAtCenter(&dp);
                if (!ok) break;
                if (s_planner.phase == SEARCH_PHASE_FAILED) {
                    App_ControlLoop_SetEnabled(false);
                    return false;
                }
                if (s_search_scope == SEARCH_SCOPE_ONE_WAY && s_planner.phase == SEARCH_PHASE_TO_START) {
                    // 片道探索: ゴールの真ん中で止まって終わる(地図は FinishSearchRun で flash に残す)
                    LED_SetDirectPattern(LED_DIRECT_ALL);
                    DelayWatching(SEARCH_GOAL_WAIT_MS);
                    LED_SetDirectPattern(LedIdlePattern());
                    App_ControlLoop_SetEnabled(false);
                    return true;
                }
                ok = TurnBack(&dp, s_planner.pos, s_planner.heading);
                if (!ok) break;
                LED_SetDirectPattern(LED_DIRECT_ALL);
                DelayWatching(SEARCH_GOAL_WAIT_MS);
                LED_SetDirectPattern(LedIdlePattern());
                if (s_planner.phase == SEARCH_PHASE_DONE) {
                    App_ControlLoop_SetEnabled(false);
                    return true;
                }
                // ゴールに着いた: 向きを変えたことをプランナーに伝え、帰りを始める
                s_planner.heading = Dir_Opposite(s_planner.heading);
                obs = WallsFromMap(s_planner.pos, s_planner.heading);
                sv = (WallSensorValues){ 0, 0, 0, 0 };
                break;
        }
        if (!ok) {
            App_SetWallPush(false);
            Ev(LOG_EV_TIMEOUT, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            App_SetTargetVelocity(0.0f);
            DelayWatching(300);
            App_ControlLoop_SetEnabled(false);
            printf("search: timeout\r\n");
            return false;
        }
    }
}

// 今のゴール(試しのゴールか、params.h の MAZE_GOALS)
static uint8_t GetGoals(const MazePos **goals) {
#if SEARCH_USE_TEST_GOAL
    static const MazePos kGoals[SEARCH_TEST_GOAL_COUNT] = SEARCH_TEST_GOALS;
    *goals = kGoals;
    return SEARCH_TEST_GOAL_COUNT;
#else
    static const MazePos kGoals[MAZE_GOAL_COUNT] = MAZE_GOALS;
    *goals = kGoals;
    return MAZE_GOAL_COUNT;
#endif
}

// 手かざしで走り出した後の準備: ジャイロを合わせ、ログのファイルを開く(探索・最短走行で共通)。
static void PrepareStart(const char *log_name) {
    HAL_Delay(SEARCH_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
    s_gyro_offset = App_RecalibrateGyroZ(GYRO_RECAL_MS);
    printf("gyro z offset: %.1f\r\n", s_gyro_offset);
    App_ResetGyroAngle(); // スタートの向きを 0° にする(ログの向きを読みやすくするため)

    // ログを流すファイルを作る(先に領域を確保するので、少し時間がかかることがある)
    SetupLogger(log_name);
    char path[SDCARD_PATH_MAX];
    s_log_failed = false;
    LED_SetDirectPattern(0x00u);
    s_log_file = Logger_StreamBegin(path, sizeof(path));
    printf("log: %s\r\n", s_log_file ? path : "(no SD, not saved)");
}

// ログのファイルを閉じる(探索・最短走行で共通)。
// 機体の壁の地図をログのイベントに残す(LOG_EV_MAP_CELLS。PC の log_viewer で描く)。
// 1つのイベントに8区画ずつ入れ、順番待ちが空くのを待ちながら入れる(16×16 で 32 個、約 0.2 秒)。
#define MAP_LOG_CELLS_PER_EVENT 8u
#define MAP_LOG_TIMEOUT_MS      2000u
static void LogMap(void) {
    uint32_t t0 = HAL_GetTick();
    for (uint32_t i = 0; i < MAZE_CELL_COUNT; i += MAP_LOG_CELLS_PER_EVENT) {
        while (Logger_EventQueued() >= Logger_EventCapacity()) {
            if (HAL_GetTick() - t0 > MAP_LOG_TIMEOUT_MS) return; // 記録が止まっているなど
            DelayWatching(SEARCH_LOG_DECIMATION);
        }
        float w[4];
        for (uint32_t k = 0; k < 4u; k++) {
            uint32_t c0 = i + 2u * k;
            uint32_t c1 = c0 + 1u;
            uint32_t lo = (c0 < MAZE_CELL_COUNT) ? s_map.cell[c0 / MAZE_SIZE][c0 % MAZE_SIZE] : 0u;
            uint32_t hi = (c1 < MAZE_CELL_COUNT) ? s_map.cell[c1 / MAZE_SIZE][c1 % MAZE_SIZE] : 0u;
            w[k] = (float)(lo | (hi << 8));
        }
        Ev(LOG_EV_MAP_CELLS, (float)i, w[0], w[1], w[2], w[3]);
    }
    // 全部が記録の行に入るのを待つ
    while (Logger_EventQueued() > 0u && HAL_GetTick() - t0 <= MAP_LOG_TIMEOUT_MS) {
        DelayWatching(SEARCH_LOG_DECIMATION);
    }
}

static void EndRunLog(void) {
    if (s_log_file) {
        uint32_t dropped = 0;
        bool saved = Logger_StreamEnd(&dropped);
        s_log_file = false;
        printf("log %s (dropped %lu rows)\r\n", saved ? "saved" : "save failed", (unsigned long)dropped);
        if (saved) {
            ShowLogSaved();
        } else {
            s_log_failed = true;
            LED_SetDirectPattern(LedIdlePattern());
        }
    } else {
        Logger_Stop();
    }
}

static void PrintSettings(const MazePos *goals, uint8_t goal_count) {
    printf("goal (%u,%u) x%u%s, turn %.0f dps\r\n",
           goals[0].x, goals[0].y, goal_count, SEARCH_USE_TEST_GOAL ? " [TEST GOAL]" : "",
           SEARCH_TURN_OMEGA_DPS);
    printf("WALL: %s WALL_KP_DEG=%.3f max %.1f deg, TH L=%d R=%d F=%d, SETPOS front %.0f mm\r\n",
           WALL_CONTROL_ENABLE ? "ON" : "OFF", WALL_KP_DEG, WALL_OFFSET_MAX_DEG,
           WALL_TH_L, WALL_TH_R, WALL_TH_FRONT_SUM,
           SEARCH_SETPOS_FRONT_MM);
}

// 探索が終わった後: ゴールまで行けていれば地図を flash に残し(最短走行のモードで使う。BlueEyes の
// store_map_in_flash)、地図をログに入れてログを閉じ、結果を UART に出す。
static void FinishSearchRun(bool done, const MazePos *goals, uint8_t goal_count) {
    // 結果をログのイベントに残せるよう、ログを閉じる前に行う
    if (s_planner.phase == SEARCH_PHASE_TO_START || s_planner.phase == SEARCH_PHASE_DONE) {
        bool saved = Flash_WriteUserData(&s_map, sizeof(s_map));
        Ev(LOG_EV_MAP_SAVED, saved ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        DelayWatching(SEARCH_LOG_DECIMATION * 2u); // イベントが記録の行に入るのを待つ
        printf("map %s\r\n", saved ? "saved to flash" : "save FAILED");
    }
    LogMap(); // 探索の終わりの地図(ゴールまで行けなかったときも)
    EndRunLog();
    printf("search %s\r\n", done ? "finished" : "stopped");
    PrintResult(goals, goal_count);
}

void SearchRun_Run(SearchAlgo algo) {
    const MazePos *goals;
    uint8_t goal_count = GetGoals(&goals);

    // 探索の速さを選ぶ: 直進の最高速度と、小回り(スラローム)の速さ(電源を切るまでそのまま)
    s_search_v = ModeUI_SelectValue("SPEED", "mm/s", kSearchSpeeds,
                                    (uint8_t)(sizeof(kSearchSpeeds) / sizeof(kSearchSpeeds[0])), SEARCH_V_MM_S);
    s_search_turn_v = ModeUI_SelectValue("SLALOM", "mm/s", kSearchTurnSpeeds,
                                         (uint8_t)(sizeof(kSearchTurnSpeeds) / sizeof(kSearchTurnSpeeds[0])),
                                         SLALOM_V_MM_S);
    printf("SEARCH (%s): v=%.0f mm/s, slalom %.0f mm/s, accel=%.0f\r\n",
           (algo == SEARCH_ALGO_ADACHI) ? "adachi" : "dijkstra", s_search_v, s_search_turn_v, SEARCH_ACCEL_MM_S2);
    PrintSettings(goals, goal_count);
    ComputeSlalomOffsets();

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    static const char *const kTypes[] = { "PIVOT", "SMALL" };
    uint8_t type = 1; // 既定はスラローム(小回り)
    while (1) {
        // 曲がり方を選ぶ: ボタンのクリックで切り替え、手かざしで走り出す(シフトレジスタの LED に番号を出す)
        printf("put in the start cell (facing north). turn type: %s  (click: change, hand: START)\r\n", kTypes[type]);
        LED_SetShiftPattern((uint16_t)(1u << type));
        while (ModeUI_WaitHandStartOrClick()) {
            type = (uint8_t)((type + 1u) % 2u);
            printf("turn type: %s\r\n", kTypes[type]);
            LED_SetShiftPattern((uint16_t)(1u << type));
        }
        s_search_slalom = (type == 1);
        s_run_kind = 0.0f; // 探索の曲がり方
        s_run_type = (float)type;

        PrepareStart(s_spin_cells ? "spin" : "search");
        bool done = RunSearch(algo, goals, goal_count);
        FinishSearchRun(done, goals, goal_count);
    }
}

void SearchSpin_Run(void) {
    s_spin_cells = true;
    printf("SEARCH SPIN: spin +%.0f / -%.0f deg (%.0f dps) at the center of every new cell\r\n",
           SENSOR_SPIN_ANGLE_DEG, SENSOR_SPIN_ANGLE_DEG, SENSOR_SPIN_OMEGA_DPS);
    SearchRun_Run(SEARCH_ALGO_DIJKSTRA);
}

// ---- RUN の SEARCH ----
// 地図(初期化 / 重ねる)・行き先(往復 / 片道 / 全面)・アルゴリズム・直進の速さ・加速度・スラロームの速さ・曲がり方を選んでから、
// 手かざしで探索する(終わったら、同じ設定で次の手かざしを待つ)。全面探索はまだ作っていない(選ぶと走らずに止まる)。
void SearchMenu_Run(void) {
    const MazePos *goals;
    uint8_t goal_count = GetGoals(&goals);
    static const float kTwo[] = { 1.0f, 2.0f };
    static const float kThree[] = { 1.0f, 2.0f, 3.0f };

    s_search_map = (ModeUI_SelectValue("MAP(1 NEW 2 CONTINUE)", "", kTwo, 2u, 1.0f) == 2.0f)
                     ? SEARCH_MAP_CONTINUE : SEARCH_MAP_NEW;
    float scope = ModeUI_SelectValue("SCOPE(1 ROUND 2 ONE-WAY 3 FULL)", "", kThree, 3u, 1.0f);
    s_search_scope = (scope == 3.0f) ? SEARCH_SCOPE_FULL : (scope == 2.0f) ? SEARCH_SCOPE_ONE_WAY : SEARCH_SCOPE_ROUND;
    SearchAlgo algo = (ModeUI_SelectValue("ALGO(1 DIJKSTRA 2 ADACHI)", "", kTwo, 2u, 1.0f) == 2.0f)
                        ? SEARCH_ALGO_ADACHI : SEARCH_ALGO_DIJKSTRA;
    s_search_v = ModeUI_SelectValue("SPEED", "mm/s", kSearchSpeeds,
                                    (uint8_t)(sizeof(kSearchSpeeds) / sizeof(kSearchSpeeds[0])), SEARCH_V_MM_S);
    s_search_accel = ModeUI_SelectValue("ACCEL", "mm/s2", kAccels, (uint8_t)(sizeof(kAccels) / sizeof(kAccels[0])),
                                        SEARCH_ACCEL_MM_S2);
    s_search_turn_v = ModeUI_SelectValue("SLALOM", "mm/s", kSearchTurnSpeeds,
                                         (uint8_t)(sizeof(kSearchTurnSpeeds) / sizeof(kSearchTurnSpeeds[0])),
                                         SLALOM_V_MM_S);
    s_search_slalom = (ModeUI_SelectValue("TURN(1 SLALOM 2 PIVOT)", "", kTwo, 2u, 1.0f) == 1.0f);

    static const char *const kMapNames[] = { "", "NEW", "CONTINUE" };
    static const char *const kScopeNames[] = { "", "ROUND", "ONE-WAY", "FULL" };
    printf("SEARCH: map %s, %s, %s, v=%.0f mm/s accel=%.0f, slalom %.0f mm/s, turn %s\r\n",
           kMapNames[s_search_map], kScopeNames[s_search_scope], (algo == SEARCH_ALGO_ADACHI) ? "adachi" : "dijkstra",
           s_search_v, s_search_accel, s_search_turn_v, s_search_slalom ? "SLALOM" : "PIVOT");
    if (s_search_scope == SEARCH_SCOPE_FULL) printf("FULL SEARCH is not implemented yet (it stops without running)\r\n");
    PrintSettings(goals, goal_count);
    ComputeSlalomOffsets();

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    while (1) {
        printf("put in the start cell (facing north). hand: START\r\n");
        LED_SetShiftPattern(0x0000u);
        while (ModeUI_WaitHandStartOrClick()) {
            // 手かざしで始める(クリックは無視する)
        }
        s_run_kind = 0.0f; // 探索の曲がり方
        s_run_type = s_search_slalom ? 1.0f : 0.0f;
        PrepareStart("search");
        bool done = RunSearch(algo, goals, goal_count);
        FinishSearchRun(done, goals, goal_count);
    }
}

// ---- 最短走行 ----
// 探索で flash に残した地図の、分かっている壁だけ(未知の壁は「ある」)で Dijkstra を計算し、
// スタートの真ん中からゴールまでの経路を指令の列にする。走り方は走る前に3つから選ぶ
// (maze_sim の最短走行と同じ指令。logic/maze/run_path):
//   PIVOT: 続く直進を1つにまとめて FAST_V で走り、曲がるときは真ん中で止まって超信地旋回
//   SMALL: 小回り 90° のスラロームだけ(RunPath_FromRoute の use_large = false)
//   LARGE: 大回り 90°・180° に置き換えられる所は置き換える(use_large = true)
// スラロームの指令の列では、直進は FAST_V まで加速し、次の旋回の速さまで減速して旋回に入る。

static CommandList s_route;
static RunList s_run_small;
static RunList s_run_large;
static bool s_run_small_ok = false;
static bool s_run_large_ok = false;

// 旋回の種類ごとの動き(速さ・角速度・角加速度・角度・前後のオフセット。logic/maze/run_path の RunTurnSpec)
typedef RunTurnSpec FastTurn;

static FastTurn s_fast_turn[RUN_TYPE_COUNT];

// 旋回の形を計算し、経路の計算に使う RunProfile も作る(計算は logic の RunProfile_ForSpeeds。maze_sim と同じ)
static RunProfile ComputeFastTurns(void) {
    RunProfile prof = RunProfile_ForSpeeds(s_fast_v, s_fast_accel, s_fast_small_v, s_fast_turn);
    prof.decel = FastDecel(); // 速度帯で減速度を決めたとき(FAST_BANDS)も、経路の時間の計算を走りに合わせる

    static const RunType kTurns[] = { RUN_SMALL90_R, RUN_LARGE90_R, RUN_LARGE180_R };
    static const char *const kNames[] = { "small90", "large90", "large180" };
    for (int i = 0; i < 3; i++) {
        const FastTurn *t = &s_fast_turn[kTurns[i]];
        printf("%-8s: v=%.0f omega=%.0f alpha=%.0f -> pre %.1f post %.1f mm%s\r\n",
               kNames[i], t->v_mm_s, t->omega_dps, t->alpha_dps2, t->pre_mm, t->post_mm,
               (t->pre_mm < 0.0f || t->post_mm < 0.0f) ? "  (WARNING: negative offset)" : "");
    }
    return prof;
}

// 次の指令が旋回ならその速さ、そうでなければ(STOP)0
static float NextTurnSpeed(const RunList *list, uint16_t i) {
    if (i + 1 >= list->count) return 0.0f;
    RunType next = (RunType)list->items[i + 1].type;
    return RunType_IsTurn(next) ? s_fast_turn[next].v_mm_s : 0.0f;
}

// 最短走行の指令の列どおりに走る(スタートの真ん中 → ゴールの真ん中)。ref_mm は今の基準点。
// 最短走行の直進。start から end まで走る(v_exit の速さで着く。0 なら止まる)。
// 壁の制御は、最初と最後の半区画を除いた所だけ(1区画以上残るときだけ)。
// 途中の壁切れで、終わりの位置(と、その先の基準)を直す。直したら、プロファイルを新しい終わりへ引き直す。
// at_center: 始まりが区画の中心なら true(最初の境界は半区画先)、境界なら false。
static void StartFastStraightTo(float target_mm, float v_end) {
    float d = target_mm - App_GetTargetDistance();
    if (d < 1.0f) d = 1.0f;
    App_StartStraightAD(d, s_fast_v, v_end, s_fast_accel, FastDecel());
}

static bool FastStraightTo(float start, float *end, float v_exit, bool at_center) {
    App_SetWallControl(false);
    StartFastStraightTo(*end, v_exit);
    bool use_wall = (*end - start >= 2.0f * SECTION_MM);
    EdgeCorr ec;
    EdgeCorr_Begin(&ec, at_center ? start + HALF_SECTION_MM : start);
    uint32_t t0 = HAL_GetTick();
    while (1) {
        CheckFailSafe();
        PollLog();
        float pos = App_GetTargetDistance();
        if (use_wall) App_SetWallControl(pos >= start + HALF_SECTION_MM && pos < *end - HALF_SECTION_MM);
        float c;
        if (EdgeCorr_Check(&ec, &c)) {
            *end += c;
            StartFastStraightTo(*end, v_exit); // 今の速さから引き直す
        }
        if (v_exit > 0.0f ? (pos >= *end) : App_IsMotionDone()) break;
        if (HAL_GetTick() - t0 > SEARCH_STEP_TIMEOUT_MS) return false;
        if (v_exit <= 0.0f) HAL_Delay(1);
    }
    App_SetWallControl(false);
    return true;
}

// v_final: 最後の直進(次が STOP)の終わりの速さ。0 なら止まる(最短走行)。探索の既知の区間では、スラロームの速さで
//   走り続けたまま次の区画へ入る。end_before: そのとき、最後の直進の終わりのこれだけ手前で待つのをやめる
//   (探索で壁を境界の手前で読むため。その所までに v_final にする)。*ref_mm は終わり(境界)の位置になる。
static bool RunList_Drive(const RunList *list, float *ref_mm, float v_final, float end_before) {
    bool at_center = true; // スタートは区画の中心
    for (uint16_t i = 0; i < list->count; i++) {
        RunType type = (RunType)list->items[i].type;
        Ev(LOG_EV_RUN_CMD, (float)i, (float)type, (float)list->items[i].halves, 0.0f, 0.0f);
        if (type == RUN_STOP) break;

        if (type == RUN_STRAIGHT) {
            float start = *ref_mm;
            float end = start + HALF_SECTION_MM * (float)list->items[i].halves;
            bool last = (i + 1u >= list->count) || (list->items[i + 1u].type == RUN_STOP);
            if (last && v_final > 0.0f) {
                float end_read = end - end_before;
                if (!FastStraightTo(start, &end_read, v_final, at_center)) return false;
                *ref_mm = end_read + end_before; // 壁切れで直した分も含めた終わり(境界)
            } else {
                if (!FastStraightTo(start, &end, NextTurnSpeed(list, i), at_center)) return false;
                *ref_mm = end;
            }
            if (list->items[i].halves % 2u) at_center = !at_center; // 半区画ごとに、中心と境界が入れ替わる
            continue;
        }

        // 旋回: 前のオフセット → 曲がる → 後ろのオフセット(どれも旋回の速さで)
        const FastTurn *t = &s_fast_turn[type];
        int q = RunType_QuarterTurns(type); // 時計回りが正
        float kind = RunType_IsLarge(type) ? ((t->angle_deg > 90.0f) ? 2.0f : 1.0f) : 0.0f;
        Ev(LOG_EV_TURN_KIND, kind, (q > 0) ? 1.0f : 0.0f, t->pre_mm, t->post_mm, 0.0f);
        App_SetWallControl(false);
        float start = *ref_mm + t->pre_mm;
        StartStraightToV(start, t->v_mm_s, t->v_mm_s, s_fast_accel);
        if (!WaitTargetDistance(start)) return false;
        App_StartSlalom(-90.0f * (float)q, t->omega_dps, t->alpha_dps2);
        if (!WaitMotionDone()) return false;
        *ref_mm = App_GetTargetDistance() + t->post_mm;
        StartStraightToV(*ref_mm, t->v_mm_s, t->v_mm_s, s_fast_accel);
        if (!WaitTargetDistance(*ref_mm)) return false;
        at_center = RunType_IsLarge(type); // 小回りは境界から境界、大回りは中心から中心
    }
    return true;
}

// 経路(区画の指令の列)をたどった後の、ゴールの区画と向き
static void RouteEnd(MazePos *pos, Direction *heading) {
    *pos = (MazePos){ MAZE_START_X, MAZE_START_Y };
    *heading = DIR_NORTH;
    for (uint16_t i = 0; i < s_route.count; i++) {
        Action act = s_route.items[i];
        if (act.type == ACTION_STOP) break;
        uint8_t n = 1;
        if (act.type == ACTION_FORWARD) {
            n = act.cells;
        } else {
            *heading = Dir_Turn(*heading, Action_QuarterTurns((ActionType)act.type));
        }
        for (uint8_t k = 0; k < n; k++) MazePos_Step(*pos, *heading, pos);
    }
}

// スラロームの指令の列で走る(スタートの真ん中 → ゴールの真ん中で止まって 180°)。打ち切ったら false。
static bool RunFastSlalom(const RunList *list) {
    DrivePos dp;
    if (!StartRun(&dp)) return false;
    float ref = dp.ref_mm;
    bool ok = RunList_Drive(list, &ref, 0.0f, 0.0f);
    if (ok) {
        dp.ref_mm = ref;
        dp.at_center = true;
        MazePos pos;
        Direction heading;
        RouteEnd(&pos, &heading);
        ok = TurnBack(&dp, pos, heading); // ゴールの真ん中で 180°(尻当てあり)
    }
    if (ok) {
        LED_SetDirectPattern(LED_DIRECT_ALL);
        DelayWatching(SEARCH_GOAL_WAIT_MS);
        LED_SetDirectPattern(LedIdlePattern());
    } else {
        Ev(LOG_EV_TIMEOUT, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        App_SetWallPush(false);
        App_SetTargetVelocity(0.0f);
        DelayWatching(300);
        printf("fast run: timeout\r\n");
    }
    App_ControlLoop_SetEnabled(false);
    return ok;
}

// 指令の列と見積もりの時間を出し、見積もりの時間[s]を返す(経路がなければ 0)
static float PrintRunList(const char *name, const RunList *list, bool ok, const RunProfile *prof) {
    if (!ok) {
        printf("[%s] no route\r\n", name);
        return 0.0f;
    }
    float total = 0.0f;
    uint16_t bad = 0;
    bool fits = RunList_EstimateTime(list, prof, &total, NULL, &bad);
    printf("[%s] %.3f s%s\r\n", name, total, fits ? "" : " (accel!)");
    RunList_Print(list);
    return total;
}

// 真ん中から真ん中まで cells 区画まっすぐ走って止まる。壁の制御は、境界から境界までの間
// (最初と最後の半区画を除く)だけ使う。
static bool FastStraight(DrivePos *dp, uint8_t cells) {
    float start = dp->ref_mm;
    float end = start + SECTION_MM * (float)cells;
    if (!FastStraightTo(start, &end, 0.0f, true)) return false; // 中心から中心へ(壁切れで終わりを直す)
    dp->ref_mm = end;
    dp->at_center = true;
    return true;
}

// 経路どおりに走る(スタートの真ん中 → ゴールの真ん中で止まって 180°)。打ち切ったら false。
static bool RunFast(void) {
    DrivePos dp;
    if (!StartRun(&dp)) return false;

    MazePos pos = { MAZE_START_X, MAZE_START_Y };
    Direction heading = DIR_NORTH;
    uint8_t cells = 0; // まとめて走る直進の区画数
    bool ok = true;

    for (uint16_t i = 0; i < s_route.count && ok; i++) {
        Action act = s_route.items[i];
        Ev(LOG_EV_ROUTE_CMD, (float)i, (float)act.type, (float)act.cells, 0.0f, 0.0f);
        if (act.type == ACTION_FORWARD) {
            cells += act.cells;
            for (uint8_t k = 0; k < act.cells; k++) MazePos_Step(pos, heading, &pos);
        } else if (act.type == ACTION_STOP) {
            if (cells > 0) ok = FastStraight(&dp, cells);
            cells = 0;
            break;
        } else {
            // 曲がる: それまでの直進を走って真ん中で止まり、回ってから隣の区画へ(次の直進にまとめる)
            if (cells > 0) ok = ok && FastStraight(&dp, cells);
            int q = Action_QuarterTurns((ActionType)act.type);
            ok = ok && Pivot((q == 1) ? -90.0f : (q == -1) ? 90.0f : 180.0f);
            heading = Dir_Turn(heading, q);
            MazePos_Step(pos, heading, &pos);
            cells = 1;
        }
    }
    if (ok) ok = TurnBack(&dp, pos, heading); // ゴールの真ん中で 180°(尻当てあり)
    if (ok) {
        LED_SetDirectPattern(LED_DIRECT_ALL);
        DelayWatching(SEARCH_GOAL_WAIT_MS);
        LED_SetDirectPattern(LedIdlePattern());
    } else {
        Ev(LOG_EV_TIMEOUT, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        App_SetWallPush(false);
        App_SetTargetVelocity(0.0f);
        DelayWatching(300);
        printf("fast run: timeout\r\n");
    }
    App_ControlLoop_SetEnabled(false);
    return ok;
}

// 最短走行の準備: 探索で flash に残した地図を s_map に読み、分かっている壁だけでスタートからゴールまでの経路を
// 計算し、小回りだけ・大回りありの指令の列も作る(maze_sim の small / large と同じ)。今の s_fast_v /
// s_fast_small_v を使う。地図や経路がなければ false。
static bool PrepareFastRoute(const MazePos *goals, uint8_t goal_count) {
    if (!Flash_ReadUserData(&s_map, sizeof(s_map))) {
        printf("no map in flash. run SEARCH first.\r\n");
        return false;
    }
    MazePos start = { MAZE_START_X, MAZE_START_Y };
    MazeSolver *solver = &s_planner.work.solver; // 探索はしないので、プランナーの作業領域を借りる
    Dijkstra_Compute(solver, &s_map, WALL_VIEW_KNOWN, NULL, goals, goal_count);
    bool route_ok = Dijkstra_BuildRoute(solver, start, DIR_NORTH, true, &s_route);
    MazePrint_Map(&s_map, solver, &start, DIR_NORTH, goals, goal_count);
    if (!route_ok) {
        printf("no route to the goal with known walls.\r\n");
        return false;
    }
    s_route_cost = (float)Dijkstra_Cost(solver, start, DIR_NORTH);
    printf("route (cost %.0f):\r\n", s_route_cost);
    CommandList_Print(&s_route);

    RunProfile prof = ComputeFastTurns();
    s_run_small_ok = RunPath_FromRoute(&s_route, &prof, false, &s_run_small);
    s_run_large_ok = RunPath_FromRoute(&s_route, &prof, true, &s_run_large);
    s_route_est_by_type[1] = PrintRunList("small", &s_run_small, s_run_small_ok, &prof);
    s_route_est_by_type[2] = PrintRunList("large", &s_run_large, s_run_large_ok, &prof);
    return true;
}

void FastRun_Run(void) {
    const MazePos *goals;
    uint8_t goal_count = GetGoals(&goals);

    // 直進の最高速度・加速度・小回り(スラローム)の速さ・走り方を選ぶ(大回りは FAST_LARGE* のまま。電源を切るまでそのまま)。
    // 加速度は、最高速度に合う値(FAST_ACCEL_FOR_SPEED_MM_S2)から始める(回さずに決めればその値)。
    // 減速度は加速度から決める(FAST_DECEL_MAX_MM_S2 まで)
    s_fast_v = ModeUI_SelectValue("SPEED", "mm/s", kFastSpeeds,
                                  (uint8_t)(sizeof(kFastSpeeds) / sizeof(kFastSpeeds[0])), FAST_V_MM_S);
    SetFastAccelForSpeed();
    const uint8_t na = (uint8_t)(sizeof(kAccels) / sizeof(kAccels[0]));
    uint8_t a_start = 0;
    for (uint8_t i = 0; i < na; i++) {
        if (kAccels[i] <= s_fast_accel) a_start = i; // 表の値以下で一番近いもの
    }
    s_fast_accel = ModeUI_SelectValueFrom("ACCEL", "mm/s2", kAccels, na, a_start, s_fast_accel);
    s_fast_decel = 0.0f; // 加速度と FAST_DECEL_MAX_MM_S2 の小さい方
    s_fast_small_v = ModeUI_SelectValue("SMALL TURN", "mm/s", kFastSmallSpeeds,
                                        (uint8_t)(sizeof(kFastSmallSpeeds) / sizeof(kFastSmallSpeeds[0])),
                                        FAST_SMALL_V_MM_S);
    // 走り方: 1 SMALL、2 LARGE、3 PIVOT(下の type は 0 PIVOT、1 SMALL、2 LARGE)
    static const float kTurnValues[] = { 1.0f, 2.0f, 3.0f };
    float turn = ModeUI_SelectValue("TURN(1 SMALL 2 LARGE 3 PIVOT)", "", kTurnValues, 3u, 1.0f);
    uint8_t type = (turn == 3.0f) ? 0u : (turn == 2.0f) ? 2u : 1u;
    printf("FAST RUN: v=%.0f mm/s accel=%.0f decel=%.0f, small turn %.0f mm/s\r\n",
           s_fast_v, s_fast_accel, FastDecel(), s_fast_small_v);
    PrintSettings(goals, goal_count);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    if (!PrepareFastRoute(goals, goal_count)) {
        ModeUI_WaitClickBlinking(LED_DIRECT_ALL); // 地図・経路がないことを全部の LED の点滅で知らせる
        while (1) {
            if (FailSafe_IsTripped()) FailSafe_Halt();
            HAL_Delay(10);
        }
    }

    static const char *const kTypes[] = { "PIVOT", "SMALL", "LARGE" };
    if ((type == 1 && !s_run_small_ok) || (type == 2 && !s_run_large_ok)) {
        printf("%s: no run list. reset and choose another type\r\n", kTypes[type]);
        ModeUI_WaitClickBlinking(LED_DIRECT_ALL);
        while (1) {
            if (FailSafe_IsTripped()) FailSafe_Halt();
            HAL_Delay(10);
        }
    }
    while (1) {
        // 手かざしで走り出す(終わったら、同じ設定で次の手かざしを待つ。クリックは無視する)
        printf("put in the start cell (facing north). run type: %s  (hand: START)\r\n", kTypes[type]);
        LED_SetShiftPattern(0x0000u);
        while (ModeUI_WaitHandStartOrClick()) {
        }

        s_run_kind = 1.0f; // 最短走行の走り方
        s_run_type = (float)type;
        s_route_count = (type == 0) ? s_route.count : (type == 1) ? s_run_small.count : s_run_large.count;
        s_route_est = s_route_est_by_type[type];
        PrepareStart("fast");
        bool done = (type == 0) ? RunFast()
                  : (type == 1) ? RunFastSlalom(&s_run_small)
                                : RunFastSlalom(&s_run_large);
        LogMap(); // 走った地図(flash から読んだもの)
        EndRunLog();
        printf("fast run (%s) %s\r\n", kTypes[type], done ? "finished" : "stopped");
    }
}

// ---- 最短走行の後の帰り道 ----
// 最短走行でゴールの真ん中に着いて 180° 回った後(pos / heading はその区画と今の向き)、探索の帰りと同じく
// スタートへ戻る。地図は最短走行に使った s_map(flash から読んだもの)。走りながら見た壁も書き足す(flash には残さない)。
// 速さは今の s_search_v / s_search_turn_v。スタートに着いて 180° 回ったら(北向き)true。
static bool ReturnToStart(MazePos pos, Direction heading, const MazePos *goals, uint8_t goal_count) {
    MazePos start = { MAZE_START_X, MAZE_START_Y };
    SearchPlanner_Init(&s_planner, &s_map, SEARCH_ALGO_DIJKSTRA, start, DIR_NORTH, goals, goal_count);
    s_planner.pos = pos;
    s_planner.heading = heading;
    s_planner.phase = SEARCH_PHASE_TO_START;
    s_event_count = 0;
    s_search_slalom = true;

    DrivePos dp;
    StartRunBegin(&dp); // 尻当てはしない(ゴールで 180° 回ったときに済ませている)
    WallObservation obs = WallsFromMap(pos, heading);
    WallSensorValues sv = { 0, 0, 0, 0 };
    return SearchLoop(&dp, obs, sv);
}

// 最短走行を1回走り(type 1: SMALL、2: LARGE。今の s_fast_v / s_fast_accel / s_fast_small_v で)、ゴールから
// スタートへ自分で戻る(長い走行・最短走行の連続のモード用)。経路は毎回作り直す(小回りの速さで変わるため)。
// それぞれ別のログ(fast / back)。うまくいかなければ *why に理由を書いて false。
static bool FastRunAndReturn(uint8_t type, const MazePos *goals, uint8_t goal_count, const char **why) {
    if (!PrepareFastRoute(goals, goal_count)) {
        *why = "no route";
        return false;
    }
    const RunList *list = (type == 1u) ? &s_run_small : &s_run_large;
    if ((type == 1u && !s_run_small_ok) || (type == 2u && !s_run_large_ok)) {
        *why = "no run list";
        return false;
    }
    s_run_kind = 1.0f;
    s_run_type = (float)type;
    s_route_count = list->count;
    s_route_est = s_route_est_by_type[type];
    printf("fast: v=%.0f accel %.0f decel %.0f small turn %.0f mm/s, %s\r\n", s_fast_v, s_fast_accel, FastDecel(),
           s_fast_small_v, (type == 1u) ? "SMALL" : "LARGE");
    PrepareStart("fast");
    bool done = RunFastSlalom(list);
    LogMap();
    EndRunLog();
    if (!done) {
        *why = "fast run stopped";
        return false;
    }

    MazePos pos;
    Direction heading;
    RouteEnd(&pos, &heading); // ゴールの区画と、着いたときの向き(180° 回ったので今は逆向き)
    s_search_v = LONG_LOG_RETURN_V_MM_S;
    s_search_turn_v = LONG_LOG_RETURN_V_MM_S;
    ComputeSlalomOffsets();
    s_run_kind = 2.0f; // 帰り道
    s_run_type = 1.0f;
    HAL_Delay(LONG_LOG_PAUSE_MS);
    PrepareStart("back");
    bool back = ReturnToStart(pos, Dir_Opposite(heading), goals, goal_count);
    LogMap();
    EndRunLog();
    if (!back) {
        *why = "return stopped";
        return false;
    }
    return true;
}

// ---- 長い走行(ログを取るための試験のモード)----
// 探索を、直進の速さ × スラロームの速さの組み合わせ(LONG_LOG_SEARCH_V_MM_S × SPEED_SELECT_SEARCH_TURN_V_MM_S)で
// LONG_LOG_REPEAT 回ずつ走り、次に最短走行を、小回り(SMALL)を小回りの速さ(SPEED_SELECT_FAST_SMALL_V_MM_S)ごとに、
// 大回り(LARGE)を1通り、LONG_LOG_REPEAT 回ずつ走る(直進は FAST_V_MM_S)。最短走行の後はゴールからスタートへ自分で戻る。
// 置き直さずに続ける(探索はスタートに戻って北を向いて終わる)。1回ごとに別のログになる(search / fast / back)。
// 走る順は「段」と「その中の番号」で数える(電池を替えた後に続きから始めるため。選ぶ画面の LED は 15 個まで):
//   段 1〜4: 探索(直進の速さが 1〜4 番目)。中の番号はスラロームの速さ × くり返し
//   段 5: 最短走行の小回り(小回りの速さ × くり返し)、段 6: 最短走行の大回り(くり返し)
// 走る前に、止まった状態の電池の電圧が LONG_LOG_MIN_VBAT_V より低ければ、次に走る段と番号を LED で出して止まる。

typedef enum { LONG_SEARCH = 0, LONG_FAST_SMALL, LONG_FAST_LARGE } LongKind;

typedef struct {
    LongKind kind;
    float v;      // 探索: 直進の速さ、最短走行: 直進の最高速度
    float v_turn; // 探索: スラロームの速さ、最短走行: 小回りの速さ
    uint8_t part; // 段(1〜)
    uint8_t step; // 段の中の番号(1〜)
} LongRun;

#define COUNT_OF(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))
#define LONG_PART_COUNT (COUNT_OF(kLongSearchSpeeds) + 2u)
#define LONG_RUN_MAX (COUNT_OF(kLongSearchSpeeds) * COUNT_OF(kSearchTurnSpeeds) * LONG_LOG_REPEAT + \
                      COUNT_OF(kFastSmallSpeeds) * LONG_LOG_REPEAT + LONG_LOG_REPEAT)
_Static_assert(COUNT_OF(kSearchTurnSpeeds) * LONG_LOG_REPEAT <= 15u, "too many steps in a part for the LED display");
_Static_assert(COUNT_OF(kFastSmallSpeeds) * LONG_LOG_REPEAT <= 15u, "too many steps in a part for the LED display");

static LongRun s_long_runs[LONG_RUN_MAX];

static uint16_t BuildLongRuns(void) {
    uint16_t n = 0;
    for (uint8_t i = 0; i < COUNT_OF(kLongSearchSpeeds); i++) {
        uint8_t step = 1;
        for (uint8_t j = 0; j < COUNT_OF(kSearchTurnSpeeds); j++) {
            for (uint8_t k = 0; k < LONG_LOG_REPEAT; k++) {
                s_long_runs[n++] = (LongRun){ LONG_SEARCH, kLongSearchSpeeds[i], kSearchTurnSpeeds[j], (uint8_t)(i + 1u), step++ };
            }
        }
    }
    uint8_t part = (uint8_t)(COUNT_OF(kLongSearchSpeeds) + 1u);
    uint8_t step = 1;
    for (uint8_t j = 0; j < COUNT_OF(kFastSmallSpeeds); j++) {
        for (uint8_t k = 0; k < LONG_LOG_REPEAT; k++) {
            s_long_runs[n++] = (LongRun){ LONG_FAST_SMALL, FAST_V_MM_S, kFastSmallSpeeds[j], part, step++ };
        }
    }
    part++;
    step = 1;
    for (uint8_t k = 0; k < LONG_LOG_REPEAT; k++) {
        s_long_runs[n++] = (LongRun){ LONG_FAST_LARGE, FAST_V_MM_S, FAST_SMALL_V_MM_S, part, step++ };
    }
    return n;
}

// 段と番号を、シフトレジスタの LED の棒グラフで交互に出し続ける(戻らない)。何かで止まったときに使う。
// 段を出している間は直結の LED も全部点け、番号と見分けられるようにする。
static void LongLogHalt(const char *why, uint8_t part, uint8_t step) {
    App_ControlLoop_SetEnabled(false);
    printf("LONG LOG stopped (%s). next: part %u step %u\r\n", why, part, step);
    while (1) {
        LED_SetDirectPattern(LED_DIRECT_ALL);
        LED_SetShiftPattern((uint16_t)((1u << part) - 1u));
        for (int i = 0; i < 100; i++) {
            if (FailSafe_IsTripped()) FailSafe_Halt();
            HAL_Delay(10);
        }
        LED_SetDirectPattern(0x00u);
        LED_SetShiftPattern(0x0000u);
        HAL_Delay(300);
        LED_SetShiftPattern((uint16_t)((1u << step) - 1u));
        for (int i = 0; i < 60; i++) {
            if (FailSafe_IsTripped()) FailSafe_Halt();
            HAL_Delay(10);
        }
        LED_SetShiftPattern(0x0000u);
        HAL_Delay(600);
    }
}

void LongLogRun_Run(void) {
    const MazePos *goals;
    uint8_t goal_count = GetGoals(&goals);
    uint16_t count = BuildLongRuns();

    // どこから始めるか(段と番号)を選ぶ
    static float part_values[LONG_PART_COUNT];
    for (uint8_t i = 0; i < LONG_PART_COUNT; i++) part_values[i] = (float)(i + 1u);
    uint8_t part = (uint8_t)ModeUI_SelectValue("PART", "", part_values, LONG_PART_COUNT, 1.0f);
    uint8_t steps_in_part = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (s_long_runs[i].part == part) steps_in_part++;
    }
    static float step_values[15];
    for (uint8_t i = 0; i < steps_in_part && i < 15u; i++) step_values[i] = (float)(i + 1u);
    uint8_t step = (uint8_t)ModeUI_SelectValue("STEP", "", step_values, steps_in_part, 1.0f);
    uint16_t first = 0;
    while (first < count && !(s_long_runs[first].part == part && s_long_runs[first].step == step)) first++;
    printf("LONG LOG: %u runs, start at #%u (part %u step %u). put in the start cell (facing north), hand: START\r\n",
           count, first + 1u, part, step);
    PrintSettings(goals, goal_count);
    if (FailSafe_IsTripped()) FailSafe_Halt();

    LED_SetShiftPattern(0x0000u);
    while (ModeUI_WaitHandStartOrClick()) {
        // 最初の1回だけ手かざしで始める(あとは置き直さずに続ける)。クリックは無視する
    }

    for (uint16_t i = first; i < count; i++) {
        const LongRun *r = &s_long_runs[i];
        // 止まった状態の電池の電圧(フィルタ済み)を見てから走る
        HAL_Delay(LONG_LOG_PAUSE_MS);
        float vbat = FailSafe_GetFilteredVoltage();
        printf("---- LONG LOG #%u/%u (part %u step %u): vbat %.2f V\r\n", i + 1u, count, r->part, r->step, vbat);
        if (vbat < LONG_LOG_MIN_VBAT_V) LongLogHalt("low battery", r->part, r->step);

        s_long_index = (uint16_t)(i + 1u);
        s_long_part = r->part;
        s_long_step = r->step;
        s_long_v = r->v;
        s_long_v_turn = r->v_turn;

        if (r->kind == LONG_SEARCH) {
            s_search_v = r->v;
            s_search_turn_v = r->v_turn;
            ComputeSlalomOffsets();
            s_search_slalom = true;
            s_run_kind = 0.0f;
            s_run_type = 1.0f;
            printf("search: v=%.0f slalom %.0f mm/s\r\n", s_search_v, s_search_turn_v);
            PrepareStart("search");
            bool done = RunSearch(SEARCH_ALGO_DIJKSTRA, goals, goal_count);
            FinishSearchRun(done, goals, goal_count);
            if (!done) LongLogHalt("search stopped", r->part, r->step);
            continue;
        }

        // 最短走行: 走ってから、ゴールからスタートへ戻る
        s_fast_v = r->v;
        s_fast_accel = FAST_ACCEL_MM_S2;
        s_fast_small_v = r->v_turn;
        const char *why = NULL;
        if (!FastRunAndReturn((r->kind == LONG_FAST_SMALL) ? 1u : 2u, goals, goal_count, &why)) {
            LongLogHalt(why, r->part, r->step);
        }
    }
    s_long_index = 0;
    printf("LONG LOG: all runs done\r\n");
    LongLogHalt("all done", (uint8_t)(LONG_PART_COUNT + 1u), 1u);
}

// ---- 最短走行の連続(速さ・加速度・小回りの速さを変えて続けて走る)----
// 探索で flash に残した地図で、直進の速さ・加速度・小回りの速さの範囲(FROM / TO)と走り方(SMALL / LARGE / 両方)を選び、
// 1回の手かざしで全部の組み合わせを走る。1本ごとにゴールからスタートへ自分で戻る(FastRunAndReturn)。
// 並びは 小回りの速さ → 加速度 → 直進の速さ → 走り方 の順に入れ子(内側ほど先に変わる)で、FAST_SWEEP_REPEAT 回ずつ。
// 打ち切り・電池の低下・全部終わったら、何本目かを LED の棒グラフで点滅させて止まる。

static uint8_t IndexOfValue(const float *values, uint8_t count, float v) {
    for (uint8_t i = 0; i < count; i++) {
        if (values[i] == v) return i;
    }
    return 0;
}

static void FastSweepHalt(const char *why, uint16_t no) {
    App_ControlLoop_SetEnabled(false);
    printf("FAST SWEEP stopped (%s) at #%u\r\n", why, no);
    uint16_t bar = (no > 15u) ? 15u : no;
    while (1) {
        LED_SetShiftPattern((uint16_t)((1u << bar) - 1u));
        for (int i = 0; i < 50; i++) {
            if (FailSafe_IsTripped()) FailSafe_Halt();
            HAL_Delay(10);
        }
        LED_SetShiftPattern(0x0000u);
        HAL_Delay(300);
    }
}

void FastSweep_Run(void) {
    const MazePos *goals;
    uint8_t goal_count = GetGoals(&goals);
    const uint8_t nv = COUNT_OF(kFastSpeeds);
    const uint8_t ns = COUNT_OF(kFastSmallSpeeds);
    // 加速度・減速度は選ばない(直進の最高速度から決まる。SetFastAccelForSpeed)
    uint8_t v0 = IndexOfValue(kFastSpeeds, nv, ModeUI_SelectValue("SPEED FROM", "mm/s", kFastSpeeds, nv, kFastSpeeds[0]));
    uint8_t v1 = IndexOfValue(kFastSpeeds, nv, ModeUI_SelectValue("SPEED TO", "mm/s", kFastSpeeds, nv, kFastSpeeds[0]));
    uint8_t s0 = IndexOfValue(kFastSmallSpeeds, ns,
                              ModeUI_SelectValue("SMALL FROM", "mm/s", kFastSmallSpeeds, ns, kFastSmallSpeeds[0]));
    uint8_t s1 = IndexOfValue(kFastSmallSpeeds, ns,
                              ModeUI_SelectValue("SMALL TO", "mm/s", kFastSmallSpeeds, ns, kFastSmallSpeeds[0]));
    static const float kTypeValues[] = { 1.0f, 2.0f, 3.0f }; // 1 SMALL、2 LARGE、3 両方
    uint8_t types = (uint8_t)ModeUI_SelectValue("TYPE(1 SMALL 2 LARGE 3 BOTH)", "", kTypeValues, 3u, 1.0f);
    if (v1 < v0) v1 = v0;
    if (s1 < s0) s1 = s0;
    uint8_t t0 = (types == 2u) ? 2u : 1u;
    uint8_t t1 = (types == 1u) ? 1u : 2u;
    uint16_t total = (uint16_t)((v1 - v0 + 1u) * (s1 - s0 + 1u) * (t1 - t0 + 1u) * FAST_SWEEP_REPEAT);
    printf("FAST SWEEP: speed %.0f..%.0f (accel/decel from speed), small %.0f..%.0f, type %u..%u, x%u = %u runs\r\n",
           kFastSpeeds[v0], kFastSpeeds[v1], kFastSmallSpeeds[s0], kFastSmallSpeeds[s1], t0, t1, FAST_SWEEP_REPEAT,
           total);
    PrintSettings(goals, goal_count);
    if (FailSafe_IsTripped()) FailSafe_Halt();
    // 地図があるかを先に確かめる(なければ走らずに止まる)
    if (!PrepareFastRoute(goals, goal_count)) FastSweepHalt("no map / no route", 0);

    printf("put in the start cell (facing north). hand: START\r\n");
    LED_SetShiftPattern(0x0000u);
    while (ModeUI_WaitHandStartOrClick()) {
        // 最初の1回だけ手かざしで始める(あとは置き直さずに続ける)。クリックは無視する
    }

    uint16_t no = 0;
    for (uint8_t si = s0; si <= s1; si++) {
        for (uint8_t vi = v0; vi <= v1; vi++) {
            for (uint8_t t = t0; t <= t1; t++) {
                for (uint8_t rep = 0; rep < FAST_SWEEP_REPEAT; rep++) {
                    no++;
                    HAL_Delay(LONG_LOG_PAUSE_MS);
                    float vbat = FailSafe_GetFilteredVoltage();
                    printf("---- FAST SWEEP #%u/%u: vbat %.2f V\r\n", no, total, vbat);
                    if (vbat < LONG_LOG_MIN_VBAT_V) FastSweepHalt("low battery", no);
                    s_fast_v = kFastSpeeds[vi];
                    SetFastAccelForSpeed();
                    s_fast_small_v = kFastSmallSpeeds[si];
                    const char *why = NULL;
                    if (!FastRunAndReturn(t, goals, goal_count, &why)) FastSweepHalt(why, no);
                }
            }
        }
    }
    printf("FAST SWEEP: all done\r\n");
    FastSweepHalt("all done", no);
}

// ---- 速度帯の最短走行(RUN の FAST_BANDS)----
// params.h の FAST_BANDS(直進の速さ・加速度・減速度・小回りの速さ・走り方の組)から、始めと終わりの速度帯を選び、
// 1回の手かざしで、遅い速度帯から順に「最短走行 → ゴールからスタートへ自分で戻る」を FAST_BAND_REPEAT 回ずつ走る。
// ログ取り(範囲を選ぶ)にも、本番(始めと終わりに同じ速度帯を選ぶ)にも使う。止まり方・LED は FAST_SWEEP と同じ。

typedef struct {
    float v;       // 直進の最高速度
    float accel;   // 直進の加速度
    float decel;   // 直進の減速度
    float small_v; // 小回りの速さ
    uint8_t type;  // 1 SMALL、2 LARGE
} FastBand;

static const FastBand kFastBands[] = FAST_BANDS;
_Static_assert(COUNT_OF(kFastBands) <= 15u, "too many fast bands for the LED display");

void FastBands_Run(void) {
    const MazePos *goals;
    uint8_t goal_count = GetGoals(&goals);
    const uint8_t nb = COUNT_OF(kFastBands);
    static float band_values[COUNT_OF(kFastBands)];
    for (uint8_t i = 0; i < nb; i++) band_values[i] = (float)(i + 1u);
    for (uint8_t i = 0; i < nb; i++) {
        const FastBand *b = &kFastBands[i];
        printf("band %u: v %.0f accel %.0f decel %.0f small %.0f %s\r\n", i + 1u, b->v, b->accel, b->decel,
               b->small_v, (b->type == 1u) ? "SMALL" : "LARGE");
    }
    uint8_t b0 = (uint8_t)ModeUI_SelectValue("BAND FROM", "", band_values, nb, 1.0f);
    uint8_t b1 = (uint8_t)ModeUI_SelectValue("BAND TO", "", band_values, nb, 1.0f);
    if (b1 < b0) b1 = b0;
    s_band_total = (uint16_t)((b1 - b0 + 1u) * FAST_BAND_REPEAT);
    printf("FAST BANDS: band %u..%u, x%u = %u runs\r\n", b0, b1, FAST_BAND_REPEAT, s_band_total);
    PrintSettings(goals, goal_count);
    if (FailSafe_IsTripped()) FailSafe_Halt();
    if (!PrepareFastRoute(goals, goal_count)) FastSweepHalt("no map / no route", 0);

    printf("put in the start cell (facing north). hand: START\r\n");
    LED_SetShiftPattern(0x0000u);
    while (ModeUI_WaitHandStartOrClick()) {
        // 最初の1回だけ手かざしで始める(あとは置き直さずに続ける)。クリックは無視する
    }

    s_band_no = 0;
    for (uint8_t b = b0; b <= b1; b++) {
        const FastBand *band = &kFastBands[b - 1u];
        for (uint8_t rep = 0; rep < FAST_BAND_REPEAT; rep++) {
            s_band_no++;
            HAL_Delay(LONG_LOG_PAUSE_MS);
            float vbat = FailSafe_GetFilteredVoltage();
            printf("---- FAST BANDS #%u/%u: band %u, vbat %.2f V\r\n", s_band_no, s_band_total, b, vbat);
            if (vbat < LONG_LOG_MIN_VBAT_V) FastSweepHalt("low battery", s_band_no);
            s_band = b;
            s_fast_v = band->v;
            s_fast_accel = band->accel;
            s_fast_decel = band->decel;
            s_fast_small_v = band->small_v;
            const char *why = NULL;
            if (!FastRunAndReturn(band->type, goals, goal_count, &why)) FastSweepHalt(why, s_band_no);
        }
    }
    printf("FAST BANDS: all done\r\n");
    FastSweepHalt("all done", s_band_no);
}

// ---- 探索の既知の区間をまとめて走る(直線の加速・大回り)----
// 探索で「まっすぐ進む」と決まったとき(区画 C0 の入口の境界の手前にいて、C0 を抜けて C1 へ)、その先をプランナーの経路
// (Dijkstra の next_dir)でたどる。壁が全部分かっている区画が続く間は、そこで壁を読む必要がないので、まとめて
// 最短走行と同じ指令の列(RunPath_FromRoute)にして走る(直線は s_search_v まで加速、「0 R 0」は大回りに置き換える)。
// 区間の終わりは、まだ分かっていない区画・目的地(ゴールかスタート)の区画 E の入口の境界。壁を読む所(境界の
// SEARCH_WALL_READ_BEFORE_MM 手前)までにスラロームの速さにして、そこで壁を読んで探索に戻る。
// 指令の列は「1つ手前の区画 P の真ん中から」作る(最短走行の部品は区画の真ん中が基準のため。P → C0 の半区画は走り済み)。
// 大回りが区間の最後の区画の真ん中で終わると境界に戻れないので、最後の2区画はまっすぐ進む形になるよう手前で切る。
// 使わなかったら *used = false(呼び出し側がふつうに1区画進む)。走って失敗したら false。

static RunList s_known_list;

static bool TryKnownRun(DrivePos *dp, MazePos c0, Direction h0, WallObservation *obs, WallSensorValues *sv,
                        bool *used) {
    *used = false;
    if (!SEARCH_KNOWN_FAST_ENABLE || dp->at_center || s_spin_cells) return true; // SEARCH_SPIN は区画を飛ばさない

    // 先読みと指令の列は logic 層(maze_sim と同じ計算)。moves[k] は C1 から数えて k 回目に進む向き(moves[0] = C1 に入る向き)
    static Direction moves[MAZE_CELL_COUNT];
    uint16_t m = SearchPlanner_KnownRun(&s_planner, SEARCH_KNOWN_MIN_MOVES, moves);
    if (m == 0u) return true; // 短すぎる・Dijkstra でない(ふつうに1区画ずつ進む)
    bool use_large = SEARCH_KNOWN_LARGE && s_search_v >= FAST_LARGE90_V_MM_S && s_search_v >= FAST_LARGE180_V_MM_S;
    if (!RunPath_FromKnownRun(moves, m, &s_known_prof, use_large, &s_known_list)) return true;

    // 区間の終わりの区画 E と、そこに入る向き
    MazePos e = c0;
    MazePos_Step(e, h0, &e); // C1
    for (uint16_t k = 1; k <= m; k++) MazePos_Step(e, moves[k], &e);

    *used = true;
    Ev(LOG_EV_KNOWN_RUN, (float)(m + 1u), (float)s_known_list.count, use_large ? 1.0f : 0.0f, (float)e.x, (float)e.y);
    float ref = dp->ref_mm - HALF_SECTION_MM; // P の真ん中
    if (!RunList_Drive(&s_known_list, &ref, s_search_turn_v, SEARCH_WALL_READ_BEFORE_MM)) return false;
    dp->ref_mm = ref; // E の入口の境界
    dp->at_center = false;
    s_planner.pos = e;
    s_planner.heading = moves[m];
    *obs = ReadWalls(sv);
    return true;
}
