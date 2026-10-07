#include "app/search_run.h"

#include <math.h>
#include "main.h"
#include "app/mode_ui.h"
#include "app/control_loop.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "interface/sdcard.h"
#include "interface/flash.h"
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
static float s_search_v = SEARCH_V_MM_S; // 探索の直進・小回りの速さ
static float s_fast_v = FAST_V_MM_S;     // 最短走行の直進の最高速度
static const float kSearchSpeeds[] = SPEED_SELECT_SEARCH_V_MM_S;
static const float kFastSpeeds[] = SPEED_SELECT_FAST_V_MM_S;
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
// WALL_EDGE_POS_MM 過ぎた所で起きるはずなので、一番近い境界からのずれを補正の量にする。
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
    // 一番近い境界(boundary0 + 1区画 × k)を選ぶ
    float k = floorf((pos - WALL_EDGE_POS_MM - ec->boundary0) / SECTION_MM + 0.5f);
    if (k < 0.0f) k = 0.0f;
    float expected = ec->boundary0 + k * SECTION_MM + WALL_EDGE_POS_MM;
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

// 目標の距離が *target に届くまで待つ。途中の壁切れで *target を直す(探索の1区画の直進用。
// 速さは一定のまま走っているので、プロファイルは直さず、待つ所だけを変える)。
static bool WaitTargetDistanceEdge(float *target, EdgeCorr *ec) {
    uint32_t t0 = HAL_GetTick();
    while (App_GetTargetDistance() < *target) {
        CheckFailSafe();
        PollLog();
        float c;
        if (EdgeCorr_Check(ec, &c)) *target += c;
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

// 探索の速さで直進する
static void StartStraightTo(float target_mm, float v_end) {
    StartStraightToV(target_mm, s_search_v, v_end, SEARCH_ACCEL_MM_S2);
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

// 次の境界まで進み、着いたら壁を読む(真ん中からなら半区画加速 = BlueEyes の half_sectionA、
// 境界からなら1区画 = one_sectionU)。
static bool GoToNextBoundary(DrivePos *dp, uint8_t cells, WallObservation *obs, WallSensorValues *sv) {
    // 壁の制御は境界から境界までの1区画だけ(真ん中からの半区画加速では使わない)
    App_SetWallControl(!dp->at_center);
    bool from_boundary = !dp->at_center;
    float boundary0 = dp->ref_mm; // 境界から走るなら、今いる境界
    dp->ref_mm += dp->at_center ? HALF_SECTION_MM + SECTION_MM * (float)(cells - 1)
                                : SECTION_MM * (float)cells;
    StartStraightTo(dp->ref_mm, s_search_v);
    if (from_boundary) {
        // 境界から境界へ走る間は、壁切れで次の境界の位置を直す
        EdgeCorr ec;
        EdgeCorr_Begin(&ec, boundary0);
        if (!WaitTargetDistanceEdge(&dp->ref_mm, &ec)) return false;
    } else {
        if (!WaitTargetDistance(dp->ref_mm)) return false;
    }
    *obs = ReadWalls(sv);
    dp->at_center = false;
    return true;
}

// ---- スラローム(小回り 90°)----
// 境界で壁を読んで「曲がる」と決まったら、止まらずに 前のオフセット → 曲がる → 後ろのオフセット で
// 隣の区画の境界へ進み、着いたら壁を読む。オフセットは SearchRun_Run の最初に計算する。
static float s_slalom_pre_mm = 0.0f;
static float s_slalom_post_mm = 0.0f;
static bool s_search_slalom = true; // 探索で曲がるとき、スラローム(true)か超信地旋回(false)か。走る前に選ぶ

static float s_slalom_omega_dps = SLALOM_OMEGA_DPS; // 選んだ速さに合わせた角速度・角加速度
static float s_slalom_alpha_dps2 = SLALOM_ALPHA_DPS2;

static void ComputeSlalomOffsets(void) {
    SlalomParams p = {
        .v_mm_s = SLALOM_V_MM_S,
        .omega_dps = SLALOM_OMEGA_DPS,
        .alpha_dps2 = SLALOM_ALPHA_DPS2,
        .angle_deg = 90.0f,
    };
    Slalom_ScaleToSpeed(&p, s_search_v); // 選んだ速さでも同じ形で曲がる
    s_slalom_omega_dps = p.omega_dps;
    s_slalom_alpha_dps2 = p.alpha_dps2;
    SlalomShape sh = Slalom_ComputeShape(&p);
    float pre, post;
    Slalom_Turn90Offsets(&sh, HALF_SECTION_MM, &pre, &post);
    s_slalom_pre_mm = pre + SLALOM_PRE_ADJ_MM;
    s_slalom_post_mm = post + SLALOM_POST_ADJ_MM;
    printf("slalom: v=%.0f omega=%.0f alpha=%.0f -> pre %.1f post %.1f mm\r\n",
           p.v_mm_s, p.omega_dps, p.alpha_dps2, s_slalom_pre_mm, s_slalom_post_mm);
}

// 境界にいる(走っている)ときに呼ぶ。right: 右(時計回り)に曲がる。
// 前壁補正: 曲がる区画の奥に壁があれば、距離で決めた曲がり始めの位置(start)の前後
// SLALOM_FRONT_WINDOW_MM の間で、FL + FR が SLALOM_FRONT_REF_SUM に届いた瞬間まで待つ。
// 届かなければ範囲の終わりまで待つ。前に壁がなければ start まで待つ。打ち切ったら false。
static bool WaitSlalomStart(float start, bool front_wall) {
    if (!SLALOM_FRONT_ENABLE || !front_wall) {
        StartStraightTo(start, s_search_v);
        return WaitTargetDistance(start);
    }
    float lo = start - SLALOM_FRONT_WINDOW_MM;
    float hi = start + SLALOM_FRONT_WINDOW_MM;
    StartStraightTo(hi, s_search_v); // 範囲の終わりまで同じ速さで進めておく
    if (!WaitTargetDistance(lo)) return false;
    uint32_t t0 = HAL_GetTick();
    bool by_sensor = false;
    uint32_t sum = 0;
    while (App_GetTargetDistance() < hi) {
        CheckFailSafe();
        PollLog();
        sum = (uint32_t)ad_fl + ad_fr;
        if (sum >= SLALOM_FRONT_REF_SUM) {
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
    StartStraightTo(dp->ref_mm, s_search_v);
    if (!WaitTargetDistance(dp->ref_mm)) return false;
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

// 走り出す前の準備(探索・最短走行で共通): 制御を有効にし、記録を始め、start_sequence で真ん中に合わせる。
static bool StartRun(DrivePos *dp) {
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
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);

    dp->ref_mm = 0.0f;
    dp->at_center = true;
    return StartSequence(dp);
}

// 1回の探索(スタート → ゴール → スタート)。打ち切ったら false。
static bool RunSearch(SearchAlgo algo, const MazePos *goals, uint8_t goal_count) {
    MazePos start = { MAZE_START_X, MAZE_START_Y };
    WallMap_Init(&s_map);
    SearchPlanner_Init(&s_planner, &s_map, algo, start, DIR_NORTH, goals, goal_count);
    s_event_count = 0;

    DrivePos dp;
    if (!StartRun(&dp)) return false;

    // スタート区画は、決まりで左右に壁があり前は開いている
    // (BlueEyes はセンサーで読んで前の壁を消しているが、yuho の斜め前のセンサーは真ん中では
    //  隣の区画の入り口を見ているので、スタート区画の壁は決まりで入れる)
    WallObservation obs = { .front = false, .right = true, .left = true };
    WallSensorValues sv = { 0, 0, 0, 0 };

    while (1) {
        MazePos pos = s_planner.pos;
        Direction heading = s_planner.heading;
        uint32_t t0 = HAL_GetTick();
        SearchPhase phase_before = s_planner.phase;
        Action act = SearchPlanner_Step(&s_planner, obs);
        Record(pos, heading, obs, sv, act, HAL_GetTick() - t0);
        if (s_planner.phase != phase_before) Ev(LOG_EV_PHASE, (float)s_planner.phase, 0.0f, 0.0f, 0.0f, 0.0f);

        bool ok = true;
        switch ((ActionType)act.type) {
            case ACTION_FORWARD:
                ok = GoToNextBoundary(&dp, act.cells, &obs, &sv);
                break;
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

void SearchRun_Run(SearchAlgo algo) {
    const MazePos *goals;
    uint8_t goal_count = GetGoals(&goals);

    // 探索の速さを選ぶ(直進と小回り。電源を切るまでそのまま)
    s_search_v = ModeUI_SelectValue("SPEED", "mm/s", kSearchSpeeds,
                                    (uint8_t)(sizeof(kSearchSpeeds) / sizeof(kSearchSpeeds[0])), SEARCH_V_MM_S);
    printf("SEARCH (%s): v=%.0f mm/s accel=%.0f\r\n",
           (algo == SEARCH_ALGO_ADACHI) ? "adachi" : "dijkstra", s_search_v, SEARCH_ACCEL_MM_S2);
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

        PrepareStart("search");
        bool done = RunSearch(algo, goals, goal_count);

        // ゴールまで行けていれば、地図を flash に残す(最短走行のモードで使う。BlueEyes の store_map_in_flash)。
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

// 旋回の種類ごとの動き(速さ・角速度・角加速度・角度・前後のオフセット)
typedef struct {
    float v_mm_s;
    float omega_dps;
    float alpha_dps2;
    float angle_deg; // 90 か 180(正の値)
    float pre_mm;
    float post_mm;
} FastTurn;

static FastTurn s_fast_turn[RUN_TYPE_COUNT];

static void SetFastTurn(RunType r, RunType l, SlalomParams p, float pre, float post) {
    FastTurn t = { p.v_mm_s, p.omega_dps, p.alpha_dps2, p.angle_deg, pre, post };
    s_fast_turn[r] = t;
    s_fast_turn[l] = t;
}

// 旋回の形を計算し、経路の計算に使う RunProfile も作る
static RunProfile ComputeFastTurns(void) {
    float pre, post;

    SlalomParams small = { SLALOM_V_MM_S, SLALOM_OMEGA_DPS, SLALOM_ALPHA_DPS2, 90.0f };
    SlalomShape sh = Slalom_ComputeShape(&small);
    Slalom_Turn90Offsets(&sh, HALF_SECTION_MM, &pre, &post);
    SetFastTurn(RUN_SMALL90_R, RUN_SMALL90_L, small, pre + SLALOM_PRE_ADJ_MM, post + SLALOM_POST_ADJ_MM);

    SlalomParams l90 = { FAST_LARGE90_V_MM_S, FAST_LARGE90_OMEGA_DPS, FAST_LARGE90_ALPHA_DPS2, 90.0f };
    sh = Slalom_ComputeShape(&l90);
    Slalom_Turn90Offsets(&sh, SECTION_MM, &pre, &post);
    SetFastTurn(RUN_LARGE90_R, RUN_LARGE90_L, l90, pre + FAST_LARGE90_PRE_ADJ_MM, post + FAST_LARGE90_POST_ADJ_MM);

    SlalomParams l180 = { FAST_LARGE180_V_MM_S, 0.0f, FAST_LARGE180_ALPHA_DPS2, 180.0f };
    Slalom_SolveOmegaForSide(&l180, SECTION_MM); // 横にちょうど1区画移る角速度
    sh = Slalom_ComputeShape(&l180);
    Slalom_Turn180Offsets(&sh, SECTION_MM, &pre, &post);
    SetFastTurn(RUN_LARGE180_R, RUN_LARGE180_L, l180, pre + FAST_LARGE180_PRE_ADJ_MM, post + FAST_LARGE180_POST_ADJ_MM);

    static const RunType kTurns[] = { RUN_SMALL90_R, RUN_LARGE90_R, RUN_LARGE180_R };
    static const char *const kNames[] = { "small90", "large90", "large180" };
    for (int i = 0; i < 3; i++) {
        const FastTurn *t = &s_fast_turn[kTurns[i]];
        printf("%-8s: v=%.0f omega=%.0f alpha=%.0f -> pre %.1f post %.1f mm%s\r\n",
               kNames[i], t->v_mm_s, t->omega_dps, t->alpha_dps2, t->pre_mm, t->post_mm,
               (t->pre_mm < 0.0f || t->post_mm < 0.0f) ? "  (WARNING: negative offset)" : "");
    }

    RunProfile prof = RunProfile_Default();
    prof.accel = FAST_ACCEL_MM_S2;
    prof.vmax = s_fast_v;
    for (int t = RUN_SMALL90_R; t <= RUN_LARGE180_L; t++) {
        prof.v_turn[t] = s_fast_turn[t].v_mm_s;
        prof.turn_pre[t] = s_fast_turn[t].pre_mm;
        prof.turn_post[t] = s_fast_turn[t].post_mm;
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
static bool FastStraightTo(float start, float *end, float v_exit, bool at_center) {
    App_SetWallControl(false);
    StartStraightToV(*end, s_fast_v, v_exit, FAST_ACCEL_MM_S2);
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
            StartStraightToV(*end, s_fast_v, v_exit, FAST_ACCEL_MM_S2); // 今の速さから引き直す
        }
        if (v_exit > 0.0f ? (pos >= *end) : App_IsMotionDone()) break;
        if (HAL_GetTick() - t0 > SEARCH_STEP_TIMEOUT_MS) return false;
        if (v_exit <= 0.0f) HAL_Delay(1);
    }
    App_SetWallControl(false);
    return true;
}

static bool RunList_Drive(const RunList *list, float *ref_mm) {
    bool at_center = true; // スタートは区画の中心
    for (uint16_t i = 0; i < list->count; i++) {
        RunType type = (RunType)list->items[i].type;
        Ev(LOG_EV_RUN_CMD, (float)i, (float)type, (float)list->items[i].halves, 0.0f, 0.0f);
        if (type == RUN_STOP) break;

        if (type == RUN_STRAIGHT) {
            float start = *ref_mm;
            float end = start + HALF_SECTION_MM * (float)list->items[i].halves;
            if (!FastStraightTo(start, &end, NextTurnSpeed(list, i), at_center)) return false;
            *ref_mm = end;
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
        StartStraightToV(start, t->v_mm_s, t->v_mm_s, FAST_ACCEL_MM_S2);
        if (!WaitTargetDistance(start)) return false;
        App_StartSlalom(-90.0f * (float)q, t->omega_dps, t->alpha_dps2);
        if (!WaitMotionDone()) return false;
        *ref_mm = App_GetTargetDistance() + t->post_mm;
        StartStraightToV(*ref_mm, t->v_mm_s, t->v_mm_s, FAST_ACCEL_MM_S2);
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
    bool ok = RunList_Drive(list, &ref);
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

void FastRun_Run(void) {
    const MazePos *goals;
    uint8_t goal_count = GetGoals(&goals);

    // 直進の最高速度を選ぶ(曲がるときの速さは SLALOM_* / FAST_LARGE* のまま。電源を切るまでそのまま)
    s_fast_v = ModeUI_SelectValue("SPEED", "mm/s", kFastSpeeds,
                                  (uint8_t)(sizeof(kFastSpeeds) / sizeof(kFastSpeeds[0])), FAST_V_MM_S);
    printf("FAST RUN: v=%.0f mm/s accel=%.0f (run type: PIVOT / SMALL / LARGE)\r\n", s_fast_v, FAST_ACCEL_MM_S2);
    PrintSettings(goals, goal_count);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    // 探索で残した地図を読み、経路を計算する
    if (!Flash_ReadUserData(&s_map, sizeof(s_map))) {
        printf("no map in flash. run SEARCH first.\r\n");
        ModeUI_WaitClickBlinking(LED_DIRECT_ALL); // 地図がないことを全部の LED の点滅で知らせる
        while (1) {
            if (FailSafe_IsTripped()) FailSafe_Halt();
            HAL_Delay(10);
        }
    }
    MazePos start = { MAZE_START_X, MAZE_START_Y };
    MazeSolver *solver = &s_planner.work.solver; // 探索はしないので、プランナーの作業領域を借りる
    Dijkstra_Compute(solver, &s_map, WALL_VIEW_KNOWN, NULL, goals, goal_count);
    bool route_ok = Dijkstra_BuildRoute(solver, start, DIR_NORTH, true, &s_route);
    MazePrint_Map(&s_map, solver, &start, DIR_NORTH, goals, goal_count);
    if (!route_ok) {
        printf("no route to the goal with known walls.\r\n");
        ModeUI_WaitClickBlinking(LED_DIRECT_ALL);
        while (1) {
            if (FailSafe_IsTripped()) FailSafe_Halt();
            HAL_Delay(10);
        }
    }
    s_route_cost = (float)Dijkstra_Cost(solver, start, DIR_NORTH);
    printf("route (cost %.0f):\r\n", s_route_cost);
    CommandList_Print(&s_route);

    // 小回りだけ・大回りありの指令の列も作る(maze_sim の small / large と同じ)
    RunProfile prof = ComputeFastTurns();
    s_run_small_ok = RunPath_FromRoute(&s_route, &prof, false, &s_run_small);
    s_run_large_ok = RunPath_FromRoute(&s_route, &prof, true, &s_run_large);
    s_route_est_by_type[1] = PrintRunList("small", &s_run_small, s_run_small_ok, &prof);
    s_route_est_by_type[2] = PrintRunList("large", &s_run_large, s_run_large_ok, &prof);

    static const char *const kTypes[] = { "PIVOT", "SMALL", "LARGE" };
    uint8_t type = 0;
    while (1) {
        // 走り方を選ぶ: ボタンのクリックで切り替え、手かざしで走り出す(シフトレジスタの LED に番号を出す)
        printf("put in the start cell (facing north). run type: %s  (click: change, hand: START)\r\n", kTypes[type]);
        LED_SetShiftPattern((uint16_t)(1u << type));
        while (ModeUI_WaitHandStartOrClick()) {
            type = (uint8_t)((type + 1u) % 3u);
            printf("run type: %s\r\n", kTypes[type]);
            LED_SetShiftPattern((uint16_t)(1u << type));
        }
        if ((type == 1 && !s_run_small_ok) || (type == 2 && !s_run_large_ok)) {
            printf("%s: no route, use another type\r\n", kTypes[type]);
            continue;
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
