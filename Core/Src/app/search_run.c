#include "app/search_run.h"

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

// ログの列(ISR が読む)。記録の何番目の区画にいるか。
static volatile float s_log_event = 0.0f;
static bool s_log_file = false;   // ログのファイルを開けたか
static bool s_log_failed = false; // 探索の途中でログの保存に失敗したか

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

static void SetupLogger(const char *file_name) {
    const ControlDebug *d = App_GetControlDebug();

    Logger_Init();
    Logger_SetDirName("search");
    Logger_SetFileName(file_name);
    Logger_AddField("event", &s_log_event);
    Logger_AddField("target", &d->target_mm_s);
    Logger_AddField("omega_ref", &d->target_omega_dps);
    Logger_AddField("pos_ref", &d->pos_ref);
    Logger_AddField("dist", &d->dist_mm);
    Logger_AddField("vl", &d->vl);
    Logger_AddField("vr", &d->vr);
    Logger_AddField("angle_ref", &d->angle_ref_deg);
    Logger_AddField("angle", &d->angle_deg);
    Logger_AddField("wall_ofs", &d->wall_offset_deg);
    Logger_AddField("ad_l", &d->ad_l);
    Logger_AddField("ad_fl", &d->ad_fl);
    Logger_AddField("ad_fr", &d->ad_fr);
    Logger_AddField("ad_r", &d->ad_r);
    Logger_SetDecimation(SEARCH_LOG_DECIMATION);
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

static uint32_t s_last_flush_ms = 0; // 前に追記した(または記録を始めた)時刻

// 前の追記から「貯められる時間 − 余裕」を過ぎたか(次に止まったときに追記する)。
static bool LogFlushDue(void) {
    if (!s_log_file) return false;
    uint32_t recordable = Logger_RecordableMs();
    uint32_t limit = (recordable > SEARCH_LOG_FLUSH_MARGIN_MS) ? recordable - SEARCH_LOG_FLUSH_MARGIN_MS : 0;
    return HAL_GetTick() - s_last_flush_ms >= limit;
}

// 止まっている間に、ここまでのログを SD へ追記する。書けたら LED で知らせる。
// 失敗したら以後は書かず、SD の失敗を知らせる LED を点けたままにする。
static void FlushLog(void) {
    if (!s_log_file) return;
    bool ok = Logger_FlushFile();
    // 追記している間は記録を休んでいるので、書き終わった時刻から数える(追記にかかった時間を含めない)
    s_last_flush_ms = HAL_GetTick();
    if (ok) {
        ShowLogSaved();
    } else {
        printf("search: log flush failed\r\n");
        s_log_file = false;
        s_log_failed = true;
        LED_SetDirectPattern(LedIdlePattern());
    }
}

// 止まっているときに呼ぶ。追記の時間が来ていれば追記する。
static void FlushLogIfDue(void) {
    if (LogFlushDue()) FlushLog();
}

// ---- 走行の待ち ----
// フェイルセーフが発動したら止まる(戻らない)。打ち切ったら false。

static void CheckFailSafe(void) {
    if (FailSafe_IsTripped()) {
        App_ControlLoop_SetEnabled(false);
        FailSafe_Halt();
    }
}

// 目標の距離が target_mm に届くまで待つ(区画の境界で壁を読むため、遅れないよう HAL_Delay しない)。
static bool WaitTargetDistance(float target_mm) {
    uint32_t t0 = HAL_GetTick();
    while (App_GetTargetDistance() < target_mm) {
        CheckFailSafe();
        if (HAL_GetTick() - t0 > SEARCH_STEP_TIMEOUT_MS) return false;
    }
    return true;
}

static bool WaitMotionDone(void) {
    uint32_t t0 = HAL_GetTick();
    while (!App_IsMotionDone()) {
        CheckFailSafe();
        if (HAL_GetTick() - t0 > SEARCH_STEP_TIMEOUT_MS) return false;
        HAL_Delay(1);
    }
    return true;
}

static void DelayWatching(uint32_t ms) {
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        CheckFailSafe();
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
    StartStraightToV(target_mm, SEARCH_V_MM_S, v_end, SEARCH_ACCEL_MM_S2);
}

// 真ん中で止まる(既に真ん中にいれば何もしない。BlueEyes の half_sectionD)。追記の時間が来ていれば追記する。
static bool StopAtCenter(DrivePos *dp) {
    if (dp->at_center) return true;
    App_SetWallControl(false); // 半区画では壁の制御を使わない(BlueEyes の half_sectionD と同じ)
    dp->ref_mm += HALF_SECTION_MM;
    StartStraightTo(dp->ref_mm, 0.0f);
    if (!WaitMotionDone()) return false;
    dp->at_center = true;
    FlushLogIfDue();
    return true;
}

static bool Pivot(float angle_deg) {
    DelayWatching(SEARCH_TURN_WAIT_MS);
    App_StartPivot(angle_deg, SEARCH_TURN_OMEGA_DPS, SEARCH_TURN_ALPHA_DPS2);
    if (!WaitMotionDone()) return false;
    DelayWatching(SEARCH_TURN_WAIT_MS);
    FlushLogIfDue();
    return true;
}

// 尻当て(BlueEyes の set_position)。真ん中から後ろの壁に押し当てて向きと位置をそろえ、真ん中へ戻る。
// 押し当てている間は向きと位置の補正を止め、両輪を同じ速さで押す(機体が壁にそろうのを打ち消さないため)。
// 押し当て終わったら制御を有効にし直し、その向き・位置を新しい基準にする(目標の距離は0になる)。
static bool SetPosition(DrivePos *dp) {
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
    FlushLogIfDue();
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
    dp->ref_mm += dp->at_center ? HALF_SECTION_MM + SECTION_MM * (float)(cells - 1)
                                : SECTION_MM * (float)cells;
    StartStraightTo(dp->ref_mm, SEARCH_V_MM_S);
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
    s_log_event = (float)s_event_count;
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

// 走り出す前の準備(探索・最短走行で共通): 制御を有効にし、記録を始め、start_sequence で真ん中に合わせる。
static bool StartRun(DrivePos *dp) {
    App_SetPositionHold(true);
    App_SetWallPush(false);
    App_SetWallControl(false); // 使うのは境界から境界までの直進の間だけ
    App_SetTargetVelocity(0.0f);
    App_ControlLoop_SetEnabled(true);
    Logger_Start();
    s_last_flush_ms = HAL_GetTick();

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
    s_log_event = 0.0f;

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
        Action act = SearchPlanner_Step(&s_planner, obs);
        Record(pos, heading, obs, sv, act, HAL_GetTick() - t0);

        bool ok = true;
        switch ((ActionType)act.type) {
            case ACTION_FORWARD:
                if (LogFlushDue()) {
                    // ログが一杯になる前に、真ん中で止まって追記してから進む(StopAtCenter の中で追記する)
                    ok = StopAtCenter(&dp) && GoToNextBoundary(&dp, act.cells, &obs, &sv);
                } else {
                    ok = GoToNextBoundary(&dp, act.cells, &obs, &sv);
                }
                break;
            case ACTION_TURN_RIGHT:
            case ACTION_TURN_LEFT:
                ok = StopAtCenter(&dp)
                  && Pivot((act.type == ACTION_TURN_RIGHT) ? -90.0f : 90.0f)
                  && GoToNextBoundary(&dp, 1, &obs, &sv);
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
                FlushLogIfDue();
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

// 手かざしで始め、ジャイロを合わせ、ログのファイルを開く(探索・最短走行で共通)。
static void WaitStart(const char *log_name) {
    printf("put in the start cell (facing north), hold hand over front-left sensor to START\r\n");
    ModeUI_WaitHandStart();
    HAL_Delay(SEARCH_START_DELAY_MS - GYRO_RECAL_MS); // 手を離す時間
    printf("gyro z offset: %.1f\r\n", App_RecalibrateGyroZ(GYRO_RECAL_MS));
    App_ResetGyroAngle(); // スタートの向きを 0° にする(ログの向きを読みやすくするため)

    SetupLogger(log_name);
    char path[SDCARD_PATH_MAX];
    s_log_failed = false;
    LED_SetDirectPattern(0x00u);
    s_log_file = Logger_BeginFile(path, sizeof(path));
    printf("log: %s\r\n", s_log_file ? path : "(no SD, not saved)");
}

// ログのファイルを閉じる(探索・最短走行で共通)。
static void EndRunLog(void) {
    if (s_log_file) {
        bool saved = Logger_EndFile();
        s_log_file = false;
        printf("log %s\r\n", saved ? "saved" : "save failed");
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

    printf("SEARCH (%s): v=%.0f mm/s accel=%.0f\r\n",
           (algo == SEARCH_ALGO_ADACHI) ? "adachi" : "dijkstra", SEARCH_V_MM_S, SEARCH_ACCEL_MM_S2);
    PrintSettings(goals, goal_count);

    if (FailSafe_IsTripped()) FailSafe_Halt(); // 起動時の低電圧など

    while (1) {
        WaitStart("search");
        bool done = RunSearch(algo, goals, goal_count);
        EndRunLog();
        printf("search %s\r\n", done ? "finished" : "stopped");
        PrintResult(goals, goal_count);

        // ゴールまで行けていれば、地図を flash に残す(最短走行のモードで使う。BlueEyes の store_map_in_flash)
        if (s_planner.phase == SEARCH_PHASE_TO_START || s_planner.phase == SEARCH_PHASE_DONE) {
            printf("map %s\r\n", Flash_WriteUserData(&s_map, sizeof(s_map)) ? "saved to flash" : "save FAILED");
        }
    }
}

// ---- 最短走行 ----
// 探索で flash に残した地図の、分かっている壁だけ(未知の壁は「ある」)で Dijkstra を計算し、
// スタートの真ん中からゴールまでの経路を指令の列にする。続く直進は1つにまとめて FAST_V で走り、
// 曲がるときは真ん中で止まって超信地旋回する(スラロームはまだない)。

static CommandList s_route;

// 真ん中から真ん中まで cells 区画まっすぐ走って止まる。壁の制御は、境界から境界までの間
// (最初と最後の半区画を除く)だけ使う。
static bool FastStraight(DrivePos *dp, uint8_t cells) {
    float start = dp->ref_mm;
    float end = start + SECTION_MM * (float)cells;
    App_SetWallControl(false);
    dp->ref_mm = end;
    StartStraightToV(end, FAST_V_MM_S, 0.0f, FAST_ACCEL_MM_S2);
    if (cells >= 2) {
        if (!WaitTargetDistance(start + HALF_SECTION_MM)) return false;
        App_SetWallControl(true);
        if (!WaitTargetDistance(end - HALF_SECTION_MM)) return false;
        App_SetWallControl(false);
    }
    if (!WaitMotionDone()) return false;
    dp->at_center = true;
    FlushLogIfDue();
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
        s_log_event = (float)(i + 1);
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
        FlushLogIfDue();
        LED_SetDirectPattern(LED_DIRECT_ALL);
        DelayWatching(SEARCH_GOAL_WAIT_MS);
        LED_SetDirectPattern(LedIdlePattern());
    } else {
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

    printf("FAST RUN: v=%.0f mm/s accel=%.0f (pivot turns)\r\n", FAST_V_MM_S, FAST_ACCEL_MM_S2);
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
    printf("route (cost %u):\r\n", Dijkstra_Cost(solver, start, DIR_NORTH));
    CommandList_Print(&s_route);

    while (1) {
        WaitStart("fast");
        bool done = RunFast();
        EndRunLog();
        printf("fast run %s\r\n", done ? "finished" : "stopped");
    }
}
