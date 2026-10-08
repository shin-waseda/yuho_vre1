#ifndef INC_LOGEVENT_H_
#define INC_LOGEVENT_H_

// ログのイベント(Logger_Event で残す)。番号はログの "ev" 列に入り、中身は "ev_a"〜"ev_e" 列に入る。
// PC の tools/get_log.py は、このファイルの「LOG_EV_名前 = 番号, // a:名前 b:名前 ...」の行を読んで、
// CSV に "ev_text" 列(例: "STEP x=1 y=0 ...")を足す。行の形を変えないこと(1行に1つ、// の後に中身の名前)。
// 番号は増やすだけにして、使っている番号は変えない(昔のログを読めるように)。
typedef enum {
    LOG_EV_NONE = 0,
    // ---- 操作・状態 ----
    LOG_EV_MODE = 1,          // a:mode
    LOG_EV_RUN_TYPE = 2,      // a:type b:kind
    LOG_EV_HAND_START = 3,    //
    LOG_EV_CLICK = 4,         //
    LOG_EV_GYRO_RECAL = 5,    // a:offset_lsb
    LOG_EV_CTRL_ENABLE = 6,   // a:on
    LOG_EV_BOOT = 7,          // a:reset_flags b:fault
    LOG_EV_FAULT_PC = 8,      // a:pc_hi b:pc_lo c:lr_hi d:lr_lo
    LOG_EV_FAULT_REG = 9,     // a:cfsr_hi b:cfsr_lo c:hfsr_hi d:bfar_hi e:bfar_lo
    // ---- 制御の切り替え ----
    LOG_EV_WALL_CTRL = 10,    // a:on
    LOG_EV_WALL_USE = 11,     // a:left b:right
    LOG_EV_WALL_EDGE = 12,    // a:side b:pos_ref c:dist
    LOG_EV_EDGE_CORR = 13,    // a:side b:corr c:expected d:edge_pos
    LOG_EV_FRONT_TRIG = 14,   // a:diff b:sum c:by_sensor
    // ---- 動き(control_loop が出す) ----
    LOG_EV_STRAIGHT = 20,     // a:dist b:v_max c:v_end d:accel
    LOG_EV_PIVOT = 21,        // a:angle b:omega c:alpha
    LOG_EV_SLALOM = 22,       // a:angle b:omega c:alpha d:v
    LOG_EV_MOTION_DONE = 23,  // a:motion
    LOG_EV_SET_VELOCITY = 24, // a:v
    // ---- 動き(search_run などが出す、意味のある単位) ----
    LOG_EV_TURN_KIND = 30,    // a:kind b:right c:pre d:post
    LOG_EV_SETPOS = 31,       // a:phase
    LOG_EV_STOP_CENTER = 32,  // a:ref
    LOG_EV_RUN_CMD = 33,      // a:index b:run_type c:halves
    LOG_EV_ROUTE_CMD = 34,    // a:index b:action c:cells
    // ---- 迷路の判断 ----
    LOG_EV_STEP = 40,         // a:x b:y c:heading d:walls e:action
    LOG_EV_STEP_INFO = 41,    // a:plan_ms b:l c:fl d:fr e:r
    LOG_EV_PHASE = 42,        // a:phase
    LOG_EV_MAP_SAVED = 43,    // a:ok
    LOG_EV_ROUTE = 44,        // a:cost b:est_s c:count d:type
    LOG_EV_MAP_CELLS = 45,    // a:index b:w0 c:w1 d:w2 e:w3
    // ---- 異常 ----
    LOG_EV_FAILSAFE = 50,     // a:cause b:value
    LOG_EV_TIMEOUT = 51,      // a:where
} LogEventCode;

// LOG_EV_BOOT: 今の起動のリセットの原因(FAULT_RESET_*、interface/fault_diag.h)と、その前に HardFault があったか(1/0)。
//   走り始めに毎回入れる。fault が 1 なら LOG_EV_FAULT_PC / LOG_EV_FAULT_REG も入れる。
//   32bit の値は上位(_hi)と下位(_lo)の 16bit に分けて入れる(CSV は有効数字6桁なので)。値 = hi × 65536 + lo。
// LOG_EV_STEP の walls: bit0 前, bit1 右, bit2 左。action: ActionType(command.h)。
// LOG_EV_RUN_CMD の run_type: RunType(run_path.h)。LOG_EV_ROUTE_CMD(最短走行の PIVOT)の action: ActionType。
// LOG_EV_TURN_KIND の kind: 0 小回り90°(s90), 1 大回り90°(l90), 2 大回り180°(l180), 3 超信地旋回。
// LOG_EV_MAP_CELLS: 機体の壁の地図(WallMap の cell、1区画1バイト)。index は最初の区画の番号(y × MAZE_SIZE + x)、
//   w0〜w3 は続く区画を2つずつ(下位8bit が先、上位8bit が次)入れた値。1つで8区画、全部で MAZE_SIZE² / 8 個。
//   (CSV は有効数字6桁なので、1つの値は 16bit まで)
// LOG_EV_SETPOS の phase: 0 始め, 1 終わり。LOG_EV_MOTION_DONE の motion: 0 直進, 1 超信地旋回, 2 スラローム。
// LOG_EV_RUN_TYPE の kind: 0 探索の曲がり方(0 PIVOT, 1 SMALL), 1 最短走行(0 PIVOT, 1 SMALL, 2 LARGE)。
// LOG_EV_TIMEOUT の where: 0 探索, 1 最短走行。LOG_EV_PHASE の phase: SearchPhase(search_planner.h)。
// LOG_EV_WALL_EDGE の side: 0 左, 1 右。pos_ref は壁切れの瞬間の目標の距離。
// LOG_EV_EDGE_CORR: 壁切れで目標の距離の基準をずらした量 corr[mm](+ なら次の境界を先へ)。
//   expected は壁切れが起きるはずだった目標の距離、edge_pos は実際に起きた目標の距離。
// LOG_EV_FRONT_TRIG: スラロームの前壁補正。diff は距離で決めた曲がり始めの位置からのずれ[mm]
//   (− なら手前で曲がり始めた)、sum はそのときの FL+FR、by_sensor は 1 なら前の壁の値で、0 なら距離で始めた。

#endif
