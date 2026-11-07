#ifndef INC_LOGEVENT_H_
#define INC_LOGEVENT_H_

// ログのイベント(Logger_Event で残す)。番号はログの "ev" 列に入り、中身は "ev_a"〜"ev_e" 列に入る。
// PC の tools/get_log.py は、このファイルの「LOG_EV_名前 = 番号, // a:名前 b:名前 ...」の行を読んで、
// CSV に "ev_text" 列(例: "STEP x=1 y=0 ...")を足す。行の形を変えないこと(1行に1つ、// の後に中身の名前)。
// 番号は増やすだけにして、使っている番号は変えない(昔のログを読めるように)。
// WALL_CTRL・WALL_USE・EDGE_CORR・FRONT_TRIG は、探索でその補正を選んだとき(2026-11-05 に戻した)だけ出る。
// SETPOS は尻当てをやめた(2026-11-04)ので出さない(昔のログ用に残す)。
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
    LOG_EV_STRAIGHT = 20,     // a:dist b:v_max c:v_end d:accel e:decel
    LOG_EV_PIVOT = 21,        // a:angle b:omega c:alpha
    LOG_EV_SLALOM = 22,       // a:angle b:omega c:alpha d:v
    LOG_EV_MOTION_DONE = 23,  // a:motion
    LOG_EV_SET_VELOCITY = 24, // a:v
    LOG_EV_PIVOT_SETTLE = 25, // a:ok b:err_deg c:ms
    LOG_EV_FRONT_ALIGN = 26,  // a:ok b:err_mm c:err_deg d:ms e:moved_mm
    LOG_EV_ALONG_SYNC = 27,   // a:along_mm b:lat_mm
    LOG_EV_GYRO_SCALE = 28,   // a:phase b:diff c:sum d:angle
    LOG_EV_GYRO_BIAS = 29,    // a:offset_lsb b:change_dps c:windows
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
    LOG_EV_LONG_RUN = 46,     // a:index b:part c:step d:v e:v_turn
    LOG_EV_FAST_PARAMS = 47,  // a:v b:accel c:small_v d:type e:decel
    LOG_EV_SEARCH_PARAMS = 48, // a:v b:turn_v c:accel d:slalom e:corr
    LOG_EV_KNOWN_RUN = 49,    // a:cells b:commands c:large d:end_x e:end_y
    // ---- 異常 ----
    LOG_EV_FAILSAFE = 50,     // a:cause b:value
    LOG_EV_TIMEOUT = 51,      // a:where
    // ---- 壁センサーのモデル用(RUN の SEARCH_SPIN)----
    LOG_EV_SENSOR_SPIN = 52,  // a:x b:y c:heading d:walls e:phase
    // ---- 速度帯の最短走行(RUN の FAST_BANDS)----
    LOG_EV_FAST_BAND = 53,    // a:band b:no c:total
    // ---- 探索の選び方(RUN の SEARCH)----
    LOG_EV_SEARCH_MODE = 54,  // a:map b:scope c:algo
    // ---- 前壁の物差しの試験(TEST の FRONT_WALL)----
    LOG_EV_FW1_POINT = 55,    // a:index b:d c:dist d:fl e:fr
    LOG_EV_FW1_POINT2 = 56,   // a:index b:l c:r d:sum_std e:vbat
    // ---- 小回りの滑りの試験(TEST の SLIP)----
    LOG_EV_SLIP_POINT = 57,   // a:kind b:v c:stop d:fl e:fr
    LOG_EV_SLIP_POINT2 = 58,  // a:dist b:l c:r d:sum_std e:vbat
    LOG_EV_SLIP_RUN = 59,     // a:kind b:v c:pre d:post
    // ---- 最短走行の ADJ CHECK(小回りの調整のあり・なしを比べる．2026-11-07 に消した．fast_0045〜0060 を読むため残す)----
    LOG_EV_FAST_CHECK = 60,   // a:adj b:fl c:fr d:l e:r
    LOG_EV_FAST_CHECK2 = 61,  // a:dist b:stop c:sum_std d:vbat e:no
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
// LOG_EV_SETPOS の phase: 0 始め, 1 終わり。LOG_EV_MOTION_DONE の motion: 0 直進, 1 超信地旋回, 2 スラローム, 3 前壁合わせ。
// LOG_EV_FW1_POINT / LOG_EV_FW1_POINT2: 前壁の物差しの試験の FW1 で、止まって 0.5 秒測った1点。index は 1 から、d は計画の距離
//   (4輪の真ん中から前壁まで[mm]．置いた所からの目標の距離から計算した名目の値)、dist はその間のエンコーダの道のりの平均[mm]、
//   fl / fr / l / r は平均、sum_std は FL + FR の標準偏差、vbat は電池の電圧。
// LOG_EV_FAST_CHECK / LOG_EV_FAST_CHECK2: 最短走行の ADJ CHECK(2026-11-07)。ゴールの真ん中から、次の境界の 12mm 手前まで
//   進んで止まり、0.5 秒前壁を読んだ平均。adj は小回りの手で足す調整を入れたか(1/0)、stop は止まる所の目標の距離、
//   dist はその間のエンコーダの道のりの平均、no は ADJ CHECK の何本目か(1 から)。
// LOG_EV_SLIP_RUN / LOG_EV_SLIP_POINT / LOG_EV_SLIP_POINT2: 小回りの滑りの試験(app/slip_turn_test)。RUN は 1 本の始め
//   (kind 1 = T1 小回り 1 回、2 = T2 右 → 左、3 = E 東へまっすぐ、4 = N 北へまっすぐ、5 = P1 超信地旋回で右、6 = P2 超信地旋回で右 → 小回りで左、
//   7 = T1L 小回りで左、8 = P1L 超信地旋回で左、9 = EW 西へまっすぐ、10 = T2L 小回りで左 → 右、11 = P2L 超信地旋回で左 → 小回りで右。
//   LR の組(7〜11 と同じ回の 1〜6)の小回りは手で足す調整を引いた形(pre / post に出る)、v は小回り(直進)の速さ、pre / post は小回りの前後のオフセット)。POINT・POINT2 は
//   止まって 0.5 秒読んだ値(stop は止まる所の目標の距離、dist はその間のエンコーダの道のりの平均。ほかは FW1 と同じ)。
// LOG_EV_GYRO_BIAS:走っている途中で止まっている間にジャイロ Z のゼロ点を測り直した続きが終わった所(app/control_loop)。
//   offset_lsb は新しいゼロ点[LSB]、change_dps はその続きの間のゼロ点の変わり[dps]、windows は寄せた窓の数(1 つ 50ms)。
// LOG_EV_GYRO_SCALE:ジャイロの倍率の試験(TEST の GYRO_SCALE)で止まって測った所。phase は 0 始め、1 左へ少し回った所、
//   2 右へ少し回った所、3 戻った所、4 左へ何回も回った後、5 右へ何回も回って戻った後。diff は FL − FR、sum は FL + FR の平均、
//   angle はそのときのジャイロの角度[deg]。
// LOG_EV_ALONG_SYNC:探索の 180° を 90° + 90° に分けたとき、止まって前後のずれを消し始める時の、目標の向きに対する
//   前後のずれ(+ 目標が前)と横のずれ(+ 機体が左)[mm]。回る前と、1回目の 90° の後(along がそれまでの横のずれ)に入る。
// LOG_EV_FRONT_ALIGN: 前の壁で合わせた結果。ok は 1 合った / 0 打ち切った、err_mm は終わった時の前後のずれ(+ 壁に近い)、
//   err_deg は向きのずれ(+ 左を向いている)、ms はかかった時間、moved_mm は合わせる間に目標の距離が動いた量(+ 前へ)。
//   この後に MOTION_DONE が入る。
// LOG_EV_PIVOT_SETTLE: 超信地旋回の台形が終わった後の合わせ込みの結果。ok は 1 合った / 0 打ち切った、
//   err_deg は終わった時の角度の差(目標 − ジャイロ)、ms は台形が終わってからの時間。この後に MOTION_DONE が入る。
// LOG_EV_RUN_TYPE の kind: 0 探索の曲がり方(0 PIVOT, 1 SMALL), 1 最短走行(0 PIVOT, 1 SMALL, 2 LARGE)。
//   2 最短走行の後の帰り道(長い走行のモード。type は 1)。
// LOG_EV_LONG_RUN: 長い走行のモードで、何番目(1〜)・段・段の中の番号・速さ(探索: 直進とスラローム、最短走行: 直進と小回り)。
// LOG_EV_FAST_PARAMS: 最短走行で選んだ直進の速さ・加速度・小回りの速さ・走り方(RUN_TYPE と同じ)・直進の減速度
//   (加速度と FAST_DECEL_MAX_MM_S2 の小さい方。前のログでは 0)。走り始めに入れる。
// LOG_EV_STRAIGHT の decel: 減速度(加速度と同じことが多い。前のログでは 0)。
// LOG_EV_SEARCH_PARAMS: 探索(と帰り道)の直進の速さ・スラロームの速さ・直進の加速度・曲がり方(1 スラローム，0 超信地旋回)。
//   走り始めに入れる(前のログでは c・d が 0)。e は使った壁の補正のビットの組み合わせ: 1 横壁追従，2 壁切れ補正，
//   4 小回りの前壁補正，8 180° の前壁合わせ，16 エンコーダとジャイロで見た横のずれの FB を切った，
//   32 小回りの手で足す調整(SLALOM_PRE/POST_ADJ_MM)を切った(2026-11-06 の比べるためだけ．search_0059 だけが 32 を持つ．
//   その後この切り替えは消した)
//   (2026-11-05 の search_0032〜0039 は 1 なら全部あり．それより前のログでは 0)。
// LOG_EV_SEARCH_MODE: 探索の地図(1 初期化，2 flash の地図に重ねる)・行き先(1 往復，2 片道，3 全面)・アルゴリズム(1 Dijkstra，2 足立法)。
//   探索と同じ所に入る(帰り道・最短走行では前の探索の値のまま)。
// LOG_EV_KNOWN_RUN: 探索で既知の区間をまとめて走り始めた。区画の数、指令の数(STOP を含む)、大回りを使ったか、終わりの区画。
//   続けて LOG_EV_RUN_CMD(指令ごと)が入る。
// LOG_EV_TIMEOUT の where: 0 探索, 1 最短走行。LOG_EV_PHASE の phase: SearchPhase(search_planner.h)。
// LOG_EV_WALL_EDGE の side: 0 左, 1 右。pos_ref は壁切れの瞬間の目標の距離。
// LOG_EV_EDGE_CORR: 壁切れで目標の距離の基準をずらした量 corr[mm](+ なら次の境界を先へ)。
//   expected は壁切れが起きるはずだった目標の距離、edge_pos は実際に起きた目標の距離。
// LOG_EV_FRONT_TRIG: スラロームの前壁補正。diff は距離で決めた曲がり始めの位置からのずれ[mm]
//   (− なら手前で曲がり始めた)、sum はそのときの FL+FR、by_sensor は 1 なら前の壁の値で、0 なら距離で始めた。
// LOG_EV_SENSOR_SPIN: 探索で初めて入った区画の真ん中で回った(壁センサーのモデル用)。区画、回る前の向き(Direction)、
//   その区画の壁(地図の向きで bit0 北, bit1 東, bit2 南, bit3 西)、phase: 0 左に回り始める, 1 右に回り始める, 2 回り終わった。

#endif
