#ifndef INC_CONTROLLOOP_H_
#define INC_CONTROLLOOP_H_


#include "global.h"
#include "params.h"
#include "logic/state_estimation/odometry.h"
#include "interface/gyro.h"

// 並進・旋回の制御/Odometry/フェイルセーフの初期化とジャイロオフセットの取り込み。
// ICM_CalibrateBlocking()の後、HAL_TIM_Base_Start_IT(&htim6)の前に、
// main()から1回呼ぶこと。
void App_ControlLoop_Init(void);

// 目標速度を直接与える(加速度0扱い)。ステップ応答の試験用。
// 直進プロファイルの実行中に呼ぶと、そのプロファイルを打ち切る。
void App_SetTargetVelocity(float mm_s);

// 台形プロファイルで直進する(停止から、または今の目標速度から)。
// distance_mm が負なら後ろへ下がる(v_max・v_end・accel は正の値で渡す)。
// 実際の開始は次のtick(ISRが取り込む)。完了はApp_IsMotionDone()で見る。
// v_end=0なら止まる。制御が有効でないと進まない。
void App_StartStraight(float distance_mm, float v_max, float v_end, float accel);
// App_StartStraight と同じで、減速度だけ別にする(最短走行の直進。減速でタイヤが滑りやすいため)
void App_StartStraightAD(float distance_mm, float v_max, float v_end, float accel, float decel);

// 台形プロファイルで超信地旋回する(その場で回る。止まって終わる)。
// 台形が終わった後、ジャイロの角度が目標に合うまで(最長 PIVOT_SETTLE_TIMEOUT_MS)完了にしない。
// angle_deg: 正=反時計回り(左)、負=時計回り(右)。
// 実際の開始は次のtick。完了はApp_IsMotionDone()で見る。制御が有効でないと進まない。
void App_StartPivot(float angle_deg, float omega_max_dps, float alpha_dps2);

// 区画の真ん中で前の壁に向いて止まっている所で、前の壁センサーが基準(FRONT_ALIGN_REF_*)になるよう、
// その場で前後と向きを合わせる(止まって終わる)。合わせた分は目標の距離と目標の向きにも入る。
// 実際の開始は次のtick。完了はApp_IsMotionDone()で見る(最長 FRONT_ALIGN_TIMEOUT_MS)。結果は LOG_EV_FRONT_ALIGN。
void App_StartFrontAlign(void);

// 目標の位置に対する推定の位置のずれ[mm](2次元。目標の向きに対して)。位置のループは前後のずれを、
// 横のずれの FB は横のずれを、いつも消そうとしている。
float App_GetPositionError(void); // 前後(+ なら目標が前)
float App_GetLateralError(void);  // 横(+ なら機体が左)

// スラローム(並進しながら曲がる)。今の並進の目標速度をそのまま保ち、角速度だけを台形で動かして
// angle_deg 回る(正=反時計回り)。走っている最中(直進のプロファイルが終わって等速のとき)に呼ぶこと。
// 実際の開始は次のtick。完了(回り終わった)はApp_IsMotionDone()で見る。並進の速さは回った後も保つ。
void App_StartSlalom(float angle_deg, float omega_max_dps, float alpha_dps2);

bool App_IsMotionDone(void);

// 目標の距離[mm](制御を有効にした時点を0とし、目標の並進速度を毎tick積分した値)。
// 超信地旋回の間は変わらない。探索で区画の境界・真ん中に着いたかを見るのに使う。
float App_GetTargetDistance(void);

// 位置の制御(前後の位置を保つ)を効かせるかどうか。起動時は true。
// false の間は目標の距離を今の距離に合わせ続けるので、true に戻しても補正は跳ばない。
// (宴会芸モードのように、向きだけ保ちたいときに false にする)
void App_SetPositionHold(bool en);

// 横壁追従(logic/control/wall_control)を使うかどうか。起動時は false。直進している間だけ効く。
// 探索で補正を選んだとき、境界から境界への直進の間だけ true にする(変わったら LOG_EV_WALL_CTRL)。
// 横壁追従のずらしは、横のずれの FB のずらしと足して目標の向きに足す。
void App_SetWallControl(bool en);

// エンコーダとジャイロで見た横のずれの FB を使うかどうか。起動時は true。
// false の間は、横のずれを返さず、目標の位置の横の成分を推定の位置に合わせ続ける(横のずれをためないので、
// 曲がった後に前後のずれに変わって位置のループに引っ張られることもない)。横は横壁追従(壁)だけで決まる。
// 探索で、横壁追従だけの走り方を作るのに使う(2026-11-05)。
void App_SetLateralControl(bool en);

// モーターの出力だけを 0 にする(true)/戻す(false)。制御の状態(目標の距離・向き・位置・積分)はそのまま。
// 走っている途中で止まって flash を書くときに使う(flash を消している間は CPU が止まって制御の割り込みが回らず、
// モーターが前の出力のまま回り続けるため。2026-11-06)
void App_SetOutputPaused(bool paused);

// 直進中に見つけた壁切れ(横の壁がなくなった瞬間)。side: 0 左、1 右、pos_ref_mm: その瞬間の目標の距離。
// 戻り値は見つけた数(新しい壁切れがあったかを、前に読んだ値と比べて見分ける)。side・pos_ref_mm は NULL 可
uint32_t App_GetWallEdge(uint8_t *side, float *pos_ref_mm);

// 補正で前後の位置を直した量[mm]を、ログの列 corr_along に足していく(壁切れ補正・小回りの前壁補正。
// 前壁合わせで動いた量は ISR が自分で足す)。+ なら、思っていたより後ろにいたので先へ進めた
void App_AddAlongCorrection(float mm);
// 速度PIDとモータ出力の有効/無効。起動時は無効。
// 無効中もオドメトリ更新は続ける(PIDとモータ出力だけ止める)。
// 有効化した直後のtickでPIDの内部状態をリセットする。
// メインループ側から呼ぶこと(ISRからは呼ばない)。
// フェイルセーフ発動中はtrueを渡しても有効化されない。
void App_ControlLoop_SetEnabled(bool en);
bool App_ControlLoop_IsEnabled(void);

// TIM6 ISR(interface/timer.c)から1kHzで呼ばれる制御tick本体。
// センサー取得→状態推定→フェイルセーフ判定→PID→モータ出力を1周期分まとめて行う。
void App_ControlTick(void);

// デバッグ表示用。App_ControlTick()が最後に計算した自己位置・実速度を返す。
// Encoder_GetDeltaL/R()は一度きりの消費関数なので、他から直接呼ばず
// 必ずこの経由で読むこと。
Pose App_GetPose(void);
WheelVelocity App_GetActualVelocity(void);

// デバッグ表示用。App_ControlTick()が最後にMotor_Drive()へ渡した
// 符号付きPWM値(無効中は0)。
typedef struct {
    int16_t left;
    int16_t right;
} MotorPWM;

MotorPWM App_GetMotorPWM(void);

// デバッグ表示用。App_ControlTick()が最後に読んだジャイロ値。
// TIM6割り込み開始後はジャイロのSPIをISRが占有するので、必ずこの経由で読むこと。
GyroData App_GetGyroRaw(void);
float App_GetGyroZ_dps(void);
// ジャイロの角速度を積分した角度[deg](起動からの累積、反時計回り正)。
float App_GetGyroAngle_deg(void);

// ジャイロ Z のゼロ点を、ms ミリ秒の平均で測り直す(ブロッキング)。機体を止めた状態で、
// 制御を無効にしているとき(走り出す直前など)に呼ぶこと。TIM6割り込みの中で測る
// (割り込みがジャイロの SPI を使っているので、メインからは読まない)。
// 測り直したゼロ点[LSB]を返す。割り込みが動いていないなどで測れなければ、元のゼロ点を返す。
float App_RecalibrateGyroZ(uint32_t ms);

// ジャイロの積分角(App_GetGyroAngle_deg)と目標の向き(迷路の軸の向き)を0に戻す(次のtickでISRが0にするまで待つ)。
// 向きの基準を決めるのはこれだけなので、手で置いて走り出す前(走りの始め)に1回呼ぶ。走りの途中では呼ばない。
// 制御を無効にしているときに呼ぶこと。
void App_ResetGyroAngle(void);

// ログ用。App_ControlTick()が毎tick更新する値(すべてfloat)。
// Logger_AddField()に各メンバのアドレスを渡して記録する。
typedef struct {
    float target_mm_s; // 目標並進速度
    float target_acc;  // 目標並進加速度[mm/s^2]
    float pos_ref;     // 目標の距離[mm](制御を有効にした時点から。止まってもリセットしない)
    float dist_mm;     // 進んだ距離[mm](pos_ref と同じ基準。左右の車輪の平均を積分)
    float pos_corr;    // 位置のループの補正量[mm/s](目標速度に足した分)
    float x_mm;        // オドメトリのX[mm]
    float vl, vr;      // 実車輪速度[mm/s]
    float vl_ref, vr_ref; // 車輪速度の目標[mm/s](位置の補正を含む。フェイルセーフの速度偏差に使う)
    float pwm_l, pwm_r;
    float ff_l, ff_r;  // 車輪ごとの FF[V](V_s,ff ∓ V_d,ff．モデルの逆)
    float i_s;         // 並進の FB の I 項[V]
    float fb_d;        // 旋回の FB[V](V_d に足した分)
    float vbat;        // フィルタ後の電池電圧[V]
    float target_omega_dps; // 目標角速度(プロファイル)
    float gyro_z_dps;       // ジャイロの角速度
    float angle_deg;        // ジャイロの積分角(起動からの累積)
    float angle_ref_deg;    // 目標の向き(angle_deg と同じ基準。止まってもリセットしない。横のずれのずらしは含まない)
    float lat_err_mm;       // 横のずれ[mm](目標の位置に対する推定の位置。目標の向きの左 +)
    // 補正を受けない推定の位置[mm](進んだ距離とジャイロの角度だけで積む。走りの始めの向きを +x、左を +y)。
    // 向きは angle_deg(補正は目標の向きを動かすだけで、ジャイロの角度は動かさない)
    float est_x_mm, est_y_mm;
    float wall_ofs_deg;     // 横壁追従のずらし[deg](目標の向きに足した分。横のずれの FB の分は含まない)
    float corr_along_mm;    // 補正で前後の位置を直した量の合計[mm](壁切れ補正・小回りの前壁補正・前壁合わせ)
    float f_ff;             // 旋回の FF のうち，タイヤが横にこする分 F[V]
    float ad_l, ad_fl, ad_fr, ad_r; // 壁センサーの値(IR 点灯 − 消灯の差。ad_* を float にしたもの)
    // 6軸(機体の座標: x 前、y 左、z 上。logic/state_estimation/imu)。制御には使わず、ログに残すだけ。
    // 加速度[m/s²]は重力を含む(止まって水平なら acc_z が約 +9.8)。ジャイロ Z は gyro_z_dps と同じなので入れない
    float acc_x, acc_y, acc_z;
    float gyro_x_dps, gyro_y_dps;
} ControlDebug;

const ControlDebug *App_GetControlDebug(void);

#endif
