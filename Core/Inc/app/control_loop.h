#ifndef INC_CONTROLLOOP_H_
#define INC_CONTROLLOOP_H_


#include "global.h"
#include "params.h"
#include "logic/state_estimation/odometry.h"
#include "interface/gyro.h"

// VelocityPID/Odometry/フェイルセーフの初期化とジャイロオフセットの取り込み。
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

// 台形プロファイルで超信地旋回する(その場で回る。止まって終わる)。
// angle_deg: 正=反時計回り(左)、負=時計回り(右)。
// 実際の開始は次のtick。完了はApp_IsMotionDone()で見る。制御が有効でないと進まない。
void App_StartPivot(float angle_deg, float omega_max_dps, float alpha_dps2);

// スラローム(並進しながら曲がる)。今の並進の目標速度をそのまま保ち、角速度だけを台形で動かして
// angle_deg 回る(正=反時計回り)。走っている最中(直進のプロファイルが終わって等速のとき)に呼ぶこと。
// 実際の開始は次のtick。完了(回り終わった)はApp_IsMotionDone()で見る。並進の速さは回った後も保つ。
void App_StartSlalom(float angle_deg, float omega_max_dps, float alpha_dps2);

bool App_IsMotionDone(void);

// 目標の距離[mm](制御を有効にした時点を0とし、目標の並進速度を毎tick積分した値)。
// 超信地旋回の間は変わらない。探索で区画の境界・真ん中に着いたかを見るのに使う。
float App_GetTargetDistance(void);

// 直進中に見つけた最後の壁切れ(横の壁がなくなった瞬間)。side(NULL可): 0 左 / 1 右、
// pos_ref_mm(NULL可): その瞬間の目標の距離。戻り値は見つけた数(増えたら新しい壁切れ)。
uint32_t App_GetWallEdge(uint8_t *side, float *pos_ref_mm);

// 位置の制御(前後の位置を保つ)を効かせるかどうか。起動時は true。
// false の間は目標の距離を今の距離に合わせ続けるので、true に戻しても補正は跳ばない。
// (宴会芸モードのように、向きだけ保ちたいときに false にする)
void App_SetPositionHold(bool en);

// 尻当て用。true の間は向きと位置の補正を止め(目標の向き・距離は今の値に合わせ続ける)、
// 車輪速度のループだけで動かす。壁に押し当てたとき、機体が壁にそろう向きを打ち消さないため。
void App_SetWallPush(bool en);

// 壁の制御を使うかどうか(起動時は true)。WALL_CONTROL_ENABLE が 1 のときだけ効く。
// 探索では BlueEyes と同じく、区画の境界から境界までの直進(1区画)だけで使い、
// 真ん中から・真ん中までの半区画と尻当てでは止める(真ん中ではセンサーが柱のあたりを見るため)。
void App_SetWallControl(bool en);

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

// ジャイロの積分角(App_GetGyroAngle_deg)を0に戻す(次のtickでISRが0にするまで待つ)。
// 制御を無効にしているときに呼ぶこと(有効な間に呼ぶと、目標の向きとの差が急に変わる)。
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
    float vl_ref, vr_ref; // 車輪速度の目標[mm/s](角速度の補正を含む。車輪速度ループに渡す値)
    float pwm_l, pwm_r;
    float ff_l, ff_r;  // FF項[V] (速度FF + 加速度FF)
    float i_l, i_r;    // PIDのI項[V]
    float vbat;        // フィルタ後の電池電圧[V]
    float target_omega_dps; // 目標角速度(プロファイル)
    float gyro_z_dps;       // ジャイロの角速度
    float angle_deg;        // ジャイロの積分角(起動からの累積)
    float angle_ref_deg;    // 目標の向き(angle_deg と同じ基準。止まってもリセットしない)
    float ang_corr_dps;     // 角速度ループの補正量(目標角速度に足した分)
    float ad_l, ad_fl, ad_fr, ad_r; // 壁センサーの値(IR 点灯 − 消灯の差。ad_* を float にしたもの)
    float wall_offset_deg;  // 壁の制御のオフセット[deg](目標の向き = 迷路の軸の向き + これ)
} ControlDebug;

const ControlDebug *App_GetControlDebug(void);

#endif
