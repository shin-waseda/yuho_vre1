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
// 実際の開始は次のtick(ISRが取り込む)。完了はApp_IsMotionDone()で見る。
// v_end=0なら止まる。制御が有効でないと進まない。
void App_StartStraight(float distance_mm, float v_max, float v_end, float accel);

// 台形プロファイルで超信地旋回する(その場で回る。止まって終わる)。
// angle_deg: 正=反時計回り(左)、負=時計回り(右)。
// 実際の開始は次のtick。完了はApp_IsMotionDone()で見る。制御が有効でないと進まない。
void App_StartPivot(float angle_deg, float omega_max_dps, float alpha_dps2);

bool App_IsMotionDone(void);

// 位置の制御(前後の位置を保つ)を効かせるかどうか。起動時は true。
// false の間は目標の距離を今の距離に合わせ続けるので、true に戻しても補正は跳ばない。
// (宴会芸モードのように、向きだけ保ちたいときに false にする)
void App_SetPositionHold(bool en);

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
} ControlDebug;

const ControlDebug *App_GetControlDebug(void);

#endif
