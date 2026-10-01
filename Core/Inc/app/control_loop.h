#ifndef INC_CONTROLLOOP_H_
#define INC_CONTROLLOOP_H_


#include "global.h"
#include "params.h"
#include "logic/state_estimation/odometry.h"

// VelocityPID/Odometryの初期化。HAL_TIM_Base_Start_IT(&htim6)を呼ぶ前に、
// main()から1回呼ぶこと。
void App_ControlLoop_Init(void);

// 速度PID調整用の簡易セッター。台形加減速プロファイル(velocity_profile)
// が未実装のため、暫定的に目標速度を外から固定値で与える。
void App_SetTargetVelocity(float mm_s);

// TIM6 ISR(interface/timer.c)から1kHzで呼ばれる制御tick本体。
// センサー取得→状態推定→PID→モータ出力を1周期分まとめて行う。
void App_ControlTick(void);

// デバッグ表示用。App_ControlTick()が最後に計算した自己位置・実速度を返す。
// Encoder_GetDeltaL/R()は一度きりの消費関数なので、他から直接呼ばず
// 必ずこの経由で読むこと。
Pose App_GetPose(void);
WheelVelocity App_GetActualVelocity(void);

#endif
