#ifndef INC_VELOCITYPID_H_
#define INC_VELOCITYPID_H_


#include "global.h"
#include "params.h"
#include "logic/control/pid.h"
#include "logic/state_estimation/kinematics.h"

// 左右輪ぶんのPID_tと、共通の出力クランプ範囲を持つ。
// ゲイン・クランプ範囲はVelocityPID_Init()でparams.hから設定する。
typedef struct {
    PID_t left;
    PID_t right;
    float output_min;
    float output_max;
} VelocityPID;

void VelocityPID_Init(VelocityPID *vpid);
void VelocityPID_Reset(VelocityPID *vpid);

// target/actualは共にWheelVelocity([mm/s])。戻り値は「速度相当の
// 補正量」であり、PWM値そのものではない(PWM変換は呼び出し側=appの責務)。
WheelVelocity VelocityPID_Update(VelocityPID *vpid, WheelVelocity target, WheelVelocity actual, float dt);

#endif