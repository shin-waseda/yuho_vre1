#include "logic/control/velocity_pid.h"

void VelocityPID_Init(VelocityPID *vpid) {
    vpid->left.kp  = VELOCITY_KP;
    vpid->left.ki  = VELOCITY_KI;
    vpid->left.kd  = VELOCITY_KD;
    vpid->right.kp = VELOCITY_KP;
    vpid->right.ki = VELOCITY_KI;
    vpid->right.kd = VELOCITY_KD;

    vpid->output_min = VELOCITY_PID_OUTPUT_MIN;
    vpid->output_max = VELOCITY_PID_OUTPUT_MAX;

    VelocityPID_Reset(vpid);
}

void VelocityPID_Reset(VelocityPID *vpid) {
    PID_Reset(&vpid->left);
    PID_Reset(&vpid->right);
}

WheelVelocity VelocityPID_Update(VelocityPID *vpid, WheelVelocity target, WheelVelocity actual, float dt) {
    WheelVelocity out;
    out.left_mm_s  = PID_Update(&vpid->left,  target.left_mm_s  - actual.left_mm_s,  dt, vpid->output_min, vpid->output_max);
    out.right_mm_s = PID_Update(&vpid->right, target.right_mm_s - actual.right_mm_s, dt, vpid->output_min, vpid->output_max);
    return out;
}
