#include "app/control_loop.h"

#include "interface/encoder.h"
#include "interface/motor.h"
#include "logic/state_estimation/kinematics.h"
#include "logic/control/velocity_pid.h"

static Odometry_t s_odo;
static VelocityPID s_vpid;
static WheelVelocity s_actual;
static float s_target_mm_s = 0.0f;

void App_ControlLoop_Init(void) {
    Odometry_Reset(&s_odo);
    VelocityPID_Init(&s_vpid);
    s_actual.left_mm_s = 0.0f;
    s_actual.right_mm_s = 0.0f;
}

void App_SetTargetVelocity(float mm_s) {
    s_target_mm_s = mm_s;
}

Pose App_GetPose(void) {
    return s_odo.pose;
}

WheelVelocity App_GetActualVelocity(void) {
    return s_actual;
}

// out.*_mm_s は「速度相当の補正量」であって、PWM値そのものではない。
// ここで符号付きPWMレンジ[-PWM_MAX, PWM_MAX]へ変換・クランプする
// (PWM_MAXはinterface層の定義なのでlogic層からは参照しない)。
// Motor_Drive()側が符号でCW/CCWを切り替えるので、前後進の判断はここでは不要。
static int16_t ClampToPWM(float v) {
    if (v > (float)PWM_MAX) return (int16_t)PWM_MAX;
    if (v < -(float)PWM_MAX) return (int16_t)-PWM_MAX;
    return (int16_t)v;
}

void App_ControlTick(void) {
    int16_t delta_l = Encoder_GetDeltaL();
    int16_t delta_r = Encoder_GetDeltaR();

    s_actual = Odometry_Update(&s_odo, delta_l, delta_r, CONTROL_DT_S);

    RobotVelocity target_robot = { .linear_mm_s = s_target_mm_s, .angular_rad_s = 0.0f };
    WheelVelocity target_wheel = Kinematics_RobotToWheel(target_robot);

    WheelVelocity out = VelocityPID_Update(&s_vpid, target_wheel, s_actual, CONTROL_DT_S);

    Motor_Drive(ClampToPWM(out.left_mm_s), ClampToPWM(out.right_mm_s));
}
