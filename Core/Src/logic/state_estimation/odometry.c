#include "logic/state_estimation/odometry.h"

#include <math.h>

void Odometry_Reset(Odometry_t *odo) {
    odo->pose.x_mm = 0.0f;
    odo->pose.y_mm = 0.0f;
    odo->pose.theta_rad = 0.0f;
}

WheelVelocity Odometry_Update(Odometry_t *odo, int16_t delta_l, int16_t delta_r, float dt) {
    WheelVelocity wheel = { 0.0f, 0.0f };

    if (dt <= 0.0f) {
        return wheel;
    }

    wheel.left_mm_s  = (ENCODER_L_SIGN * (float)delta_l) * MM_PER_PULSE / dt;
    wheel.right_mm_s = (ENCODER_R_SIGN * (float)delta_r) * MM_PER_PULSE / dt;

    RobotVelocity robot = Kinematics_WheelToRobot(wheel);

    // 微小時間(dt)の範囲ではthetaを一定とみなして積分する
    odo->pose.x_mm      += robot.linear_mm_s * cosf(odo->pose.theta_rad) * dt;
    odo->pose.y_mm      += robot.linear_mm_s * sinf(odo->pose.theta_rad) * dt;
    odo->pose.theta_rad += robot.angular_rad_s * dt;

    return wheel;
}
