#include "logic/state_estimation/kinematics.h"

RobotVelocity Kinematics_WheelToRobot(WheelVelocity wheel) {
    RobotVelocity r;
    r.linear_mm_s   = (wheel.left_mm_s + wheel.right_mm_s) / 2.0f;
    r.angular_rad_s = (wheel.right_mm_s - wheel.left_mm_s) / TREAD_WIDTH_MM;
    return r;
}

WheelVelocity Kinematics_RobotToWheel(RobotVelocity robot) {
    float turn_term = (robot.angular_rad_s * TREAD_WIDTH_MM) / 2.0f;

    WheelVelocity w;
    w.left_mm_s  = robot.linear_mm_s - turn_term;
    w.right_mm_s = robot.linear_mm_s + turn_term;
    return w;
}
