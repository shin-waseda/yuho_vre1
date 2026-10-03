#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_


#include "global.h"
#include "params.h"

typedef struct {
    float left_mm_s;
    float right_mm_s;
} WheelVelocity;

typedef struct {
    float linear_mm_s;   // 機体前進速度
    float angular_rad_s; // 機体旋回角速度(反時計回り正)
} RobotVelocity;

RobotVelocity Kinematics_WheelToRobot(WheelVelocity wheel);
WheelVelocity  Kinematics_RobotToWheel(RobotVelocity robot);

#endif