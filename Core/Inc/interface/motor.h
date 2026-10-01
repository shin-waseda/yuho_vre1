#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "global.h"
#include "params.h"

#define PWM_MAX 4199

void Motor_Init(void);
void Motor_Forward(uint16_t left, uint16_t right);
void Motor_Stop(void);

#endif