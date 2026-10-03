#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "global.h"
#include "params.h"

#define PWM_MAX 4199

void Motor_Init(void);

// STBYピンでドライバ自体を有効/無効にする。
void Motor_Enable(void);
void Motor_Disable(void);

// 正負で前進/後退を切り替える。|left|/|right|がPWM_MAXを超える場合は
// クランプする。向きの判断は呼び出し側で意識する必要はない。
void Motor_Drive(int16_t left, int16_t right);

void Motor_Stop(void);

#endif