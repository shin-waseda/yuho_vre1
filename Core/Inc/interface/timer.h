#ifndef INC_TIMER_H_
#define INC_TIMER_H_


#include "global.h"
#include "params.h"

// HAL_TIM_PeriodElapsedCallback (TIM6, 1kHz) の実体は Core/Src/interface/timer.c。
// HALが名前を決め打ちするコールバックなのでここでは宣言しない。
// 中身は Sensor_ReadAll() 等、appの関数を呼ぶだけに留める(ISR例外ルール)。

#endif