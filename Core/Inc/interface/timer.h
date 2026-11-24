#ifndef INC_TIMER_H_
#define INC_TIMER_H_


#include "global.h"
#include "params.h"

// HAL_TIM_PeriodElapsedCallback (TIM6, 1kHz) の実体は Core/Src/interface/timer.c。
// HALが名前を決め打ちするコールバックなのでここでは宣言しない。
// 中身は Sensor_ReadAll() 等、appの関数を呼ぶだけに留める(ISR例外ルール)。

// 時間(app が HAL を直接呼ばないための窓口。中身は HAL_GetTick / HAL_Delay と同じ)
uint32_t Timer_GetMs(void);        // 起動からの時間[ms](SysTick。約49日で0に戻るので差で使う)
void Timer_DelayMs(uint32_t ms);   // ms だけ待つ(HAL_Delay と同じく、割り込みの中では使わない)

// 割り込みを止める区間(メインと ISR の両方からさわる変数を書く間)。入れ子にしてよい。
//   uint32_t s = Irq_Save(); ... Irq_Restore(s);
uint32_t Irq_Save(void);
void Irq_Restore(uint32_t saved);

#endif