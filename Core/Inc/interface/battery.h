#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_


#include "global.h"
#include "params.h"

// ISR(Sensor_ReadAll)が最後に読んだvabtを電圧[V]へ換算して返す。
// フィルタはかけていない(生値の換算のみ)。
float Battery_GetVoltage(void);

// ADCを直接読んで、samples回平均した電圧[V]を返す。
// TIM6割り込み開始前の起動時チェック専用(ISRとADCを取り合うため)。
float Battery_MeasureVoltageBlocking(uint16_t samples);

#endif
