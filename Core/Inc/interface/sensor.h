#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_


#include "global.h"
#include "params.h"

// IR壁センサー(R/FR/FL/L)を全点灯・消灯シーケンスで走査し、
// 差分値を ad_r/ad_fr/ad_fl/ad_l (global.h) へ書き込む。
// TIM6周期割り込み(interface/timer.c)から1kHzで呼ばれる想定。
void Sensor_ReadAll(void);

// バッテリー電圧のADC生値を1回だけ取得する(IR LEDは操作しない)。
// ADCのrankを1周させて戻すので、後のSensor_ReadAll()の前提は崩れない。
// TIM6割り込み開始前(=Sensor_ReadAll()と同時に動かない時)専用。
uint16_t Sensor_ReadBatteryRawBlocking(void);

#endif