#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_


#include "global.h"
#include "params.h"

// IR壁センサー(R/FR/FL/L)を全点灯・消灯シーケンスで走査し、
// 差分値を ad_r/ad_fr/ad_fl/ad_l (global.h) へ書き込む。
// TIM6周期割り込み(interface/timer.c)から1kHzで呼ばれる想定。
void Sensor_ReadAll(void);

#endif