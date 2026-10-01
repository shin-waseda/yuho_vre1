#include "main.h"
#include "interface/sensor.h"

// TIM6周期割り込み(1kHz)。中身は書かず、appの関数を呼ぶだけに留める。
// (速度PID実装後は、ここに App_ControlTick() の呼び出しが追加される)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance != TIM6) return;

  Sensor_ReadAll();
}
