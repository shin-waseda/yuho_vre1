#include "main.h"
#include "interface/sensor.h"
#include "app/control_loop.h"

// TIM6周期割り込み(1kHz)。中身は書かず、appの関数を呼ぶだけに留める。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance != TIM6) return;

  Sensor_ReadAll();
  App_ControlTick();
}
