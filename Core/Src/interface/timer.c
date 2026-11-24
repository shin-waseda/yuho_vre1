#include "main.h"
#include "interface/sensor.h"
#include "app/control_loop.h"
#include "app/logger.h"

// TIM6周期割り込み(1kHz)。中身は書かず、appの関数を呼ぶだけに留める。
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance != TIM6) return;

  Sensor_ReadAll();
  App_ControlTick();
  Logger_Sample(); // このtickで確定した値を記録する
}

uint32_t Timer_GetMs(void) {
  return HAL_GetTick();
}

void Timer_DelayMs(uint32_t ms) {
  HAL_Delay(ms);
}

uint32_t Irq_Save(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void Irq_Restore(uint32_t saved) {
  if (!saved) __enable_irq();
}
