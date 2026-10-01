#include "main.h"
#include "interface/sensor.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;

volatile uint16_t ad_r, ad_fr, ad_fl, ad_l, vabt;

#define ADC_TIMEOUT_MS  10   // Discontinuous化済みなので通常は一瞬で終わる
#define IR_SETTLE_US    50   // LED点灯からセンサ読み取りまでの安定待ち

// TIM6は Prescaler=84-1 で 84MHz/84=1MHz、つまり1カウント=1us
static void tim6_wait_us(uint32_t us) {
  uint16_t start = __HAL_TIM_GET_COUNTER(&htim6);
  while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - start) < us);
}

// 事前設定済みのrank(1→2→3→4→5→wrap→1…)を1つだけ進めて変換値を取得
// Rank順: 1=CH1(Sensor_R) 2=CH0(Sensor_FR) 3=CH2(Sensor_FL) 4=CH3(Sensor_L) 5=CH10(Vol_Check)
static uint16_t adc_next(void) {
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK) {
    return 0;
  }
  return HAL_ADC_GetValue(&hadc1);
}

void Sensor_ReadAll(void) {
  uint16_t r_on, fr_on, fl_on, l_on, r_off, fr_off, fl_off, l_off;

  // --- 1. 全LED消灯でOFF値を取得 (rank1..5: R, FR, FL, L, Vbat の順) ---
  HAL_GPIO_WritePin(IR_R_GPIO_Port,  IR_R_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IR_L_GPIO_Port,  IR_L_Pin,  GPIO_PIN_RESET);
  tim6_wait_us(IR_SETTLE_US);

  r_off  = adc_next(); // rank1: CH1
  fr_off = adc_next(); // rank2: CH0
  fl_off = adc_next(); // rank3: CH2
  l_off  = adc_next(); // rank4: CH3
  vabt            = adc_next(); // rank5: CH10 → 次はwrapしてrank1に戻る

  // --- 2. R ---
  HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);
  tim6_wait_us(IR_SETTLE_US);
  r_on = adc_next(); // rank1: CH1
  HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);

  // --- 3. FR ---
  HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_SET);
  tim6_wait_us(IR_SETTLE_US);
  fr_on = adc_next(); // rank2: CH0
  HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);

  // --- 4. FL ---
  HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_SET);
  tim6_wait_us(IR_SETTLE_US);
  fl_on = adc_next(); // rank3: CH2
  HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);

  // --- 5. L ---
  HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_SET);
  tim6_wait_us(IR_SETTLE_US);
  l_on = adc_next(); // rank4: CH3
  HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);

  uint16_t vabt2 = adc_next(); // rank5: CH10
  vabt = (uint16_t)(((uint32_t)vabt + vabt2) / 2); // 簡易平均でノイズ低減

  // --- 6. 差分計算（アンダーフロー防止） ---
  ad_r  = (r_on  > r_off ) ? (r_on  - r_off ) : 0;
  ad_fr = (fr_on > fr_off) ? (fr_on - fr_off) : 0;
  ad_fl = (fl_on > fl_off) ? (fl_on - fl_off) : 0;
  ad_l  = (l_on  > l_off ) ? (l_on  - l_off ) : 0;
}
