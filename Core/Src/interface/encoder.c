#include "interface/encoder.h"

extern TIM_HandleTypeDef htim4; // ENC_L
extern TIM_HandleTypeDef htim8; // ENC_R

void Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

// 前回読み取り時からの差分(符号付き)を返し、カウンタは読み捨てずにそのまま維持
int16_t Encoder_GetDeltaL(void) {
    static uint16_t last = 0;
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = (int16_t)(now - last); // 16bitラップアラウンドを利用した差分計算
    last = now;
    return delta;
}

int16_t Encoder_GetDeltaR(void) {
    static uint16_t last = 0;
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim8);
    int16_t delta = (int16_t)(now - last);
    last = now;
    return delta;
}
