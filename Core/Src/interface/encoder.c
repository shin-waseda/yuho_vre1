#include "main.h"
#include "tim.h" // htim4 (ENC_L) / htim8 (ENC_R) の宣言
#include "interface/encoder.h"

void Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

// 前回読み取り時のカウンタ値。Encoder_SyncDelta()で現在値に合わせられるよう、
// 関数内staticではなくファイルスコープに置く。
static uint16_t s_last_l = 0;
static uint16_t s_last_r = 0;

void Encoder_SyncDelta(void) {
    s_last_l = __HAL_TIM_GET_COUNTER(&htim4);
    s_last_r = __HAL_TIM_GET_COUNTER(&htim8);
}

// 前回読み取り時からの差分(符号付き)を返し、カウンタは読み捨てずにそのまま維持
int16_t Encoder_GetDeltaL(void) {
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta = (int16_t)(now - s_last_l); // 16bitラップアラウンドを利用した差分計算
    s_last_l = now;
    return delta;
}

int16_t Encoder_GetDeltaR(void) {
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim8);
    int16_t delta = (int16_t)(now - s_last_r);
    s_last_r = now;
    return delta;
}
