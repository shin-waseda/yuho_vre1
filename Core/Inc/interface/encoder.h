#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "global.h"
#include "params.h"

void Encoder_Init(void);
int16_t Encoder_GetDeltaL(void);
int16_t Encoder_GetDeltaR(void);

// 次回のGetDeltaL/R()が0から数え始めるよう、前回値を現在のカウンタに合わせる。
// 1kHz制御ループ開始(HAL_TIM_Base_Start_IT)の直前に呼び、
// それまでに溜まった回転がオドメトリへ一気に入るのを防ぐ。
void Encoder_SyncDelta(void);

#endif