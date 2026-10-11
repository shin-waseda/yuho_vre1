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

// 右のカウンタの今の値(16bit。読んでも GetDeltaR の前回値は変わらない)。
// 制御の割り込みが GetDeltaR を使っている間に、メインで右タイヤの回転を読むとき(モードの UI の値選び)に使う。
// 差は呼び出し側で (int16_t)(now - last) として取る。
uint16_t Encoder_GetCountR(void);

#endif