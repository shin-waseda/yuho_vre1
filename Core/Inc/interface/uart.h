#ifndef INC_UART_H_
#define INC_UART_H_


#include "global.h"
#include "params.h"

// USART1へバイト列をそのまま送る(ブロッキング)。printf(文字列)と同じ線を使う。
// ログの本体(float32の生データ)の送信用。
void UART_WriteBytes(const uint8_t *data, uint32_t len);

#endif
