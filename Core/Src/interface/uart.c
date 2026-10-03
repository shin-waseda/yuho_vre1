#include "main.h"
#include "usart.h" // huart1 の宣言
#include "interface/uart.h"

// HAL_UART_Transmit()の長さはuint16_tなので、それを超えないよう分けて送る。
#define UART_CHUNK_BYTES 0x8000u

void UART_WriteBytes(const uint8_t *data, uint32_t len) {
    while (len > 0) {
        uint16_t n = (len > UART_CHUNK_BYTES) ? (uint16_t)UART_CHUNK_BYTES : (uint16_t)len;
        HAL_UART_Transmit(&huart1, (uint8_t *)data, n, HAL_MAX_DELAY);
        data += n;
        len -= n;
    }
}
