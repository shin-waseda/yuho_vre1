#include "main.h"
#include "interface/led.h"

static void Pulse(GPIO_TypeDef *port, uint16_t pin) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

// MSBから送ると、最初に送ったbit15が2個目の595のQ7まで押し出され、
// 最後に送ったbit0が1個目の595のQ0に収まる。
void LED_SetShiftPattern(uint16_t pattern) {
    for (int i = LED_SHIFT_COUNT - 1; i >= 0; i--) {
        GPIO_PinState bit = ((pattern >> i) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(SER_595_GPIO_Port, SER_595_Pin, bit);
        Pulse(SCLK_595_GPIO_Port, SCLK_595_Pin);
    }
    Pulse(Latch_595_GPIO_Port, Latch_595_Pin);
}
