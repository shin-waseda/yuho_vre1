#include "app/led_test.h"

#include "main.h"
#include "interface/led.h"

// 1個あたりの点灯時間[ms]。
#define LED_TEST_STEP_MS 500

// 各周回の最初に全消灯する時間[ms]。
// ここで光っているLEDは、595の出力に関係なく点いていることになる。
#define LED_TEST_ALL_OFF_MS 2000

void LEDTest_Run(void) {
    printf("LED TEST\r\n");

    while (1) {
        printf("ALL OFF\r\n");
        LED_SetShiftPattern(0x0000u);
        HAL_Delay(LED_TEST_ALL_OFF_MS);

        for (int i = 0; i < LED_SHIFT_COUNT; i++) {
            // bit0〜7が1個目の595(SER直結側)、bit8〜15が2個目の595
            printf("LED %2d : 595#%d Q%d\r\n", i + 1, i / 8 + 1, i % 8);
            LED_SetShiftPattern((uint16_t)(1u << i));
            HAL_Delay(LED_TEST_STEP_MS);
        }
    }
}
