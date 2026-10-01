#include "app/mode_ui.h"

#include "main.h"
#include "interface/encoder.h"
#include "interface/button.h"
#include "app/test_mode.h"

RobotMode ModeUI_Select(void) {
    RobotMode mode = MODE_TEST;
    float accumulated = 0.0f;

    while (1) {
        if (Button_IsPressed()) {
            HAL_Delay(20); // デバウンス
            if (Button_IsPressed()) {
                break;
            }
        }

        int16_t delta_r = Encoder_GetDeltaR();
        accumulated += (float)delta_r;

        while (accumulated >= MODE_SELECT_PULSES_PER_STEP) {
            accumulated -= MODE_SELECT_PULSES_PER_STEP;
            mode = (RobotMode)((mode + 1) % MODE_COUNT);
        }
        while (accumulated <= -MODE_SELECT_PULSES_PER_STEP) {
            accumulated += MODE_SELECT_PULSES_PER_STEP;
            mode = (RobotMode)((mode - 1 + MODE_COUNT) % MODE_COUNT);
        }

        HAL_Delay(10);
    }

    return mode;
}

void ModeUI_Run(RobotMode mode) {
    switch (mode) {
        case MODE_TEST:
            TestMode_Run();
            break;
        default:
            break;
    }
}
