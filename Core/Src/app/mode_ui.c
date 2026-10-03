#include "app/mode_ui.h"

#include "main.h"
#include "interface/encoder.h"
#include "interface/button.h"
#include "app/test_mode.h"
#include "app/vel_pid_test.h"
#include "app/led_test.h"
#include "app/straight_test.h"
#include "app/failsafe.h"

static const char *ModeName(RobotMode mode) {
    switch (mode) {
        case MODE_TEST:          return "TEST";
        case MODE_VEL_PID:       return "VEL_PID";
        case MODE_LED_TEST:      return "LED_TEST";
        case MODE_STRAIGHT_TEST: return "STRAIGHT";
        default:                 return "EMPTY";
    }
}

static void WaitButtonRelease(void) {
    while (Button_IsPressed()) {
        HAL_Delay(10);
    }
    HAL_Delay(20); // 離した直後のチャタリングを読まない
}

void ModeUI_WaitClick(void) {
    while (1) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        if (Button_IsPressed()) {
            HAL_Delay(20);
            if (Button_IsPressed()) break;
        }
        HAL_Delay(10);
    }
    WaitButtonRelease();
}

// 2個点灯の窓をずらして表示するため、窓が収まるモード数に制限する。
// (モードm≥1はLED m, m+1を使うので、最大モード番号は LED_SHIFT_COUNT-1)
_Static_assert(MODE_COUNT <= LED_SHIFT_COUNT, "MODE_COUNT exceeds shift LED display range");

// モード0は全点灯。モードm(≥1)はLED m, m+1 (1始まり)の2個点灯で示す。
// 例: TEST(0)→全点灯 / VEL_PID(1)→LED1,2
static void ShowMode(RobotMode mode) {
    printf("MODE %2d: %s\r\n", (int)mode, ModeName(mode));
    if (mode == 0) {
        LED_SetShiftPattern(0xFFFFu);
    } else {
        LED_SetShiftPattern((uint16_t)(0x3u << (mode - 1)));
    }
}

// U6のはんだ不良対策で、確実に光るLED1〜7(D23〜D17)だけを使う。
// 基板修理後はLED_SHIFT_COUNTに戻してよい。
#define MODE_UI_BAR_LED_COUNT 7

void ModeUI_ShowBattery(float vbat, uint32_t hold_ms) {
    float ratio = (vbat - FAILSAFE_LOW_VOLTAGE_V) / (BATTERY_FULL_V - FAILSAFE_LOW_VOLTAGE_V);
    int n;
    if (ratio <= 0.0f) {
        n = 0;
    } else if (ratio >= 1.0f) {
        n = MODE_UI_BAR_LED_COUNT;
    } else {
        n = (int)(ratio * MODE_UI_BAR_LED_COUNT) + 1; // しきい値を少しでも超えていれば1個は点ける
        if (n > MODE_UI_BAR_LED_COUNT) n = MODE_UI_BAR_LED_COUNT;
    }

    LED_SetShiftPattern((uint16_t)((1u << n) - 1u));
    HAL_Delay(hold_ms);
}

RobotMode ModeUI_Select(void) {
    RobotMode mode = MODE_TEST;
    float accumulated = 0.0f;

    ShowMode(mode);

    while (1) {
        if (Button_IsPressed()) {
            HAL_Delay(20);
            if (Button_IsPressed()) {
                break;
            }
        }

        int16_t delta_r = Encoder_GetDeltaR();
        accumulated += (float)delta_r;

        while (accumulated >= MODE_SELECT_PULSES_PER_STEP) {
            accumulated -= MODE_SELECT_PULSES_PER_STEP;
            mode = (RobotMode)((mode + 1) % MODE_COUNT);
            ShowMode(mode);
        }
        while (accumulated <= -MODE_SELECT_PULSES_PER_STEP) {
            accumulated += MODE_SELECT_PULSES_PER_STEP;
            mode = (RobotMode)((mode - 1 + MODE_COUNT) % MODE_COUNT);
            ShowMode(mode);
        }

        HAL_Delay(10);
    }

    // 決定のボタンを離すまで待つ。でないと各モードの最初のボタン待ちが、
    // 押されたままの決定ボタンを拾ってしまう。
    WaitButtonRelease();
    return mode;
}

void ModeUI_Run(RobotMode mode) {
    switch (mode) {
        case MODE_TEST:
            TestMode_Run();
            break;
        case MODE_VEL_PID:
            VelPIDTest_Run();
            break;
        case MODE_LED_TEST:
            LEDTest_Run();
            break;
        case MODE_STRAIGHT_TEST:
            StraightTest_Run();
            break;
        default:
            break;
    }
}
