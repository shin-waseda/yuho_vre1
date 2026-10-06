#include "app/mode_ui.h"

#include "main.h"
#include "interface/encoder.h"
#include "interface/button.h"
#include "app/test_mode.h"
#include "app/vel_pid_test.h"
#include "app/led_test.h"
#include "app/straight_test.h"
#include "app/sd_dump.h"
#include "app/pivot_test.h"
#include "app/party_mode.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "interface/sdcard.h"

static const char *ModeName(RobotMode mode) {
    switch (mode) {
        case MODE_SENSOR:        return "SENSOR";
        case MODE_VEL_PID:       return "VEL_PID";
        case MODE_STRAIGHT_TEST: return "STRAIGHT";
        case MODE_PIVOT_TEST:    return "PIVOT";
        case MODE_LED_TEST:      return "LED_TEST";
        case MODE_PARTY:         return "PARTY";
        case MODE_SD_DUMP:       return "SD_DUMP";
        case MODE_SD_DUMP_ALL:   return "SD_DUMP_ALL";
        default:                 return "UNKNOWN";
    }
}

// ---- メニューの階層 ----
// 一番上の階層の各項目が、その中のモードの一覧を持つ。並び順がエンコーダで送る順になる。
static const RobotMode s_test_modes[] = {
    MODE_SENSOR, MODE_VEL_PID, MODE_STRAIGHT_TEST, MODE_PIVOT_TEST, MODE_LED_TEST, MODE_PARTY,
};
static const RobotMode s_sd_modes[] = {
    MODE_SD_DUMP, MODE_SD_DUMP_ALL,
};

typedef struct {
    const char *name;
    const RobotMode *modes;
    uint8_t count;
} ModeMenu;

#define MENU_COUNT_OF(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

static const ModeMenu s_menus[] = {
    { "TEST", s_test_modes, MENU_COUNT_OF(s_test_modes) },
    { "SD",   s_sd_modes,   MENU_COUNT_OF(s_sd_modes) },
};
#define MENU_TOP_COUNT MENU_COUNT_OF(s_menus)

static void WaitButtonRelease(void) {
    while (Button_IsPressed()) {
        HAL_Delay(10);
    }
    HAL_Delay(20); // 離した直後のチャタリングを読まない
}

#define SD_SAVE_OK_LIGHT_MS   500 // 保存できたときに全部点灯する時間
#define ERROR_BLINK_HALF_MS   50  // エラーの点滅の半周期(0.1秒周期)

SdSaveResult ModeUI_SaveLogToSD(void) {
    if (!SDCard_IsMounted()) {
        printf("SD: not mounted, skip saving\r\n");
        return SD_SAVE_SKIPPED;
    }
    char path[64];
    if (!Logger_SaveCSV(path, sizeof(path))) {
        printf("SD: save failed\r\n");
        return SD_SAVE_FAILED;
    }
    printf("SD: saved %s\r\n", path);
    LED_SetDirectPattern(LED_DIRECT_ALL);
    HAL_Delay(SD_SAVE_OK_LIGHT_MS);
    LED_SetDirectPattern(0x00u);
    return SD_SAVE_OK;
}

// ボタンが押されたら(チャタリングを除いて)true
static bool ButtonClicked(void) {
    if (!Button_IsPressed()) return false;
    HAL_Delay(20);
    return Button_IsPressed();
}

void ModeUI_WaitClick(void) {
    while (1) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        if (ButtonClicked()) break;
        HAL_Delay(10);
    }
    WaitButtonRelease();
}

void ModeUI_WaitClickBlinking(uint8_t leds) {
    bool on = false;
    uint32_t last_toggle = HAL_GetTick() - ERROR_BLINK_HALF_MS; // すぐ1回目を点ける
    while (1) {
        if (FailSafe_IsTripped()) FailSafe_Halt(); // フェイルセーフの表示に切り替わる
        if (ButtonClicked()) break;
        if (HAL_GetTick() - last_toggle >= ERROR_BLINK_HALF_MS) {
            last_toggle += ERROR_BLINK_HALF_MS;
            on = !on;
            LED_SetDirectPattern(on ? leds : 0x00u);
        }
        HAL_Delay(1);
    }
    LED_SetDirectPattern(0x00u);
    WaitButtonRelease();
}

void ModeUI_WaitHandStart(void) {
    // かざす: 左前のセンサーがしきい値を続けて超えるまで待つ
    uint32_t above_since = HAL_GetTick();
    while (1) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        if (ad_fl <= SENSOR_START_THRESHOLD) {
            above_since = HAL_GetTick();
        } else if (HAL_GetTick() - above_since >= SENSOR_START_HOLD_MS) {
            break;
        }
        HAL_Delay(1);
    }
    printf("hand detected (FL:%u)\r\n", ad_fl); // しきい値を見直すときの目安
    LED_SetDirectPattern(LED_FRONT_LEFT);

    // 離す: しきい値を下回るまで待つ
    while (ad_fl > SENSOR_START_THRESHOLD) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        HAL_Delay(1);
    }
    LED_SetDirectPattern(0x00u);
}

// 2個点灯の窓をずらして表示するため、窓が収まる項目数に制限する。
// (番号m≥1はLED m, m+1を使うので、最大の番号は LED_SHIFT_COUNT-1)
_Static_assert(MENU_COUNT_OF(s_menus) <= LED_SHIFT_COUNT, "too many top menus for shift LED display");
_Static_assert(MENU_COUNT_OF(s_test_modes) <= LED_SHIFT_COUNT, "too many TEST modes for shift LED display");
_Static_assert(MENU_COUNT_OF(s_sd_modes) <= LED_SHIFT_COUNT, "too many SD modes for shift LED display");

// 階層ごとの、今選んでいる項目の番号をLEDで示す。番号0は全点灯、番号m(≥1)はLED m, m+1 (1始まり)の2個点灯。
// 例: 0番目→全点灯 / 1番目→LED1,2
static void ShowIndex(uint8_t index) {
    if (index == 0) {
        LED_SetShiftPattern(0xFFFFu);
    } else {
        LED_SetShiftPattern((uint16_t)(0x3u << (index - 1)));
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

// menu が NULL なら一番上の階層、そうでなければその中のモードの i 番目を表示する
static void ShowItem(const ModeMenu *menu, uint8_t i) {
    if (menu == NULL) {
        printf("MENU %d: %s\r\n", (int)i, s_menus[i].name);
    } else {
        printf("%s %d: %s\r\n", menu->name, (int)i, ModeName(menu->modes[i]));
    }
    ShowIndex(i);
}

// 右エンコーダの回転で 0〜count-1 を送り、ボタンで確定した番号を返す。
static uint8_t SelectIndex(const ModeMenu *menu, uint8_t count) {
    uint8_t index = 0;
    float accumulated = 0.0f;

    (void)Encoder_GetDeltaR(); // 前の階層で決定するまでに回った分を捨てる
    ShowItem(menu, index);

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
            index = (uint8_t)((index + 1u) % count);
            ShowItem(menu, index);
        }
        while (accumulated <= -MODE_SELECT_PULSES_PER_STEP) {
            accumulated += MODE_SELECT_PULSES_PER_STEP;
            index = (uint8_t)((index + count - 1u) % count);
            ShowItem(menu, index);
        }

        HAL_Delay(10);
    }

    // 決定のボタンを離すまで待つ。でないと次の階層や各モードの最初のボタン待ちが、
    // 押されたままの決定ボタンを拾ってしまう。
    WaitButtonRelease();
    return index;
}

RobotMode ModeUI_Select(void) {
    const ModeMenu *menu = &s_menus[SelectIndex(NULL, MENU_TOP_COUNT)];
    RobotMode mode = menu->modes[SelectIndex(menu, menu->count)];
    printf("MODE: %s / %s\r\n", menu->name, ModeName(mode));
    return mode;
}

void ModeUI_Run(RobotMode mode) {
    switch (mode) {
        case MODE_SENSOR:
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
        case MODE_PARTY:
            PartyMode_Run();
            break;
        case MODE_SD_DUMP:
            SdDump_Run();
            break;
        case MODE_SD_DUMP_ALL:
            SdDumpAll_Run();
            break;
        case MODE_PIVOT_TEST:
            PivotTest_Run();
            break;
        default:
            break;
    }
}
