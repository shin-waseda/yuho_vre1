#include "app/mode_ui.h"

#include <math.h>
#include <stdio.h>
#include "main.h"
#include "interface/encoder.h"
#include "interface/button.h"
#include "app/sd_dump.h"
#include "app/sysid_mode.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "interface/sdcard.h"

static const char *ModeName(RobotMode mode) {
    switch (mode) {
        case MODE_SYSID_E0:    return "E0(start voltage)";
        case MODE_SYSID_E1:    return "E1(translation step)";
        case MODE_SYSID_E2:    return "E2(pivot step)";
        case MODE_SYSID_E2S:   return "E2S(pivot stair)";
        case MODE_SYSID_E1S:   return "E1S(forward steps)";
        case MODE_SYSID_E1V:   return "E1V(FF + yaw FB)";
        case MODE_SYSID_E1B:   return "E1B(brake)";
        case MODE_SYSID_E1T:   return "E1T(turn step)";
        case MODE_SYSID_E1K0:  return "E1K0(curve at V_d = 0)";
        case MODE_SYSID_E1W:   return "E1W(V_d jumps, b_w)";
        case MODE_SD_DUMP:     return "SD_DUMP";
        case MODE_SD_DUMP_ALL: return "SD_DUMP_ALL";
        default:               return "UNKNOWN";
    }
}

// モードの一覧。並び順がエンコーダで送る順になる。
static const RobotMode s_modes[] = {
    MODE_SYSID_E0, MODE_SYSID_E1, MODE_SYSID_E2, MODE_SYSID_E2S, MODE_SYSID_E1S, MODE_SYSID_E1V, MODE_SYSID_E1B,
    MODE_SYSID_E1T, MODE_SYSID_E1K0, MODE_SYSID_E1W, MODE_SD_DUMP, MODE_SD_DUMP_ALL,
};

#define MENU_COUNT_OF(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

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
    if (!Logger_SaveFile(path, sizeof(path))) {
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

bool ModeUI_WaitHandStartOrClick(void) {
    uint32_t above_since = HAL_GetTick();
    while (1) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        if (ButtonClicked()) {
            WaitButtonRelease();
            return true;
        }
        if (ad_fl <= SENSOR_START_THRESHOLD) {
            above_since = HAL_GetTick();
        } else if (HAL_GetTick() - above_since >= SENSOR_START_HOLD_MS) {
            break;
        }
        HAL_Delay(1);
    }
    // 手をかざした: 離すまで待つ(ModeUI_WaitHandStart と同じ)
    printf("hand detected (FL:%u)\r\n", ad_fl);
    LED_SetDirectPattern(LED_FRONT_LEFT);
    while (ad_fl > SENSOR_START_THRESHOLD) {
        if (FailSafe_IsTripped()) FailSafe_Halt();
        HAL_Delay(1);
    }
    LED_SetDirectPattern(0x00u);
    return false;
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
// (番号n はLED n, n+1を使うので、最大の番号は LED_SHIFT_COUNT-1)
#define SELECT_MAX_ITEMS (LED_SHIFT_COUNT - 1)
_Static_assert(MENU_COUNT_OF(s_modes) <= SELECT_MAX_ITEMS, "too many modes for shift LED display");

// 今選んでいる項目をLEDで示す。番号は1始まり(index 0 が 1番)で、番号n はLED n, n+1 の2個点灯。
// 例: 1番→LED1,2 / 2番→LED2,3
static void ShowIndex(uint8_t index) {
    LED_SetShiftPattern((uint16_t)(0x3u << index));
}

// 値(電圧など)を選んでいるときの表示: 番号n なら LED1〜n を点ける棒グラフ(1番が一番小さい値)。
static void ShowValueIndex(uint8_t index) {
    LED_SetShiftPattern((uint16_t)((1u << (index + 1u)) - 1u));
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

// 選んでいる項目の表示(UART と LED)。番号は1始まりで出す。
// s_show_values があれば値(ModeUI_SelectValue)、なければモードを選んでいる。
static const float *s_show_values = NULL;
static const char *s_show_name = "";
static const char *s_show_unit = "";

static void ShowItem(uint8_t i) {
    if (s_show_values != NULL) {
        printf("%s %d: %.0f %s\r\n", s_show_name, (int)i + 1, s_show_values[i], s_show_unit);
        ShowValueIndex(i);
        return;
    }
    printf("MODE %d: %s\r\n", (int)i + 1, ModeName(s_modes[i]));
    ShowIndex(i);
}

// 右エンコーダの回転で 0〜count-1 を送り(start から始める)、ボタンで確定した番号を返す。
// 回転はカウンタの値を直接読み、ここで差を取る(Encoder_GetDeltaR は使わない)。モードを決めた後は
// 1kHz の割り込みが Encoder_GetDeltaR で差分を持っていくので、それを使うと回しても動かなかった。
static uint8_t SelectIndex(uint8_t count, uint8_t start) {
    uint8_t index = start;
    float accumulated = 0.0f;

    uint16_t last = Encoder_GetCountR(); // ここから数える(前に決定するまでに回った分は数えない)
    ShowItem(index);

    while (1) {
        if (Button_IsPressed()) {
            HAL_Delay(20);
            if (Button_IsPressed()) {
                break;
            }
        }

        uint16_t now = Encoder_GetCountR();
        int16_t delta_r = (int16_t)(now - last); // 16bit の折り返しを使った差
        last = now;
        accumulated += (float)delta_r;

        while (accumulated >= MODE_SELECT_PULSES_PER_STEP) {
            accumulated -= MODE_SELECT_PULSES_PER_STEP;
            index = (uint8_t)((index + 1u) % count);
            ShowItem(index);
        }
        while (accumulated <= -MODE_SELECT_PULSES_PER_STEP) {
            accumulated += MODE_SELECT_PULSES_PER_STEP;
            index = (uint8_t)((index + count - 1u) % count);
            ShowItem(index);
        }

        HAL_Delay(10);
    }

    // 決定のボタンを離すまで待つ。でないと次の選択や各モードの最初のボタン待ちが、
    // 押されたままの決定ボタンを拾ってしまう。
    WaitButtonRelease();
    return index;
}

static RobotMode s_current_mode = MODE_SYSID_E0;

RobotMode ModeUI_CurrentMode(void) {
    return s_current_mode;
}

RobotMode ModeUI_Select(void) {
    s_show_values = NULL;
    RobotMode mode = s_modes[SelectIndex(MENU_COUNT_OF(s_modes), 0)];
    printf("MODE: %s\r\n", ModeName(mode));
    s_current_mode = mode;
    return mode;
}

float ModeUI_SelectValue(const char *name, const char *unit, const float *values, uint8_t count, float def) {
    return ModeUI_SelectValueFrom(name, unit, values, count, 0u, def);
}

float ModeUI_SelectValueFrom(const char *name, const char *unit, const float *values, uint8_t count, uint8_t start,
                             float def) {
    if (count == 0u) return def;
    if (count > SELECT_MAX_ITEMS) count = SELECT_MAX_ITEMS;
    printf("select %s (turn right wheel, press button to set)\r\n", name);
    s_show_values = values;
    s_show_name = name;
    s_show_unit = unit;
    if (start >= count) start = 0u;
    uint8_t i = SelectIndex(count, start);
    s_show_values = NULL;
    LED_SetShiftPattern(0x0000u);
    printf("%s: %.0f %s\r\n", name, values[i], unit);
    return values[i];
}

void ModeUI_Run(RobotMode mode) {
    switch (mode) {
        case MODE_SYSID_E0:
            SysidMode_Run(SYSID_E0);
            break;
        case MODE_SYSID_E1:
            SysidMode_Run(SYSID_E1);
            break;
        case MODE_SYSID_E2:
            SysidMode_Run(SYSID_E2);
            break;
        case MODE_SYSID_E2S:
            SysidMode_Run(SYSID_E2S);
            break;
        case MODE_SYSID_E1S:
            SysidMode_Run(SYSID_E1S);
            break;
        case MODE_SYSID_E1V:
            SysidMode_Run(SYSID_E1V);
            break;
        case MODE_SYSID_E1B:
            SysidMode_Run(SYSID_E1B);
            break;
        case MODE_SYSID_E1T:
            SysidMode_Run(SYSID_E1T);
            break;
        case MODE_SYSID_E1K0:
            SysidMode_Run(SYSID_E1K0);
            break;
        case MODE_SYSID_E1W:
            SysidMode_Run(SYSID_E1W);
            break;
        case MODE_SD_DUMP:
            SdDump_Run();
            break;
        case MODE_SD_DUMP_ALL:
            SdDumpAll_Run();
            break;
        default:
            break;
    }
}
