#include "app/mode_ui.h"

#include <math.h>
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
#include "app/sensor_log.h"
#include "app/search_run.h"
#include "app/slalom_test.h"
#include "app/stream_test.h"
#include "app/sensor_spin.h"
#include "app/failsafe.h"
#include "app/logger.h"
#include "interface/sdcard.h"

static const char *ModeName(RobotMode mode) {
    switch (mode) {
        case MODE_SENSOR:        return "SENSOR";
        case MODE_SENSOR_LOG:    return "SENSOR_LOG";
        case MODE_VEL_PID:       return "VEL_PID";
        case MODE_STRAIGHT_TEST: return "STRAIGHT";
        case MODE_PIVOT_TEST:    return "PIVOT";
        case MODE_SLALOM_TEST:   return "SLALOM";
        case MODE_LED_TEST:      return "LED_TEST";
        case MODE_PARTY:         return "PARTY";
        case MODE_SEARCH:        return "SEARCH";
        case MODE_SEARCH_ADACHI: return "SEARCH_ADACHI";
        case MODE_FAST_RUN:      return "FAST_RUN";
        case MODE_SD_DUMP:       return "SD_DUMP";
        case MODE_SD_DUMP_ALL:   return "SD_DUMP_ALL";
        case MODE_STREAM_TEST:   return "STREAM_TEST";
        case MODE_LONG_LOG:      return "LONG_LOG";
        case MODE_SLALOM_SWEEP:  return "SLALOM_SWEEP";
        case MODE_STRAIGHT_SWEEP: return "STRAIGHT_SWEEP";
        case MODE_FAST_SWEEP:    return "FAST_SWEEP";
        case MODE_SENSOR_SPIN:   return "SENSOR_SPIN";
        case MODE_SEARCH_SPIN:   return "SEARCH_SPIN";
        case MODE_FAST_BANDS:    return "FAST_BANDS";
        case MODE_RUN_TEST:      return "TEST";
        default:                return "UNKNOWN";
    }
}

// ---- メニューの階層 ----
// 一番上の階層の各項目が、その中のモードの一覧を持つ。並び順がエンコーダで送る順になる。
static const RobotMode s_test_modes[] = {
    MODE_SENSOR, MODE_SENSOR_LOG, MODE_VEL_PID, MODE_STRAIGHT_TEST, MODE_PIVOT_TEST, MODE_SLALOM_TEST, MODE_LED_TEST, MODE_PARTY,
    MODE_LONG_LOG, MODE_SLALOM_SWEEP, MODE_STRAIGHT_SWEEP, MODE_SENSOR_SPIN,
};
static const RobotMode s_run_modes[] = {
    // SEARCH は地図・行き先・アルゴリズムなどを中で選ぶ(SearchMenu_Run)。ログ取りの走行は TEST の中で選ぶ(RunTest_Run)
    MODE_SEARCH, MODE_FAST_RUN, MODE_RUN_TEST,
};
static const RobotMode s_sd_modes[] = {
    MODE_SD_DUMP, MODE_SD_DUMP_ALL, MODE_STREAM_TEST,
};
// ログ取りのモードへのショートカット(一番上の階層で選ぶとすぐ決まる。TEST / RUN の中にも残してある)。
// preset があれば、そのモードで値を選ぶ所(ModeUI_SelectValue)を、選ぶ順に preset の値で埋めて飛ばす(値は params.h の SHORTCUT_*)
static const RobotMode s_long_log_mode[] = { MODE_LONG_LOG };
static const RobotMode s_slalom_sweep_mode[] = { MODE_SLALOM_SWEEP };
static const RobotMode s_straight_sweep_mode[] = { MODE_STRAIGHT_SWEEP };
static const RobotMode s_fast_sweep_mode[] = { MODE_FAST_SWEEP };
static const RobotMode s_sensor_spin_mode[] = { MODE_SENSOR_SPIN };
static const float s_preset_straight_v[] = SHORTCUT_STRAIGHT_V;
static const float s_preset_straight_acc[] = SHORTCUT_STRAIGHT_ACC;
static const float s_preset_slalom_low[] = SHORTCUT_SLALOM_LOW;
static const float s_preset_slalom_high[] = SHORTCUT_SLALOM_HIGH;
static const float s_preset_long_log[] = SHORTCUT_LONG_LOG;

typedef struct {
    const char *name;
    const RobotMode *modes;
    uint8_t count;
    const float *preset;  // NULL なら値を選ぶ
    uint8_t preset_count;
} ModeMenu;

#define MENU_COUNT_OF(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

static const ModeMenu s_menus[] = {
    { "RUN",  s_run_modes,  MENU_COUNT_OF(s_run_modes),  NULL, 0u },
    { "TEST", s_test_modes, MENU_COUNT_OF(s_test_modes), NULL, 0u },
    { "SD",   s_sd_modes,   MENU_COUNT_OF(s_sd_modes),   NULL, 0u },
    { "STRAIGHT_SWEEP(v)",   s_straight_sweep_mode, 1u, s_preset_straight_v,   MENU_COUNT_OF(s_preset_straight_v) },
    { "STRAIGHT_SWEEP(acc)", s_straight_sweep_mode, 1u, s_preset_straight_acc, MENU_COUNT_OF(s_preset_straight_acc) },
    { "SLALOM_SWEEP(low)",   s_slalom_sweep_mode,   1u, s_preset_slalom_low,   MENU_COUNT_OF(s_preset_slalom_low) },
    { "SLALOM_SWEEP(high)",  s_slalom_sweep_mode,   1u, s_preset_slalom_high,  MENU_COUNT_OF(s_preset_slalom_high) },
    { "FAST_SWEEP",          s_fast_sweep_mode,     1u, NULL, 0u }, // 範囲は解析の後で決めるので選ぶ
    { "LONG_LOG",            s_long_log_mode,       1u, s_preset_long_log,     MENU_COUNT_OF(s_preset_long_log) },
    { "SENSOR_SPIN",         s_sensor_spin_mode,    1u, NULL, 0u }, // 値を選ぶ所はない
};

// 選んだショートカットの値と、次に使う番号
static const float *s_preset = NULL;
static uint8_t s_preset_count = 0u;
static uint8_t s_preset_next = 0u;
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
_Static_assert(MENU_COUNT_OF(s_menus) <= SELECT_MAX_ITEMS, "too many top menus for shift LED display");
_Static_assert(MENU_COUNT_OF(s_test_modes) <= SELECT_MAX_ITEMS, "too many TEST modes for shift LED display");
_Static_assert(MENU_COUNT_OF(s_sd_modes) <= SELECT_MAX_ITEMS, "too many SD modes for shift LED display");
_Static_assert(MENU_COUNT_OF(s_run_modes) <= SELECT_MAX_ITEMS, "too many RUN modes for shift LED display");

// 今選んでいる項目をLEDで示す。番号は1始まり(index 0 が 1番)で、番号n はLED n, n+1 の2個点灯。
// 例: 1番→LED1,2 / 2番→LED2,3
static void ShowIndex(uint8_t index) {
    LED_SetShiftPattern((uint16_t)(0x3u << index));
}

// 値(速さなど)を選んでいるときの表示: 番号n なら LED1〜n を点ける棒グラフ(1番が一番遅い値)。
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
// s_show_values があれば値(ModeUI_SelectValue)、なければ s_show_menu が NULL なら一番上の階層、
// そうでなければその中のモードを選んでいる。
static const ModeMenu *s_show_menu = NULL;
static const float *s_show_values = NULL;
static const char *s_show_name = "";
static const char *s_show_unit = "";

static void ShowItem(uint8_t i) {
    if (s_show_values != NULL) {
        printf("%s %d: %.0f %s\r\n", s_show_name, (int)i + 1, s_show_values[i], s_show_unit);
        ShowValueIndex(i);
        return;
    } else if (s_show_menu == NULL) {
        printf("MENU %d: %s\r\n", (int)i + 1, s_menus[i].name);
    } else {
        printf("%s %d: %s\r\n", s_show_menu->name, (int)i + 1, ModeName(s_show_menu->modes[i]));
    }
    ShowIndex(i);
}

// 右エンコーダの回転で 0〜count-1 を送り(start から始める)、ボタンで確定した番号を返す。
// 回転はカウンタの値を直接読み、ここで差を取る(Encoder_GetDeltaR は使わない)。モードを決めた後は
// 1kHz の制御の割り込みが Encoder_GetDeltaR で差分を持っていくので、それを使うと回しても動かなかった。
static uint8_t SelectIndex(uint8_t count, uint8_t start) {
    uint8_t index = start;
    float accumulated = 0.0f;

    uint16_t last = Encoder_GetCountR(); // ここから数える(前の階層で決定するまでに回った分は数えない)
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

    // 決定のボタンを離すまで待つ。でないと次の階層や各モードの最初のボタン待ちが、
    // 押されたままの決定ボタンを拾ってしまう。
    WaitButtonRelease();
    return index;
}

static RobotMode s_current_mode = MODE_SENSOR;

RobotMode ModeUI_CurrentMode(void) {
    return s_current_mode;
}

RobotMode ModeUI_Select(void) {
    s_show_values = NULL;
    s_show_menu = NULL;
    const ModeMenu *menu = &s_menus[SelectIndex(MENU_TOP_COUNT, 0)];
    // 中のモードが1つだけなら、もう一度選ばせずにそれに決める
    s_show_menu = menu;
    RobotMode mode = (menu->count == 1) ? menu->modes[0] : menu->modes[SelectIndex(menu->count, 0)];
    printf("MODE: %s / %s\r\n", menu->name, ModeName(mode));
    s_current_mode = mode;
    s_preset = menu->preset;
    s_preset_count = menu->preset_count;
    s_preset_next = 0u;
    return mode;
}

float ModeUI_SelectValue(const char *name, const char *unit, const float *values, uint8_t count, float def) {
    return ModeUI_SelectValueFrom(name, unit, values, count, 0u, def);
}

float ModeUI_SelectValueFrom(const char *name, const char *unit, const float *values, uint8_t count, uint8_t start,
                             float def) {
    if (count == 0u) return def;
    if (count > SELECT_MAX_ITEMS) count = SELECT_MAX_ITEMS;
    // ショートカットの値があれば、選ばずにそれを使う(表にない値なら、ふつうに選ぶ)
    if (s_preset != NULL && s_preset_next < s_preset_count) {
        float v = s_preset[s_preset_next++];
        for (uint8_t k = 0; k < count; k++) {
            if (values[k] == v) {
                printf("%s: %.0f %s (shortcut)\r\n", name, v, unit);
                return v;
            }
        }
        printf("shortcut value %.0f is not in the list of %s: select it\r\n", v, name);
    }
    printf("select %s (turn right wheel, press button to set)\r\n", name);
    s_show_values = values;
    s_show_name = name;
    s_show_unit = unit;
    if (start >= count) start = 0u;
    uint8_t i = SelectIndex(count, start); // ModeUI_SelectValue は1番(一番遅い値)から始める
    s_show_values = NULL;
    LED_SetShiftPattern(0x0000u);
    printf("%s: %.0f %s\r\n", name, values[i], unit);
    return values[i];
}

// RUN の中の TEST: ログ取りの走行を、値を選ぶのと同じ画面(棒グラフ)で選んで始める
static void RunTest_Run(void) {
    static const RobotMode kTests[] = { MODE_FAST_SWEEP, MODE_SEARCH_SPIN, MODE_FAST_BANDS, MODE_LONG_LOG };
    static const float kValues[] = { 1.0f, 2.0f, 3.0f, 4.0f };
    printf("RUN TEST: 1 FAST_SWEEP, 2 SEARCH_SPIN, 3 FAST_BANDS, 4 LONG_LOG\r\n");
    uint8_t n = (uint8_t)ModeUI_SelectValue("TEST(1 FAST_SWEEP 2 SEARCH_SPIN 3 FAST_BANDS 4 LONG_LOG)", "", kValues,
                                            MENU_COUNT_OF(kValues), 1.0f);
    RobotMode mode = kTests[n - 1u];
    printf("MODE: RUN / TEST / %s\r\n", ModeName(mode));
    s_current_mode = mode; // ログの MODE に、選んだ走行を残す
    ModeUI_Run(mode);
}

void ModeUI_Run(RobotMode mode) {
    switch (mode) {
        case MODE_SENSOR:
            TestMode_Run();
            break;
        case MODE_SENSOR_LOG:
            SensorLog_Run();
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
        case MODE_SEARCH:
            SearchMenu_Run();
            break;
        case MODE_SEARCH_ADACHI:
            SearchRun_Run(SEARCH_ALGO_ADACHI);
            break;
        case MODE_FAST_RUN:
            FastRun_Run();
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
        case MODE_STREAM_TEST:
            StreamTest_Run();
            break;
        case MODE_PIVOT_TEST:
            PivotTest_Run();
            break;
        case MODE_SLALOM_TEST:
            SlalomTest_Run();
            break;
        case MODE_LONG_LOG:
            LongLogRun_Run();
            break;
        case MODE_SLALOM_SWEEP:
            SlalomSweep_Run();
            break;
        case MODE_STRAIGHT_SWEEP:
            StraightSweep_Run();
            break;
        case MODE_FAST_SWEEP:
            FastSweep_Run();
            break;
        case MODE_SENSOR_SPIN:
            SensorSpin_Run();
            break;
        case MODE_SEARCH_SPIN:
            SearchSpin_Run();
            break;
        case MODE_FAST_BANDS:
            FastBands_Run();
            break;
        case MODE_RUN_TEST:
            RunTest_Run();
            break;
        default:
            break;
    }
}
