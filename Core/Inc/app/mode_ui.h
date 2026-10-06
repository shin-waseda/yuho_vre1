#ifndef INC_MODEUI_H_
#define INC_MODEUI_H_


#include "global.h"

#include "interface/led.h"
#include "interface/uart.h"
#include "robot_state.h"

typedef enum {
    MODE_TEST = 0,
    MODE_VEL_PID,
    MODE_LED_TEST,
    MODE_STRAIGHT_TEST,
    MODE_SD_DUMP,
    MODE_SD_DUMP_ALL,
    MODE_PIVOT_TEST,
    // 空きモード。中身を実装する時は名前を付け替える
    MODE_7,
    MODE_8,
    MODE_9,
    MODE_10,
    MODE_11,
    MODE_12,
    MODE_13,
    MODE_14,
    MODE_15,
    MODE_COUNT // シフトレジスタLED表示の都合で最大16(mode_ui.cの_Static_assert参照)
} RobotMode;

// 起動時のバッテリー残量をLEDバーで表示する(ブロッキング、hold_ms待つ)。
// FAILSAFE_LOW_VOLTAGE_V〜BATTERY_FULL_Vを、現在光るLED(1〜MODE_UI_BAR_LED_COUNT)
// の点灯本数に線形に割り当てる。しきい値未満は全消灯。
void ModeUI_ShowBattery(float vbat, uint32_t hold_ms);

// 起動時に一度呼ぶブロッキング処理。右エンコーダの回転
// (MODE_SELECT_PULSES_PER_STEPごと)でモードを送り、ボタン押下で
// 確定して選択されたモードを返す。
RobotMode ModeUI_Select(void);

typedef enum {
    SD_SAVE_SKIPPED, // SDカードがない(マウントされていない)ので保存しなかった
    SD_SAVE_OK,
    SD_SAVE_FAILED,
} SdSaveResult;

// 試験モードの走行後、記録したログをSDカードへCSVで保存し、結果をUARTに出す。
// 成功ならマイコン直結のLED全部を0.5秒点灯して消す(ブロッキング)。
// 失敗のときはLEDを変えずに SD_SAVE_FAILED を返すので、呼び出し側が
// 次のボタン待ちを ModeUI_WaitClickBlinking(MODE_UI_LED_SD_ERROR) にして知らせる。
SdSaveResult ModeUI_SaveLogToSD(void);

// SDへの保存の失敗を知らせる直結LED(左後ろ = LED_6)
#define MODE_UI_LED_SD_ERROR LED_REAR_LEFT

// ボタンが押されて離されるまで待つ(チャタリング除去付き)。
// 待っている間にフェイルセーフが発動したらFailSafe_Halt()へ入る(戻らない)。
void ModeUI_WaitClick(void);

// ModeUI_WaitClick() と同じだが、待っている間マイコン直結のLED(leds のビット。led.h の
// LED_FRONT_RIGHT など)を0.1秒周期で点滅させる(エラー表示)。押されたらLEDを消して戻る。
void ModeUI_WaitClickBlinking(uint8_t leds);

// 非接触スタート。左前の壁センサー(FL)に手をかざして(SENSOR_START_THRESHOLD を
// SENSOR_START_HOLD_MS 続けて超える)、離すまで待つ。かざしている間は左前の直結LEDを点ける。
// 待っている間にフェイルセーフが発動したらFailSafe_Halt()へ入る(戻らない)。
void ModeUI_WaitHandStart(void);

// 選択されたモードを実行する。未知のモードは何もしない。
void ModeUI_Run(RobotMode mode);

#endif