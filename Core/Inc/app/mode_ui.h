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

// 試験モードの走行後、記録したログをSDカードへCSVで保存し、結果をUARTとLEDで示す。
// LED: 成功=LED1〜7点灯 / 失敗=LED1,3,5,7点灯 / SDなし=変えない。次の操作まで表示が残る。
void ModeUI_SaveLogToSD(void);

// ボタンが押されて離されるまで待つ(チャタリング除去付き)。
// 待っている間にフェイルセーフが発動したらFailSafe_Halt()へ入る(戻らない)。
void ModeUI_WaitClick(void);

// 選択されたモードを実行する。未知のモードは何もしない。
void ModeUI_Run(RobotMode mode);

#endif