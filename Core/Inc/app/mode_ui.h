#ifndef INC_MODEUI_H_
#define INC_MODEUI_H_


#include "global.h"

#include "interface/led.h"
#include "interface/uart.h"
#include "robot_state.h"

typedef enum {
    MODE_TEST = 0,
    MODE_COUNT // 現在はTESTのみ。モードを増やす時はここに追加する
} RobotMode;

// 起動時に一度呼ぶブロッキング処理。右エンコーダの回転
// (MODE_SELECT_PULSES_PER_STEPごと)でモードを送り、ボタン押下で
// 確定して選択されたモードを返す。
RobotMode ModeUI_Select(void);

// 選択されたモードを実行する。TEST以外は現状未実装(何もしない)。
void ModeUI_Run(RobotMode mode);

#endif