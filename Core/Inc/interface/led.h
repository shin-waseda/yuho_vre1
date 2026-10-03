#ifndef INC_LED_H_
#define INC_LED_H_


#include "global.h"
#include "params.h"

// 74HC595×2のデイジーチェーンにつながったLEDの個数。
// LED n (1始まり) が pattern の bit(n-1) に対応する。
// bit0 = SER_595が直結している1個目の595のQ0。
#define LED_SHIFT_COUNT 16

// 16個のLEDの点灯状態をまとめて書き込む(1=点灯)。
// 16bitをシフトし終えてからラッチするので、途中の状態は表示されない。
void LED_SetShiftPattern(uint16_t pattern);

#endif
