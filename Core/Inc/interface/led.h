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

// マイコンに直結したLED(main.h の LED_1〜LED_6)の個数。
// LED n (1始まり) が pattern の bit(n-1) に対応する。
// シフトレジスタを通らないので、595 や配線が壊れていても光る(エラー表示用)。
#define LED_DIRECT_COUNT 6

// 6個の直結LEDの点灯状態をまとめて書き込む(1=点灯。ピンを High にすると光る想定で、極性は未確認)。
void LED_SetDirectPattern(uint8_t pattern);

#endif
