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

// 直結LEDの機体上の位置(LED_SetDirectPattern に渡すビット。| で組み合わせる)
#define LED_FRONT_RIGHT ((uint8_t)(1u << 0)) // LED_1 右前
#define LED_LEFT        ((uint8_t)(1u << 1)) // LED_2 左
#define LED_FRONT_LEFT  ((uint8_t)(1u << 2)) // LED_3 左前
#define LED_RIGHT       ((uint8_t)(1u << 3)) // LED_4 右
#define LED_REAR_RIGHT  ((uint8_t)(1u << 4)) // LED_5 右後ろ
#define LED_REAR_LEFT   ((uint8_t)(1u << 5)) // LED_6 左後ろ
#define LED_DIRECT_ALL  ((uint8_t)((1u << LED_DIRECT_COUNT) - 1u))

// 6個の直結LEDの点灯状態をまとめて書き込む(1=点灯。ピンを High にすると光る。実機で確認済み)。
void LED_SetDirectPattern(uint8_t pattern);

#endif
