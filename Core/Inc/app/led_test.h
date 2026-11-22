#ifndef INC_LEDTEST_H_
#define INC_LEDTEST_H_


#include "global.h"
#include "params.h"

// シフトレジスタLEDの配線確認用のブロッキングテストループ。
// 全消灯→LED1〜16を1個ずつ点灯、を繰り返し、点灯中のLED番号と
// 595のどの出力に当たるかをUARTへ表示する。電源を切るまで戻らない。
void LEDTest_Run(void);

// カメラ(マーカーで位置を出すもの)の校正用。直結の LED_1〜6(機体の外周の6個)だけを点けたままにし、
// シフトレジスタの LED1〜16 は消す。電源を切るまで戻らない。
void CameraLED_Run(void);

#endif
