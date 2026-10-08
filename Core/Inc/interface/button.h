#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_


#include "global.h"
#include "params.h"

// Push_IN_1 (PA6) の状態を返す。
// 注: gpio.cではGPIO_NOPULL設定になっており、外部プルアップ/プルダウンの
// 有無を確認できていない。ここでは「押下時にLOWになるアクティブLOW」と
// 仮定している。実機で逆(常にHIGHのまま反応しない等)なら反転すること。
bool Button_IsPressed(void);

#endif
