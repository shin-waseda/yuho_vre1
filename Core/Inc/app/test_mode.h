#ifndef INC_TESTMODE_H_
#define INC_TESTMODE_H_


#include "global.h"
#include "params.h"

// センサー・ジャイロ・自己位置・速度PID調整用の値をUARTへ表示し続ける
// ブロッキングのテストループ。電源を切るまで戻らない。
void TestMode_Run(void);

#endif
