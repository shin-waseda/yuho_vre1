#ifndef INC_STRAIGHTTEST_H_
#define INC_STRAIGHTTEST_H_


#include "global.h"
#include "params.h"

// 台形プロファイルの直進試験(床上)。
// ボタン → 数区画ぶん加速・等速・減速して止まる走行を1回、Loggerに記録 →
// ボタン → PCへ送信、を繰り返す。PC側は tools/get_log.py で受ける。
// 電源を切るまで戻らない。
void StraightTest_Run(void);

#endif
