#ifndef INC_PIVOTTEST_H_
#define INC_PIVOTTEST_H_


#include "global.h"
#include "params.h"

// 超信地旋回の試験(床上)。
// ボタン → +90°(左)回って止まる → 少し待つ → −90°(右)回って元の向きへ戻る、を1回
// Loggerに記録(SDがあれば自動保存) → ボタン → PCへ送信、を繰り返す。
// 電源を切るまで戻らない。
void PivotTest_Run(void);

#endif
