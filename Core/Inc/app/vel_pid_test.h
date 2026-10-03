#ifndef INC_VELPIDTEST_H_
#define INC_VELPIDTEST_H_


#include "global.h"
#include "params.h"

// 速度PID調整用のブロッキングテストループ。
// ボタン → 0→HIGH→0 のステップを1回走らせてLoggerに記録 → ボタン → PCへ送信、を繰り返す。
// PC側は tools/get_log.py で受ける。電源を切るまで戻らない。
void VelPIDTest_Run(void);

#endif
