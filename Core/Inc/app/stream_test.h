#ifndef INC_STREAMTEST_H_
#define INC_STREAMTEST_H_


#include "global.h"
#include "params.h"

// 走りながら SD へ流すログ(Logger_Stream*)の試験。モーターは動かさない(机の上でよい)。
// 記録の間隔(1ms / 5ms)をボタンのクリックで切り替えて選び、手かざしで始める。
// 制御の値・センサーの値を、ボタンを押すまで(最大 STREAM_TEST_MAX_MS)SD へ流し続け、
// 書いたブロックの数・捨てた行の数・1ブロックを書くのにかかった時間(平均・最大)を UART に出す。
// ファイルは stream/test_NNNN.bin(SD_DUMP で受け取ると、get_log.py が .csv にする)。
// 電源を切るまで戻らない。
void StreamTest_Run(void);

#endif
