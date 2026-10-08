#ifndef INC_RUNLOG_H_
#define INC_RUNLOG_H_


#include "global.h"
#include "params.h"
#include "app/logger.h"
#include "app/log_event.h"

// 走りのログ(全部の値 + イベント)の列を Logger に登録する。探索・最短走行・宴会芸・流す試験で共通。
// 列: time_s、ControlDebug の全部(目標・速度・PWM・FF・I 項・向き・壁センサー・電圧など 27 列)、
//     イベントの 6 列(ev, ev_a〜ev_e。番号と中身の意味は app/log_event.h)。合わせて 34 列。
// 記録の間隔は decimation tick に1回。SD へ流すとき(Logger_StreamBegin)に使う想定。
void RunLog_Setup(const char *dir, const char *file, uint32_t decimation);

#endif
