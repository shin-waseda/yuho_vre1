#ifndef INC_PARTYMODE_H_
#define INC_PARTYMODE_H_


#include "global.h"
#include "params.h"

// 宴会芸モード。ジャイロで向きを保ち、床(板など)を回されても同じ方向を向き続ける。
// 手をかざして離す → ジャイロのゼロ点を測り直す → 向きを保つ → ボタンで止める、を繰り返す。
// 位置の制御は止めて、向きだけを保つ(回す中心と機体の中心がずれていても暴れないように)。
// 電源を切るまで戻らない。
void PartyMode_Run(void);

#endif
