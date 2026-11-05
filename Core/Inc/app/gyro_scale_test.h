#ifndef INC_GYROSCALETEST_H_
#define INC_GYROSCALETEST_H_


#include "global.h"
#include "params.h"

// ジャイロの倍率の試験(床上)。区画の真ん中に、前の壁にまっすぐ向けて置く(横の壁はないほうがよい)。
// 手かざし → 止まって前の壁の FL − FR を測る → ±GYRO_SCALE_SLOPE_DEG 回って、1° あたりの FL − FR の変わり方を測る
// → 左へ GYRO_SCALE_TURNS 回(ジャイロの角度で)回って止まり、FL − FR を測る → 右へ同じだけ回って戻り、FL − FR を測る。
// 回る前と後の FL − FR の差から、本当に回った角度とジャイロの角度の違いを出し、左回り・右回りの倍率を UART に出す。
// 同じ壁を同じ場所で続けて見るので、前壁の基準の値やマウントが合っていなくても、差は測れる。
// Logger に記録(SDがあれば自動保存) → ボタン → PCへ送信、を繰り返す。電源を切るまで戻らない。
void GyroScaleTest_Run(void);

#endif
