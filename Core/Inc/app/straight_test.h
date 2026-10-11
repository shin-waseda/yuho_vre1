#ifndef INC_STRAIGHTTEST_H_
#define INC_STRAIGHTTEST_H_


#include "global.h"
#include "params.h"

// 台形プロファイルの直進試験(床上)。
// ボタン → 数区画ぶん加速・等速・減速して止まる走行を1回、Loggerに記録 →
// ボタン → PCへ送信、を繰り返す。PC側は tools/get_log.py で受ける。
// 電源を切るまで戻らない。
void StraightTest_Run(void);

// 直進の連続の試験。速さ(SPEED_SELECT_STRAIGHT_V_MM_S)と加速度(SPEED_SELECT_ACCEL_MM_S2)の範囲を選び、加速度ごとに
// 速さを順に上げながら、各組み合わせで STRAIGHT_SWEEP_REPEAT 回走る。両側と両端に壁のある通路
// (STRAIGHT_TEST_SECTIONS + 1 区画)の端の区画に通路の向きに置いて手かざしで始めると、尻当てでそろえて走り、
// 反対の端で 180° 回って尻当てして戻る、をくり返す(置き直さない)。1回ごとに SD に sweep_NNNN で保存する。
// 打ち切り・電池の低下・全部終わったら、何回目かを LED の棒グラフで点滅させて止まる。電源を切るまで戻らない。
void StraightSweep_Run(void);

#endif
