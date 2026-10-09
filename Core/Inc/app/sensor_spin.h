#ifndef INC_SENSORSPIN_H_
#define INC_SENSORSPIN_H_


#include "global.h"
#include "params.h"

// 区画の真ん中でゆっくり超信地旋回しながら、壁センサーの値を記録する(PC の plant_sim の壁センサーのモデルを合わせる用)。
// 手かざし → 左に SENSOR_SPIN_ANGLE_DEG 回る → 右に同じだけ回って戻る、を1回。SD の sensor/spin_NNNN へ流す
// (SENSOR_SPIN_LOG_DECIMATION ms ごと)。終わったら次の手かざしを待つ(壁の組み合わせを変えて続けて取る)。
// 置く所は手で区画の真ん中に合わせる。どの区画で、どの壁があったか、スタートの向きは別に書き留めておく。
// 電源を切るまで戻らない。
void SensorSpin_Run(void);

#endif
