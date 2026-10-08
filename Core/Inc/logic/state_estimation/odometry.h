#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_


#include "global.h"
#include "params.h"
#include "logic/state_estimation/kinematics.h"

typedef struct {
    float x_mm;
    float y_mm;
    float theta_rad; // 正規化せず積分し続ける(unbounded)
} Pose;

// 呼び出し側(app)が所有・保持する自己位置の累積状態。
typedef struct {
    Pose pose;
} Odometry_t;

void Odometry_Reset(Odometry_t *odo);

// エンコーダ差分パルス数(Encoder_GetDeltaL/R)と経過時間から、
// 瞬時の左右輪速度[mm/s]を返す。あわせて内部でkinematicsを使い、
// odo->pose(自己位置推定)をその場で積分更新する。
WheelVelocity Odometry_Update(Odometry_t *odo, int16_t delta_l, int16_t delta_r, float dt);

#endif