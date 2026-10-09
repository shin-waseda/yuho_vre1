#ifndef INC_VELOCITYPROFILE_H_
#define INC_VELOCITYPROFILE_H_


#include "global.h"

// 距離指定の台形速度プロファイル(加速 → 等速 → 減速)。
// 毎tick VelocityProfile_Step() で目標の速度・加速度・位置を1ステップ進める。
// 実測は使わない(目標軌道だけを作る)ので、FFにそのまま使える。
// 距離が短く最高速度に届かない場合は、自動的に三角形のプロファイルになる。
typedef struct {
    // 指令
    float distance_mm; // 進む距離(>0)
    float v_max;       // 最高速度[mm/s]
    float v_end;       // 終点の速度[mm/s](0なら止まる)
    float accel;       // 加速度の大きさ[mm/s^2]
    float decel;       // 減速度の大きさ[mm/s^2](VelocityProfile_Start なら accel と同じ)

    // 現在の目標(Stepで更新)
    float pos_mm;  // 開始からの目標位置
    float v;       // 目標速度[mm/s]
    float a;       // 目標加速度[mm/s^2](このtickで実際にかけた値)
    bool decelerating; // 減速に入った(以後は加速に戻らない)
    bool done;     // 距離を進み終えた(v_end=0なら止まった)
} VelocityProfile;

// v_start: 開始時の速度(停止からなら0)。
void VelocityProfile_Start(VelocityProfile *p, float distance_mm, float v_start,
                           float v_max, float v_end, float accel);
// 加速度と減速度を別にする(減速でタイヤが滑りやすいので、減速だけ小さくする。最短走行の直進)。
void VelocityProfile_StartAD(VelocityProfile *p, float distance_mm, float v_start,
                             float v_max, float v_end, float accel, float decel);

// dt秒進める。done後は v=v_end, a=0 を保つ。
void VelocityProfile_Step(VelocityProfile *p, float dt);

#endif
