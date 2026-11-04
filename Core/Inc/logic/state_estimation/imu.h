#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "global.h"
#include "params.h"

// 6軸(加速度・ジャイロ)の生値を、機体の座標の物理量にする。ハードには触らない(生値を受け取るだけ)。
// 自己位置推定(姿勢・速度の推定)でもそのまま使えるよう、次の形にそろえる:
//   - 機体の座標: x = 前、y = 左、z = 上(右手系)。センサーの軸との対応は IMU_AXIS_MAP / IMU_AXIS_SIGN
//   - 加速度[m/s²]: 重力を含んだまま(引かない)。止まって水平なら約 (0, 0, +9.8)。
//     傾き(重力の向き)は姿勢の推定に使う情報なので、0 点として引かない。止まっているときの値は ImuCalib.acc_rest に残す
//   - 角速度[dps]: ゼロ点(止まっているときの平均)を引く。反時計回り(z 軸の正の向きに右ねじ)が正
// 機体の前後だけの加速度が欲しいときは acc − acc_rest(機体が傾かない平らな床の上なら、これで重力と0点のずれが消える)。

typedef struct {
    float acc[3];  // 加速度[m/s²](機体の x, y, z。重力を含む)
    float gyro[3]; // 角速度[dps](機体の x, y, z。ゼロ点を引いた後)
} ImuSample;

// 止まっているときに測った値(センサーの軸、生値[LSB]の平均)
typedef struct {
    float gyro_bias[3]; // ジャイロのゼロ点
    float acc_rest[3];  // 止まっているときの加速度(重力 + 0 点のずれ)
} ImuCalib;

// 止まっている間の平均を取るための足し込み
typedef struct {
    float sum_acc[3];
    float sum_gyro[3];
    uint32_t count;
} ImuCalAcc;

void Imu_CalReset(ImuCalAcc *acc);
void Imu_CalAdd(ImuCalAcc *acc, const int16_t acc_raw[3], const int16_t gyro_raw[3]);
// 平均を cal に入れる(1つも足していなければ cal はそのまま)
void Imu_CalFinish(const ImuCalAcc *acc, ImuCalib *cal);

// 生値(センサーの軸)を機体の座標の物理量にする
void Imu_ToBody(const int16_t acc_raw[3], const int16_t gyro_raw[3], const ImuCalib *cal, ImuSample *out);

// cal.acc_rest を機体の座標の[m/s²]にする(acc − これ で、止まっていたときからの加速度の変化)
void Imu_RestAccBody(const ImuCalib *cal, float out[3]);

#endif
