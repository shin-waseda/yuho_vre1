#include "logic/state_estimation/imu.h"

static const uint8_t kAxisMap[3] = IMU_AXIS_MAP;
static const float kAxisSign[3] = IMU_AXIS_SIGN;

#define ACC_M_S2_PER_LSB (GRAVITY_M_S2 / ACCEL_SENSITIVITY_LSB_PER_G)
#define GYRO_DPS_PER_LSB (1.0f / GYRO_SENSITIVITY_LSB_PER_DPS)

void Imu_CalReset(ImuCalAcc *acc) {
    for (int i = 0; i < 3; i++) {
        acc->sum_acc[i] = 0.0f;
        acc->sum_gyro[i] = 0.0f;
    }
    acc->count = 0u;
}

void Imu_CalAdd(ImuCalAcc *acc, const int16_t acc_raw[3], const int16_t gyro_raw[3]) {
    for (int i = 0; i < 3; i++) {
        acc->sum_acc[i] += (float)acc_raw[i];
        acc->sum_gyro[i] += (float)gyro_raw[i];
    }
    acc->count++;
}

void Imu_CalFinish(const ImuCalAcc *acc, ImuCalib *cal) {
    if (acc->count == 0u) return;
    for (int i = 0; i < 3; i++) {
        cal->acc_rest[i] = acc->sum_acc[i] / (float)acc->count;
        cal->gyro_bias[i] = acc->sum_gyro[i] / (float)acc->count;
    }
}

void Imu_ToBody(const int16_t acc_raw[3], const int16_t gyro_raw[3], const ImuCalib *cal, ImuSample *out) {
    for (int i = 0; i < 3; i++) {
        uint8_t s = kAxisMap[i]; // 機体の i 軸 = センサーの s 軸 × 符号
        out->acc[i] = kAxisSign[i] * (float)acc_raw[s] * ACC_M_S2_PER_LSB;
        out->gyro[i] = kAxisSign[i] * ((float)gyro_raw[s] - cal->gyro_bias[s]) * GYRO_DPS_PER_LSB;
    }
}

void Imu_RestAccBody(const ImuCalib *cal, float out[3]) {
    for (int i = 0; i < 3; i++) {
        uint8_t s = kAxisMap[i];
        out[i] = kAxisSign[i] * cal->acc_rest[s] * ACC_M_S2_PER_LSB;
    }
}
