#ifndef GYRO_H
#define GYRO_H

#include "global.h"
#include "params.h"


#define ICM_WHO_AM_I     0x75
#define ICM_PWR_MGMT0    0x4E
#define ICM_GYRO_CONFIG0 0x4F
#define ICM_GYRO_X_H     0x25
#define ICM_GYRO_Y_H     0x27
#define ICM_GYRO_Z_H     0x29

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} GyroData;

// X/Y/Zの生値をバースト読み出しで1回のSPI通信にまとめて取得する。
// TIM6割り込み開始後はISR(App_ControlTick)だけが呼ぶこと。
// メインループから同時に呼ぶとSPIを取り合う(値はApp_GetGyroRaw()で読む)。
GyroData ICM_ReadGyro(void);

// 静止時の生値の平均(ゼロ点オフセット)[LSB]。
typedef struct {
    float x;
    float y;
    float z;
} GyroOffset;

// 静止状態でsamples回(1ms間隔)読んで平均し、オフセットとして保持する。
// 機体を動かさないこと。TIM6割り込み開始前(ICM_Init()直後)専用。
void ICM_CalibrateBlocking(uint16_t samples);
GyroOffset ICM_GetOffset(void);

void ICM_Init(void);
void ICM_Write(uint8_t reg, uint8_t data);
uint8_t ICM_Read(uint8_t reg);

#endif
