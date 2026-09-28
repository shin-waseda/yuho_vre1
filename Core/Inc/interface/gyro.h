#ifndef GYRO_H
#define GYRO_H

#include "global.h"
#include "params.h"


#define ICM_WHO_AM_I     0x75
#define ICM_PWR_MGMT0    0x4E
#define ICM_GYRO_X_H     0x25
#define ICM_GYRO_Y_H     0x27
#define ICM_GYRO_Z_H     0x29

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} GyroData;

GyroData ICM_ReadGyro(void);

void ICM_Init(void);
void ICM_Write(uint8_t reg, uint8_t data);
uint8_t ICM_Read(uint8_t reg);

#endif
