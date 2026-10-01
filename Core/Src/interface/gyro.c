#include "main.h"
#include "interface/gyro.h"

extern SPI_HandleTypeDef hspi2;

void ICM_Init(void) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // ソフトリセット
    ICM_Write(0x11, 0x01);
    HAL_Delay(10);

    // ジャイロ+加速度 有効化
    ICM_Write(ICM_PWR_MGMT0, 0x0F);
    HAL_Delay(10);
}

void ICM_Write(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    uint8_t buf[2] = {reg & 0x7F, data};
    HAL_SPI_Transmit(&hspi2, buf, 2, 10);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

uint8_t ICM_Read(uint8_t reg) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    HAL_SPI_Transmit(&hspi2, &tx, 1, 10);
    HAL_SPI_Receive(&hspi2, &rx, 1, 10);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    return rx;
}

GyroData ICM_ReadGyro(void) {
    GyroData g;
    g.x = (int16_t)(ICM_Read(ICM_GYRO_X_H) << 8 | ICM_Read(ICM_GYRO_X_H + 1));
    g.y = (int16_t)(ICM_Read(ICM_GYRO_Y_H) << 8 | ICM_Read(ICM_GYRO_Y_H + 1));
    g.z = (int16_t)(ICM_Read(ICM_GYRO_Z_H) << 8 | ICM_Read(ICM_GYRO_Z_H + 1));
    return g;
}