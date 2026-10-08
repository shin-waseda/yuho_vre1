#include "main.h"
#include "spi.h" // hspi2 の宣言
#include "interface/gyro.h"

static GyroOffset s_offset = { 0.0f, 0.0f, 0.0f };

// 設定変更直後の出力が安定するまでの待ち[ms](データシート値は未確認、余裕を持たせた値)
#define ICM_CALIB_SETTLE_MS 100

void ICM_Init(void) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // ソフトリセット
    ICM_Write(0x11, 0x01);
    HAL_Delay(10);

    // ジャイロ+加速度 有効化
    ICM_Write(ICM_PWR_MGMT0, 0x0F);
    HAL_Delay(10);

    // ジャイロ ±2000dps / ODR 4kHz (divergence_v3と同じ設定値)
    ICM_Write(ICM_GYRO_CONFIG0, 0x04);
    HAL_Delay(1);
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

// 先頭アドレス(GYRO_DATA_X1)から6byte連続で読む(アドレス自動インクリメント)。
GyroData ICM_ReadGyro(void) {
    uint8_t tx = ICM_GYRO_X_H | 0x80;
    uint8_t rx[6] = {0};

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &tx, 1, 10);
    HAL_SPI_Receive(&hspi2, rx, 6, 10);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    GyroData g;
    g.x = (int16_t)((uint16_t)rx[0] << 8 | rx[1]);
    g.y = (int16_t)((uint16_t)rx[2] << 8 | rx[3]);
    g.z = (int16_t)((uint16_t)rx[4] << 8 | rx[5]);
    return g;
}

void ICM_CalibrateBlocking(uint16_t samples) {
    if (samples == 0) samples = 1;

    HAL_Delay(ICM_CALIB_SETTLE_MS);

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    for (uint16_t i = 0; i < samples; i++) {
        GyroData g = ICM_ReadGyro();
        sum_x += g.x;
        sum_y += g.y;
        sum_z += g.z;
        HAL_Delay(1);
    }

    s_offset.x = (float)sum_x / samples;
    s_offset.y = (float)sum_y / samples;
    s_offset.z = (float)sum_z / samples;
}

GyroOffset ICM_GetOffset(void) {
    return s_offset;
}
