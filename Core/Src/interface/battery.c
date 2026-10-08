#include "interface/battery.h"
#include "interface/sensor.h"

static float RawToVoltage(uint16_t raw) {
    return (float)raw * ADC_VREF_V / ADC_FULL_SCALE * BATTERY_DIVIDER_RATIO;
}

float Battery_GetVoltage(void) {
    return RawToVoltage(vabt);
}

float Battery_MeasureVoltageBlocking(uint16_t samples) {
    if (samples == 0) samples = 1;

    uint32_t sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        sum += Sensor_ReadBatteryRawBlocking();
    }
    return RawToVoltage((uint16_t)(sum / samples));
}
