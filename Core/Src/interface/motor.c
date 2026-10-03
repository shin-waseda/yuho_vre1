#include "main.h"
#include "tim.h" // htim2 の宣言
#include "interface/motor.h"

// PWM開始のみ。STBYはLowのまま(ドライバ無効)にしておき、
// 実際に駆動するときだけMotor_Enable()で有効化する。
void Motor_Init(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void Motor_Enable(void) {
    HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_SET);
}

void Motor_Disable(void) {
    HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);
}

static void DriveWheel(GPIO_TypeDef *cw_port, uint16_t cw_pin,
                        GPIO_TypeDef *ccw_port, uint16_t ccw_pin,
                        uint32_t channel, int16_t value) {
    uint16_t pwm;

    if (value >= 0) {
        HAL_GPIO_WritePin(cw_port, cw_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ccw_port, ccw_pin, GPIO_PIN_RESET);
        pwm = (uint16_t)value;
    } else {
        HAL_GPIO_WritePin(cw_port, cw_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ccw_port, ccw_pin, GPIO_PIN_SET);
        pwm = (uint16_t)(-value);
    }

    if (pwm > PWM_MAX) pwm = PWM_MAX;
    __HAL_TIM_SET_COMPARE(&htim2, channel, pwm);
}

void Motor_Drive(int16_t left, int16_t right) {
    DriveWheel(Motor_L_CW_GPIO_Port, Motor_L_CW_Pin, Motor_L_CCW_GPIO_Port, Motor_L_CCW_Pin, TIM_CHANNEL_1, left);
    DriveWheel(Motor_R_CW_GPIO_Port, Motor_R_CW_Pin, Motor_R_CCW_GPIO_Port, Motor_R_CCW_Pin, TIM_CHANNEL_4, right);
}

void Motor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    HAL_GPIO_WritePin(Motor_L_CW_GPIO_Port, Motor_L_CW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_L_CCW_GPIO_Port, Motor_L_CCW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_R_CW_GPIO_Port, Motor_R_CW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_R_CCW_GPIO_Port, Motor_R_CCW_Pin, GPIO_PIN_RESET);
}
