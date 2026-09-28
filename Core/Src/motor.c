#include "interface/motor.h"

extern TIM_HandleTypeDef htim2;

void Motor_Init(void) {
    HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void Motor_Forward(uint16_t left, uint16_t right) {
    HAL_GPIO_WritePin(Motor_L_CW_GPIO_Port, Motor_L_CW_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_L_CCW_GPIO_Port, Motor_L_CCW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_R_CW_GPIO_Port, Motor_R_CW_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Motor_R_CCW_GPIO_Port, Motor_R_CCW_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right);
}

void Motor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    HAL_GPIO_WritePin(Motor_L_CW_GPIO_Port, Motor_L_CW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_L_CCW_GPIO_Port, Motor_L_CCW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_R_CW_GPIO_Port, Motor_R_CW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor_R_CCW_GPIO_Port, Motor_R_CCW_Pin, GPIO_PIN_RESET);
}
