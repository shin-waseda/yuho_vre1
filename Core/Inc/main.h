/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "global.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// ad_r/ad_fr/ad_fl/ad_l/vabt は global.h (via このファイルの Includes) で宣言済み

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_4_Pin GPIO_PIN_13
#define LED_4_GPIO_Port GPIOC
#define IR_FL_Pin GPIO_PIN_14
#define IR_FL_GPIO_Port GPIOC
#define IR_R_Pin GPIO_PIN_15
#define IR_R_GPIO_Port GPIOC
#define IR_FR_Pin GPIO_PIN_0
#define IR_FR_GPIO_Port GPIOH
#define IR_L_Pin GPIO_PIN_1
#define IR_L_GPIO_Port GPIOH
#define Vol_Check_Pin GPIO_PIN_0
#define Vol_Check_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOC
#define Sensor_L_Pin GPIO_PIN_0
#define Sensor_L_GPIO_Port GPIOA
#define Sensor_FR_Pin GPIO_PIN_1
#define Sensor_FR_GPIO_Port GPIOA
#define Sensor_R_Pin GPIO_PIN_2
#define Sensor_R_GPIO_Port GPIOA
#define Sensor_FL_Pin GPIO_PIN_3
#define Sensor_FL_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define Motor_L_PWM_Pin GPIO_PIN_5
#define Motor_L_PWM_GPIO_Port GPIOA
#define Push_IN_1_Pin GPIO_PIN_6
#define Push_IN_1_GPIO_Port GPIOA
#define Latch_595_Pin GPIO_PIN_7
#define Latch_595_GPIO_Port GPIOA
#define Motor_L_CCW_Pin GPIO_PIN_4
#define Motor_L_CCW_GPIO_Port GPIOC
#define Motor_L_CW_Pin GPIO_PIN_5
#define Motor_L_CW_GPIO_Port GPIOC
#define Motor_STBY_Pin GPIO_PIN_0
#define Motor_STBY_GPIO_Port GPIOB
#define Motor_R_CW_Pin GPIO_PIN_1
#define Motor_R_CW_GPIO_Port GPIOB
#define Motor_R_CCW_Pin GPIO_PIN_10
#define Motor_R_CCW_GPIO_Port GPIOB
#define Motor_R_PWM_Pin GPIO_PIN_11
#define Motor_R_PWM_GPIO_Port GPIOB
#define LED_6_Pin GPIO_PIN_12
#define LED_6_GPIO_Port GPIOB
#define SCLK_Pin GPIO_PIN_13
#define SCLK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define ENC_R_A_Pin GPIO_PIN_6
#define ENC_R_A_GPIO_Port GPIOC
#define ENC_R_B_Pin GPIO_PIN_7
#define ENC_R_B_GPIO_Port GPIOC
#define SD_DAT0_Pin GPIO_PIN_8
#define SD_DAT0_GPIO_Port GPIOC
#define SD_DAT1_Pin GPIO_PIN_9
#define SD_DAT1_GPIO_Port GPIOC
#define SCLK_595_Pin GPIO_PIN_8
#define SCLK_595_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_11
#define LED_5_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SER_595_Pin GPIO_PIN_15
#define SER_595_GPIO_Port GPIOA
#define SD_DAT2_Pin GPIO_PIN_10
#define SD_DAT2_GPIO_Port GPIOC
#define SD_DAT3_Pin GPIO_PIN_11
#define SD_DAT3_GPIO_Port GPIOC
#define SD_CLK_Pin GPIO_PIN_12
#define SD_CLK_GPIO_Port GPIOC
#define SD_CMD_Pin GPIO_PIN_2
#define SD_CMD_GPIO_Port GPIOD
#define FAN_Pin GPIO_PIN_4
#define FAN_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_5
#define Buzzer_GPIO_Port GPIOB
#define ENC_L_A_Pin GPIO_PIN_6
#define ENC_L_A_GPIO_Port GPIOB
#define ENC_L_B_Pin GPIO_PIN_7
#define ENC_L_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
