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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NTC1_ADC_Pin GPIO_PIN_0
#define NTC1_ADC_GPIO_Port GPIOC
#define NTC2_ADC_Pin GPIO_PIN_1
#define NTC2_ADC_GPIO_Port GPIOC
#define VIN_DC_DC_ADC_Pin GPIO_PIN_2
#define VIN_DC_DC_ADC_GPIO_Port GPIOC
#define VIN_ADC_Pin GPIO_PIN_3
#define VIN_ADC_GPIO_Port GPIOC
#define FAN0_FG_IO_Pin GPIO_PIN_0
#define FAN0_FG_IO_GPIO_Port GPIOA
#define FAN1_FG_IO_Pin GPIO_PIN_1
#define FAN1_FG_IO_GPIO_Port GPIOA
#define CD4051B_A_Pin GPIO_PIN_4
#define CD4051B_A_GPIO_Port GPIOA
#define CD4051B_B_Pin GPIO_PIN_5
#define CD4051B_B_GPIO_Port GPIOA
#define CD4051B_C_Pin GPIO_PIN_6
#define CD4051B_C_GPIO_Port GPIOA
#define CD4051B_ADC_Pin GPIO_PIN_4
#define CD4051B_ADC_GPIO_Port GPIOC
#define RS485_EN_Pin GPIO_PIN_5
#define RS485_EN_GPIO_Port GPIOC
#define BUZZ_PWM_Pin GPIO_PIN_0
#define BUZZ_PWM_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define E_STOP_ON_Pin GPIO_PIN_9
#define E_STOP_ON_GPIO_Port GPIOC
#define LM5060_PGOOD_Pin GPIO_PIN_8
#define LM5060_PGOOD_GPIO_Port GPIOA
#define LOG_UART_TX_Pin GPIO_PIN_9
#define LOG_UART_TX_GPIO_Port GPIOA
#define LOG_UART_RX_Pin GPIO_PIN_10
#define LOG_UART_RX_GPIO_Port GPIOA
#define VIN_DC_DC_EN_Pin GPIO_PIN_11
#define VIN_DC_DC_EN_GPIO_Port GPIOA
#define PGOOD_12V_Pin GPIO_PIN_12
#define PGOOD_12V_GPIO_Port GPIOA
#define MOTOR_POWER_PGD_Pin GPIO_PIN_10
#define MOTOR_POWER_PGD_GPIO_Port GPIOC
#define MOTOR_POWER_EN_Pin GPIO_PIN_11
#define MOTOR_POWER_EN_GPIO_Port GPIOC
#define AUX_PWER_PGD_Pin GPIO_PIN_12
#define AUX_PWER_PGD_GPIO_Port GPIOC
#define AUX_POWER_EN_Pin GPIO_PIN_2
#define AUX_POWER_EN_GPIO_Port GPIOD
#define DC_DC_24V_PGOOD_Pin GPIO_PIN_5
#define DC_DC_24V_PGOOD_GPIO_Port GPIOB
#define LED_PWM_Pin GPIO_PIN_6
#define LED_PWM_GPIO_Port GPIOB
#define DC_DC_24V_EN_Pin GPIO_PIN_7
#define DC_DC_24V_EN_GPIO_Port GPIOB
#define FAN1_PWM_IO_Pin GPIO_PIN_8
#define FAN1_PWM_IO_GPIO_Port GPIOB
#define FAN0_PWM_IO_Pin GPIO_PIN_9
#define FAN0_PWM_IO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
