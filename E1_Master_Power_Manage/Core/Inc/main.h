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
#define VIN_ADC_Pin GPIO_PIN_2
#define VIN_ADC_GPIO_Port GPIOC
#define MOTOR_POWER_ADC_Pin GPIO_PIN_3
#define MOTOR_POWER_ADC_GPIO_Port GPIOC
#define E_STOP1_ADC1_Pin GPIO_PIN_0
#define E_STOP1_ADC1_GPIO_Port GPIOA
#define E_STOP1_ADC2_Pin GPIO_PIN_1
#define E_STOP1_ADC2_GPIO_Port GPIOA
#define E_STOP2_ADC1_Pin GPIO_PIN_2
#define E_STOP2_ADC1_GPIO_Port GPIOA
#define E_STOP2_ADC2_Pin GPIO_PIN_3
#define E_STOP2_ADC2_GPIO_Port GPIOA
#define E_STOP3_ADC1_Pin GPIO_PIN_4
#define E_STOP3_ADC1_GPIO_Port GPIOA
#define E_STOP3_ADC2_Pin GPIO_PIN_5
#define E_STOP3_ADC2_GPIO_Port GPIOA
#define E_STOP4_ADC1_Pin GPIO_PIN_6
#define E_STOP4_ADC1_GPIO_Port GPIOA
#define E_STOP4_ADC2_Pin GPIO_PIN_7
#define E_STOP4_ADC2_GPIO_Port GPIOA
#define AUX_POWER_ADC_Pin GPIO_PIN_4
#define AUX_POWER_ADC_GPIO_Port GPIOC
#define CD4051B_ADC_Pin GPIO_PIN_5
#define CD4051B_ADC_GPIO_Port GPIOC
#define MOTOR_POWER_CHG_IN_Pin GPIO_PIN_0
#define MOTOR_POWER_CHG_IN_GPIO_Port GPIOB
#define MOTOR_POWER_CHG_OCP_FLAG_Pin GPIO_PIN_1
#define MOTOR_POWER_CHG_OCP_FLAG_GPIO_Port GPIOB
#define EXT_PGOOD_12V_Pin GPIO_PIN_8
#define EXT_PGOOD_12V_GPIO_Port GPIOE
#define EXT_PGOOD_24V_Pin GPIO_PIN_9
#define EXT_PGOOD_24V_GPIO_Port GPIOE
#define COMP_PGOOD_24V_Pin GPIO_PIN_10
#define COMP_PGOOD_24V_GPIO_Port GPIOE
#define HSD1_IN_24V_Pin GPIO_PIN_11
#define HSD1_IN_24V_GPIO_Port GPIOE
#define HSD1_DIAG_EN_24V_Pin GPIO_PIN_12
#define HSD1_DIAG_EN_24V_GPIO_Port GPIOE
#define HSD2_DIAG_EN_24V_Pin GPIO_PIN_13
#define HSD2_DIAG_EN_24V_GPIO_Port GPIOE
#define AUX_POWER_PGD_Pin GPIO_PIN_14
#define AUX_POWER_PGD_GPIO_Port GPIOE
#define MOTOR_POWER_PGD_Pin GPIO_PIN_15
#define MOTOR_POWER_PGD_GPIO_Port GPIOE
#define DBR_LSD_OCP_FLAG_Pin GPIO_PIN_10
#define DBR_LSD_OCP_FLAG_GPIO_Port GPIOB
#define E_STOP_ON_Pin GPIO_PIN_11
#define E_STOP_ON_GPIO_Port GPIOB
#define MOTOR_POWER_CHG_EN_Pin GPIO_PIN_12
#define MOTOR_POWER_CHG_EN_GPIO_Port GPIOB
#define LED_B_PB13_Pin GPIO_PIN_13
#define LED_B_PB13_GPIO_Port GPIOB
#define LED_R_PB14_Pin GPIO_PIN_14
#define LED_R_PB14_GPIO_Port GPIOB
#define BUZZER_IN_Pin GPIO_PIN_15
#define BUZZER_IN_GPIO_Port GPIOB
#define DBR_LSD_EN_Pin GPIO_PIN_8
#define DBR_LSD_EN_GPIO_Port GPIOD
#define MOTOR_POWER_EN_Pin GPIO_PIN_9
#define MOTOR_POWER_EN_GPIO_Port GPIOD
#define AUX_POWER_EN_Pin GPIO_PIN_10
#define AUX_POWER_EN_GPIO_Port GPIOD
#define HSD_FAULT_Pin GPIO_PIN_11
#define HSD_FAULT_GPIO_Port GPIOD
#define HSD2_IN_24V_Pin GPIO_PIN_12
#define HSD2_IN_24V_GPIO_Port GPIOD
#define HSD1_DIAG_EN_12V_Pin GPIO_PIN_13
#define HSD1_DIAG_EN_12V_GPIO_Port GPIOD
#define HSD1_IN_12V_Pin GPIO_PIN_14
#define HSD1_IN_12V_GPIO_Port GPIOD
#define DC_DC_EN_Pin GPIO_PIN_15
#define DC_DC_EN_GPIO_Port GPIOD
#define P_CAN_STB_Pin GPIO_PIN_8
#define P_CAN_STB_GPIO_Port GPIOA
#define P_CAN_RXD_Pin GPIO_PIN_11
#define P_CAN_RXD_GPIO_Port GPIOA
#define P_CAN_TXD_Pin GPIO_PIN_12
#define P_CAN_TXD_GPIO_Port GPIOA
#define RGB1_DI_IO_Pin GPIO_PIN_12
#define RGB1_DI_IO_GPIO_Port GPIOC
#define REV_PD0_Pin GPIO_PIN_0
#define REV_PD0_GPIO_Port GPIOD
#define REV_PD1_Pin GPIO_PIN_1
#define REV_PD1_GPIO_Port GPIOD
#define REV_PD2_Pin GPIO_PIN_2
#define REV_PD2_GPIO_Port GPIOD
#define CD4051B_A_Pin GPIO_PIN_3
#define CD4051B_A_GPIO_Port GPIOD
#define CD4051B_B_Pin GPIO_PIN_4
#define CD4051B_B_GPIO_Port GPIOD
#define CD4051B_C_Pin GPIO_PIN_5
#define CD4051B_C_GPIO_Port GPIOD
#define RGB2_DI_IO_Pin GPIO_PIN_5
#define RGB2_DI_IO_GPIO_Port GPIOB
#define FAN0_PWM_IO_Pin GPIO_PIN_8
#define FAN0_PWM_IO_GPIO_Port GPIOB
#define FAN1_PWM_IO_Pin GPIO_PIN_9
#define FAN1_PWM_IO_GPIO_Port GPIOB
#define FAN0_FG_IO_Pin GPIO_PIN_0
#define FAN0_FG_IO_GPIO_Port GPIOE
#define FAN0_FG_IO_EXTI_IRQn EXTI0_IRQn
#define FAN1_FG_IO_Pin GPIO_PIN_1
#define FAN1_FG_IO_GPIO_Port GPIOE
#define FAN1_FG_IO_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
