/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_POWER_CHG_IN_Pin|MOTOR_POWER_CHG_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, HSD1_IN_24V_Pin|HSD1_DIAG_EN_24V_Pin|HSD2_DIAG_EN_24V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DBR_LSD_EN_Pin|MOTOR_POWER_EN_Pin|AUX_POWER_EN_Pin|HSD2_IN_24V_Pin
                          |HSD1_DIAG_EN_12V_Pin|HSD1_IN_12V_Pin|DC_DC_EN_Pin|CD4051B_A_Pin
                          |CD4051B_B_Pin|CD4051B_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(P_CAN_STB_GPIO_Port, P_CAN_STB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOTOR_POWER_CHG_IN_Pin MOTOR_POWER_CHG_EN_Pin */
  GPIO_InitStruct.Pin = MOTOR_POWER_CHG_IN_Pin|MOTOR_POWER_CHG_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_POWER_CHG_OCP_FLAG_Pin DBR_LSD_OCP_FLAG_Pin E_STOP_ON_Pin */
  GPIO_InitStruct.Pin = MOTOR_POWER_CHG_OCP_FLAG_Pin|DBR_LSD_OCP_FLAG_Pin|E_STOP_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_PGOOD_12V_Pin EXT_PGOOD_24V_Pin COMP_PGOOD_24V_Pin AUX_POWER_PGD_Pin
                           MOTOR_POWER_PGD_Pin */
  GPIO_InitStruct.Pin = EXT_PGOOD_12V_Pin|EXT_PGOOD_24V_Pin|COMP_PGOOD_24V_Pin|AUX_POWER_PGD_Pin
                          |MOTOR_POWER_PGD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : HSD1_IN_24V_Pin HSD1_DIAG_EN_24V_Pin HSD2_DIAG_EN_24V_Pin */
  GPIO_InitStruct.Pin = HSD1_IN_24V_Pin|HSD1_DIAG_EN_24V_Pin|HSD2_DIAG_EN_24V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DBR_LSD_EN_Pin MOTOR_POWER_EN_Pin AUX_POWER_EN_Pin HSD2_IN_24V_Pin
                           HSD1_DIAG_EN_12V_Pin HSD1_IN_12V_Pin DC_DC_EN_Pin */
  GPIO_InitStruct.Pin = DBR_LSD_EN_Pin|MOTOR_POWER_EN_Pin|AUX_POWER_EN_Pin|HSD2_IN_24V_Pin
                          |HSD1_DIAG_EN_12V_Pin|HSD1_IN_12V_Pin|DC_DC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : HSD_FAULT_Pin REV_PD0_Pin REV_PD1_Pin REV_PD2_Pin */
  GPIO_InitStruct.Pin = HSD_FAULT_Pin|REV_PD0_Pin|REV_PD1_Pin|REV_PD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : P_CAN_STB_Pin */
  GPIO_InitStruct.Pin = P_CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(P_CAN_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CD4051B_A_Pin CD4051B_B_Pin CD4051B_C_Pin */
  GPIO_InitStruct.Pin = CD4051B_A_Pin|CD4051B_B_Pin|CD4051B_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN0_FG_IO_Pin FAN1_FG_IO_Pin */
  GPIO_InitStruct.Pin = FAN0_FG_IO_Pin|FAN1_FG_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
