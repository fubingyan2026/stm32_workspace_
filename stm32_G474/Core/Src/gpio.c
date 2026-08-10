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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VBAT2_EN_Pin|VBAT1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|RELAY1_IN_Pin|RELAY2_IN_Pin|PRECHARGE_EN_Pin
                          |POWER_CAN_STB_Pin|V_CHARGE_VBAT1_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY3_IN_GPIO_Port, RELAY3_IN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BAT1_CAN_STB_GPIO_Port, BAT1_CAN_STB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BAT2_CAN_STB_Pin|V_CHARGE_VBAT2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VBAT2_EN_Pin VBAT1_EN_Pin */
  GPIO_InitStruct.Pin = VBAT2_EN_Pin|VBAT1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin RELAY1_IN_Pin RELAY3_IN_Pin RELAY2_IN_Pin
                           PRECHARGE_EN_Pin V_CHARGE_VBAT1_EN_Pin */
  GPIO_InitStruct.Pin = LED_Pin|RELAY1_IN_Pin|RELAY3_IN_Pin|RELAY2_IN_Pin
                          |PRECHARGE_EN_Pin|V_CHARGE_VBAT1_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_IN_IO_Pin D_IN2_IO_Pin KEY_LED_IN_Pin */
  GPIO_InitStruct.Pin = KEY1_IN_IO_Pin|D_IN2_IO_Pin|KEY_LED_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY2_IN_IO_Pin D_IN1_IO_Pin VBAT1_PGD_Pin VBAT2_PGD_Pin */
  GPIO_InitStruct.Pin = KEY2_IN_IO_Pin|D_IN1_IO_Pin|VBAT1_PGD_Pin|VBAT2_PGD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_CAN_STB_Pin */
  GPIO_InitStruct.Pin = POWER_CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_CAN_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT1_CAN_STB_Pin */
  GPIO_InitStruct.Pin = BAT1_CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT1_CAN_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT2_CAN_STB_Pin */
  GPIO_InitStruct.Pin = BAT2_CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT2_CAN_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : V_CHARGE_VBAT2_EN_Pin */
  GPIO_InitStruct.Pin = V_CHARGE_VBAT2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(V_CHARGE_VBAT2_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
