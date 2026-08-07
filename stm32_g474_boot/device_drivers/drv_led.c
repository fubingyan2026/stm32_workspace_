/**
 * @file    drv_led.c
 * @brief   运行 LED 设备驱动实现（GPIO，PA2）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_led.h"

#include "main.h"

/* Private constants ---------------------------------------------------------*/

/** 运行 LED 引脚（CubeMX 已配置 GPIO_Output；与 main.h 中同名定义保持一致） */
#ifndef LED_Pin
#define LED_Pin GPIO_PIN_2
#endif
#ifndef LED_GPIO_Port
#define LED_GPIO_Port GPIOA
#endif

/* Exported functions --------------------------------------------------------*/

void drv_led_init(void)
{
    /* PA2 已由 CubeMX MX_GPIO_Init() 配置为推挽输出（gpio.c），无需重复初始化 */
}

void drv_led_on(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void drv_led_off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void drv_led_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
