/**
 * @file    drv_buzzer.c
 * @author  maximillian
 * @version V2.1.0
 * @date    2026-07-8
 * @brief   蜂鸣器设备驱动实现（TIM12_CH2 PWM 输出，句柄自包含）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_buzzer.h"

#include "tim.h"

/* Private constants ---------------------------------------------------------*/

/** @brief 蜂鸣器 PWM 硬件（来自 CubeMX tim.c: PB15 → TIM12_CH2） */
#define BUZZER_HTIM (&htim12)
#define BUZZER_CH   (TIM_CHANNEL_2)

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void drv_buzzer_init(void)
{
    HAL_TIM_PWM_Start(BUZZER_HTIM, BUZZER_CH);
    __HAL_TIM_SET_COMPARE(BUZZER_HTIM, BUZZER_CH, 0);
    s_initialized = true;
}

void drv_buzzer_deinit(void)
{
    HAL_TIM_PWM_Stop(BUZZER_HTIM, BUZZER_CH);
    s_initialized = false;
}

void drv_buzzer_set(uint8_t duty)
{
    if (!s_initialized) {
        return;
    }

    if (duty > 100) {
        duty = 100;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(BUZZER_HTIM);
    uint32_t cmp = (uint32_t)duty * (arr + 1) / 200; /* 50% 占空对应最响 */

    __HAL_TIM_SET_COMPARE(BUZZER_HTIM, BUZZER_CH, cmp);
}
