/**
 * @file    drv_buzzer.c
 * @author  maximillian
 * @version V2.1.0
 * @date    2026-07-8
 * @brief   蜂鸣器设备驱动实现（TIM12_CH2 PWM 输出，句柄自包含）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_buzzer.h"

#include "log.h"
#include "tim.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_BUZZER_LOG_ENABLE 1

#if DRV_BUZZER_LOG_ENABLE
#define DRV_BUZZER_LOG_E(...) LOG_E("drv_buzzer", __VA_ARGS__)
#define DRV_BUZZER_LOG_W(...) LOG_W("drv_buzzer", __VA_ARGS__)
#define DRV_BUZZER_LOG_I(...) LOG_I("drv_buzzer", __VA_ARGS__)
#define DRV_BUZZER_LOG_D(...) LOG_D("drv_buzzer", __VA_ARGS__)
#else
#define DRV_BUZZER_LOG_E(...) ((void)0)
#define DRV_BUZZER_LOG_W(...) ((void)0)
#define DRV_BUZZER_LOG_I(...) ((void)0)
#define DRV_BUZZER_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 蜂鸣器 PWM 硬件（来自 CubeMX tim.c: PB15 → TIM12_CH2） */
#define BUZZER_HTIM (&htim12)
#define BUZZER_CH (TIM_CHANNEL_2)

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void drv_buzzer_init(void)
{
    HAL_TIM_PWM_Start(BUZZER_HTIM, BUZZER_CH);
    __HAL_TIM_SET_COMPARE(BUZZER_HTIM, BUZZER_CH, 0);
    s_initialized = true;

    DRV_BUZZER_LOG_I("蜂鸣器初始化完成 (TIM12_CH2 PWM 启动)");
}

void drv_buzzer_deinit(void)
{
    HAL_TIM_PWM_Stop(BUZZER_HTIM, BUZZER_CH);
    s_initialized = false;

    DRV_BUZZER_LOG_I("蜂鸣器反初始化完成");
}

void drv_buzzer_set(uint8_t duty)
{
    if (!s_initialized) {
        return;
    }

    if (duty > 100) {
        DRV_BUZZER_LOG_W("蜂鸣器占空比超限被截断: 输入=%u, 上限=100", (unsigned)duty);
        duty = 100;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(BUZZER_HTIM);
    uint32_t cmp = (uint32_t)duty * (arr + 1) / 200; /* 50% 占空对应最响 */

    __HAL_TIM_SET_COMPARE(BUZZER_HTIM, BUZZER_CH, cmp);

    DRV_BUZZER_LOG_D("蜂鸣器占空比=%u%%", (unsigned)duty);
}
