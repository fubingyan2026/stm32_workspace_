/**
 * @file    drv_buzzer.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   蜂鸣器设备驱动实现（TIM3_CH3 PWM 输出，句柄自包含，E1_MASTER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_buzzer.h"

#include "log.h"
#include "tim.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_BUZZER_LOG_ENABLE 0

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

/** @brief 蜂鸣器 PWM 硬件（来自 CubeMX tim.c: PB0 → TIM3_CH3） */
#define BUZZER_HTIM (&htim3)
#define BUZZER_CH (TIM_CHANNEL_3)

/** @brief 蜂鸣器默认频率（Hz），待实机确认谐振频率 */
#define BUZZER_FREQ_HZ (4000U)

/** @brief 蜂鸣器定时器输入时钟（TIM3 在 APB1，72MHz） */
#define BUZZER_TIM_CLK_HZ (72000000U)

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void drv_buzzer_init(void)
{
    /* 按默认频率重设 ARR（PSC=0），再启动 PWM 输出 */
    __HAL_TIM_SET_AUTORELOAD(BUZZER_HTIM, BUZZER_TIM_CLK_HZ / BUZZER_FREQ_HZ - 1U);
    __HAL_TIM_SET_COUNTER(BUZZER_HTIM, 0);
    BUZZER_HTIM->Instance->EGR = TIM_EGR_UG;

    HAL_TIM_PWM_Start(BUZZER_HTIM, BUZZER_CH);
    __HAL_TIM_SET_COMPARE(BUZZER_HTIM, BUZZER_CH, 0);
    s_initialized = true;

    DRV_BUZZER_LOG_I("蜂鸣器初始化完成 (TIM3_CH3 PWM 启动, %uHz)", (unsigned)BUZZER_FREQ_HZ);
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
