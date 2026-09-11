/**
 * @file    drv_led.c
 * @brief   LED 设备驱动实现 — 状态灯亮度控制 (PB6 → TIM4_CH1)
 * @attention
 *
 * 已将原 drv_pwm 的最小 PWM 控制合并进本文件：直接管理 TIM4_CH1。
 * 亮度 0~1023 线性映射到 PWM 占空比（0=灭，1023=恒亮）。
 */

#include "drv_led.h"

#include "log.h"
#include "tim.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印（Boot 精简体积） */
#define DRV_LED_LOG_ENABLE 0

#if DRV_LED_LOG_ENABLE
#define DRV_LED_LOG_E(...) LOG_E("drv_led", __VA_ARGS__)
#define DRV_LED_LOG_W(...) LOG_W("drv_led", __VA_ARGS__)
#define DRV_LED_LOG_I(...) LOG_I("drv_led", __VA_ARGS__)
#define DRV_LED_LOG_D(...) LOG_D("drv_led", __VA_ARGS__)
#else
#define DRV_LED_LOG_E(...) ((void)0)
#define DRV_LED_LOG_W(...) ((void)0)
#define DRV_LED_LOG_I(...) ((void)0)
#define DRV_LED_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 亮度分辨率上限 (srv_signal 输出 0-1023) */
#define LED_DUTY_MAX (1023U)

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void drv_led_init(void)
{
    if (!s_initialized) {
        (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
        s_initialized = true;
    }
    DRV_LED_LOG_I("LED 初始化完成 (TIM4_CH1=状态灯)");
}

void drv_led_deinit(void)
{
    if (s_initialized) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
        (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        s_initialized = false;
    }
    DRV_LED_LOG_I("LED 反初始化完成");
}

void drv_led_set_duty(drv_led_ch_t ch, uint16_t duty)
{
    if ((ch != DRV_LED_CH_STATUS) || !s_initialized) {
        return;
    }
    if (duty > LED_DUTY_MAX) {
        duty = LED_DUTY_MAX;
    }

    /* 线性映射到定时器比较值（ARR 由 CubeMX 配置） */
    const uint32_t arr = (uint32_t)htim4.Init.Period;
    const uint32_t ccr = (arr * (uint32_t)duty) / (uint32_t)LED_DUTY_MAX;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}
