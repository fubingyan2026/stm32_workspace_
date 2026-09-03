/**
 * @file    drv_led.c
 * @brief   LED PWM 驱动实现 — 单颗状态指示灯 (经 drv_pwm → TIM4_CH1)
 */

#include "drv_led.h"

#include "log.h"
#include "drv_pwm.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_LED_LOG_ENABLE 1

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

/**
 * @brief 亮度分辨率上限 (srv_signal 输出 0-1023)
 * @note  经 drv_pwm 千分比通道输出：1023 → 1000‰。
 */
#define LED_DUTY_MAX (1023U)

void drv_led_init(void)
{
    drv_pwm_init(); /* 幂等：确保 TIM4 组已启动 */
    drv_pwm_set_duty(DRV_PWM_CH_LED, 0);

    DRV_LED_LOG_I("LED PWM 初始化完成 (TIM4_CH1=状态灯)");
}

void drv_led_deinit(void)
{
    drv_pwm_set_duty(DRV_PWM_CH_LED, 0);

    DRV_LED_LOG_I("LED PWM 反初始化完成");
}

void drv_led_set_duty(drv_led_ch_t ch, uint16_t duty)
{
    if (ch != DRV_LED_CH_STATUS) {
        return;
    }

    if (duty > LED_DUTY_MAX) {
        duty = LED_DUTY_MAX;
    }

    uint16_t permille = (uint16_t)((uint32_t)duty * 1000U / (uint32_t)LED_DUTY_MAX);
    drv_pwm_set_duty(DRV_PWM_CH_LED, permille);
}
