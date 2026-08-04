/**
 * @file    drv_led.c
 * @brief   LED PWM 驱动实现 — TIM1 互补输出
 */

#include "drv_led.h"

#include "log.h"
#include "tim.h"

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
 * @brief 亮度分辨率上限 (srv_led 输出 0-1023)
 * @note  TIM1 ARR = Period(1023)-1 = 1022；PWM2 互补输出下 CCR=duty 直接映射:
 *        CCR=0 → N 通道 0% → 灭；CCR≥1023 → N 通道 100% → 最亮。
 *        CCR 允许略大于 ARR，等效 100% 占空，无副作用。
 */
#define LED_DUTY_MAX (1023U)

void drv_led_init(void)
{
    /* 启动 TIM1 CH1N (PB13, 蓝色) 互补 PWM */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

    /* 启动 TIM1 CH2N (PB14, 红色) 互补 PWM */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    DRV_LED_LOG_I("LED PWM 初始化完成 (TIM1 CH1N=蓝, CH2N=红)");
}

void drv_led_deinit(void)
{
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    DRV_LED_LOG_I("LED PWM 反初始化完成");
}

void drv_led_set_duty(drv_led_ch_t ch, uint16_t duty)
{
    uint32_t tim_ch = (ch == DRV_LED_CH_BLUE) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;

    if (duty > LED_DUTY_MAX) {
        duty = LED_DUTY_MAX;
    }

    __HAL_TIM_SET_COMPARE(&htim1, tim_ch, duty);
}
