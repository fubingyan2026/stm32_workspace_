/**
 * @file    drv_led.h
 * @brief   LED PWM 驱动 — 单颗状态指示灯 (PB6 → TIM4_CH1, 经 drv_pwm 输出)
 * @attention
 *
 * TIM4 由 drv_pwm 统一管理（默认 25kHz），本驱动仅按亮度占空比映射
 * 到 TIM4_CH1。亮度 0-1023 对应 srv_signal 的输出范围。
 */

#ifndef __DRV_LED_H
#define __DRV_LED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief LED 通道 */
typedef enum {
    DRV_LED_CH_STATUS = 0, /**< 状态指示灯 — TIM4_CH1, PB6 */
    DRV_LED_CH_NUM,
} drv_led_ch_t;

/** @brief 初始化 LED PWM（经 drv_pwm 启动 TIM4_CH1 并置灭） */
void drv_led_init(void);

/** @brief 反初始化（占空比清零，不影响同组其他通道） */
void drv_led_deinit(void);

/**
 * @brief 设置 LED 亮度
 * @param ch    LED 通道 (DRV_LED_CH_STATUS)
 * @param duty  亮度 0-1023 (0=灭, 1023=最亮)
 */
void drv_led_set_duty(drv_led_ch_t ch, uint16_t duty);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LED_H */
