/**
 * @file    drv_led.h
 * @brief   LED 设备驱动 — 单颗状态指示灯 (PB6 → TIM4_CH1)
 * @attention
 *
 * 本驱动直接管理 TIM4_CH1（原 drv_pwm 最小实现已合并入 drv_led.c）。
 * 亮度 0-1023 线性映射到 PWM 占空比（0=灭，1023=恒亮）。
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

/** @brief 初始化 LED（启动 TIM4_CH1 并置灭；幂等） */
void drv_led_init(void);

/** @brief 反初始化（占空比清零并停止输出） */
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
