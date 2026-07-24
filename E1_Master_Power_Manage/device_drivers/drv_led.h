/**
 * @file    drv_led.h
 * @brief   LED PWM 驱动 — TIM1 CH1N(PB13 蓝) / CH2N(PB14 红)
 * @attention
 *
 * CubeMX tim.c 已将 PB13/PB14 配置为 TIM1 互补 PWM 输出:
 * - TIM1 时钟 84MHz, Prescaler=84→1MHz, Period=1024→0-1023 分辨率
 * - PWM2 模式，互补 N 通道输出
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
    DRV_LED_CH_BLUE = 0, /**< 蓝色 LED — TIM1_CH1N, PB13 */
    DRV_LED_CH_RED,      /**< 红色 LED — TIM1_CH2N, PB14 */
    DRV_LED_CH_NUM,
} drv_led_ch_t;

/** @brief 初始化 LED PWM（启动 TIM1 CH1N + CH2N 互补输出） */
void drv_led_init(void);

/** @brief 反初始化（停止 PWM） */
void drv_led_deinit(void);

/**
 * @brief 设置 LED 亮度
 * @param ch    LED 通道 (DRV_LED_CH_BLUE / DRV_LED_CH_RED)
 * @param duty  亮度 0-1023 (0=灭, 1023=最亮)
 */
void drv_led_set_duty(drv_led_ch_t ch, uint16_t duty);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LED_H */
