/**
 * @file    drv_pwm.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-18
 * @brief   PWM 设备驱动（内置通道路由表，多通道通用）
 * @attention
 *
 * 硬件配置（路由表）内置在 drv_pwm.c 中，上层无需传参。
 * drv_pwm_init() 自动初始化全部已注册 PWM 通道。
 *
 * ## 通道分配
 * - DRV_PWM_CH_MOTOR_CHG_IN: MOTOR_POWER_CHG_IN (PB0 → TIM3_CH3, 84MHz)
 *
 * ## 用法
 * @code
 *   drv_pwm_init();
 *   drv_pwm_set_frequency(DRV_PWM_CH_MOTOR_CHG_IN, 20000);
 *   drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 500);
 * @endcode
 *
 * 占空比使用千分比 0~1000（0.1% 精度）。PWM1 模式下 duty=0 输出恒低（关），
 * duty=1000 输出恒高（开）。
 */

#ifndef __DRV_PWM_H
#define __DRV_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PWM 逻辑通道（与 drv_pwm.c 路由表严格对应）
 */
typedef enum {
    DRV_PWM_CH_MOTOR_CHG_IN = 0, /**< MOTOR_POWER_CHG_IN — 电机预充电输入 (PB0, TIM3_CH3) */
    DRV_PWM_CH_MAX,
} drv_pwm_channel_t;

typedef enum {
    DRV_PWM_OK = 0,
    DRV_PWM_ERROR_UNINITIALIZED,  /**< 通道未初始化 */
    DRV_PWM_ERROR_INVALID_PARAM,  /**< 参数越界 */
} drv_pwm_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化全部已注册 PWM 通道（启动 PWM 并置 0% 占空比） */
void drv_pwm_init(void);

/** @brief 反初始化全部 PWM 通道（停止 PWM 输出） */
void drv_pwm_deinit_all(void);

/**
 * @brief 直接给定占空比（千分比 0~1000，0.1% 精度）
 * @param ch  逻辑通道
 * @param duty_permille 0~1000（0=恒低关闭, 1000=恒高开启）
 * @return 错误码
 */
drv_pwm_error_t drv_pwm_set_duty(drv_pwm_channel_t ch, uint16_t duty_permille);

/**
 * @brief 更改 PWM 输出频率（自动重新计算 PSC/ARR 并保持已设定占空比）
 * @param ch      逻辑通道
 * @param freq_hz 目标频率 (Hz)，合法区间 [1, timer_clk_hz/2]
 * @return 错误码
 */
drv_pwm_error_t drv_pwm_set_frequency(drv_pwm_channel_t ch, uint32_t freq_hz);

/**
 * @brief 回读通道实际 PWM 频率
 * @param ch 逻辑通道
 * @return 频率 (Hz)，通道未初始化或越界返回 0
 */
uint32_t drv_pwm_get_frequency(drv_pwm_channel_t ch);

/**
 * @brief 回读通道当前占空比
 * @param ch 逻辑通道
 * @return 千分比 0~1000，通道未初始化或越界返回 0
 */
uint16_t drv_pwm_get_duty(drv_pwm_channel_t ch);

/** @brief 获取通道名称字符串 */
const char* drv_pwm_channel_name(drv_pwm_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_PWM_H */
