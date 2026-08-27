/**
 * @file    drv_pwm.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   PWM 设备驱动（TIM1/TIM4 输出比较通道）
 * @attention
 *
 * G0 手套 CubeMX 配置：
 *   TIM1_CH1 (PA8)  周期 1024
 *   TIM4_CH1 (PB6)  周期 1024
 *   TIM4_CH2 (PB7)  周期 1024
 * 占空比接口统一 0~1023（与 srv_signal 的 uint16_t 接口对齐）。
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
 * @brief PWM 通道枚举
 */
typedef enum {
    DRV_PWM_TIM1_CH1 = 0, /**< TIM1_CH1 — PA8 */
    DRV_PWM_TIM4_CH1,     /**< TIM4_CH1 — PB6 */
    DRV_PWM_TIM4_CH2,     /**< TIM4_CH2 — PB7 */
    DRV_PWM_CH_NUM,       /**< 通道总数 */
} drv_pwm_channel_t;

/**
 * @brief PWM 驱动错误码
 */
typedef enum {
    DRV_PWM_OK = 0,
    DRV_PWM_ERROR_UNINITIALIZED,
    DRV_PWM_ERROR_INVALID_PARAM,
} drv_pwm_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化全部 PWM 通道（启动定时器与输出） */
drv_pwm_error_t drv_pwm_init(void);

/** @brief 停止全部 PWM 通道 */
void drv_pwm_deinit_all(void);

bool drv_pwm_is_initialized(drv_pwm_channel_t ch);

/* --- 占空比 --- */

/**
 * @brief 设置 PWM 占空比
 * @param ch    通道号
 * @param duty  占空比 0~1023（0=全低, 1023=全高）
 * @return DRV_PWM_OK 成功
 */
drv_pwm_error_t drv_pwm_set_duty(drv_pwm_channel_t ch, uint16_t duty);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_PWM_H */
