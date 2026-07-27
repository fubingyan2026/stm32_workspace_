/**
 * @file    drv_pwm.h
 * @brief   TIM PWM 输出驱动 — 6 通道，硬件输出，无需中断
 */

#ifndef __DRV_PWM_H
#define __DRV_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ====== 错误码 =============================================================*/

typedef enum {
    DRV_PWM_OK = 0,
    DRV_PWM_ERR_PARAM = -1,
    DRV_PWM_ERR_INIT  = -2,
} drv_pwm_error_t;

/* ====== 通道枚举 ===========================================================*/

typedef enum {
    DRV_PWM_CH_TIM1_CH1  = 0, /**< PE9  TIM1_CH1,  ~1kHz, 50% */
    DRV_PWM_CH_TIM1_CH3,      /**< PE13 TIM1_CH3,  ~1kHz, 50% */
    DRV_PWM_CH_TIM2_CH1,      /**< PA0  TIM2_CH1,  ~1kHz, 50% */
    DRV_PWM_CH_TIM2_CH3,      /**< PA2  TIM2_CH3,  ~1kHz, 50% */
    DRV_PWM_CH_TIM3_CH4,      /**< PB1  TIM3_CH4,  ~1kHz,  0% */
    DRV_PWM_CH_TIM12_CH2,     /**< PB15 TIM12_CH2, ~500Hz, 0% */
    DRV_PWM_CH_NUM,
} drv_pwm_channel_t;

/* ====== API ================================================================*/

drv_pwm_error_t drv_pwm_init(void);
drv_pwm_error_t drv_pwm_set_duty(drv_pwm_channel_t ch, uint32_t pulse);
drv_pwm_error_t drv_pwm_set_duty_percent(drv_pwm_channel_t ch, uint32_t percent);
uint32_t        drv_pwm_get_period(drv_pwm_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_PWM_H */
