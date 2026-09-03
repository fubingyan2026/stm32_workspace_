/**
 * @file    drv_pwm.h
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-09-03
 * @brief   PWM 设备驱动（内置通道路由表，多通道通用，E1_MASTER_POWER_CTU）
 * @attention
 *
 * 硬件配置（路由表）内置在 drv_pwm.c 中，上层无需传参。
 * drv_pwm_init() 自动初始化全部已注册 PWM 通道（幂等，可被多个驱动重复调用）。
 *
 * ## 通道分配（共用 TIM4，定时器时钟 72MHz，默认 25kHz）
 * - DRV_PWM_CH_LED : 状态指示灯 (PB6 → TIM4_CH1)
 * - DRV_PWM_CH_FAN1: 外部风扇 1 (PB8 → TIM4_CH3)
 * - DRV_PWM_CH_FAN0: 内部风扇 0 (PB9 → TIM4_CH4)
 *
 * ## 用法
 * @code
 *   drv_pwm_init();
 *   drv_pwm_set_duty(DRV_PWM_CH_FAN0, 500);
 * @endcode
 *
 * 占空比使用千分比 0~1000（0.1% 精度）。PWM1 模式下 duty=0 输出恒低（关），
 * duty=1000 输出恒高（开）。
 *
 * @note 三个逻辑通道共享同一 TIM4，修改频率按「组」生效（所有通道同时变）。
 *       LED 与风扇因此固定工作在同一频率（默认 25kHz），亮度/转速仅靠占空比调节。
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
    DRV_PWM_CH_LED = 0,      /**< LED_PWM — 状态指示灯 (PB6, TIM4_CH1) */
    DRV_PWM_CH_FAN1,         /**< FAN1_PWM_IO — 外部风扇 1 (PB8, TIM4_CH3) */
    DRV_PWM_CH_FAN0,         /**< FAN0_PWM_IO — 内部风扇 0 (PB9, TIM4_CH4) */
    DRV_PWM_CH_MAX,
} drv_pwm_channel_t;

typedef enum {
    DRV_PWM_OK = 0,
    DRV_PWM_ERROR_UNINITIALIZED,  /**< 通道未初始化 */
    DRV_PWM_ERROR_INVALID_PARAM,  /**< 参数越界 */
} drv_pwm_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化全部已注册 PWM 通道（启动 PWM 并置 0% 占空比；幂等） */
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
 * @brief 更改 PWM 输出频率（自动重新计算 PSC/ARR 并保持各通道已设定占空比）
 * @note  同组通道共享定时器：任一通道调用即整组变频（其他通道占空比保持不变）。
 * @param ch      逻辑通道（用于校验/命名）
 * @param freq_hz 目标频率 (Hz)，合法区间 [1, timer_clk_hz/2]
 * @return 错误码
 */
drv_pwm_error_t drv_pwm_set_frequency(drv_pwm_channel_t ch, uint32_t freq_hz);

/**
 * @brief 回读通道实际 PWM 频率（整组相同）
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
