/**
 * @file    drv_fan.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-8
 * @brief   风扇驱动（PWM 调速 + EXTI 脉冲计数测速，自包含引脚配置）
 * @attention
 *
 * 硬件配置内置在 drv_fan.c 中，上层无需传递引脚参数。
 * PWM: TIM10_CH1(FAN0) / TIM11_CH1(FAN1)，FG: EXTI0(PE0) / EXTI1(PE1)。
 */

#ifndef __DRV_FAN_H
#define __DRV_FAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

#define DRV_FAN_MAX (2U) /**< 最大风扇数量 */

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化全部风扇（内部引脚表，无需传参） */
void drv_fan_init(void);

/** @brief 反初始化全部风扇 */
void drv_fan_deinit_all(void);

/** @brief 获取已配置的风扇数量 */
uint32_t drv_fan_get_count(void);

/** @brief 设置占空比 0-100 */
void drv_fan_set_duty(uint32_t id, uint8_t duty);

/**
 * @brief 获取自上次调用以来的 FG 脉冲计数（读取后清零）
 * @param id 风扇编号
 * @return 脉冲数（0=无有效数据）
 */
uint32_t drv_fan_get_tach_delta(uint32_t id);

/** @brief 获取每转脉冲数 */
uint8_t drv_fan_get_pulse_per_rev(uint32_t id);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_FAN_H */
