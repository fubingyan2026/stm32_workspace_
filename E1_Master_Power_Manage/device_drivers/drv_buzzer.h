/**
 * @file    drv_buzzer.h
 * @author  maximillian
 * @version V2.1.0
 * @date    2026-07-8
 * @brief   蜂鸣器设备驱动（TIM12_CH2 PWM 输出，句柄自包含）
 * @attention
 *
 * CubeMX tim.c 已将 PB15 配置为 TIM12_CH2 PWM。句柄内置在 drv_buzzer.c 中。
 * 仅负责硬件配置和 PWM 输出，不包含超时保护、开机提示音等行为逻辑。
 */

#ifndef __DRV_BUZZER_H
#define __DRV_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化蜂鸣器 PWM（内部句柄，无需传参）
 * @note  启动 PWM 输出，初始占空比 0（静音）
 */
void drv_buzzer_init(void);

/** @brief 反初始化，停止 PWM */
void drv_buzzer_deinit(void);

/**
 * @brief 设置蜂鸣器占空比
 * @param duty 0-100（0=静音，100=最响）
 */
void drv_buzzer_set(uint8_t duty);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_BUZZER_H */
