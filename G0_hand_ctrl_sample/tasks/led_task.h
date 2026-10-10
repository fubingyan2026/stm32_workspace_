/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.h
 * @brief   LED 状态指示任务 — TIM1_CH1 呼吸模式
 */

#ifndef LED_TASK_H
#define LED_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void led_task_init(void);

/**
 * @brief 指示“上电时按键被按下”的初始化异常：LED 只显示红色
 * @note  非锁存：按键松开后自动恢复正常（白色）；供上电自检调用
 */
void led_task_indicate_boot_fault(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_TASK_H */
