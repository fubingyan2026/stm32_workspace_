/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.h
 * @brief   运行 LED 状态指示任务（sw_timer 驱动）
 */

#ifndef __LED_TASK_H
#define __LED_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化运行 LED 任务
 *
 * 点亮运行 LED 并启动 sw_timer：正常运行常亮，升级期间闪烁。
 */
void led_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_TASK_H */
