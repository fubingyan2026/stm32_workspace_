/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    log_task.h
 * @brief   日志输出任务（本地副本，E1_MASTER_POWER_CTU）
 *
 * 支持 UART（USART1 DMA）与 SEGGER RTT 两种输出后端，默认 UART。
 * 运行时可通过 log_task_set_output() 切换。
 */

#ifndef LOG_TASK_H
#define LOG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 初始化日志任务
 * @note  初始化 log 模块 + 输出后端 + 启动 sw_timer，输出后端默认为 UART
 */
void log_task_init(void);

/**
 * @brief 强制排空当前 log 缓冲到当前输出后端
 * @note  有界等待 TX 完成（UART 模式下），可在任意时刻调用
 */
void log_task_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* LOG_TASK_H */
