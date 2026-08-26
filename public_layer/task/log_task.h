/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    log_task.h
 * @brief   日志输出任务（sw_timer 驱动，无需外部 poll）
 *
 * 支持 UART / SEGGER RTT 两种输出后端。App 默认 RTT（USART1 让位给上位机
 * 二进制主机协议），Boot 默认 UART；由 log_task.c 内 LOG_TASK_DEFAULT_OUTPUT_RTT 决定。
 */

#ifndef LOG_TASK_H
#define LOG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化日志任务
 * @note  初始化 log 模块 + 启动 sw_timer，输出后端按编译配置（App 默认 RTT）
 */
void log_task_init(void);

/**
 * @brief 强制排空当前 log 缓冲到输出后端
 * @note  有界等待 TX 完成（UART 模式下），可在任意时刻调用
 */
void log_task_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* LOG_TASK_H */
