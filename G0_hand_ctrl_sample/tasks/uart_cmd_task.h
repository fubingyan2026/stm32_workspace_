/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    uart_cmd_task.h
 * @brief   UART 命令任务 — sw_timer 轮询收发服务 + 周期错误恢复
 */

#ifndef UART_CMD_TASK_H
#define UART_CMD_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void uart_cmd_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_CMD_TASK_H */
