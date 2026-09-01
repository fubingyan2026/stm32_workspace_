/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    daemon_task.h
 * @brief   守护进程任务 — sw_timer 周期驱动 daemon_task() 检测各服务在线状态
 */

#ifndef DAEMON_TASK_H
#define DAEMON_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void daemon_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DAEMON_TASK_H */
