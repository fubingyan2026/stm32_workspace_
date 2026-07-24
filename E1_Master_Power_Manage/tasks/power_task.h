/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    power_task.h
 * @brief   电源管理任务 — sw_timer 驱动 power_control 服务
 */

#ifndef POWER_TASK_H
#define POWER_TASK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void power_task_init(void);
void power_task_request_on(void);
void power_task_emergency_off(void);
bool power_task_is_powered_on(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_TASK_H */
