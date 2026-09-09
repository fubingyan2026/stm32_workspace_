/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    com_task.h
 * @brief   RS485 通信任务 — 主机查询应答式从站 (E1_MASTER_POWER_CTU)
 */

#ifndef COM_TASK_H
#define COM_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void com_task_init(void);

/**
 * @brief 主循环高频服务（RX 搬运/解析应答/TX 排空），由 app_main 每轮迭代调用
 */
void com_task_service(void);

#ifdef __cplusplus
}
#endif

#endif /* COM_TASK_H */
