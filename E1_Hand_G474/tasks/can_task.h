/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.h
 * @brief   CAN 通信任务 — 主机上报 + 从板控制 + RX 接收
 */

#ifndef CAN_TASK_H
#define CAN_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void can_task_init(void);
void can_task_tick(void);

/**
 * @brief 高速步进（主循环全速调用）：驱动电机/传感器控制（如橘虾 MIT 1ms→全速、Mz 响应门控轮询）
 * @note  仅包含需要最高周期的步骤（CAN1/CAN2 测试模块 step），
 *        总线状态轮询与 UART 主机协议仍在 1ms 定时器中执行
 */
void can_task_fast_step(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TASK_H */
