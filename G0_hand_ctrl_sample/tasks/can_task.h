/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.h
 * @brief   CAN 通信任务 — RX 队列消费 + TX 队列排空 + DM4310 电机
 */

#ifndef CAN_TASK_H
#define CAN_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void can_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TASK_H */
