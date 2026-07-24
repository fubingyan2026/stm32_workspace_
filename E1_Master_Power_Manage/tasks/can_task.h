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

#include "srv_can_mst.h"
#include "srv_can_slv.h"

#ifdef __cplusplus
extern "C" {
#endif

void can_task_init(void);
void can_task_tick(void);
void can_task_request(uint8_t feedback_select);

/**
 * @brief 设置当前从板控制状态（由 host RX 或电源时序触发）
 */
void can_task_set_slave_ctrl(bool hsd_12v_on, bool reserved_channel);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TASK_H */
