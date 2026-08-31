/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.c
 * @brief   CAN 通信任务 — sw_timer 驱动：RX 队列消费 + TX 队列排空 + DM4310 电机
 */

#include "can_task.h"

#include "drv_can.h"
#include "log.h"
#include "srv_dm4310_ctrl.h"
#include "sw_timer.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define CAN_TASK_LOG_ENABLE 1

#if CAN_TASK_LOG_ENABLE
#define CAN_TASK_LOG_E(...) LOG_E("can_task", __VA_ARGS__)
#define CAN_TASK_LOG_W(...) LOG_W("can_task", __VA_ARGS__)
#define CAN_TASK_LOG_I(...) LOG_I("can_task", __VA_ARGS__)
#define CAN_TASK_LOG_D(...) LOG_D("can_task", __VA_ARGS__)
#else
#define CAN_TASK_LOG_E(...) ((void)0)
#define CAN_TASK_LOG_W(...) ((void)0)
#define CAN_TASK_LOG_I(...) ((void)0)
#define CAN_TASK_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS 1U

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void can_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void can_task_init(void)
{
    drv_can_error_t err = drv_can_init();
    if (err != DRV_CAN_OK) {
        CAN_TASK_LOG_E("drv_can_init failed: %d", (int)err);
        return; /* CAN 不可用，不启动周期任务 */
    }

    /* DM4310 电机服务（MIT 模式，注册 CAN 发送回调） */
    srv_dm4310_ctrl_init();

    const sw_timer_config_t cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = can_timer_cb,
    };
    sw_timer_init(&s_timer, &cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);

    CAN_TASK_LOG_I("CAN 任务初始化完成 (period=%ums)",
        (unsigned)TASK_PERIOD_MS);
}

/* Private functions ---------------------------------------------------------*/

static void can_timer_cb(void* user_data)
{
    (void)user_data;

    /* Bus-Off 检测/恢复（bxCAN） */
    if (drv_can_is_bus_off(DRV_CAN_CH_1)) {
        (void)drv_can_recover(DRV_CAN_CH_1);
    }

    /* RX 队列消费 → DM4310 电机反馈解析（主循环上下文，非 ISR） */
    srv_dm4310_ctrl_poll_rx();

    /* DM4310 电机状态机步进（初始化/使能/禁用状态转换） */
    srv_dm4310_ctrl_step();

    /* TX 队列排空到 bxCAN TX 邮箱 */
    drv_can_tx_flush(DRV_CAN_CH_1);
}
