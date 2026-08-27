/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.c
 * @brief   CAN 通信任务 — sw_timer 驱动 srv_can 处理 + 周期心跳
 */

#include "can_task.h"

#include "drv_can.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_can.h"
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

#define TASK_PERIOD_MS 10U
#define HEARTBEAT_INTERVAL_MS 1000U

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static uint16_t s_heartbeat_ms;

/* Private function prototypes -----------------------------------------------*/

static void can_timer_cb(void* user_data);

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions --------------------------------------------------------*/

void can_task_init(void)
{
    drv_can_error_t err = drv_can_init();
    if (err != DRV_CAN_OK) {
        CAN_TASK_LOG_E("drv_can_init failed: %d", (int)err);
        return; /* CAN 不可用，不启动周期任务 */
    }

    srv_can_init();

    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);

    s_heartbeat_ms = 0;

    const sw_timer_config_t cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = can_timer_cb,
    };
    sw_timer_init(&s_timer, &cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);

    CAN_TASK_LOG_I("CAN 任务初始化完成 (period=%ums, heartbeat=%ums)",
        (unsigned)TASK_PERIOD_MS, (unsigned)HEARTBEAT_INTERVAL_MS);
}

/* Private functions ---------------------------------------------------------*/

static void can_timer_cb(void* user_data)
{
    (void)user_data;

    /* Bus-Off 检测/恢复（bxCAN） */
    if (drv_can_is_bus_off(DRV_CAN_CH_1)) {
        (void)drv_can_recover(DRV_CAN_CH_1);
    }

    srv_can_process();

    /* 周期心跳（链路验证） */
    s_heartbeat_ms += TASK_PERIOD_MS;
    if (s_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        s_heartbeat_ms = 0;
        srv_can_send_heartbeat();
    }
}

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    (void)ch;
    srv_can_on_rx(msg);
}
