/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    uart_cmd_task.c
 * @brief   UART 命令任务 — sw_timer 驱动底层收发轮询 + 周期错误恢复
 * @attention
 *
 * 本任务只负责底层驱动（drv_uart 恢复、TX 队列排空、RX step/tick）与
 * 应用层步进（app_uart_interact_step：心跳）。协议帧的业务收发与
 * 相关初始化（srv_uart_tx_cmd_init / srv_uart_rx_cmd_init / 按键上报）
 * 全部集中在 applications/app_uart_interact。
 */

#include "uart_cmd_task.h"

#include "app_uart_interact.h"
#include "drv_uart.h"
#include "log.h"
#include "srv_uart_rx_cmd.h"
#include "sw_timer.h"

/** @brief UART 命令轮询周期 (ms) */
#define UART_CMD_TASK_PERIOD_MS (1U)

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void uart_cmd_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void uart_cmd_task_init(void)
{
    /* UART 交互：按键事件上报（需在 key_task 与 uart_cmd_task 之后） */
    app_uart_interact_init();

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = uart_cmd_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, UART_CMD_TASK_PERIOD_MS, 0);

    LOG_I("uart_cmd_task", "UART 命令任务初始化完成 (period=%ums)",
        (unsigned)UART_CMD_TASK_PERIOD_MS);
}

/* Private functions ---------------------------------------------------------*/

static void uart_cmd_timer_cb(void* user_data)
{
    (void)user_data;

    /* 周期 UART 错误兜底恢复（状态正常时零开销返回） */
    (void)drv_uart_recover(DRV_UART_CH_2);

    /* 排空 TX 缓冲队列到 DMA（忙时入队的帧在此发出，保证不丢失） */
    drv_uart_tx_flush(DRV_UART_CH_2);

    /* 喂数据 + 解析 + 分派 RX 回调（回调在 app_uart_interact） */
    srv_uart_rx_cmd_step();

    /* 空闲超时 tick */
    srv_uart_rx_cmd_tick();

    /* 应用层步进：周期心跳 */
    app_uart_interact_step();
}
