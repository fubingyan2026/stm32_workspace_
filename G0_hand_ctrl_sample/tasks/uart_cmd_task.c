/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    uart_cmd_task.c
 * @brief   UART 命令任务 — sw_timer 轮询收发服务 + 周期错误恢复
 */

#include "uart_cmd_task.h"

#include "drv_systick.h"
#include "drv_uart.h"
#include "log.h"
#include "srv_uart_rx_cmd.h"
#include "srv_uart_tx_cmd.h"
#include "sw_timer.h"

/** @brief UART 命令轮询周期 (ms) */
#define UART_CMD_TASK_PERIOD_MS (10U)

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void uart_cmd_timer_cb(void* user_data);

static void uart_cmd_rx_callback(uint8_t cmd, const uint8_t* data,
    uint8_t data_len);

/* Exported functions --------------------------------------------------------*/

void uart_cmd_task_init(void)
{
    srv_uart_tx_cmd_init();
    srv_uart_rx_cmd_init(uart_cmd_rx_callback);

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

    /* 喂数据 + 解析 + 分派 RX 回调 */
    srv_uart_rx_cmd_step();

    /* 空闲超时 tick */
    srv_uart_rx_cmd_tick();
}

/**
 * @brief RX 命令回调：当前打印透传，业务命令表后续由应用层扩展
 */
static void uart_cmd_rx_callback(uint8_t cmd, const uint8_t* data,
    uint8_t data_len)
{
    (void)data;
    LOG_I("uart_cmd_task", "收到命令 cmd=0x%02X data_len=%u",
        (unsigned)cmd, (unsigned)data_len);
}
