/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    log_task.c
 * @brief   日志输出任务（本地副本，E1_MASTER_POWER_CTU）
 *
 * 支持 USART1 DMA（量产默认）与 SEGGER RTT（J-Link 调试）两种输出后端，
 * 运行时通过 log_task_set_output() 切换。参考 G0_hand_ctrl_sample 本地
 * log_task 双后端设计；本工程不启用控制台命令，仅日志 TX。
 */

/* Includes ------------------------------------------------------------------*/
#include "log_task.h"

#include <string.h>

#include "SEGGER_RTT.h"
#include "drv_log_uart.h"
#include "drv_systick.h"
#include "log.h"
#include "sw_timer.h"

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 日志输出后端
 */
typedef enum {
    LOG_TASK_OUTPUT_NONE = 0, /**< 关闭输出 */
    LOG_TASK_OUTPUT_UART, /**< USART1 DMA 输出（量产默认） */
    LOG_TASK_OUTPUT_RTT, /**< SEGGER RTT 输出（J-Link 调试） */
} log_task_output_t;

#define LOG_TASK_TX_BUF_SIZE (1024U)
#define LOG_TASK_PERIOD_MS (10U)

/* Private variables ---------------------------------------------------------*/

static uint8_t s_tx_buf[LOG_TASK_TX_BUF_SIZE];
static sw_timer_t s_log_timer;
static const log_task_output_t s_output_mode = LOG_TASK_OUTPUT_UART;

/* Private function prototypes -----------------------------------------------*/

static void log_timer_cb(void* user_data);
static void log_task_drain_tx(void);

/* Exported functions --------------------------------------------------------*/

void log_task_init(void)
{
    log_config_t log_cfg = {
        .name = "E1_CTU",
        .get_timestamp_cb = millis,
    };
    log_init(&log_cfg);
    log_set_level(LOG_LEVEL_DEBUG);

    /* UART 后端驱动初始化（本任务是 drv_log_uart 的唯一消费者，先于 log_init
     * 调用其内部“初始化完成”日志会被静默丢弃，故须放在 log_init 之后） */
    if (s_output_mode == LOG_TASK_OUTPUT_UART) {
        (void)drv_log_uart_init();
    }

    /* RTT 预初始化，便于随时切换后端 */
    if (s_output_mode == LOG_TASK_OUTPUT_RTT) {
        SEGGER_RTT_Init();
    }

    /* 启动 sw_timer 驱动 TX 发送 */
    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = log_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_log_timer, &timer_cfg);
    sw_timer_start(&s_log_timer, LOG_TASK_PERIOD_MS, 0);
}

void log_task_flush(void)
{
    static uint8_t buf[LOG_TASK_TX_BUF_SIZE];

    /* 把 log 缓冲全部搬移到当前输出后端 */
    for (uint32_t guard = 0U; guard < 128U; guard++) {
        const uint32_t pending = log_tx_len();
        if (pending == 0U) {
            break;
        }
        const uint32_t len = (pending > sizeof(buf)) ? sizeof(buf) : pending;

        switch (s_output_mode) {
        case LOG_TASK_OUTPUT_NONE:
            (void)log_tx_get(buf, len); /* 丢弃 */
            break;
        case LOG_TASK_OUTPUT_RTT: {
            const uint32_t actual = log_tx_get(buf, len);
            if (actual > 0U) {
                SEGGER_RTT_Write(0, buf, actual);
            }
            break;
        }
        case LOG_TASK_OUTPUT_UART:
        default: {
            const uint32_t t0 = millis();
            while (drv_log_uart_is_tx_busy()) {
                if ((uint32_t)(millis() - t0) > 100U) {
                    break;
                }
            }
            if (!drv_log_uart_is_tx_busy()) {
                const uint32_t actual = log_tx_get(buf, len);
                if (actual > 0U) {
                    drv_log_uart_send(buf, actual);
                }
            }
            break;
        }
        }
    }

    /* 等最后一段 UART DMA 传输完成（有界等待） */
    {
        const uint32_t t0 = millis();
        while (drv_log_uart_is_tx_busy()) {
            if ((uint32_t)(millis() - t0) > 200U) {
                break;
            }
        }
    }
}

log_task_output_t log_task_get_output(void)
{
    return s_output_mode;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief sw_timer 回调：排空 log 缓冲到当前输出后端
 */
static void log_timer_cb(void* user_data)
{
    (void)user_data;
    log_task_drain_tx();
}

/**
 * @brief 将 log 缓冲最新一段输出到当前后端
 */
static void log_task_drain_tx(void)
{
    uint32_t log_len = log_tx_len();
    if (log_len == 0U) {
        return;
    }
    if (log_len > sizeof(s_tx_buf)) {
        log_len = sizeof(s_tx_buf);
    }

    switch (s_output_mode) {
    case LOG_TASK_OUTPUT_NONE:
        /* 不输出 */
        break;
    case LOG_TASK_OUTPUT_RTT: {
        const uint32_t actual = log_tx_get(s_tx_buf, log_len);
        if (actual > 0U) {
            SEGGER_RTT_Write(0, s_tx_buf, actual);
        }
        break;
    }
    case LOG_TASK_OUTPUT_UART:
    default:
        if (!drv_log_uart_is_tx_busy()) {
            const uint32_t actual = log_tx_get(s_tx_buf, log_len);
            if (actual > 0U) {
                drv_log_uart_send(s_tx_buf, actual);
            }
        }
        break;
    }
}
