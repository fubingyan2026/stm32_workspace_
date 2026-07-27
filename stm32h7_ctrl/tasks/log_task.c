/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    log_task.c
 * @brief   日志输出任务实现（FreeRTOS 线程驱动，无需外部 poll）
 *
 * TX：线程定期检查 log 模块 kfifo，有数据则通过 UART DMA 发送。
 * RX：由 IDLE 中断驱动 drv_uart 自动将数据推入 kfifo。
 */

#include "log_task.h"

#include "cmsis_os2.h"
#include "drv_systick.h"
#include "drv_uart.h"
#include "log.h"

#include "SEGGER_RTT.h"

/* Private constants ---------------------------------------------------------*/

#define LOG_TASK_TX_BUF_SIZE (64)
#define LOG_TASK_PERIOD_MS   (20U)

#define TASK_STACK_SIZE      256U
#define TASK_PRIORITY        osPriorityBelowNormal

/* Private variables ---------------------------------------------------------*/

static uint8_t s_tx_buf[LOG_TASK_TX_BUF_SIZE];
static osThreadId_t s_task_handle;
static log_task_output_t s_output_mode = LOG_OUTPUT_RTT;

/* Private function prototypes -----------------------------------------------*/

static void log_task_entry(void* argument);

static void log_rx_cb(drv_uart_channel_t ch, const uint8_t* data, uint32_t len)
{
    (void)ch;
    log_hexdump("UART0", data, len);
}

/* Exported functions --------------------------------------------------------*/

void log_task_init(void)
{
    log_config_t log_cfg = {
        .name = "E1_PRO",
        .get_timestamp_cb = millis,
    };
    log_init(&log_cfg);
    log_set_level(LOG_LEVEL_DEBUG);

    drv_uart_register_rx_callback(DRV_UART_CH_1, log_rx_cb);

    /* SEGGER RTT 初始化（预初始化以便随时切换） */
    SEGGER_RTT_Init();

    const osThreadAttr_t attr = {
        .name       = "log_task",
        .stack_size = TASK_STACK_SIZE * 4,
        .priority   = TASK_PRIORITY,
    };
    s_task_handle = osThreadNew(log_task_entry, NULL, &attr);
}

void log_task_set_output(log_task_output_t mode)
{
    s_output_mode = mode;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief FreeRTOS 任务入口：发送待输出日志 + 读取接收数据
 */
static void log_task_entry(void* argument)
{
    (void)argument;

    for (;;) {
        /* ── TX ── */
        uint32_t log_len = log_tx_len();
        if (log_len > 0) {
            if (log_len > sizeof(s_tx_buf)) {
                log_len = sizeof(s_tx_buf);
            }
            uint32_t actual = log_tx_get(s_tx_buf, log_len);
            if (actual > 0) {
                switch (s_output_mode) {
                case LOG_OUTPUT_NONE:
                    break;
                case LOG_OUTPUT_RTT:
                    SEGGER_RTT_Write(0, s_tx_buf, actual);
                    break;
                case LOG_OUTPUT_UART:
                    if (!drv_uart_is_tx_busy(DRV_UART_CH_1)) {
                        drv_uart_send(DRV_UART_CH_1, s_tx_buf, actual);
                    }
                    break;
                default:
                    break;
                }
            }
        }

        osDelay(LOG_TASK_PERIOD_MS);
    }
}
