/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    log_task.c
 * @brief   日志输出任务实现（sw_timer 驱动，无需外部 poll）
 *
 * TX：sw_timer 定期检查 log 模块 kfifo，有数据则通过 UART DMA 发送。
 * RX：由 IDLE 中断驱动 drv_uart 自动将数据推入 kfifo。
 */

#include "log_task.h"

#include "drv_log_uart.h"
#include "drv_systick.h"
#include "log.h"
#include "sw_timer.h"

#include "SEGGER_RTT.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define LOG_TASK_LOG_ENABLE 1

#if LOG_TASK_LOG_ENABLE
#define LOG_TASK_LOG_E(...) LOG_E("log_task", __VA_ARGS__)
#define LOG_TASK_LOG_W(...) LOG_W("log_task", __VA_ARGS__)
#define LOG_TASK_LOG_I(...) LOG_I("log_task", __VA_ARGS__)
#define LOG_TASK_LOG_D(...) LOG_D("log_task", __VA_ARGS__)
#else
#define LOG_TASK_LOG_E(...) ((void)0)
#define LOG_TASK_LOG_W(...) ((void)0)
#define LOG_TASK_LOG_I(...) ((void)0)
#define LOG_TASK_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define LOG_TASK_TX_BUF_SIZE (256)
#define LOG_TASK_PERIOD_MS (20U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 日志输出后端
 */
typedef enum {
    LOG_OUTPUT_NONE = 0, /**< 关闭输出 */
    LOG_OUTPUT_UART, /**< USART1 DMA 输出（默认） */
    LOG_OUTPUT_RTT, /**< SEGGER RTT 输出 */
} log_task_output_t;

/* Private variables ---------------------------------------------------------*/

static uint8_t s_tx_buf[LOG_TASK_TX_BUF_SIZE];
static sw_timer_t s_log_timer;
static log_task_output_t s_output_mode = LOG_OUTPUT_UART;

/* Private function prototypes -----------------------------------------------*/

static void log_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void log_task_init(void)
{
    log_config_t log_cfg = {
        .name = "E1_PRO",
        .get_timestamp_cb = millis,
    };
    log_init(&log_cfg);
    log_set_level(LOG_LEVEL_INFO);

    /* 注：drv_uart_init() 由 app_main 统一调用，此处不再单独调用 */
    drv_log_uart_init();
    /* SEGGER RTT 初始化（无论当前模式，预初始化以便随时切换） */
    SEGGER_RTT_Init();

    /* 启动 sw_timer 驱动 TX 发送 */
    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = log_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_log_timer, &timer_cfg);
    sw_timer_start(&s_log_timer, LOG_TASK_PERIOD_MS, 0);

    LOG_TASK_LOG_I("日志任务初始化完成: 输出=UART DMA, 周期=%ums, 级别=DEBUG",
        (unsigned)LOG_TASK_PERIOD_MS);
}

void log_task_flush(void)
{
    uint8_t buf[LOG_TASK_TX_BUF_SIZE];

    /* 1) 把 log 缓冲全部搬移到输出后端（UART 非阻塞发送前先等上一段 DMA 空闲） */
    for (uint32_t guard = 0U; guard < 256U; guard++) {
        const uint32_t pending = log_tx_len();
        if (pending == 0U) {
            break;
        }
        const uint32_t len = (pending > sizeof(buf)) ? sizeof(buf) : pending;

        switch (s_output_mode) {
        case LOG_OUTPUT_NONE:
            (void)log_tx_get(buf, len); /* 丢弃 */
            break;
        case LOG_OUTPUT_RTT: {
            const uint32_t actual = log_tx_get(buf, len);
            if (actual > 0U) {
                SEGGER_RTT_Write(0, buf, actual);
            }
            break;
        }
        case LOG_OUTPUT_UART:
        default: {
            /* 等上一段 DMA 空闲（有界等待），避免非阻塞发送被丢弃 */
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

    /* 2) 等最后一段 UART DMA 传输完成（有界等待，UART 波特率 115200 下 256B ≈ 22ms） */
    {
        const uint32_t t0 = millis();
        while (drv_log_uart_is_tx_busy()) {
            if ((uint32_t)(millis() - t0) > 200U) {
                break;
            }
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief sw_timer 回调：发送待输出日志 + 读取接收数据
 */
static void log_timer_cb(void* user_data)
{
    (void)user_data;
    /* 注：此处为 kfifo→UART 排空路径，禁止添加任何日志（会回灌自身 kfifo） */
    /* ── TX ── */
    uint32_t log_len = log_tx_len();
    if (log_len > 0) {
        if (log_len > sizeof(s_tx_buf)) {
            log_len = sizeof(s_tx_buf);
        }
        switch (s_output_mode) {
        case LOG_OUTPUT_NONE:
            /* 不输出 */
            break;
        case LOG_OUTPUT_RTT:
            uint32_t actual = log_tx_get(s_tx_buf, log_len);
            if (actual > 0) {
                SEGGER_RTT_Write(0, s_tx_buf, actual);
            }

            break;
        case LOG_OUTPUT_UART:
            if (!drv_log_uart_is_tx_busy()) {
                uint32_t actual = log_tx_get(s_tx_buf, log_len);
                if (actual > 0) {
                    drv_log_uart_send(s_tx_buf, actual);
                }
            }
            break;
        default:
            break;
        }
    }

    /* ── RX ── */
    {
        uint8_t rx_buf[128];
        uint32_t rx_len = drv_log_uart_rx_read(rx_buf, sizeof(rx_buf));
        if (rx_len > 0) {
            log_hexdump("UART1", rx_buf, rx_len);
        }
    }
}
