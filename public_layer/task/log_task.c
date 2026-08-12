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
 * RX：USART1 控制台命令解析（log / logclear / help），并驱动日志 Flash 落盘。
 */

#include "log_task.h"

#include <string.h>

#include "drv_log_uart.h"
#include "drv_systick.h"
#include "log.h"
#include "sw_timer.h"

/* srv_log_flash 为 App 专属服务（落盘区域位于 Boot 代码区与 App 之间），
 * Boot 镜像不编译该服务，故将相关调用一并排除。 */
#if !defined(E1_BUILD_BOOT)
#include "srv_log_flash.h"
#endif

#include "SEGGER_RTT.h"

/* Private constants ---------------------------------------------------------*/

#define LOG_TASK_TX_BUF_SIZE (1024)
#define LOG_TASK_PERIOD_MS (10U)
#define LOG_TASK_CMD_BUF_SIZE (32U)

/* Private types -------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 日志输出后端
 */
typedef enum {
    LOG_OUTPUT_NONE = 0, /**< 关闭输出 */
    LOG_OUTPUT_UART, /**< USART1 DMA 输出（默认） */
    LOG_OUTPUT_RTT, /**< SEGGER RTT 输出 */
} log_task_output_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 控制台命令枚举（USART1 收到整行后置位，主循环 timer 消费）
 */
typedef enum {
    LOG_TASK_CMD_NONE = 0, /**< 无命令 */
    LOG_TASK_CMD_DUMP, /**< 打印 Flash 存储日志 */
    LOG_TASK_CMD_CLEAR, /**< 清空 Flash 存储日志 */
    LOG_TASK_CMD_HELP, /**< 显示帮助 */
    LOG_TASK_CMD_UNKNOWN, /**< 未知命令 */
} log_task_cmd_t;

/* Private variables ---------------------------------------------------------*/

static uint8_t s_tx_buf[LOG_TASK_TX_BUF_SIZE];
static sw_timer_t s_log_timer;
static log_task_output_t s_output_mode = LOG_OUTPUT_UART;

/** @brief 控制台命令行累积缓冲（主循环从 drv_log_uart kfifo 读取后填充） */
static char s_cmd_buf[LOG_TASK_CMD_BUF_SIZE];
static uint8_t s_cmd_len = 0;

/** @brief 待处理命令（主循环置位并消费） */
static log_task_cmd_t s_pending_cmd = LOG_TASK_CMD_NONE;

/* Private function prototypes -----------------------------------------------*/

static void log_timer_cb(void* user_data);

static void log_task_print_help(void);

static log_task_cmd_t log_task_parse_cmd(const char* line);

static void log_task_poll_rx(void);

/* Exported functions --------------------------------------------------------*/

void log_task_init(void)
{
    log_config_t log_cfg = {
        .name = "E1_PRO",
        .get_timestamp_cb = millis,
    };
    log_init(&log_cfg);
    log_set_level(LOG_LEVEL_DEBUG);

    /* 日志串口（USART1 DMA）驱动初始化：本任务是 drv_log_uart 的唯一消费者，
     * 由 log_task 自行完成初始化，避免各 app_main（App/Boot 多工程）遗漏。
     * 需放在 log_init 之后：drv_log_uart_init() 内部的“初始化完成”日志
     * 经 log 缓冲输出，若先于 log_init 调用会被静默丢弃。 */
    (void)drv_log_uart_init();

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
 * @brief sw_timer 回调：控制台命令 + 日志落盘驱动 + 发送待输出日志
 */
static void log_timer_cb(void* user_data)
{
    (void)user_data;

    /* ── 控制台 RX：从 drv_log_uart kfifo 读取并累积命令行（主循环上下文） ── */
    log_task_poll_rx();

    /* ── 控制台命令处理 ── */
    const log_task_cmd_t cmd = s_pending_cmd;
    s_pending_cmd = LOG_TASK_CMD_NONE;

    switch (cmd) {
#if !defined(E1_BUILD_BOOT)
    case LOG_TASK_CMD_DUMP:
        srv_log_flash_dump();
        break;
    case LOG_TASK_CMD_CLEAR:
        srv_log_flash_clear();
        break;
#endif
    case LOG_TASK_CMD_HELP:
        log_task_print_help();
        break;
    case LOG_TASK_CMD_UNKNOWN:
        LOG_W("log_task", "未知命令，输入 help 查看可用命令");
        log_task_print_help();
        break;
    default:
        break;
    }

#if !defined(E1_BUILD_BOOT)
    /* ── 日志 Flash 落盘驱动（有新增记录且到限流间隔时写入） ── */
    srv_log_flash_step();
#endif

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
}

/**
 * @brief 打印控制台帮助
 */
static void log_task_print_help(void)
{
    static const char help_text[] = "可用命令:\r\n"
                                    "  log       打印 Flash 存储的 WARN/ERROR 日志\r\n"
                                    "  logclear  清空 Flash 日志\r\n"
                                    "  help / ?  显示本帮助\r\n";
    log_write((const uint8_t*)help_text, (uint32_t)strlen(help_text));
}

/**
 * @brief 解析一行控制台命令
 * @param line 以 '\0' 结尾的命令行
 * @return 命令枚举
 */
static log_task_cmd_t log_task_parse_cmd(const char* line)
{
    if (strcmp(line, "log") == 0) {
        return LOG_TASK_CMD_DUMP;
    }
    if (strcmp(line, "logclear") == 0) {
        return LOG_TASK_CMD_CLEAR;
    }
    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        return LOG_TASK_CMD_HELP;
    }
    return LOG_TASK_CMD_UNKNOWN;
}

/**
 * @brief 从 drv_log_uart 接收 kfifo 读取字节，累积命令行，整行到达即解析
 * @note  主循环上下文轮询（drv_log_uart 的 IDLE 中断只把字节推进 kfifo，
 *        不在中断里做命令解析），对任意长度/任意分块的输入都稳定。
 */
static void log_task_poll_rx(void)
{
    uint8_t buf[16];
    uint32_t n = drv_log_uart_rx_read(buf, sizeof(buf));

    for (uint32_t i = 0; i < n; i++) {
        const uint8_t c = buf[i];

        if (c == '\r' || c == '\n') {
            if (s_cmd_len > 0) {
                s_cmd_buf[s_cmd_len] = '\0';
                s_pending_cmd = log_task_parse_cmd(s_cmd_buf);
                s_cmd_len = 0;
            }
            continue;
        }

        if (s_cmd_len >= sizeof(s_cmd_buf) - 1) {
            s_cmd_len = 0; /* 命令过长，丢弃并复位 */
            continue;
        }
        s_cmd_buf[s_cmd_len++] = (char)c;
    }
}
