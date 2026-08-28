/**
 * @file    srv_uart_rx_cmd.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   UART 命令接收服务实现 — protocol_parser 帧解析 + 回调上抛
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_uart_rx_cmd.h"

#include "crc.h"
#include "drv_uart.h"
#include "log.h"
#include "main.h"
#include "protocol_parser.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_UART_RX_CMD_LOG_ENABLE 1

#if SRV_UART_RX_CMD_LOG_ENABLE
#define SRV_UART_RX_CMD_LOG_E(...) LOG_E("srv_uart_rx_cmd", __VA_ARGS__)
#define SRV_UART_RX_CMD_LOG_W(...) LOG_W("srv_uart_rx_cmd", __VA_ARGS__)
#define SRV_UART_RX_CMD_LOG_I(...) LOG_I("srv_uart_rx_cmd", __VA_ARGS__)
#define SRV_UART_RX_CMD_LOG_D(...) LOG_D("srv_uart_rx_cmd", __VA_ARGS__)
#else
#define SRV_UART_RX_CMD_LOG_E(...) ((void)0)
#define SRV_UART_RX_CMD_LOG_W(...) ((void)0)
#define SRV_UART_RX_CMD_LOG_I(...) ((void)0)
#define SRV_UART_RX_CMD_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 帧头/帧尾（协议定义） */
static const uint8_t s_header[] = { 'z' };
static const uint8_t s_footer[] = { '\n' };

/** @brief 从 drv_uart 读取数据的临时缓冲 */
#define SRV_UART_RX_CMD_READ_BUF_SIZE (32U)

/** @brief 错误日志限频窗口 (ms) */
#define SRV_UART_RX_CMD_ERR_LOG_PERIOD_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static protocol_parser_context_t s_parser;
static uint8_t s_input_buf[512]; /**< parser 输入 kfifo 缓冲（2 的幂） */
static uint8_t s_output_buf[SRV_UART_RX_CMD_MAX_FRAME + 4U]; /**< parser 输出帧缓冲 */
static srv_uart_rx_cmd_cb_t s_rx_cb;
static uint32_t s_rx_count; /**< 已收完整命令帧计数 */
static uint32_t s_last_err_log; /**< 上次错误日志时间戳 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static uint16_t rx_get_len_cb(uint8_t* buffer, uint16_t len);

static protocol_parser_error_t rx_check_cb(uint8_t* buffer, uint16_t len);

static void rx_log_error(const char* tag, int32_t err);

/* Exported functions --------------------------------------------------------*/

void srv_uart_rx_cmd_init(srv_uart_rx_cmd_cb_t callback)
{
    s_rx_cb = callback;
    s_rx_count = 0;
    s_last_err_log = 0;

    const protocol_parser_config_t cfg = {
        .name = "srv_uart_rx_cmd",
        .header = s_header,
        .footer = s_footer,
        .output_buffer = s_output_buf,
        .input_buffer = s_input_buf,
        .get_len_cb = rx_get_len_cb,
        .check_cb = rx_check_cb,
        .header_len = 1,
        .footer_len = 1,
        .input_buffer_len = (uint16_t)sizeof(s_input_buf),
        .output_buffer_len = (uint16_t)sizeof(s_output_buf),
    };

    (void)protocol_parser_init(&s_parser, &cfg);
    s_initialized = true;

    SRV_UART_RX_CMD_LOG_I("UART 命令接收服务初始化完成");
}

void srv_uart_rx_cmd_step(void)
{
    if (!s_initialized) {
        return;
    }

    /* 1) 从 drv_uart 读取并喂入解析器 */
    uint8_t buf[SRV_UART_RX_CMD_READ_BUF_SIZE];
    uint32_t n = drv_uart_rx_available(DRV_UART_CH_2);
    while (n > 0U) {
        const uint32_t chunk = (n > sizeof(buf)) ? (uint32_t)sizeof(buf) : n;
        const uint32_t rd = drv_uart_rx_read(DRV_UART_CH_2, buf, chunk);
        if (rd == 0U) {
            break;
        }
        (void)protocol_parser_feed(&s_parser, buf, rd);
        n -= rd;
    }

    /* 2) 解析完整帧并分派回调 */
    uint16_t frame_len = 0;
    uint8_t* frame = NULL;

    for (;;) {
        const protocol_parser_error_t err =
            protocol_parser_parse(&s_parser, &frame_len, &frame);

        if (err == PROTOCOL_PARSER_OK) {
            /* 帧校验：结构合法性（frame[0] 帧头 / frame[len-1] 帧尾 / data_len 一致） */
            if (frame == NULL || frame_len < 5U
                || frame[0] != 'z' || frame[frame_len - 1] != '\n') {
                rx_log_error("非法帧结构", (int32_t)err);
                continue;
            }

            const uint8_t data_len = frame[2];
            if (data_len != (frame_len - 5U)) {
                rx_log_error("data_len 不一致", (int32_t)err);
                continue;
            }

            s_rx_count++;

            if (s_rx_cb) {
                s_rx_cb(frame[1], &frame[3], data_len);
            }
            continue;
        }

        /* 不完整 / 空闲超时：等待更多数据，终止本轮解析 */
        if (err == PROTOCOL_PARSER_ERROR_INCOMPLETE
            || err == PROTOCOL_PARSER_ERROR_IDLE_TIMEOUT) {
            break;
        }

        /* 其他错误（CHECKSUM/FOOTER_MISMATCH/HEADER_MISMATCH 等）：
         * parser 已自行跳过垃圾字节，限频告警后继续 */
        rx_log_error("解析错误", (int32_t)err);
    }
}

void srv_uart_rx_cmd_tick(void)
{
    if (!s_initialized) {
        return;
    }
    (void)protocol_parser_tick(&s_parser);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 帧长度计算回调
 * @param buffer 含帧头的帧数据（header 已匹配）
 * @param len    当前已缓冲长度
 * @return 完整帧总长（data_len + 5）；不足返回 0
 * @note  buffer[0]='z', buffer[1]=cmd, buffer[2]=data_len
 */
static uint16_t rx_get_len_cb(uint8_t* buffer, uint16_t len)
{
    if (buffer == NULL) {
        return 0;
    }
    if (len < 3U) {
        return 0; /* 尚不足 cmd + data_len 两个字段 */
    }
    return (uint16_t)buffer[2] + 5U;
}

/**
 * @brief 帧校验回调（CRC8）
 * @param buffer 完整帧（含 header + cmd + data_len + payload + crc + footer）
 * @param len    帧总长 = data_len + 5
 * @note  crc 字节位于 len-2（footer '\n' 在 len-1）
 */
static protocol_parser_error_t rx_check_cb(uint8_t* buffer, uint16_t len)
{
    if (buffer == NULL || len < 3U) {
        return PROTOCOL_PARSER_ERROR_INVALID_PARAM;
    }

    const uint8_t expected = get_CRC8_check_sum(buffer, len - 2U, 0xFF);
    if (expected == buffer[len - 2U]) {
        return PROTOCOL_PARSER_OK;
    }
    return PROTOCOL_PARSER_ERROR_CHECKSUM;
}

static void rx_log_error(const char* tag, int32_t err)
{
    if ((uint32_t)(HAL_GetTick() - s_last_err_log) >= SRV_UART_RX_CMD_ERR_LOG_PERIOD_MS) {
        s_last_err_log = HAL_GetTick();
        SRV_UART_RX_CMD_LOG_W("%s: err=%d", tag, (int)err);
    }
}
