/**
 * @file    srv_uart_tx_cmd.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   UART 命令发送服务实现 — protocol_packer 帧打包 + drv_uart 发送
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_uart_tx_cmd.h"

#include "crc.h"
#include "drv_uart.h"
#include "log.h"
#include "protocol_packer.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_UART_TX_CMD_LOG_ENABLE 1

#if SRV_UART_TX_CMD_LOG_ENABLE
#define SRV_UART_TX_CMD_LOG_E(...) LOG_E("srv_uart_tx_cmd", __VA_ARGS__)
#define SRV_UART_TX_CMD_LOG_W(...) LOG_W("srv_uart_tx_cmd", __VA_ARGS__)
#define SRV_UART_TX_CMD_LOG_I(...) LOG_I("srv_uart_tx_cmd", __VA_ARGS__)
#define SRV_UART_TX_CMD_LOG_D(...) LOG_D("srv_uart_tx_cmd", __VA_ARGS__)
#else
#define SRV_UART_TX_CMD_LOG_E(...) ((void)0)
#define SRV_UART_TX_CMD_LOG_W(...) ((void)0)
#define SRV_UART_TX_CMD_LOG_I(...) ((void)0)
#define SRV_UART_TX_CMD_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 帧头/帧尾（协议定义） */
static const uint8_t s_header[] = { 'z' };
static const uint8_t s_footer[] = { '\n' };

/* Private variables ---------------------------------------------------------*/

static protocol_packer_context_t s_packer;
static uint8_t s_pack_out[SRV_UART_TX_CMD_MAX_FRAME];
static uint8_t s_cmd_buf[SRV_UART_TX_CMD_MAX_PAYLOAD + 2U]; /* cmd + data_len占位 + payload */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static protocol_packer_error_t tx_fill_len_cb(uint8_t* buffer,
    uint16_t payload_len);

static protocol_packer_error_t tx_checksum_cb(const uint8_t* data, uint16_t len,
    uint8_t* checksum_out, uint16_t* checksum_len);

/* Exported functions --------------------------------------------------------*/

void srv_uart_tx_cmd_init(void)
{
    const protocol_packer_config_t cfg = {
        .name = "srv_uart_tx_cmd",
        .header = s_header,
        .footer = s_footer,
        .output_buffer = s_pack_out,
        .checksum_cb = tx_checksum_cb,
        .fill_len_cb = tx_fill_len_cb,
        .header_len = 1,
        .footer_len = 1,
        .checksum_len = 1,
        .output_buffer_len = (uint16_t)sizeof(s_pack_out),
    };

    (void)protocol_packer_init(&s_packer, &cfg);
    s_initialized = true;

    SRV_UART_TX_CMD_LOG_I("UART 命令发送服务初始化完成");
}

srv_uart_tx_cmd_error_t srv_uart_tx_cmd_send(uint8_t cmd, const uint8_t* data,
    uint8_t data_len)
{
    if (!s_initialized) {
        return SRV_UART_TX_CMD_ERROR_UNINITIALIZED;
    }
    if (data_len > SRV_UART_TX_CMD_MAX_PAYLOAD) {
        return SRV_UART_TX_CMD_ERROR_INVALID_PARAM;
    }
    if (data_len > 0 && data == NULL) {
        return SRV_UART_TX_CMD_ERROR_NULL_PTR;
    }

    /* 构造 data 块：[cmd][data_len占位][payload]（由 fill_len_cb 回填 data_len） */
    s_cmd_buf[0] = cmd;
    s_cmd_buf[1] = 0;
    if (data_len > 0) {
        memcpy(&s_cmd_buf[2], data, data_len);
    }

    uint8_t* frame = NULL;
    uint16_t frame_len = 0;
    const protocol_packer_error_t err = protocol_packer_pack(&s_packer,
        s_cmd_buf, (uint16_t)(2U + data_len), &frame, &frame_len);

    if (err != PROTOCOL_PACKER_OK) {
        SRV_UART_TX_CMD_LOG_E("打包失败: %d (cmd=0x%02X, data_len=%u)",
            (int)err, (unsigned)cmd, (unsigned)data_len);
        return SRV_UART_TX_CMD_ERROR_INTERNAL;
    }

    const drv_uart_error_t uart_err = drv_uart_send(DRV_UART_CH_2, frame, frame_len);
    if (uart_err == DRV_UART_ERROR_TX_BUSY) {
        return SRV_UART_TX_CMD_ERROR_TX_BUSY;
    }
    if (uart_err != DRV_UART_OK) {
        SRV_UART_TX_CMD_LOG_E("UART 发送失败: %d", (int)uart_err);
        return SRV_UART_TX_CMD_ERROR_INTERNAL;
    }

    SRV_UART_TX_CMD_LOG_D("发送 cmd=0x%02X data_len=%u 帧长=%u",
        (unsigned)cmd, (unsigned)data_len, (unsigned)frame_len);

    return SRV_UART_TX_CMD_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 帧长度字段回填回调
 * @note  由 protocol_packer 在 data 拷贝后调用；payload_len 参数为传入 pack 的
 *        data 块长度（cmd + data_len占位 + payload = 2 + 实际payload）。
 *        data_len 字段位于 buffer[2]（header_len=1 + cmd 1 字节之后）。
 */
static protocol_packer_error_t tx_fill_len_cb(uint8_t* buffer,
    uint16_t payload_len)
{
    if (buffer == NULL) {
        return PROTOCOL_PACKER_ERROR_NULL_PTR;
    }
    if (payload_len < 2U) {
        return PROTOCOL_PACKER_ERROR_INVALID_PARAM;
    }

    buffer[2] = (uint8_t)(payload_len - 2U);
    return PROTOCOL_PACKER_OK;
}

/**
 * @brief CRC8 计算回调（覆盖 data 块全部字节：header + cmd + data_len + payload）
 */
static protocol_packer_error_t tx_checksum_cb(const uint8_t* data, uint16_t len,
    uint8_t* checksum_out, uint16_t* checksum_len)
{
    if (data == NULL || checksum_out == NULL || checksum_len == NULL) {
        return PROTOCOL_PACKER_ERROR_NULL_PTR;
    }

    checksum_out[0] = get_CRC8_check_sum((uint8_t*)data, len, 0xFF);
    *checksum_len = 1;
    return PROTOCOL_PACKER_OK;
}
