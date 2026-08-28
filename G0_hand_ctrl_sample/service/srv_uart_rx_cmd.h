/**
 * @file    srv_uart_rx_cmd.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   UART 命令接收服务 — protocol_parser 帧解析 + 回调上抛
 * @attention
 *
 * 协议（收发一致，变长负载）：
 *   [z][cmd][data_len][payload...][crc][\n]
 *   帧头 'z'(0x7A) 1B | cmd 1B | data_len 1B | payload data_len B |
 *   crc 1B (CRC8) | 帧尾 '\n'(0x0A) 1B
 *   帧总长 = data_len + 5
 *
 * 解析由 task 层周期调用 srv_uart_rx_cmd_step() 驱动（喂数据 + 解析 + 分派）。
 */

#ifndef SRV_UART_RX_CMD_H
#define SRV_UART_RX_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief 最大可变长负载长度（data_len 为 1 字节，上限 255） */
#define SRV_UART_RX_CMD_MAX_PAYLOAD (255U)

/** @brief 最大帧长度 = payload + 5（头1 + cmd1 + len1 + payload + crc1 + 尾1） */
#define SRV_UART_RX_CMD_MAX_FRAME (SRV_UART_RX_CMD_MAX_PAYLOAD + 5U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART 命令接收回调函数类型（主循环上下文执行）
 * @param cmd      命令字节
 * @param data     负载数据指针（仅回调期间有效）
 * @param data_len 负载长度
 */
typedef void (*srv_uart_rx_cmd_cb_t)(uint8_t cmd, const uint8_t* data,
    uint8_t data_len);

/**
 * @brief 接收服务错误码
 */
typedef enum {
    SRV_UART_RX_CMD_OK = 0, /**< 操作成功 */
    SRV_UART_RX_CMD_ERROR_NULL_PTR, /**< 空指针错误 */
    SRV_UART_RX_CMD_ERROR_UNINITIALIZED, /**< 未初始化 */
    SRV_UART_RX_CMD_ERROR_INTERNAL, /**< 内部错误 */
} srv_uart_rx_cmd_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 UART 命令接收服务
 * @param callback 收到完整命令帧后的回调（可为 NULL 表示只统计不处理）
 * @note  内部完成 protocol_parser 初始化，需在 drv_uart_init() 之后调用
 */
void srv_uart_rx_cmd_init(srv_uart_rx_cmd_cb_t callback);

/**
 * @brief 接收处理步进：从 drv_uart 读取数据 → 喂解析器 → 解析完整帧并回调
 * @note  需在 task 层 sw_timer 中周期调用（主循环上下文）
 */
void srv_uart_rx_cmd_step(void);

/**
 * @brief 空闲超时 tick：转发给 protocol_parser_tick
 * @note  与 step 同周期调用即可
 */
void srv_uart_rx_cmd_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* SRV_UART_RX_CMD_H */
