/**
 * @file    srv_uart_tx_cmd.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   UART 命令发送服务 — protocol_packer 帧打包 + drv_uart 发送
 * @attention
 *
 * 协议（收发一致，变长负载）：
 *   [z][cmd][data_len][payload...][crc][\n]
 *   帧头 'z'(0x7A) 1B | cmd 1B | data_len 1B | payload data_len B |
 *   crc 1B (CRC8) | 帧尾 '\n'(0x0A) 1B
 *   帧总长 = data_len + 5
 */

#ifndef SRV_UART_TX_CMD_H
#define SRV_UART_TX_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief 最大可变长负载长度（data_len 为 1 字节，上限 255） */
#define SRV_UART_TX_CMD_MAX_PAYLOAD (64U)

/** @brief 最大帧长度 = payload + 5（头1 + cmd1 + len1 + payload + crc1 + 尾1） */
#define SRV_UART_TX_CMD_MAX_FRAME (SRV_UART_TX_CMD_MAX_PAYLOAD + 5U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 发送错误码
 */
typedef enum {
    SRV_UART_TX_CMD_OK = 0, /**< 操作成功 */
    SRV_UART_TX_CMD_ERROR_NULL_PTR, /**< 空指针错误 */
    SRV_UART_TX_CMD_ERROR_UNINITIALIZED, /**< 未初始化 */
    SRV_UART_TX_CMD_ERROR_INVALID_PARAM, /**< 无效参数（负载超长等） */
    SRV_UART_TX_CMD_ERROR_TX_BUSY, /**< TX DMA 忙 */
    SRV_UART_TX_CMD_ERROR_QUEUE_FULL, /**< TX 队列满（帧被丢弃） */
    SRV_UART_TX_CMD_ERROR_INTERNAL, /**< 内部错误（打包失败等） */
} srv_uart_tx_cmd_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 获取发送错误码的描述字符串
 * @param err 错误码
 * @return 描述字符串（如 "OK" / "QUEUE_FULL"）
 */
const char* srv_uart_tx_cmd_err_str(srv_uart_tx_cmd_error_t err);

/**
 * @brief 初始化 UART 命令发送服务
 * @note  内部完成 protocol_packer 初始化，需在 drv_uart_init() 之后调用
 */
void srv_uart_tx_cmd_init(void);

/**
 * @brief 打包并发送一条 UART 命令帧（非阻塞）
 * @param cmd      命令字节
 * @param data     负载数据（可为 NULL，当 data_len=0）
 * @param data_len 负载长度（0~255）
 * @return 操作结果错误码
 */
srv_uart_tx_cmd_error_t srv_uart_tx_cmd_send(uint8_t cmd, const uint8_t* data,
    uint8_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* SRV_UART_TX_CMD_H */
