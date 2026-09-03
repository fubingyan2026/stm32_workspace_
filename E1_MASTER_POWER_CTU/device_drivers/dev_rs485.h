/**
 * @file    dev_rs485.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-03
 * @brief   RS485 半双工驱动（方向控制 DE/RE 封装，底层收发由通用 drv_uart 承担）
 * @attention
 *
 * 主机查询应答式 485 从站：本板仅应答主机查询/控制。
 *   - 底层串口收发（USART3 DMA circular + IDLE → kfifo、TX 帧队列）由 drv_uart 实现
 *   - 本驱动负责 RS485_EN(PC5, DE/!RE) 半双工方向控制：
 *       · 发送前拉高 EN，TX DMA 完成（队列排空、TC 置位）后拉低 EN
 *       · 其余时间保持接收方向
 *
 * 注意：TX 完成 hook 由本驱动注册到 drv_uart（ISR 上下文调用）。
 */

#ifndef __DEV_RS485_H
#define __DEV_RS485_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief RS485 驱动错误码枚举
 */
typedef enum {
    DEV_RS485_OK = 0,                /**< 操作成功 */
    DEV_RS485_ERROR_NULL_PTR,        /**< 空指针错误 */
    DEV_RS485_ERROR_UNINITIALIZED,   /**< 未初始化 */
    DEV_RS485_ERROR_TX_BUSY,         /**< TX 队列满/发送忙（帧被丢弃） */
} dev_rs485_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化 RS485（底层 drv_uart + 方向控制，默认接收方向） */
dev_rs485_error_t dev_rs485_init(void);
void dev_rs485_deinit(void);
bool dev_rs485_is_initialized(void);

/* --- TX（半双工，自动控制 DE/RE 方向） --- */

/**
 * @brief 非阻塞发送一帧（底层经 drv_uart 帧队列，忙时不阻塞）
 * @param data 数据指针
 * @param len  数据长度（≤ DRV_UART_TX_MAX_FRAME_LEN）
 * @return 操作结果错误码
 */
dev_rs485_error_t dev_rs485_send(const uint8_t* data, uint32_t len);

bool dev_rs485_is_tx_busy(void);

/**
 * @brief 排空底层 TX 队列（task 层周期调用；发送时保持 EN 拉高）
 * @note  在 com_task 的 sw_timer 回调中每周期调用
 */
void dev_rs485_tx_flush(void);

/* --- RX（底层 kfifo，字节流读取） --- */

/**
 * @brief 读取自上次调用以来新收到的字节
 * @param buf     目标缓冲区
 * @param max_len 最大读取字节数
 * @return 实际读取的字节数
 */
uint32_t dev_rs485_rx_read(uint8_t* buf, uint32_t max_len);

/** @brief 查询当前可读字节数 */
uint32_t dev_rs485_rx_available(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_RS485_H */
