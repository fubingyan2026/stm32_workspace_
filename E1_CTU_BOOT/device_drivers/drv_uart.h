/**
 * @file    drv_uart.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   通用串口设备驱动（DMA 输出 + DMA circular + IDLE 中断 → kfifo 环形缓冲）
 * @attention
 *
 * 多实例驱动：内部句柄表，drv_uart_init() 无需传参。
 * TX：DMA normal，gState + s_tx_busy 双状态检忙（忙时经 msg_fifo 帧队列入队，不丢帧）。
 * RX：DMA circular + IDLE 事件 (ReceiveToIdle)，IDLE 中断只做 kfifo_move_in 推进读指针，
 *     消费方在主循环通过 drv_uart_rx_read()/drv_uart_rx_available() 轮询读取。
 *
 * HAL 回调按 per-instance 注册（USE_HAL_UART_REGISTER_CALLBACKS=1，见 stm32f1xx_hal_conf.h）。
 * 当前仅登记 USART3（RS485 通道）；USART1 由 drv_log_uart 独立接管，互不冲突。
 *
 * 参考：G0_hand_ctrl_sample/device_drivers/drv_uart.c（F103 同族 HAL）。
 */

#ifndef __DRV_UART_H
#define __DRV_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief TX 单帧最大长度（容纳 RS485 协议应答帧 + 余量） */
#define DRV_UART_TX_MAX_FRAME_LEN (70U)

/** @brief TX 缓冲队列深度（帧数），队列满才丢帧 */
#define DRV_UART_TX_QUEUE_DEPTH (8U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART 通道枚举
 */
typedef enum {
    DRV_UART_CH_RS485 = 0, /**< USART3 — PB10(TX) / PB11(RX)，RS485 总线 */
    DRV_UART_CH_NUM, /**< 通道总数 */
} drv_uart_channel_t;

/**
 * @brief UART 驱动错误码
 */
typedef enum {
    DRV_UART_OK = 0, /**< 操作成功 */
    DRV_UART_ERROR_NULL_PTR, /**< 空指针错误 */
    DRV_UART_ERROR_UNINITIALIZED, /**< 未初始化 */
    DRV_UART_ERROR_TX_BUSY, /**< TX 队列满（帧被丢弃） */
    DRV_UART_ERROR_TX_QUEUE_FULL, /**< TX 缓冲队列已满（帧被丢弃） */
    DRV_UART_ERROR_INVALID_PARAM, /**< 无效参数 */
} drv_uart_error_t;

/** @brief TX DMA 完成回调 hook（ISR 上下文；RS485 驱动用于释放总线方向 DE） */
typedef void (*drv_uart_tx_cplt_hook_t)(drv_uart_channel_t ch);

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化所有 UART 通道（当前为 USART3/RS485） */
drv_uart_error_t drv_uart_init(void);

/** @brief 反初始化所有 UART 通道 */
void drv_uart_deinit_all(void);

/** @brief 检查指定通道是否已初始化 */
bool drv_uart_is_initialized(drv_uart_channel_t ch);

/**
 * @brief 串口错误恢复（主循环周期调用，处理错误回调未能恢复的残留状态）
 * @param ch 通道号
 * @return DRV_UART_OK 恢复成功/无需恢复；DRV_UART_ERROR_* 参数非法或未初始化
 */
drv_uart_error_t drv_uart_recover(drv_uart_channel_t ch);

/* --- TX（经 msg_fifo 发送缓冲队列，忙时入队不丢帧） --- */

/**
 * @brief 发送数据（非阻塞，经 TX 缓冲队列）
 * @param ch   通道号
 * @param data 数据指针
 * @param len  数据长度（≤ DRV_UART_TX_MAX_FRAME_LEN）
 * @return DRV_UART_ERROR_TX_BUSY 表示 TX 队列满
 * @note   TX 空闲时立即 DMA 发送；忙时入队，由 drv_uart_tx_flush() 排空。
 */
drv_uart_error_t drv_uart_send(drv_uart_channel_t ch, const uint8_t* data, uint32_t len);

/** @brief 查询 TX DMA 是否忙碌 */
bool drv_uart_is_tx_busy(drv_uart_channel_t ch);

/**
 * @brief 排空 TX 缓冲队列到 DMA（主循环/任务周期调用）
 */
void drv_uart_tx_flush(drv_uart_channel_t ch);

/** @brief 查询 TX 缓冲队列中待发送的帧数 */
uint32_t drv_uart_tx_pending(drv_uart_channel_t ch);

/** @brief 注册 TX DMA 完成 hook（ISR 上下文回调，用于 RS485 释放发送方向） */
drv_uart_error_t drv_uart_register_tx_cplt_hook(drv_uart_channel_t ch,
    drv_uart_tx_cplt_hook_t hook);

/**
 * @brief 等待 USART TX 移位寄存器清空（TC 置位，有界自旋）
 * @param ch 通道号
 * @return true=TC 已置位（最后字节已完全发出）；false=达内部保护次数仍未置位
 * @note  半双工（RS485）释放总线方向前调用；ISR/主循环上下文均可，等待为微秒级
 */
bool drv_uart_tx_shift_wait(drv_uart_channel_t ch);

/* --- RX（DMA circular + IDLE 中断 → kfifo 环形缓冲） --- */

/**
 * @brief 从接收 kfifo 读取数据
 * @param ch      通道号
 * @param buf     目标缓冲区
 * @param max_len 最大读取字节数
 * @return 实际读取的字节数
 */
uint32_t drv_uart_rx_read(drv_uart_channel_t ch, uint8_t* buf, uint32_t max_len);

/** @brief 查询接收 kfifo 中可读字节数 */
uint32_t drv_uart_rx_available(drv_uart_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_H */
