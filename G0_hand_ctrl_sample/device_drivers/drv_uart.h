/**
 * @file    drv_uart.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-12
 * @brief   通用串口设备驱动（DMA 输出 + DMA circular + IDLE 中断 → kfifo 环形缓冲）
 * @attention
 *
 * 多实例驱动：内部句柄表，drv_uart_init() 无需传参。
 * TX：DMA normal，gState + s_tx_busy 双状态检忙。
 * RX：DMA circular + IDLE 事件 (ReceiveToIdle) 接收，IDLE 中断只做
 *     kfifo_move_in 推进读指针（与 drv_log_uart 同构），消费方在主循环
 *     通过 drv_uart_rx_read()/drv_uart_rx_available() 轮询读取。
 *
 * HAL 回调 (HAL_UART_TxCpltCallback / HAL_UARTEx_RxEventCallback /
 * HAL_UART_ErrorCallback) 集中在此驱动中（per-instance 注册，
 * 需要 USE_HAL_UART_REGISTER_CALLBACKS=1）。
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

/** @brief TX 单帧最大长度（容纳 UART 命令协议最大帧 269B + 余量） */
#define DRV_UART_TX_MAX_FRAME_LEN (70U)

/** @brief TX 缓冲队列深度（帧数），队列满才丢帧（高频反馈场景需足够大） */
#define DRV_UART_TX_QUEUE_DEPTH (16U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART 通道枚举
 * @note  DRV_UART_CH_1 (USART1) 已由独立 drv_log_uart 控制台驱动接管，
 *        本驱动不再初始化/管理该通道（s_inst 中 CH_1 的 huart 置空）。
 *        DRV_UART_CH_2 (USART2) 为本驱动管理的通用串口。
 */
typedef enum {
    DRV_UART_CH_1 = 0, /**< USART1 — PA9(TX) / PA10(RX)（由 drv_log_uart 接管） */
    DRV_UART_CH_2, /**< USART2 — PA2(TX) / PA3(RX) */
    DRV_UART_CH_NUM, /**< 通道总数 */
} drv_uart_channel_t;

/**
 * @brief UART 驱动错误码
 */
typedef enum {
    DRV_UART_OK = 0, /**< 操作成功 */
    DRV_UART_ERROR_NULL_PTR, /**< 空指针错误 */
    DRV_UART_ERROR_UNINITIALIZED, /**< 未初始化 */
    DRV_UART_ERROR_TX_BUSY, /**< TX DMA 忙（正在发送，应稍后重试） */
    DRV_UART_ERROR_TX_QUEUE_FULL, /**< TX 缓冲队列已满（帧被丢弃） */
    DRV_UART_ERROR_INVALID_PARAM, /**< 无效参数 */
} drv_uart_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化所有 UART 通道（USART2） */
drv_uart_error_t drv_uart_init(void);

/** @brief 反初始化所有 UART 通道 */
void drv_uart_deinit_all(void);

/** @brief 检查指定通道是否已初始化 */
bool drv_uart_is_initialized(drv_uart_channel_t ch);

/**
 * @brief 串口错误恢复（主循环周期调用，处理错误回调未能恢复的残留状态）
 * @param ch 通道号
 * @return DRV_UART_OK 恢复成功/无需恢复；DRV_UART_ERROR_* 参数非法或未初始化
 * @note  幂等：清除 UART 错误标志与 HAL RX 状态 → 丢弃接收缓冲 →
 *        重启 RX DMA（含重使能 IDLE 中断）→ 释放卡死的 TX 忙标志。
 *        建议在 task 层 sw_timer 中周期调用，成本极低（状态正常时直接返回）。
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
 *        队列满才返回 TX_BUSY，正常流量下数据不丢失。
 */
drv_uart_error_t drv_uart_send(drv_uart_channel_t ch, const uint8_t* data, uint32_t len);

/** @brief 查询 TX DMA 是否忙碌 */
bool drv_uart_is_tx_busy(drv_uart_channel_t ch);

/**
 * @brief 排空 TX 缓冲队列到 DMA（主循环/任务周期调用）
 * @note  在 uart_cmd_task 的 sw_timer 回调中周期调用
 */
void drv_uart_tx_flush(drv_uart_channel_t ch);

/** @brief 查询 TX 缓冲队列中待发送的帧数 */
uint32_t drv_uart_tx_pending(drv_uart_channel_t ch);

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
