/**
 * @file    drv_uart.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   通用串口设备驱动（USART1，DMA 输出 + DMA circular + IDLE 中断接收）
 * @attention
 *
 * 多实例驱动：内部句柄表，drv_uart_init() 无需传参。
 * TX：DMA normal，gState + s_tx_busy 双状态检忙。
 * RX：DMA circular + IDLE 事件 (ReceiveToIdle) 接收，
 *     总线空闲即按实际长度上抛（天然帧分块，错位自愈）。
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

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART 通道枚举
 * @note  DRV_UART_CH_2 (USART2) 已由独立 drv_log_uart 控制台驱动接管，
 *        本驱动不再初始化/管理该通道（s_inst 中 CH_2 的 huart 置空）。
 */
typedef enum {
    DRV_UART_CH_1 = 0, /**< USART1 — PA9(TX) / PA10(RX) */
    DRV_UART_CH_2, /**< USART2 — PA2(TX) / PA3(RX)（由 drv_log_uart 接管） */
    DRV_UART_CH_NUM, /**< 通道总数 */
} drv_uart_channel_t;

/**
 * @brief UART 接收回调函数类型（中断上下文执行）
 * @param ch     通道号
 * @param data   数据指针
 * @param len    数据长度
 */
typedef void (*drv_uart_rx_callback_t)(drv_uart_channel_t ch, const uint8_t* data, uint32_t len);

/**
 * @brief UART 驱动错误码
 */
typedef enum {
    DRV_UART_OK = 0, /**< 操作成功 */
    DRV_UART_ERROR_NULL_PTR, /**< 空指针错误 */
    DRV_UART_ERROR_UNINITIALIZED, /**< 未初始化 */
    DRV_UART_ERROR_TX_BUSY, /**< TX DMA 忙 */
    DRV_UART_ERROR_INVALID_PARAM, /**< 无效参数 */
} drv_uart_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化所有 UART 通道（USART1） */
drv_uart_error_t drv_uart_init(void);

/** @brief 反初始化所有 UART 通道 */
void drv_uart_deinit_all(void);

/** @brief 检查指定通道是否已初始化 */
bool drv_uart_is_initialized(drv_uart_channel_t ch);

/* --- TX --- */

/**
 * @brief 非阻塞 DMA 发送
 * @param ch   通道号
 * @param data 数据指针
 * @param len  数据长度
 * @return 操作结果错误码
 */
drv_uart_error_t drv_uart_send(drv_uart_channel_t ch, const uint8_t* data, uint32_t len);

/** @brief 查询 TX DMA 是否忙碌 */
bool drv_uart_is_tx_busy(drv_uart_channel_t ch);

/* --- RX --- */

/**
 * @brief 注册接收回调（每通道独立注册）
 * @param ch       通道号
 * @param callback 回调函数（NULL=取消）
 * @note  回调在 IDLE 中断上下文中执行，应尽量简短
 */
void drv_uart_register_rx_callback(drv_uart_channel_t ch,
    drv_uart_rx_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_H */
