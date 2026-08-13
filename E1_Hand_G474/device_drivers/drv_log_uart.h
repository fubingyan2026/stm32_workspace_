/**
 * @file    drv_log_uart.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   USART1 控制台串口设备驱动（DMA 输出 + DMA circular + IDLE 中断 → kfifo 接收）
 * @attention
 *
 * 参考 E1_Master_Power_Manage 工程的 drv_log_uart 设计：
 *   - USART1 从通用 drv_uart 中剥离，独立管理（日志输出 TX + 控制台命令 RX）
 *   - RX：DMA circular + IDLE 中断，按 DMA 计数器同步进 kfifo（环形缓冲），
 *     消费方在主循环通过 drv_log_uart_rx_read() 轮询读取
 *   - TX：非阻塞 DMA 发送
 *
 * 依赖：USART1 RX DMA 需为 circular 模式（CubeMX 生成的是 normal，
 *       在 drv_log_uart_init() 中运行期切换，见实现注释）。
 */

#ifndef __DRV_LOG_UART_H
#define __DRV_LOG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief USART1 控制台驱动错误码枚举
 */
typedef enum {
    DRV_LOG_UART_OK = 0, /**< 操作成功 */
    DRV_LOG_UART_ERROR_NULL_PTR, /**< 空指针错误 */
    DRV_LOG_UART_ERROR_UNINITIALIZED, /**< 未初始化 */
    DRV_LOG_UART_ERROR_TX_BUSY, /**< TX DMA 忙 */
} drv_log_uart_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/**
 * @brief 初始化 USART1 控制台驱动（内部状态，直接使用 CubeMX 的 huart1）
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_init(void);

/** @brief 反初始化 USART1 控制台驱动 */
void drv_log_uart_deinit(void);

/** @brief 检查驱动是否已初始化 */
bool drv_log_uart_is_initialized(void);

/* --- TX（日志输出） --- */

/**
 * @brief 非阻塞 DMA 发送
 * @param data 数据指针
 * @param len  数据长度
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_send(const uint8_t* data, uint32_t len);

/** @brief 查询 TX DMA 是否忙碌 */
bool drv_log_uart_is_tx_busy(void);

/* --- RX（DMA circular + IDLE 中断 → kfifo 环形缓冲） --- */

/**
 * @brief 从接收 kfifo 读取数据
 * @param buf     目标缓冲区
 * @param max_len 最大读取字节数
 * @return 实际读取的字节数
 */
uint32_t drv_log_uart_rx_read(uint8_t* buf, uint32_t max_len);

/** @brief 查询接收 kfifo 中可读字节数 */
uint32_t drv_log_uart_rx_available(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LOG_UART_H */
