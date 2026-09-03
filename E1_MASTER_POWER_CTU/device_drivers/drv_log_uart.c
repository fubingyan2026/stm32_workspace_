/**
 * @file    drv_log_uart.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   USART1 设备驱动实现（TX DMA 日志输出，E1_MASTER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_log_uart.h"

#include "log.h"
#include "usart.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_LOG_UART_LOG_ENABLE 1

#if DRV_LOG_UART_LOG_ENABLE
#define DRV_LOG_UART_LOG_E(...) LOG_E("drv_log_uart", __VA_ARGS__)
#define DRV_LOG_UART_LOG_W(...) LOG_W("drv_log_uart", __VA_ARGS__)
#define DRV_LOG_UART_LOG_I(...) LOG_I("drv_log_uart", __VA_ARGS__)
#define DRV_LOG_UART_LOG_D(...) LOG_D("drv_log_uart", __VA_ARGS__)
#else
#define DRV_LOG_UART_LOG_E(...) ((void)0)
#define DRV_LOG_UART_LOG_W(...) ((void)0)
#define DRV_LOG_UART_LOG_I(...) ((void)0)
#define DRV_LOG_UART_LOG_D(...) ((void)0)
#endif

/* 注：本文件是日志输出后端本体。仅允许在 drv_log_uart_init()/deinit() 打印；
 * send/TxCplt 为 TX 热路径，禁止任何日志（自引用回灌）。 */

/* Private constants ---------------------------------------------------------*/

/** @brief 日志串口 HAL 句柄（来自 CubeMX usart.c: USART1） */
#define LOG_HUART (&huart1)

/* Private variables ---------------------------------------------------------*/

/** @brief TX DMA 传输中标志 */
static volatile bool s_tx_busy;

/** @brief 初始化标志 */
static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化日志串口驱动（内部状态，直接使用 CubeMX 的 huart1）
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_init(void)
{
    if (s_initialized) {
        drv_log_uart_deinit();
    }

    s_tx_busy = false;
    s_initialized = true;

    DRV_LOG_UART_LOG_I("日志串口初始化完成 (USART1 TX DMA)");

    return DRV_LOG_UART_OK;
}

/**
 * @brief 反初始化日志串口驱动
 */
void drv_log_uart_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    HAL_UART_DMAStop(LOG_HUART);

    s_tx_busy = false;
    s_initialized = false;

    DRV_LOG_UART_LOG_I("日志串口反初始化完成");
}

/**
 * @brief 检查驱动是否已初始化
 * @return true表示已初始化，false表示未初始化
 */
bool drv_log_uart_is_initialized(void)
{
    return s_initialized;
}

/**
 * @brief 非阻塞 DMA 发送
 * @param data 数据指针
 * @param len  数据长度
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_send(const uint8_t* data, uint32_t len)
{
    if (!data) {
        return DRV_LOG_UART_ERROR_NULL_PTR;
    }

    if (!s_initialized) {
        return DRV_LOG_UART_ERROR_UNINITIALIZED;
    }

    if (len == 0 || len > UINT16_MAX) {
        return DRV_LOG_UART_OK;
    }

    if (s_tx_busy || LOG_HUART->gState != HAL_UART_STATE_READY) {
        return DRV_LOG_UART_ERROR_TX_BUSY;
    }

    if (HAL_UART_Transmit_DMA(LOG_HUART, (uint8_t*)data, (uint16_t)len) != HAL_OK) {
        return DRV_LOG_UART_ERROR_TX_BUSY;
    }

    s_tx_busy = true;

    return DRV_LOG_UART_OK;
}

/**
 * @brief 查询 TX DMA 是否忙碌
 * @return true表示正在发送
 */
bool drv_log_uart_is_tx_busy(void)
{
    if (!s_initialized) {
        return false;
    }

    return s_tx_busy || LOG_HUART->gState != HAL_UART_STATE_READY;
}

/**
 * @brief UART TX DMA 完成回调（全局弱回调，本工程 USART1 专用）
 * @note  USART3(RS485) 的 TX 完成由 drv_uart per-instance 回调接管，不经过此处
 * @param huart UART 句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == LOG_HUART) {
        s_tx_busy = false;
    }
}
