/**
 * @file    drv_log_uart.c
 * @author  maximillian
 * @version V1.2.0
 * @date    2026-07-1
 * @brief   USART1 设备驱动实现（DMA 输出 + DMA circular + IDLE 中断接收）
 * @attention
 *
 * Copyright (c) 2026 E1_PRO 项目组
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_log_uart.h"

#include "kfifo.h"
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
 *send/TxCplt/RxEvent/sync_rx_dma 为 TX/RX 热路径，禁止任何日志（自引用回灌）。 */

/* Private constants ---------------------------------------------------------*/

/** @brief DMA circular 接收缓冲区大小（字节，必须为2的幂） */
#define DRV_LOG_UART_RX_CIRC_BUF_SIZE (1024U)

/** @brief 日志串口 HAL 句柄（来自 CubeMX usart.c: USART1） */
#define LOG_HUART (&huart1)

/* Private variables ---------------------------------------------------------*/

/** @brief USART1 RX DMA circular 缓冲区 */
static uint8_t s_rx_dma_buf[DRV_LOG_UART_RX_CIRC_BUF_SIZE];

/** @brief 接收 FIFO 实例 */
static kfifo_t s_rx_fifo;

/** @brief TX DMA 传输中标志 */
static volatile bool s_tx_busy;

/** @brief 初始化标志 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void drv_log_uart_sync_rx_dma(void);

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
    s_initialized = false;

    kfifo_init(&s_rx_fifo, s_rx_dma_buf, sizeof(s_rx_dma_buf), NULL);

    if (HAL_UARTEx_ReceiveToIdle_DMA(LOG_HUART, s_rx_dma_buf, sizeof(s_rx_dma_buf)) != HAL_OK) {
        DRV_LOG_UART_LOG_E("日志串口 RX DMA 启动失败 (state=%d)", (int)LOG_HUART->gState);
        return DRV_LOG_UART_ERROR_UNINITIALIZED;
    }

    s_initialized = true;

    DRV_LOG_UART_LOG_I("日志串口初始化完成 (USART1 DMA, RX缓冲区=%u字节)",
        (unsigned)sizeof(s_rx_dma_buf));

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

    __HAL_UART_DISABLE_IT(LOG_HUART, UART_IT_IDLE);
    HAL_UART_DMAStop(LOG_HUART);
    kfifo_reset(&s_rx_fifo);

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
 * @brief 从接收 FIFO 读取数据
 * @param buf     目标缓冲区
 * @param max_len 最大读取字节数
 * @return 实际读取的字节数
 */
uint32_t drv_log_uart_rx_read(uint8_t* buf, uint32_t max_len)
{
    if (!buf || max_len == 0) {
        return 0;
    }

    return kfifo_get(&s_rx_fifo, buf, max_len);
}

/**
 * @brief 查询接收 FIFO 中可读字节数
 * @return 可读字节数
 */
uint32_t drv_log_uart_rx_available(void)
{
    return kfifo_len(&s_rx_fifo);
}

/**
 * @brief UART TX DMA 完成回调
 * @param huart UART 句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == LOG_HUART) {
        s_tx_busy = false;
    }
}

/**
 * @brief UART RX IDLE 事件回调（由 HAL_UART_IRQHandler 内部触发）
 * @param huart UART 句柄
 * @param Size  本次接收的字节数（未使用，实际同步由 kfifo_move_in 按 DMA 位置计算）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
    (void)Size;

    if (huart == LOG_HUART) {
        drv_log_uart_sync_rx_dma();
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 将 USART1 RX circular DMA 写入位置同步到接收 FIFO
 * @note  由 IDLE 中断回调 HAL_UARTEx_RxEventCallback 驱动，
 *        将 DMA 硬件写指针位置同步到 kfifo，避免逐字节拷贝
 */
static void drv_log_uart_sync_rx_dma(void)
{
    if (!s_initialized || !LOG_HUART->hdmarx) {
        return;
    }

    const uint32_t remaining = __HAL_DMA_GET_COUNTER(LOG_HUART->hdmarx);
    const uint32_t dma_hw_index = sizeof(s_rx_dma_buf) - remaining;

    kfifo_move_in(&s_rx_fifo, dma_hw_index);
}
