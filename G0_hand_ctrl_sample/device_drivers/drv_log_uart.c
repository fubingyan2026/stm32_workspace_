/**
 * @file    drv_log_uart.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   USART2 控制台串口设备驱动实现（DMA 输出 + DMA circular + IDLE → kfifo 接收）
 * @attention
 *
 * 参考 E1_Master_Power_Manage 工程的 drv_log_uart 设计：
 *   - RX：DMA circular 缓冲区与接收 kfifo 共用同一块 RAM，IDLE 中断里按
 *     DMA 计数器同步写指针（kfifo_move_in），消费方主循环轮询读取
 *   - TX：非阻塞 DMA 发送，TxCplt 回调清忙
 *
 * G0 仅一路 UART（USART2），采用全局 weak 回调（USE_HAL_UART_REGISTER_CALLBACKS=0）。
 * F1 无 UART_CLEAR_OREF/NEF/FEF/PEF 与 UART_RXDATA_FLUSH_REQUEST，用
 * __HAL_UART_CLEAR_OREFLAG/NEFLAG/FEFLAG/PEFLAG 清标志。
 *
 * 注意：本文件热路径（send/TxCplt/RxEvent/sync_rx_dma）禁止打印日志，
 * 防止日志回灌 USART2 TX 造成自引用。
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_log_uart.h"

#include "kfifo.h"
#include "log.h"
#include "usart.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_LOG_UART_LOG_ENABLE 0

#if DRV_LOG_UART_LOG_ENABLE
#define DRV_LOG_UART_LOG_E(...) LOG_E("drv_log_uart", __VA_ARGS__)
#define DRV_LOG_UART_LOG_W(...) LOG_W("drv_log_uart", __VA_ARGS__)
#define DRV_LOG_UART_LOG_I(...) LOG_I("drv_log_uart", __VA_ARGS__)
#else
#define DRV_LOG_UART_LOG_E(...) ((void)0)
#define DRV_LOG_UART_LOG_W(...) ((void)0)
#define DRV_LOG_UART_LOG_I(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief DMA circular 接收缓冲区大小（字节，必须为 2 的幂） */
#define DRV_LOG_UART_RX_CIRC_BUF_SIZE (256U)

/** @brief 日志串口 HAL 句柄（来自 CubeMX usart.c: USART2） */
#define LOG_HUART (&huart2)

/* Private variables ---------------------------------------------------------*/

/** @brief USART2 RX DMA circular 缓冲区（与接收 kfifo 共用） */
static uint8_t s_rx_dma_buf[DRV_LOG_UART_RX_CIRC_BUF_SIZE];

/** @brief 接收 kfifo（环形缓冲）实例 */
static kfifo_t s_rx_fifo;

/** @brief TX DMA 传输中标志 */
static volatile bool s_tx_busy;

/** @brief 初始化标志 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void drv_log_uart_sync_rx_dma(void);

static void drv_log_uart_restart_rx(void);

/* 注册到 HAL 的 per-instance 回调（热路径，禁止日志） */
static void drv_log_uart_tx_cplt_cb(UART_HandleTypeDef* huart);

static void drv_log_uart_rx_event_cb(UART_HandleTypeDef* huart, uint16_t size);

static void drv_log_uart_error_cb(UART_HandleTypeDef* huart);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 USART2 控制台驱动
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_init(void)
{
    if (s_initialized) {
        drv_log_uart_deinit();
    }

    s_tx_busy = false;
    s_initialized = false;

    /* 接收 kfifo 与 DMA 共享缓冲，kfifo_move_in 按 DMA 写指针推进 */
    kfifo_init(&s_rx_fifo, s_rx_dma_buf, sizeof(s_rx_dma_buf), NULL);

    /* CubeMX 生成的 USART2 RX DMA 已是 circular（DMA_CIRCULAR）；
     * 此处幂等检查，兼容从 normal 生成的项目。 */
    if (LOG_HUART->hdmarx != NULL && LOG_HUART->hdmarx->Init.Mode != DMA_CIRCULAR) {
        LOG_HUART->hdmarx->Init.Mode = DMA_CIRCULAR;
        LOG_HUART->hdmarx->Instance->CCR |= DMA_CCR_CIRC;
    }

    /* 注册 per-instance 回调（USE_HAL_UART_REGISTER_CALLBACKS=1 时 HAL IRQ
       按 huart->xxxCallback 指针分发，本驱动独占 huart2） */
    HAL_UART_RegisterCallback(LOG_HUART, HAL_UART_TX_COMPLETE_CB_ID,
        drv_log_uart_tx_cplt_cb);
    HAL_UART_RegisterCallback(LOG_HUART, HAL_UART_ERROR_CB_ID,
        drv_log_uart_error_cb);
    HAL_UART_RegisterRxEventCallback(LOG_HUART, drv_log_uart_rx_event_cb);

    if (HAL_UARTEx_ReceiveToIdle_DMA(LOG_HUART, s_rx_dma_buf,
            sizeof(s_rx_dma_buf)) != HAL_OK) {
        DRV_LOG_UART_LOG_E("日志串口 RX DMA 启动失败 (state=%d)",
            (int)LOG_HUART->gState);
        return DRV_LOG_UART_ERROR_UNINITIALIZED;
    }

    s_initialized = true;

    DRV_LOG_UART_LOG_I("初始化完成 (USART2 DMA circular + kfifo, RX缓冲=%u字节)",
        (unsigned)sizeof(s_rx_dma_buf));

    return DRV_LOG_UART_OK;
}

/**
 * @brief 反初始化 USART2 控制台驱动
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
 */
bool drv_log_uart_is_tx_busy(void)
{
    if (!s_initialized) {
        return false;
    }

    return s_tx_busy || LOG_HUART->gState != HAL_UART_STATE_READY;
}

/**
 * @brief 从接收 kfifo 读取数据
 */
uint32_t drv_log_uart_rx_read(uint8_t* buf, uint32_t max_len)
{
    if (!buf || max_len == 0) {
        return 0;
    }

    return kfifo_get(&s_rx_fifo, buf, max_len);
}

/**
 * @brief 查询接收 kfifo 中可读字节数
 */
uint32_t drv_log_uart_rx_available(void)
{
    return kfifo_len(&s_rx_fifo);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 将 USART2 RX circular DMA 写入位置同步到接收 kfifo
 * @note  IDLE 中断回调中调用，将 DMA 硬件写指针位置推进 kfifo->in，
 *        消费方在主循环读取 [out, in) 区间。
 */
static void drv_log_uart_sync_rx_dma(void)
{
    if (!s_initialized || LOG_HUART->hdmarx == NULL) {
        return;
    }

    const uint32_t remaining = __HAL_DMA_GET_COUNTER(LOG_HUART->hdmarx);
    const uint32_t dma_hw_index = sizeof(s_rx_dma_buf) - remaining;

    kfifo_move_in(&s_rx_fifo, dma_hw_index);
}

/**
 * @brief 清错误标志并重启 RX DMA（错误回调 / 初始化共用）
 * @note  F1 无 UART_CLEAR_OREF/NEF/FEF/PEF 与 RXDATA_FLUSH_REQUEST，
 *        用 __HAL_UART_CLEAR_*FLAG 清 SR 各错误位。
 */
static void drv_log_uart_restart_rx(void)
{
    __HAL_UART_CLEAR_OREFLAG(LOG_HUART);
    __HAL_UART_CLEAR_NEFLAG(LOG_HUART);
    __HAL_UART_CLEAR_FEFLAG(LOG_HUART);
    __HAL_UART_CLEAR_PEFLAG(LOG_HUART);

    HAL_UARTEx_ReceiveToIdle_DMA(LOG_HUART, s_rx_dma_buf, sizeof(s_rx_dma_buf));
}

/* ===== 注册到 HAL 的 per-instance 回调（USE_HAL_UART_REGISTER_CALLBACKS=1） ===== */

/**
 * @brief UART TX DMA 完成回调（per-instance）
 */
static void drv_log_uart_tx_cplt_cb(UART_HandleTypeDef* huart)
{
    if (huart == LOG_HUART) {
        s_tx_busy = false;
    }
}

/**
 * @brief UART RX IDLE 事件回调（per-instance）— 同步 DMA 写指针到 kfifo
 */
static void drv_log_uart_rx_event_cb(UART_HandleTypeDef* huart, uint16_t size)
{
    (void)size;

    if (huart == LOG_HUART) {
        drv_log_uart_sync_rx_dma();
    }
}

/**
 * @brief UART 错误回调（per-instance）— 丢弃缓冲数据并重启接收
 */
static void drv_log_uart_error_cb(UART_HandleTypeDef* huart)
{
    if (huart != LOG_HUART) {
        return;
    }

    DRV_LOG_UART_LOG_W("RX 错误 0x%08lX，丢弃缓冲并重启接收",
        (unsigned long)HAL_UART_GetError(huart));

    kfifo_reset(&s_rx_fifo);
    drv_log_uart_restart_rx();
}
