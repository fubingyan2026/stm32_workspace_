/**
 * @file    drv_uart.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-12
 * @brief   通用串口设备驱动实现（DMA 输出 + DMA circular + IDLE → kfifo 环形缓冲）
 * @attention
 *
 * 参考 E1_Hand_G474 的 drv_uart 设计（per-instance HAL 回调）：
 *   - TX：DMA normal，gState + s_tx_busy 双状态检忙
 *   - RX：DMA circular + IDLE 事件 (ReceiveToIdle)，与 drv_log_uart 同构：
 *     IDLE 中断只做 kfifo_move_in 推进读指针（不重启、不上抛），
 *     消费方主循环轮询 drv_uart_rx_read/available
 *   - HAL 回调通过 HAL_UART_RegisterCallback/RegisterRxEventCallback 注册
 *     （USE_HAL_UART_REGISTER_CALLBACKS=1），与 drv_log_uart 互不冲突
 *
 * F1 差异：无 UART_CLEAR_OREF/NEF/FEF/PEF 与 UART_RXDATA_FLUSH_REQUEST，
 * 错误清理改用 __HAL_UART_CLEAR_*FLAG。
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

#include "kfifo.h"
#include "log.h"
#include "main.h"
#include "msg_fifo.h"
#include "usart.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define UART_LOG_ENABLE 1

#if UART_LOG_ENABLE
#define UART_LOG_E(...) LOG_E("drv_uart", __VA_ARGS__)
#define UART_LOG_W(...) LOG_W("drv_uart", __VA_ARGS__)
#define UART_LOG_I(...) LOG_I("drv_uart", __VA_ARGS__)
#define UART_LOG_D(...) LOG_D("drv_uart", __VA_ARGS__)
#else
#define UART_LOG_E(...) ((void)0)
#define UART_LOG_W(...) ((void)0)
#define UART_LOG_I(...) ((void)0)
#define UART_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief RX DMA circular 缓冲区大小（字节，必须为 2 的幂，与接收 kfifo 共用） */
#define DRV_UART_RX_BUF_SIZE (256U)

/** @brief 错误日志聚合窗口 (ms)：窗口内的错误只在首次打印，附累计次数 */
#define DRV_UART_ERR_LOG_PERIOD_MS (1000U)

/* Private types -------------------------------------------------------------*/

/** @brief TX 队列元素：单帧数据（长度 + 负载） */
typedef struct {
    uint16_t len;
    uint8_t data[DRV_UART_TX_MAX_FRAME_LEN];
} drv_uart_tx_frame_t;

typedef struct {
    UART_HandleTypeDef* huart; /**< HAL UART 句柄 */
    uint8_t rx_buf[DRV_UART_RX_BUF_SIZE]; /**< RX DMA circular 缓冲（与 kfifo 共用） */
    uint8_t tx_dma_buf[DRV_UART_TX_MAX_FRAME_LEN]; /**< TX DMA 缓冲（持久，防 DMA 读被覆盖） */
    kfifo_t rx_fifo; /**< 接收 kfifo（环形缓冲，SPSC：ISR 写指针/主循环读） */
    msg_fifo_t tx_fifo; /**< 发送缓冲队列（帧级，忙时入队不丢帧） */
    uint8_t tx_fifo_buf[DRV_UART_TX_QUEUE_DEPTH * sizeof(drv_uart_tx_frame_t)];
    bool tx_busy; /**< TX DMA 传输中 */
    bool initialized; /**< 初始化标志 */
    uint32_t err_count; /**< 日志聚合窗口内错误累计次数 */
    uint32_t err_flags; /**< 日志聚合窗口内错误标志并集 */
    uint32_t err_last_log; /**< 上次错误日志时间戳 (ms) */
} drv_uart_inst_t;

/* Private variables ---------------------------------------------------------*/

static drv_uart_inst_t s_inst[DRV_UART_CH_NUM] = {
    /* CH_1 (USART1) 已由独立 drv_log_uart 控制台驱动接管，此处置空跳过 */
    [DRV_UART_CH_1] = { .huart = NULL },
    /* CH_2 (USART2)：通用串口 */
    [DRV_UART_CH_2] = { .huart = &huart2 },
};

/* Private functions prototypes ----------------------------------------------*/

/**
 * @brief 启动/重启 RX DMA 到缓冲（ISR 与主循环共用）
 * @note  DMA circular：IDLE 事件后不停止，硬件继续接收，只需同步 kfifo 读指针。
 * @return true=启动成功；false=HAL 句柄忙等，需主循环兜底重试
 */
static bool drv_uart_rx_start(drv_uart_inst_t* inst)
{
    __HAL_UART_CLEAR_OREFLAG(inst->huart);
    __HAL_UART_CLEAR_NEFLAG(inst->huart);
    __HAL_UART_CLEAR_FEFLAG(inst->huart);
    __HAL_UART_CLEAR_PEFLAG(inst->huart);

    if (HAL_UARTEx_ReceiveToIdle_DMA(inst->huart,
            inst->rx_buf, DRV_UART_RX_BUF_SIZE)
        != HAL_OK) {
        return false;
    }
    return true;
}

/**
 * @brief 将 RX circular DMA 写指针同步到接收 kfifo（IDLE 中断回调中调用）
 */
static void drv_uart_sync_rx_dma(drv_uart_inst_t* inst)
{
    if (inst->huart == NULL || inst->huart->hdmarx == NULL) {
        return;
    }

    const uint32_t remaining = __HAL_DMA_GET_COUNTER(inst->huart->hdmarx);
    const uint32_t dma_hw_index = DRV_UART_RX_BUF_SIZE - remaining;

    kfifo_move_in(&inst->rx_fifo, dma_hw_index);
}

/* 注册到 HAL 的 per-instance 回调 */
static void drv_uart_tx_cplt_cb(UART_HandleTypeDef* huart);

static void drv_uart_rx_event_cb(UART_HandleTypeDef* huart, uint16_t size);

static void drv_uart_error_cb(UART_HandleTypeDef* huart);

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_uart_error_t drv_uart_init(void)
{
    drv_uart_error_t err = DRV_UART_OK;

    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        drv_uart_inst_t* inst = &s_inst[ch];

        /* CH_1 (USART1) 由 drv_log_uart 接管，跳过 */
        if (inst->huart == NULL) {
            continue;
        }

        inst->tx_busy = false;
        inst->initialized = false;

        /* 接收 kfifo 与 DMA 共用缓冲，kfifo_move_in 按 DMA 写指针推进 */
        kfifo_init(&inst->rx_fifo, inst->rx_buf, sizeof(inst->rx_buf), NULL);

        /* TX 发送缓冲队列（帧级，元素 = drv_uart_tx_frame_t） */
        (void)msg_fifo_init(&inst->tx_fifo, inst->tx_fifo_buf,
            (uint32_t)sizeof(inst->tx_fifo_buf), (uint16_t)sizeof(drv_uart_tx_frame_t));

        if (!drv_uart_rx_start(inst)) {
            UART_LOG_E("ch%u rx start failed at init (HAL state=%d)",
                (unsigned)ch + 1U, (int)inst->huart->RxState);
            err = DRV_UART_ERROR_UNINITIALIZED;
            continue;
        }

        /* 注册 per-instance HAL 回调（USE_HAL_UART_REGISTER_CALLBACKS=1） */
        HAL_UART_RegisterCallback(inst->huart, HAL_UART_TX_COMPLETE_CB_ID,
            drv_uart_tx_cplt_cb);
        HAL_UART_RegisterCallback(inst->huart, HAL_UART_ERROR_CB_ID,
            drv_uart_error_cb);
        HAL_UART_RegisterRxEventCallback(inst->huart, drv_uart_rx_event_cb);

        inst->initialized = true;
    }

    return err;
}

void drv_uart_deinit_all(void)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        drv_uart_inst_t* inst = &s_inst[ch];
        if (!inst->initialized) {
            continue;
        }

        HAL_UART_DMAStop(inst->huart);
        kfifo_reset(&inst->rx_fifo);
        msg_fifo_deinit(&inst->tx_fifo);

        inst->tx_busy = false;
        inst->initialized = false;
    }
}

bool drv_uart_is_initialized(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM) {
        return false;
    }
    return s_inst[ch].initialized;
}

/* --- 错误恢复 --- */

drv_uart_error_t drv_uart_recover(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM) {
        return DRV_UART_ERROR_INVALID_PARAM;
    }
    drv_uart_inst_t* inst = &s_inst[ch];
    if (!inst->initialized || inst->huart == NULL) {
        return DRV_UART_ERROR_UNINITIALIZED;
    }

    const uint32_t err = HAL_UART_GetError(inst->huart);
    /* 注意：HAL_UART_GetState() 返回 gState|RxState 组合值，TX 忙时(BUSY_TX)
       会让组合结果 != BUSY_RX 导致误判。这里只用 RxState 独立判断接收状态。 */
    const HAL_UART_StateTypeDef rx_state = (HAL_UART_StateTypeDef)inst->huart->RxState;

    /* 无错误且 RX 正在运行（DMA circular 正常工作中）→ 无需恢复 */
    if (err == HAL_UART_ERROR_NONE && rx_state == HAL_UART_STATE_BUSY_RX) {
        return DRV_UART_OK;
    }

    UART_LOG_W("ch%u recover: err=0x%08lX rx_state=%d，重置接收链路",
        (unsigned)ch + 1U, (unsigned long)err, (int)rx_state);

    /* 中止当前接收（阻塞式，释放 DMA + 清 RX 状态，IDLE 中断一并关闭） */
    if (rx_state != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortReceive(inst->huart);
    }

    /* 丢弃残留接收数据并复位 kfifo */
    kfifo_reset(&inst->rx_fifo);

    /* 重启 RX DMA（含重使能 IDLE 中断），失败返回未初始化 */
    if (!drv_uart_rx_start(inst)) {
        UART_LOG_E("ch%u recover: rx restart failed (state=%d)",
            (unsigned)ch + 1U, (int)HAL_UART_GetState(inst->huart));
        return DRV_UART_ERROR_UNINITIALIZED;
    }

    /* TX 侧若已恢复 READY 则释放卡死的忙标志 */
    if (inst->huart->gState == HAL_UART_STATE_READY) {
        inst->tx_busy = false;
    }

    return DRV_UART_OK;
}

/* --- TX（经 msg_fifo 发送缓冲队列） --- */

drv_uart_error_t drv_uart_send(drv_uart_channel_t ch, const uint8_t* data, uint32_t len)
{
    if (ch >= DRV_UART_CH_NUM || !data) {
        return DRV_UART_ERROR_NULL_PTR;
    }
    if (!s_inst[ch].initialized) {
        return DRV_UART_ERROR_UNINITIALIZED;
    }
    if (len == 0 || len > UINT16_MAX) {
        return DRV_UART_OK;
    }

    drv_uart_inst_t* inst = &s_inst[ch];

    if (len > DRV_UART_TX_MAX_FRAME_LEN) {
        return DRV_UART_ERROR_INVALID_PARAM;
    }

    /* TX 空闲：拷贝到持久 DMA 缓冲后立即发送（不进队列） */
    if (!inst->tx_busy && inst->huart->gState == HAL_UART_STATE_READY
        && msg_fifo_empty(&inst->tx_fifo)) {
        memcpy(inst->tx_dma_buf, data, len);
        if (HAL_UART_Transmit_DMA(inst->huart, inst->tx_dma_buf,
                (uint16_t)len)
            == HAL_OK) {
            inst->tx_busy = true;
            return DRV_UART_OK;
        } else {
            return DRV_UART_ERROR_TX_BUSY;
        }
    }

    /* TX 忙或 DMA 启动失败：入队缓冲，由 drv_uart_tx_flush() 排空，保证不丢帧 */
    drv_uart_tx_frame_t frame;
    frame.len = (uint16_t)len;
    memcpy(frame.data, data, len);

    if (!msg_fifo_push(&inst->tx_fifo, &frame)) {
        return DRV_UART_ERROR_TX_QUEUE_FULL; /* 队列满，帧被丢弃 */
    }

    return DRV_UART_OK;
}

bool drv_uart_is_tx_busy(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) {
        return false;
    }

    return s_inst[ch].tx_busy || s_inst[ch].huart->gState != HAL_UART_STATE_READY;
}

void drv_uart_tx_flush(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) {
        return;
    }

    drv_uart_inst_t* inst = &s_inst[ch];

    /* 主循环驱动：DMA 空闲且有帧时启动，TxCplt 清忙后由本函数下轮续发。
       单次调用仅发一帧即返回，避免长时间占用主循环（帧间隔由 1ms 周期保证）。 */
    if (inst->huart->gState == HAL_UART_STATE_READY
        && !inst->tx_busy
        && !msg_fifo_empty(&inst->tx_fifo)) {
        drv_uart_tx_frame_t frame;
        if (!msg_fifo_pop(&inst->tx_fifo, &frame)) {
            return;
        }

        /* 拷贝到持久 DMA 缓冲再启动发送，防止队列后续入队覆盖 DMA 读区域 */
        memcpy(inst->tx_dma_buf, frame.data, frame.len);

        if (HAL_UART_Transmit_DMA(inst->huart, inst->tx_dma_buf,
                frame.len)
            == HAL_OK) {
            inst->tx_busy = true;
        } else {
            /* DMA 启动失败：放回队列由下轮重试 */
            (void)msg_fifo_push(&inst->tx_fifo, &frame);
        }
    }
}

uint32_t drv_uart_tx_pending(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) {
        return 0;
    }
    if (s_inst[ch].tx_fifo.element_size == 0U) {
        return 0;
    }
    return (uint32_t)(kfifo_len(&s_inst[ch].tx_fifo.fifo) / s_inst[ch].tx_fifo.element_size);
}

/* --- RX（DMA circular + IDLE → kfifo 环形缓冲） --- */

uint32_t drv_uart_rx_read(drv_uart_channel_t ch, uint8_t* buf, uint32_t max_len)
{
    if (ch >= DRV_UART_CH_NUM || !buf || max_len == 0) {
        return 0;
    }
    if (!s_inst[ch].initialized) {
        return 0;
    }

    return kfifo_get(&s_inst[ch].rx_fifo, buf, max_len);
}

uint32_t drv_uart_rx_available(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) {
        return 0;
    }

    return kfifo_len(&s_inst[ch].rx_fifo);
}

/* ===== 注册到 HAL 的 per-instance 回调（USE_HAL_UART_REGISTER_CALLBACKS=1） ===== */

/**
 * @brief UART TX DMA 完成回调（per-instance）
 * @note  仅清除 tx_busy；下一帧由主循环 drv_uart_tx_flush() 周期驱动，
 *        中断只做最小操作，不在 ISR 中发起新 DMA 发送。
 */
static void drv_uart_tx_cplt_cb(UART_HandleTypeDef* huart)
{
    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        if (s_inst[ch].initialized && s_inst[ch].huart == huart) {
            s_inst[ch].tx_busy = false;
            return;
        }
    }
}

/**
 * @brief UART RX 事件回调（per-instance，IDLE/HT）
 * @note  DMA circular：IDLE 事件后硬件继续接收，仅同步 kfifo 读指针，不重启。
 */
static void drv_uart_rx_event_cb(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }

    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        if (s_inst[ch].initialized && s_inst[ch].huart == huart) {
            drv_uart_sync_rx_dma(&s_inst[ch]);
            return;
        }
    }

    (void)Size;
}

/**
 * @brief UART 错误回调（per-instance）— 限频打印错误原因，丢弃缓冲并重启接收
 * @note  F1 无 UART_CLEAR_OREF/NEF/FEF/PEF，用 __HAL_UART_CLEAR_*FLAG。
 */
static void drv_uart_error_cb(UART_HandleTypeDef* huart)
{
    uint32_t err = HAL_UART_GetError(huart);

    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        drv_uart_inst_t* inst = &s_inst[ch];
        if (!inst->initialized || inst->huart != huart) {
            continue;
        }

        inst->err_count++;
        inst->err_flags |= err;

        uint32_t now = HAL_GetTick();
        if (now - inst->err_last_log >= DRV_UART_ERR_LOG_PERIOD_MS) {
            UART_LOG_E("ch%u err=0x%02lX x%lu%s%s%s%s%s",
                (unsigned)ch + 1U, (unsigned long)inst->err_flags, (unsigned long)inst->err_count,
                (inst->err_flags & HAL_UART_ERROR_PE) ? " PE(parity)" : "",
                (inst->err_flags & HAL_UART_ERROR_NE) ? " NE(noise)" : "",
                (inst->err_flags & HAL_UART_ERROR_FE) ? " FE(frame)" : "",
                (inst->err_flags & HAL_UART_ERROR_ORE) ? " ORE(overrun)" : "",
                (inst->err_flags & HAL_UART_ERROR_DMA) ? " DMA(transfer)" : "");
            inst->err_last_log = now;
            inst->err_count = 0;
            inst->err_flags = 0;
        }

        /* RX DMA 已被 HAL 中止，丢弃缓冲数据并原地立即重启 */
        kfifo_reset(&inst->rx_fifo);
        if (!drv_uart_rx_start(inst)) {
            UART_LOG_E("ch%u rx err and receive restart failed", (unsigned)ch + 1U);
        }

        /* TX DMA 错误同样会中止发送，防止 tx_busy 永久卡死 */
        if (huart->gState == HAL_UART_STATE_READY) {
            inst->tx_busy = false;
        }
        return;
    }
}
