/**
 * @file    drv_uart.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   通用串口设备驱动实现（USART1，DMA 输出 + DMA circular + IDLE 中断接收）
 * @attention
 *
 * 参考 E1_Hand_G474 的 drv_uart 设计（per-instance HAL 回调）：
 *   - TX：DMA normal，gState + s_tx_busy 双状态检忙
 *   - RX：DMA circular + IDLE 事件 (ReceiveToIdle)，总线空闲即按实际长度上抛，
 *     天然帧分块、错位自愈
 *   - HAL 回调通过 HAL_UART_RegisterCallback/RegisterRxEventCallback 注册
 *     （USE_HAL_UART_REGISTER_CALLBACKS=1），与 drv_log_uart 互不冲突
 *
 * F1 差异：无 UART_CLEAR_OREF/NEF/FEF/PEF 与 UART_RXDATA_FLUSH_REQUEST，
 * 错误清理改用 __HAL_UART_CLEAR_*FLAG。
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

#include "log.h"
#include "main.h"
#include "usart.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define UART_LOG_ENABLE 0

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

/** @brief RX DMA 单缓冲字节数（IDLE 事件分帧） */
#define DRV_UART_RX_BUF_SIZE (32U)

/** @brief TX 最大单次发送量 */
#define DRV_UART_TX_BUF_SIZE (128U)

/** @brief 错误日志聚合窗口 (ms)：窗口内的错误只在首次打印，附累计次数 */
#define DRV_UART_ERR_LOG_PERIOD_MS (1000U)

/* Private types -------------------------------------------------------------*/

typedef struct {
    UART_HandleTypeDef* huart;          /**< HAL UART 句柄 */
    uint8_t             rx_buf[DRV_UART_RX_BUF_SIZE]; /**< RX DMA 缓冲 */
    uint8_t             tx_buf[DRV_UART_TX_BUF_SIZE]; /**< TX DMA 发送缓冲 */
    bool                tx_busy;        /**< TX DMA 传输中 */
    bool                initialized;    /**< 初始化标志 */
    drv_uart_rx_callback_t rx_callback; /**< 接收回调（中断中执行） */
    uint32_t            err_count;      /**< 日志聚合窗口内错误累计次数 */
    uint32_t            err_flags;      /**< 日志聚合窗口内错误标志并集 */
    uint32_t            err_last_log;   /**< 上次错误日志时间戳 (ms) */
} drv_uart_inst_t;

/* Private variables ---------------------------------------------------------*/

static drv_uart_inst_t s_inst[DRV_UART_CH_NUM] = {
    /* CH_1 (USART1)：通用串口 */
    [DRV_UART_CH_1] = { .huart = &huart1 },
    /* CH_2 (USART2) 已由独立 drv_log_uart 控制台驱动接管，此处置空跳过 */
    [DRV_UART_CH_2] = { .huart = NULL },
};

/* Private functions prototypes ----------------------------------------------*/

/**
 * @brief 启动/重启 RX DMA 到缓冲（ISR 与主循环共用）
 * @note  IDLE 事件模式：总线空闲即上报本次已收长度并重启，天然按帧分块。
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

        /* CH_2 (USART2) 由 drv_log_uart 接管，跳过 */
        if (inst->huart == NULL) {
            continue;
        }

        inst->tx_busy = false;
        inst->initialized = false;

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

/* --- TX --- */

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

    if (inst->tx_busy || inst->huart->gState != HAL_UART_STATE_READY) {
        return DRV_UART_ERROR_TX_BUSY;
    }

    if (len > DRV_UART_TX_BUF_SIZE) {
        return DRV_UART_ERROR_INVALID_PARAM;
    }

    /* 拷贝到内部持久缓冲区再启动 DMA，防止调用者栈回收后 DMA 读脏数据 */
    memcpy(inst->tx_buf, data, len);

    if (HAL_UART_Transmit_DMA(inst->huart, inst->tx_buf, (uint16_t)len) != HAL_OK) {
        return DRV_UART_ERROR_TX_BUSY;
    }

    inst->tx_busy = true;

    return DRV_UART_OK;
}

bool drv_uart_is_tx_busy(drv_uart_channel_t ch)
{
    if (ch >= DRV_UART_CH_NUM || !s_inst[ch].initialized) {
        return false;
    }

    return s_inst[ch].tx_busy || s_inst[ch].huart->gState != HAL_UART_STATE_READY;
}

/* --- RX --- */

void drv_uart_register_rx_callback(drv_uart_channel_t ch,
    drv_uart_rx_callback_t callback)
{
    if (ch >= DRV_UART_CH_NUM) return;
    s_inst[ch].rx_callback = callback;
}

/* ===== 注册到 HAL 的 per-instance 回调（USE_HAL_UART_REGISTER_CALLBACKS=1） ===== */

/**
 * @brief UART TX DMA 完成回调（per-instance）— 清除 tx_busy
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
 * @brief UART RX 事件回调（per-instance，IDLE/TC）— 上抛数据
 * @note  IDLE：总线空闲，本次突发结束，Size = 实际已收字节数。
 */
static void drv_uart_rx_event_cb(UART_HandleTypeDef* huart, uint16_t Size)
{
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }

    for (uint32_t ch = 0; ch < DRV_UART_CH_NUM; ch++) {
        if (s_inst[ch].initialized && s_inst[ch].huart == huart) {
            drv_uart_inst_t* inst = &s_inst[ch];

            /* 重启接收（幂等：IDLE 事件后 HAL 已停止本次接收） */
            if (!drv_uart_rx_start(inst)) {
                UART_LOG_E("ch%u rx restart failed in ISR", (unsigned)ch + 1U);
            }

            /* 上抛本次空闲间隙前收到的数据 */
            if (Size > 0 && inst->rx_callback) {
                inst->rx_callback((drv_uart_channel_t)ch, inst->rx_buf, Size);
            }
            return;
        }
    }
}

/**
 * @brief UART 错误回调（per-instance）— 限频打印错误原因，立即重启接收
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

        /* RX DMA 已被 HAL 中止，原地立即重启 */
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
