/**
 * @file    drv_can.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-12
 * @brief   CAN 设备驱动实现（经典 bxCAN，中断接收 + msg_fifo 收发缓冲队列）
 * @attention
 *
 * 参考 E1_Hand_G474 的 drv_can 队列架构：
 *   - RX：中断回调 HAL_CAN_RxFifo0MsgPendingCallback 入队 msg_fifo（ISR 单生产者），
 *         主循环 drv_can_rx_pop 出队消费
 *   - TX：主循环 drv_can_send/drv_can_tx_enqueue 入队（单生产者），
 *         drv_can_tx_flush 排空到 bxCAN TX 邮箱
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

#include "can.h"
#include "drv_systick.h"
#include "log.h"
#include "main.h"
#include "msg_fifo.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_CAN_LOG_ENABLE 1

#if DRV_CAN_LOG_ENABLE
#define DRV_CAN_LOG_E(...) LOG_E("drv_can", __VA_ARGS__)
#define DRV_CAN_LOG_W(...) LOG_W("drv_can", __VA_ARGS__)
#define DRV_CAN_LOG_I(...) LOG_I("drv_can", __VA_ARGS__)
#define DRV_CAN_LOG_D(...) LOG_D("drv_can", __VA_ARGS__)
#else
#define DRV_CAN_LOG_E(...) ((void)0)
#define DRV_CAN_LOG_W(...) ((void)0)
#define DRV_CAN_LOG_I(...) ((void)0)
#define DRV_CAN_LOG_D(...) ((void)0)
#endif

/** @brief TX 失败日志限频窗口 (ms)：邮箱满时防止刷屏 */
#define DRV_CAN_ERR_LOG_PERIOD_MS (1000U)

/* Private constants ---------------------------------------------------------*/

/** @brief 接收缓冲队列深度（报文数）：ISR 入队 / 主循环 drv_can_rx_pop 出队 */
#define DRV_CAN_RX_FIFO_DEPTH 16U

/** @brief 发送缓冲队列深度（报文数）：主循环入队 / drv_can_tx_flush 排空 */
#define DRV_CAN_TX_FIFO_DEPTH 16U

/* Private types -------------------------------------------------------------*/

typedef struct {
    drv_can_rx_callback_t rx_callback;
    bool initialized;
} drv_can_ctx_t;

/* Private constants ---------------------------------------------------------*/

/** @brief 通道 → HAL 句柄映射（基于 CubeMX can.c，仅 CAN1） */
static CAN_HandleTypeDef* const s_hcan[DRV_CAN_CH_NUM] = {
    [DRV_CAN_CH_1] = &hcan,
};

/* Private variables ---------------------------------------------------------*/

static drv_can_ctx_t s_ctx[DRV_CAN_CH_NUM];
static uint32_t s_tx_err_log_ts; /**< 上次 TX 忙告警时间戳 (ms) */

/** @brief 每通道接收缓冲队列（元素 = drv_can_msg_t，ISR 单生产者） */
static msg_fifo_t s_rx_fifo[DRV_CAN_CH_NUM];
static uint8_t s_rx_fifo_buf[DRV_CAN_CH_NUM][DRV_CAN_RX_FIFO_DEPTH * sizeof(drv_can_msg_t)];

/** @brief 每通道发送缓冲队列（元素 = drv_can_msg_t，主循环单生产者） */
static msg_fifo_t s_tx_fifo[DRV_CAN_CH_NUM];
static uint8_t s_tx_fifo_buf[DRV_CAN_CH_NUM][DRV_CAN_TX_FIFO_DEPTH * sizeof(drv_can_msg_t)];

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_can_error_t drv_can_init(void)
{
    for (uint32_t ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        memset(&s_ctx[ch], 0, sizeof(s_ctx[ch]));

        /* 配置 RX 滤波：掩码全 0 → 全通过，接入 FIFO0。
         * 不配滤波则 bxCAN 滤波 bank 未激活，FIFO0 收不到任何报文，RX 中断永不触发。
         * ID 过滤由 can_task 的 can_rx_callback 按 CAN ID 软件分发。 */
        CAN_FilterTypeDef filter = { 0 };
        filter.FilterActivation = CAN_FILTER_ENABLE;
        filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;
        filter.FilterIdHigh = 0x0000U;
        filter.FilterIdLow = 0x0000U;
        filter.FilterMaskIdHigh = 0x0000U;
        filter.FilterMaskIdLow = 0x0000U;
        filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        filter.FilterBank = 0;
        if (HAL_CAN_ConfigFilter(s_hcan[ch], &filter) != HAL_OK) {
            DRV_CAN_LOG_E("CAN%u RX 滤波配置失败", (unsigned)ch + 1U);
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        if (HAL_CAN_Start(s_hcan[ch]) != HAL_OK) {
            DRV_CAN_LOG_E("CAN%u Start 失败 (HAL state=%d)",
                (unsigned)ch + 1U, (int)HAL_CAN_GetState(s_hcan[ch]));
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        /* 使能 RX FIFO 0 消息待处理中断 */
        if (HAL_CAN_ActivateNotification(s_hcan[ch], CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
            HAL_CAN_Stop(s_hcan[ch]);
            DRV_CAN_LOG_E("CAN%u 使能 RX 中断失败 (state=%d)",
                (unsigned)ch + 1U, (int)HAL_CAN_GetState(s_hcan[ch]));
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        /* 初始化消息缓冲队列（接收/发送各一条，元素 = 完整报文） */
        (void)msg_fifo_init(&s_rx_fifo[ch], s_rx_fifo_buf[ch],
            (uint32_t)sizeof(s_rx_fifo_buf[ch]), (uint16_t)sizeof(drv_can_msg_t));
        (void)msg_fifo_init(&s_tx_fifo[ch], s_tx_fifo_buf[ch],
            (uint32_t)sizeof(s_tx_fifo_buf[ch]), (uint16_t)sizeof(drv_can_msg_t));

        s_ctx[ch].initialized = true;
    }

    DRV_CAN_LOG_I("CAN%u 初始化完成, RX FIFO0 中断已使能", (unsigned)DRV_CAN_CH_NUM);
    return DRV_CAN_OK;
}

void drv_can_deinit_all(void)
{
    for (uint32_t ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        if (!s_ctx[ch].initialized) {
            continue;
        }
        HAL_CAN_DeactivateNotification(s_hcan[ch], CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_Stop(s_hcan[ch]);
        msg_fifo_deinit(&s_rx_fifo[ch]);
        msg_fifo_deinit(&s_tx_fifo[ch]);
        memset(&s_ctx[ch], 0, sizeof(s_ctx[ch]));
    }

    DRV_CAN_LOG_I("CAN 反初始化完成");
}

bool drv_can_is_initialized(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM) {
        return false;
    }
    return s_ctx[ch].initialized;
}

/* --- 发送（经 TX 队列） --- */

drv_can_error_t drv_can_send(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    if (ch >= DRV_CAN_CH_NUM || !msg) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }
    if (!s_ctx[ch].initialized) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }
    if (msg->dlc > 8) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }

    /* 入队 TX 缓冲队列（主循环单生产者），由 drv_can_tx_flush 排空 */
    return msg_fifo_push(&s_tx_fifo[ch], msg) ? DRV_CAN_OK : DRV_CAN_ERROR_TX_BUSY;
}

uint32_t drv_can_tx_pending(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return 0;
    }
    if (s_tx_fifo[ch].element_size == 0U) {
        return 0;
    }
    return (uint32_t)(kfifo_len(&s_tx_fifo[ch].fifo) / s_tx_fifo[ch].element_size);
}

void drv_can_tx_flush(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return;
    }
    while (!msg_fifo_empty(&s_tx_fifo[ch]) && drv_can_tx_all_done(ch)) {
        drv_can_msg_t msg;
        if (!msg_fifo_pop(&s_tx_fifo[ch], &msg)) {
            break;
        }

        CAN_TxHeaderTypeDef tx = {
            .StdId = msg.is_extended ? 0 : msg.id,
            .ExtId = msg.is_extended ? msg.id : 0,
            .IDE = msg.is_extended ? CAN_ID_EXT : CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = msg.dlc,
            .TransmitGlobalTime = DISABLE,
        };

        uint32_t mailbox;
        if (HAL_CAN_AddTxMessage(s_hcan[ch], &tx, msg.data, &mailbox) != HAL_OK) {
            /* 邮箱满多为瞬时/可恢复（无 ACK / 总线异常），限频告警 */
            const uint32_t now_ms = millis();
            if ((uint32_t)(now_ms - s_tx_err_log_ts) >= DRV_CAN_ERR_LOG_PERIOD_MS) {
                s_tx_err_log_ts = now_ms;
                DRV_CAN_LOG_W("CAN1 发送失败 TX忙: id=0x%03X dlc=%u, 空闲邮箱=%d",
                    (unsigned)msg.id, (unsigned)msg.dlc,
                    (int)HAL_CAN_GetTxMailboxesFreeLevel(s_hcan[ch]));
            }
            break;
        }
    }
}

bool drv_can_tx_all_done(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return true; /* 未初始化视作空闲 */
    }
    /* bxCAN 3 个 TX 邮箱全部空闲 = 所有已提交帧均已发出（TSR.TME 硬件置位） */
    return HAL_CAN_GetTxMailboxesFreeLevel(s_hcan[ch]) == 3U;
}

/* --- 接收（经 RX 队列） --- */

bool drv_can_rx_pop(drv_can_channel_t ch, drv_can_msg_t* msg)
{
    if (ch >= DRV_CAN_CH_NUM || !msg) {
        return false;
    }
    if (!s_ctx[ch].initialized) {
        return false;
    }
    return msg_fifo_pop(&s_rx_fifo[ch], msg);
}

uint32_t drv_can_rx_pending(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return 0;
    }
    if (s_rx_fifo[ch].element_size == 0U) {
        return 0;
    }
    return (uint32_t)(kfifo_len(&s_rx_fifo[ch].fifo) / s_rx_fifo[ch].element_size);
}

void drv_can_rx_fifo_reset(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM) {
        return;
    }
    msg_fifo_deinit(&s_rx_fifo[ch]);
    (void)msg_fifo_init(&s_rx_fifo[ch], s_rx_fifo_buf[ch],
        (uint32_t)sizeof(s_rx_fifo_buf[ch]), (uint16_t)sizeof(drv_can_msg_t));
}

/* ===== HAL 回调 ===== */

/**
 * @brief CAN Rx FIFO 0 消息待处理回调
 *
 * 由 HAL_CAN_IRQHandler 内部触发。
 * 读取报文后入队 RX 缓冲队列，并调用用户注册的接收回调（若注册）。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    /* 查找通道 */
    drv_can_channel_t ch;
    for (ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        if (s_ctx[ch].initialized && s_hcan[ch] == hcan) {
            break;
        }
    }
    if (ch >= DRV_CAN_CH_NUM) {
        return;
    }

    CAN_RxHeaderTypeDef rx;
    drv_can_msg_t msg;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, msg.data) != HAL_OK) {
        return;
    }

    msg.id = rx.IDE == CAN_ID_EXT ? rx.ExtId : rx.StdId;
    msg.is_extended = (rx.IDE == CAN_ID_EXT);
    msg.dlc = rx.DLC;

    /* 缓冲队列：ISR 写入（单生产者），主循环 drv_can_rx_pop 出队消费 */
    (void)msg_fifo_push(&s_rx_fifo[ch], &msg);

    if (s_ctx[ch].rx_callback) {
        s_ctx[ch].rx_callback(ch, &msg);
    }
}

/* --- 接收回调注册 --- */

drv_can_error_t drv_can_register_rx_callback(drv_can_channel_t ch,
    drv_can_rx_callback_t callback)
{
    if (ch >= DRV_CAN_CH_NUM) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }
    if (!s_ctx[ch].initialized) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }

    s_ctx[ch].rx_callback = callback;
    DRV_CAN_LOG_I("CAN%u 接收回调已注册", (unsigned)ch + 1U);
    return DRV_CAN_OK;
}

/* --- Bus-Off 检测 / 自恢复 --- */

bool drv_can_is_bus_off(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return false;
    }
    /* bxCAN ESR.BOFF：内核已进入 Bus-Off 并离线 */
    return (s_hcan[ch]->Instance->ESR & CAN_ESR_BOFF) != 0U;
}

drv_can_error_t drv_can_recover(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }
    if (!drv_can_is_bus_off(ch)) {
        return DRV_CAN_OK;
    }

    /* Stop/Start 触发内核 128×11 隐性位恢复；滤波器硬件配置保留 */
    if (HAL_CAN_Stop(s_hcan[ch]) != HAL_OK) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }
    if (HAL_CAN_Start(s_hcan[ch]) != HAL_OK) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }
    /* 重使能 RX FIFO0 中断（幂等），确保恢复后继续接收 */
    if (HAL_CAN_ActivateNotification(s_hcan[ch], CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }

    DRV_CAN_LOG_I("CAN%u 已从 Bus-Off 恢复", (unsigned)ch + 1U);
    return DRV_CAN_OK;
}
