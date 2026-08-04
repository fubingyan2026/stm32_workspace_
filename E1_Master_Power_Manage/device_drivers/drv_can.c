/**
 * @file    drv_can.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-8
 * @brief   CAN 设备驱动实现（经典 bxCAN，中断接收，句柄自包含）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

#include "can.h"
#include "drv_systick.h"
#include "log.h"
#include "main.h"

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

/* Private types -------------------------------------------------------------*/

typedef struct {
    drv_can_rx_callback_t rx_callback;
    bool                  initialized;
} drv_can_ctx_t;

/* Private constants ---------------------------------------------------------*/

/** @brief 通道 → HAL 句柄映射（基于 CubeMX can.c，仅 CAN1） */
static CAN_HandleTypeDef* const s_hcan[DRV_CAN_CH_NUM] = {
    [DRV_CAN_CH_1] = &hcan1,
};

/* Private variables ---------------------------------------------------------*/

static drv_can_ctx_t s_ctx[DRV_CAN_CH_NUM];
static uint32_t s_tx_err_log_ts; /**< 上次 TX 忙告警时间戳 (ms) */

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_can_error_t drv_can_init(void)
{
    /* 拉低 CAN 收发器 STB 引脚（正常模式，低电平有效） */
    HAL_GPIO_WritePin(P_CAN_STB_GPIO_Port, P_CAN_STB_Pin, GPIO_PIN_RESET);

    for (uint32_t ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        memset(&s_ctx[ch], 0, sizeof(s_ctx[ch]));

        /* 配置 RX 滤波：掩码全 0 → 全通过，接入 FIFO0。
         * 不配滤波则 bxCAN 滤波 bank 未激活，FIFO0 收不到任何报文，RX 中断永不触发。
         * ID 过滤由 can_task 的 can_rx_callback 按 CAN ID 软件分发（各服务忽略非本属帧）。 */
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
        memset(&s_ctx[ch], 0, sizeof(s_ctx[ch]));
    }

    /* 拉高 CAN 收发器 STB 引脚（待机模式） */
    HAL_GPIO_WritePin(P_CAN_STB_GPIO_Port, P_CAN_STB_Pin, GPIO_PIN_SET);

    DRV_CAN_LOG_I("CAN 反初始化完成 (STB 待机)");
}

bool drv_can_is_initialized(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM) {
        return false;
    }
    return s_ctx[ch].initialized;
}

/* --- 发送 --- */

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

    CAN_TxHeaderTypeDef tx = {
        .StdId = msg->is_extended ? 0 : msg->id,
        .ExtId = msg->is_extended ? msg->id : 0,
        .IDE   = msg->is_extended ? CAN_ID_EXT : CAN_ID_STD,
        .RTR   = CAN_RTR_DATA,
        .DLC   = msg->dlc,
        .TransmitGlobalTime = DISABLE,
    };

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(s_hcan[ch], &tx, (uint8_t*)msg->data, &mailbox) != HAL_OK) {
        /* 邮箱满多为瞬时/可恢复（无 ACK / 总线异常），限频告警 */
        const uint32_t now_ms = millis();
        if ((uint32_t)(now_ms - s_tx_err_log_ts) >= DRV_CAN_ERR_LOG_PERIOD_MS) {
            s_tx_err_log_ts = now_ms;
            DRV_CAN_LOG_W("CAN1 发送失败 TX忙: id=0x%03X dlc=%u, 空闲邮箱=%d",
                (unsigned)msg->id, (unsigned)msg->dlc,
                (int)HAL_CAN_GetTxMailboxesFreeLevel(s_hcan[ch]));
        }
        return DRV_CAN_ERROR_TX_BUSY;
    }

    return DRV_CAN_OK;
}

bool drv_can_tx_ready(drv_can_channel_t ch)
{
    if (ch >= DRV_CAN_CH_NUM || !s_ctx[ch].initialized) {
        return false;
    }
    return HAL_CAN_GetTxMailboxesFreeLevel(s_hcan[ch]) > 0;
}

/* ===== HAL 回调 ===== */

/**
 * @brief CAN Rx FIFO 0 消息待处理回调
 *
 * 由 HAL_CAN_IRQHandler 内部触发。
 * 读取报文后调用用户注册的接收回调。
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
