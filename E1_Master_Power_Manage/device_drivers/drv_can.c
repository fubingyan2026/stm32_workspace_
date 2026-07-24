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
#include "main.h"

#include <string.h>

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

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_can_error_t drv_can_init(void)
{
    /* 拉低 CAN 收发器 STB 引脚（正常模式，低电平有效） */
    HAL_GPIO_WritePin(P_CAN_STB_GPIO_Port, P_CAN_STB_Pin, GPIO_PIN_RESET);

    for (uint32_t ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        memset(&s_ctx[ch], 0, sizeof(s_ctx[ch]));

        if (HAL_CAN_Start(s_hcan[ch]) != HAL_OK) {
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        /* 使能 RX FIFO 0 消息待处理中断 */
        if (HAL_CAN_ActivateNotification(s_hcan[ch], CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
            HAL_CAN_Stop(s_hcan[ch]);
            return DRV_CAN_ERROR_UNINITIALIZED;
        }

        s_ctx[ch].initialized = true;
    }

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
    return DRV_CAN_OK;
}
