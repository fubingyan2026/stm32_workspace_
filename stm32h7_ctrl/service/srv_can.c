/**
 * @file    srv_can.c
 * @brief   CAN FD 电机控制协议服务实现
 *
 * 控制帧解析 → srv_motor_behavior API。
 * 反馈帧打包 → drv_can_send (CAN FD, 64B max)。
 */

#include "srv_can.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void srv_can_init(void)
{
}

/**
 * @brief ISR 回调：快速复制数据，置标志位
 */
void srv_can_on_rx(const drv_can_msg_t* msg)
{
    if (!msg || msg->id != SRV_CAN_ID_CTRL || msg->dlc < SRV_CAN_CTRL_LEN)
        return;

}

/**
 * @brief 主循环处理：将缓存的控制数据下发到电机行为层
 * @note  ctrl 字节（bit0/bit1 使能位）已废弃：电机使能由 srv_motor 上电自治，
 *        FAULT 由 err_code 清零自动恢复；目标值仅在 RUNNING 态被接受。
 */
void srv_can_process(void)
{
  
}

/**
 * @brief 打包并发送 9 电机反馈数据 (CAN FD)
 */
void srv_can_send_feedback(void)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    uint8_t buf[SRV_CAN_FB_LEN];
    memset(buf, 0, sizeof(buf));

    drv_can_msg_t tx = {
        .id = SRV_CAN_ID_FEEDBACK,
        .is_extended = false,
        .is_fd = true,
        .dlc = SRV_CAN_FB_LEN,
    };
    memcpy(tx.data, buf, SRV_CAN_FB_LEN);
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 打包并发送 9 电机低频状态数据 (CAN FD, 500ms)
 */
void srv_can_send_status(void)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    uint8_t buf[SRV_CAN_STATUS_LEN];
    memset(buf, 0, sizeof(buf));

    drv_can_msg_t tx = {
        .id = SRV_CAN_ID_STATUS,
        .is_extended = false,
        .is_fd = true,
        .dlc = SRV_CAN_STATUS_LEN,
    };
    memcpy(tx.data, buf, SRV_CAN_STATUS_LEN);
    drv_can_send(DRV_CAN_CH_1, &tx);
}
