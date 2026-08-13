/**
 * @file    srv_can.c
 * @brief   旧 CAN FD 上位机控制协议服务实现
 *
 * 保留的上位机协议（can_protocol.md V1.1.0）：
 *   控制帧 (Host→Device, CAN ID 0x100)：一帧携带 9 电机使能 + 位置/速度/电流目标
 *   反馈帧 (Device→Host, CAN ID 0x101, 100ms)：状态 + 角度/速度/Q电流
 *   状态帧 (Device→Host, CAN ID 0x102, 500ms)：状态 + 故障/温度/电压
 *
 * 苇熠伺服执行器测试协议已独立到 srv_ht_temp_test.c/h 与 srv_ht_torque_test.c/h
 * （经典 CAN 2.0A，1 Mbps，设备地址寻址），激活哪个模块由 srv_ht_test_mode.h 的
 * SRV_HT_TEST_MODE_TORQUE 宏决定。srv_can_on_rx() 先将测试协议帧路由给选中的
 * 测试模块 on_rx，返回 true 表示已处理；剩余帧按旧协议解析 0x100。
 */

#include "srv_can.h"

#include "srv_ht_test_mode.h"
#include "srv_motor_behavior.h"

/* 苇熠测试模式选择（srv_ht_test_mode.h）：temp=速度模式原测试，torque=位置模式往复耐久测试 */
#if SRV_HT_TEST_MODE_TORQUE
#include "srv_ht_torque_test.h"
#define HT_TEST_ON_RX srv_ht_torque_test_on_rx
#else
#include "srv_ht_temp_test.h"
#define HT_TEST_ON_RX srv_ht_temp_test_on_rx
#endif

#include <string.h>

/* Private variables ---------------------------------------------------------*/

/** @brief ISR → 主循环控制数据暂存（旧 CAN FD 上位机控制帧，保留） */
static struct {
    bool pending;
    uint8_t ctrl;
    int16_t pos[9];
    int16_t spd[9];
    int16_t cur[9];
} s_rx;

/* Exported functions --------------------------------------------------------*/

void srv_can_init(void)
{
    memset(&s_rx, 0, sizeof(s_rx));
}

/**
 * @brief ISR 回调：先路由给测试协议（选中模块 on_rx），再按旧协议解析 0x100 控制帧
 * @note  快速复制数据，置标志位，不打日志
 */
void srv_can_on_rx(const drv_can_msg_t* msg)
{
    if (!msg)
        return;

    /* 苇熠测试协议帧（扫描应答/位置/报警/电压/在线心跳）由选中的测试模块消费 */
    if (HT_TEST_ON_RX(msg))
        return;

    if (msg->id != SRV_CAN_ID_CTRL || msg->dlc < SRV_CAN_CTRL_LEN)
        return;

    s_rx.ctrl = msg->data[0];
    memcpy(s_rx.pos, &msg->data[1], 18);
    memcpy(s_rx.spd, &msg->data[19], 18);
    memcpy(s_rx.cur, &msg->data[37], 18);
    s_rx.pending = true;
}

/**
 * @brief 主循环处理：将缓存的控制数据下发到电机行为层
 * @note  ctrl 字节（bit0/bit1 使能位）已废弃：电机使能由 srv_motor 上电自治，
 *        FAULT 由 err_code 清零自动恢复；目标值仅在 RUNNING 态被接受。
 */
void srv_can_process(void)
{
    if (!s_rx.pending)
        return;
    s_rx.pending = false;

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++)
        srv_motor_behavior_set_setpoint(i, s_rx.pos[i], s_rx.spd[i], s_rx.cur[i]);
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

    buf[0] = (uint8_t)srv_motor_behavior_get_state();

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);

        buf[1 + i] = fb ? (uint8_t)fb->fsm_state : 0;

        if (fb) {
            buf[10 + i * 2] = (uint8_t)fb->angle_fb;
            buf[10 + i * 2 + 1] = (uint8_t)(fb->angle_fb >> 8);
            buf[28 + i * 2] = (uint8_t)fb->speed_fb;
            buf[28 + i * 2 + 1] = (uint8_t)(fb->speed_fb >> 8);
            buf[46 + i * 2] = (uint8_t)fb->q_cur;
            buf[46 + i * 2 + 1] = (uint8_t)(fb->q_cur >> 8);
        }
    }

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

    buf[0] = (uint8_t)srv_motor_behavior_get_state();

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);

        if (fb) {
            buf[1 + i] = (uint8_t)fb->fsm_state;
            buf[10 + i] = (uint8_t)fb->err_code;
            buf[19 + i] = (uint8_t)fb->temp;
            buf[28 + i * 2] = (uint8_t)fb->vbus;
            buf[28 + i * 2 + 1] = (uint8_t)(fb->vbus >> 8);
        }
    }

    drv_can_msg_t tx = {
        .id = SRV_CAN_ID_STATUS,
        .is_extended = false,
        .is_fd = true,
        .dlc = SRV_CAN_STATUS_LEN,
    };
    memcpy(tx.data, buf, SRV_CAN_STATUS_LEN);
    drv_can_send(DRV_CAN_CH_1, &tx);
}
