/**
 * @file    srv_dm4310_ctrl.c
 * @author  maximillian
 * @version V3.0.0
 * @date    2026-08-28
 * @brief   DM4310 电机控制服务（单电机，MIT 模式）
 * @attention
 *
 * 精简自多电机多模式版本：仅保留一路 DM4310 电机的 MIT 模式控制与反馈。
 * 发送走 dev_dm4310 的 MIT 帧（经 drv_can 队列），接收走 drv_can_rx_pop 轮询。
 */

#include "srv_dm4310_ctrl.h"

#include "drv_can.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_DM4310_CTRL_LOG_ENABLE 1

#if SRV_DM4310_CTRL_LOG_ENABLE
#define SRV_DM4310_CTRL_LOG_E(...) LOG_E("srv_dm4310_ctrl", __VA_ARGS__)
#define SRV_DM4310_CTRL_LOG_W(...) LOG_W("srv_dm4310_ctrl", __VA_ARGS__)
#define SRV_DM4310_CTRL_LOG_I(...) LOG_I("srv_dm4310_ctrl", __VA_ARGS__)
#define SRV_DM4310_CTRL_LOG_D(...) LOG_D("srv_dm4310_ctrl", __VA_ARGS__)
#else
#define SRV_DM4310_CTRL_LOG_E(...) ((void)0)
#define SRV_DM4310_CTRL_LOG_W(...) ((void)0)
#define SRV_DM4310_CTRL_LOG_I(...) ((void)0)
#define SRV_DM4310_CTRL_LOG_D(...) ((void)0)
#endif

/* Private variables ---------------------------------------------------------*/

static motor_t s_motor[MOTOR_NUM];
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void srv_dm4310_can_send(uint16_t id, const uint8_t* data, uint8_t len);

/* Exported functions --------------------------------------------------------*/

void srv_dm4310_ctrl_init(void)
{
    memset(&s_motor[MOTOR_1], 0, sizeof(s_motor[MOTOR_1]));

    s_motor[MOTOR_1].id = 1; /* 按实际总线 ID 配置 */
    s_motor[MOTOR_1].ctrl.mode = MIT_MODE;
    s_motor[MOTOR_1].cmd.mode = MIT_MODE;

    /* 注册 CAN 发送回调（桥接到 drv_can 队列发送，与驱动解耦） */
    dm4310_can_send_register(srv_dm4310_can_send);

    s_initialized = true;

    SRV_DM4310_CTRL_LOG_I("DM4310 初始化完成 (ID=%d, MIT 模式, CAN1)",
        (int)s_motor[MOTOR_1].id);
}

void srv_dm4310_ctrl_enable(void)
{
    if (!s_initialized) {
        return;
    }
    s_motor[MOTOR_1].start_flag = 1;
    dm4310_enable(&s_motor[MOTOR_1]);
}

void srv_dm4310_ctrl_disable(void)
{
    if (!s_initialized) {
        return;
    }
    s_motor[MOTOR_1].start_flag = 0;
    dm4310_disable(&s_motor[MOTOR_1]);
}

void srv_dm4310_ctrl_set_target(float pos, float vel, float kp, float kd, float tor)
{
    if (!s_initialized) {
        return;
    }

    s_motor[MOTOR_1].cmd.pos_set = pos;
    s_motor[MOTOR_1].cmd.vel_set = vel;
    s_motor[MOTOR_1].cmd.kp_set = kp;
    s_motor[MOTOR_1].cmd.kd_set = kd;
    s_motor[MOTOR_1].cmd.tor_set = tor;

    dm4310_set(&s_motor[MOTOR_1]);
}

void srv_dm4310_ctrl_send(void)
{
    if (!s_initialized) {
        return;
    }
    dm4310_ctrl_send(&s_motor[MOTOR_1]);
}

void srv_dm4310_ctrl_poll_rx(void)
{
    if (!s_initialized) {
        return;
    }

    drv_can_msg_t msg;
    while (drv_can_rx_pending(DRV_CAN_CH_1) > 0U) {
        if (!drv_can_rx_pop(DRV_CAN_CH_1, &msg)) {
            break;
        }

        /* MIT 模式：反馈帧 ID = 电机 ID；按 ID 分发到对应电机槽位 */
        for (motor_num_t i = MOTOR_1; i < MOTOR_NUM; i++) {
            if (msg.id == (uint32_t)s_motor[i].id) {
                dm4310_fbdata(&s_motor[i], msg.data);
                break;
            }
        }
    }
}

motor_t* srv_dm4310_ctrl_get_motor(void)
{
    return &s_motor[MOTOR_1];
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief DM4310 CAN 发送回调：桥接到 drv_can 队列发送（与驱动解耦）
 */
static void srv_dm4310_can_send(uint16_t id, const uint8_t* data, uint8_t len)
{
    drv_can_msg_t msg;
    msg.id = id;
    msg.is_extended = false;
    msg.dlc = len;
    if (len > 8U) {
        len = 8U;
    }
    memcpy(msg.data, data, len);

    (void)drv_can_send(DRV_CAN_CH_1, &msg);
}
