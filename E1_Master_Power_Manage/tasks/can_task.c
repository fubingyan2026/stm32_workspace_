/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.c
 * @brief   CAN 通信任务 — 主机上报 + 从板控制 + RX 接收
 */

#include "can_task.h"
#include "app_status_report.h"

#include "drv_can.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_can_dual.h"
#include "srv_can_mst.h"
#include "srv_can_slv.h"
#include "srv_pwr_det.h"
#include "sw_timer.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define CAN_TASK_LOG_ENABLE 1

#if CAN_TASK_LOG_ENABLE
#define CAN_TASK_LOG_E(...) LOG_E("can_task", __VA_ARGS__)
#define CAN_TASK_LOG_W(...) LOG_W("can_task", __VA_ARGS__)
#define CAN_TASK_LOG_I(...) LOG_I("can_task", __VA_ARGS__)
#define CAN_TASK_LOG_D(...) LOG_D("can_task", __VA_ARGS__)
#else
#define CAN_TASK_LOG_E(...) ((void)0)
#define CAN_TASK_LOG_W(...) ((void)0)
#define CAN_TASK_LOG_I(...) ((void)0)
#define CAN_TASK_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS (10U)
#define REPORT_INTERVAL_MS (100U)

/** @brief CAN TX/RX 日志限频窗口 (ms)：总线繁忙时防止刷屏 */
#define CAN_TASK_ERR_LOG_PERIOD_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static uint16_t s_report_ms;
static uint32_t s_tx_err_log_ts;
static uint32_t s_rx_log_ts;

/** @brief 当前从板控制状态（由外部通过 can_task_set_slave_ctrl 设置） */
static srv_can_slv_ctrl_t s_slave_ctrl;

/* Private function prototypes -----------------------------------------------*/

static void can_timer_cb(void* user_data);

static bool can_send_frame(uint16_t can_id, const uint8_t* data, uint8_t len);
static void can_read_slave_ctrl(srv_can_slv_ctrl_t* ctrl);
static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions --------------------------------------------------------*/

void can_task_init(void)
{
    const drv_can_error_t can_err = drv_can_init();
    if (can_err != DRV_CAN_OK) {
        CAN_TASK_LOG_E("CAN 驱动初始化失败 (err=%d)", (int)can_err);
    }

    memset(&s_slave_ctrl, 0, sizeof(s_slave_ctrl));

    srv_pwr_det_init();

    /* 主机上报服务（read_data 由应用层 app_status_report 聚合填充） */
    const srv_can_mst_config_t master_cfg = {
        .read_data = app_status_report_fill,
        .send_frame = can_send_frame,
    };
    srv_can_mst_init(&master_cfg);

    /* 从板控制服务 */
    const srv_can_slv_config_t slaver_cfg = {
        .send_frame = can_send_frame,
        .get_ctrl = can_read_slave_ctrl,
    };
    srv_can_slv_init(&slaver_cfg);

    /* 双电池协议解析 */
    const srv_can_dual_config_t dual_cfg = { .send_frame = can_send_frame };
    srv_can_dual_init(&dual_cfg);

    /* 注册 CAN 接收回调 */
    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);

    s_report_ms = 0;

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = can_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);

    CAN_TASK_LOG_I("CAN 任务初始化完成 (period=%ums, report=%ums)",
        (unsigned)TASK_PERIOD_MS, (unsigned)REPORT_INTERVAL_MS);
}

void can_task_tick(void)
{
    srv_can_mst_task();
    srv_can_slv_task();
}

void can_task_request(uint8_t feedback_select)
{
    srv_can_mst_request(feedback_select);
}

void can_task_set_slave_ctrl(bool hsd_12v_on, bool reserved_channel)
{
    s_slave_ctrl.hsd1_12v_on = hsd_12v_on;
    s_slave_ctrl.reserved_channel = reserved_channel;

    CAN_TASK_LOG_I("从板控制更新: hsd1_12v=%d reserved=%d",
        (int)hsd_12v_on, (int)reserved_channel);
    srv_can_slv_request();
}

/* Private functions ---------------------------------------------------------*/

static void can_timer_cb(void* user_data)
{
    (void)user_data;

    srv_can_mst_task();
    srv_can_slv_task();

    /* 周期触发主机上报 */
    s_report_ms += TASK_PERIOD_MS;
    if (s_report_ms >= REPORT_INTERVAL_MS) {
        s_report_ms = 0;
        srv_can_mst_request(0x00); /* 仅发送 0x001 状态帧，电池帧按需由主机触发 */
    }
}

static bool can_send_frame(uint16_t can_id, const uint8_t* data, uint8_t len)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1)) {
        const uint32_t now_ms = millis();
        if ((uint32_t)(now_ms - s_tx_err_log_ts) >= CAN_TASK_ERR_LOG_PERIOD_MS) {
            s_tx_err_log_ts = now_ms;
            CAN_TASK_LOG_W("CAN 发送忙, 帧丢弃待重试: id=0x%03X len=%u",
                (unsigned)can_id, (unsigned)len);
        }
        return false;
    }

    drv_can_msg_t msg;
    msg.id = can_id;
    msg.is_extended = false;
    msg.dlc = len;
    memcpy(msg.data, data, len);

    return drv_can_send(DRV_CAN_CH_1, &msg) == DRV_CAN_OK;
}

static void can_read_slave_ctrl(srv_can_slv_ctrl_t* ctrl)
{
    if (!ctrl)
        return;
    *ctrl = s_slave_ctrl;
}

/* --- CAN RX 回调 --- */

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    (void)ch;

    if (!msg)
        return;

    /* RX 事件日志（限频 1s，防止总线繁忙时刷屏；ISR 上下文，仅 kfifo 入队） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_rx_log_ts) >= CAN_TASK_ERR_LOG_PERIOD_MS) {
        s_rx_log_ts = now_ms;
        CAN_TASK_LOG_D("CAN RX: id=0x%03X dlc=%u", (unsigned)msg->id, (unsigned)msg->dlc);
    }

    /* 主机控制指令解析（0x001, len=7） */
    if (msg->id == 0x001 && msg->dlc == 7) {
        srv_can_mst_process_rx(msg->data, msg->dlc);
        return;
    }

    /* 从板控制 ACK 处理（0x002） */
    srv_can_slv_process_rx(msg->id, msg->data, msg->dlc);

    /* 双电池上报解析（0x200/0x201/0x202） */
    srv_can_dual_process_rx(msg->id, msg->data, msg->dlc);
}
