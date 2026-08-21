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
#include "srv_boot_ctrl.h"
#include "srv_can_dual.h"
#include "srv_can_mst.h"
#include "srv_can_slv.h"
#include "srv_device_monitor.h"
#include "srv_pwr_det.h"
#include "srv_ws2812b.h"
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

/** @brief 从板存活探测间隔 (ms)：周期发 0x002，ACK 到达即在线 */
#define SLAVE_POLL_PERIOD_MS (100U)

/** @brief CAN TX/RX 日志限频窗口 (ms)：总线繁忙时防止刷屏 */
#define CAN_TASK_ERR_LOG_PERIOD_MS (1000U)

/** @brief 进 boot 命令帧（主机 → 板卡）：触发 App 置 upgrade_flag 并复位进入升级模式 */
#define BOOT_REQUEST_CAN_ID (0x003U)
#define BOOT_REQUEST_LEN (1U)
#define BOOT_REQUEST_MAGIC (0x01U)

/** @brief RGB 输出控制帧（主机 → 板卡，8 字节：每灯 4 字节 = index + RGB，一帧控 2 灯） */
#define CAN_RGB_CTRL_ID (0x004U)
#define CAN_RGB_CTRL_LEN (8U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static uint16_t s_report_ms;
static uint16_t s_slave_poll_ms;
static uint32_t s_tx_err_log_ts;
static uint32_t s_rx_log_ts;

/** @brief 当前从板控制状态（由外部通过 can_task_set_slave_ctrl 设置） */
static srv_can_slv_ctrl_t s_slave_ctrl;

/** @brief 收到 0x003 进 boot 命令标志（ISR 置位，主循环 can_timer_cb 消费） */
static volatile bool s_enter_boot_requested;

/** @brief 0x004 单灯控制数据（每灯 4 字节：索引 + RGB 亮度） */
typedef struct {
    uint8_t index; /**< LED 索引：0-31=通道1, 32-63=通道2 */
    uint8_t r; /**< 红亮度 */
    uint8_t g; /**< 绿亮度 */
    uint8_t b; /**< 蓝亮度 */
} can_rgb_pixel_t;

/** @brief 收到 0x004 RGB 控制帧快照（一帧两灯，ISR 仅存数据，主循环 can_timer_cb 应用） */
typedef struct {
    can_rgb_pixel_t led[2]; /**< 两个 LED 控制块 */
    bool valid; /**< 有待应用命令 */
} can_rgb_pending_t;

static can_rgb_pending_t s_rgb_pending;

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

    /* 设备在线监控（daemon 封装；心跳喂狗点见 can_rx_callback） */
    srv_device_monitor_init(NULL);

    /* 注册 CAN 接收回调 */
    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);

    s_report_ms = 0;
    s_slave_poll_ms = 0;

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

/* Private functions ---------------------------------------------------------*/

static void can_timer_cb(void* user_data)
{
    (void)user_data;

    /* 消费进 boot 请求（主循环上下文；request_boot 涉及 Flash 写 + 复位，不能放 ISR） */
    if (s_enter_boot_requested) {
        s_enter_boot_requested = false;
        CAN_TASK_LOG_W("收到 bootloader 请求,准备调转");

        if (srv_boot_ctrl_request_boot() != SRV_BOOT_CTRL_OK) {
            CAN_TASK_LOG_E("进入 bootloader 请求失败");
        }
    }

    /* 应用 0x004 RGB 控制命令（主循环上下文，避免 ISR 内驱动 SPI DMA；一帧控 2 灯） */
    if (s_rgb_pending.valid) {
        s_rgb_pending.valid = false;
        int err = srv_ws2812b_set_pixel(s_rgb_pending.led[0].index,
            s_rgb_pending.led[0].r, s_rgb_pending.led[0].g, s_rgb_pending.led[0].b);
        if (err == 0) {
            err = srv_ws2812b_set_pixel(s_rgb_pending.led[1].index,
                s_rgb_pending.led[1].r, s_rgb_pending.led[1].g, s_rgb_pending.led[1].b);
        }
        if (err != 0) {
            CAN_TASK_LOG_W("RGB 控制应用失败: idx=%u",
                (unsigned)s_rgb_pending.led[0].index);
        }
    }

    srv_can_mst_task();
    srv_can_slv_task();
    srv_device_monitor_step();

    /* 周期触发主机上报 */
    s_report_ms += TASK_PERIOD_MS;
    if (s_report_ms >= REPORT_INTERVAL_MS) {
        s_report_ms = 0;
        srv_can_mst_request(0x00); /* 仅发送 0x001 状态帧，电池帧按需由主机触发 */
    }

    /* 周期从板存活探测：发送 0x002，ACK 到达即喂狗判在线 */
    s_slave_poll_ms += TASK_PERIOD_MS;
    if (s_slave_poll_ms >= SLAVE_POLL_PERIOD_MS) {
        s_slave_poll_ms = 0;
        srv_can_slv_request();
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

    /* 进 boot 命令（0x003, 载荷首字节 0x01）：仅置标志，由主循环 can_timer_cb 消费。
     * dlc 放宽为 >=1（≥ BOOT_REQUEST_LEN）：上位机可能按 1 字节或 8 字节填充发送，
     * 只要首字节为 magic 即触发。 */
    if (msg->id == BOOT_REQUEST_CAN_ID && msg->dlc >= BOOT_REQUEST_LEN
        && msg->data[0] == BOOT_REQUEST_MAGIC) {
        s_enter_boot_requested = true;
        return;
    }

    /* 主机控制指令解析（0x001, len=3） */
    if (msg->id == 0x001 && msg->dlc == 3) {
        srv_can_mst_process_rx(msg->data, msg->dlc);
        return;
    }

    /* RGB 输出控制帧（0x004, len=8，一帧控 2 灯，每灯 4 字节）：ISR 仅快照，主循环 can_timer_cb 应用 */
    if (msg->id == CAN_RGB_CTRL_ID && msg->dlc == CAN_RGB_CTRL_LEN) {
        s_rgb_pending.led[0].index = msg->data[0];
        s_rgb_pending.led[0].r = msg->data[1];
        s_rgb_pending.led[0].g = msg->data[2];
        s_rgb_pending.led[0].b = msg->data[3];
        s_rgb_pending.led[1].index = msg->data[4];
        s_rgb_pending.led[1].r = msg->data[5];
        s_rgb_pending.led[1].g = msg->data[6];
        s_rgb_pending.led[1].b = msg->data[7];
        s_rgb_pending.valid = true;
        return;
    }

    /* 设备在线喂狗（ISR 安全：daemon_reload 仅时间戳更新） */
    if (msg->id == SRV_CAN_SLV_ID_CTRL && msg->dlc == SRV_CAN_SLV_ACK_LEN) {
        srv_device_monitor_feed(SRV_DEVICE_SLAVER);
    } else if (msg->id == SRV_CAN_DUAL_ID_CORE || msg->id == SRV_CAN_DUAL_ID_INFO
        || msg->id == SRV_CAN_DUAL_ID_FAULT) {
        srv_device_monitor_feed(SRV_DEVICE_DUAL);
    }

    /* 从板控制 ACK 处理（0x002） */
    srv_can_slv_process_rx(msg->id, msg->data, msg->dlc);

    /* 双电池上报解析（0x200/0x201/0x202） */
    srv_can_dual_process_rx(msg->id, msg->data, msg->dlc);
}
