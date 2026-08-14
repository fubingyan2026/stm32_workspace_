/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.c
 * @brief   CAN 通信任务 — sw_timer 驱动 srv_can 处理 + 反馈上报
 */

#include "can_task.h"

#include "drv_can.h"
#include "log.h"
#include "srv_can.h"
#include "srv_ht_test_mode.h"
#include "srv_pa430_torque_test.h"
#include "sw_timer.h"

/* CAN1（FDCAN1）：苇熠(HT) 伺服执行器测试，按 srv_ht_test_mode.h 选择 temp/torque；
 * CAN2（FDCAN2）：PA430 (Motorevo) MIT 测试由 SRV_PA430_TORQUE_TEST_ENABLE 独立控制，
 * 两条总线并行运行，互不干扰 */
#if SRV_HT_TEST_MODE_TORQUE
#include "srv_ht_torque_test.h"
#define HT_TEST_INIT srv_ht_torque_test_init
#define HT_TEST_STEP srv_ht_torque_test_step
#else
#include "srv_ht_temp_test.h"
#define HT_TEST_INIT srv_ht_temp_test_init
#define HT_TEST_STEP srv_ht_temp_test_step
#endif

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS 5U
#define FB_INTERVAL_MS 100U
#define STATUS_INTERVAL_MS 500U

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static uint8_t s_fb_tick;
static uint8_t s_status_tick;

/* Private function prototypes -----------------------------------------------*/

static void can_timer_cb(void* user_data);
static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions --------------------------------------------------------*/

void can_task_init(void)
{
    drv_can_error_t err = drv_can_init();
    if (err != DRV_CAN_OK) {
        LOG_E("can_task", "drv_can_init failed: %d (FDCAN start error?)", (int)err);
        return; /* CAN 不可用，不启动周期任务 */
    }
    srv_can_init();
    HT_TEST_INIT(); /* CAN1：苇熠伺服执行器测试（模式见 srv_ht_test_mode.h，AUTO_START=1 自动启动） */
#if SRV_PA430_TORQUE_TEST_ENABLE
    srv_pa430_torque_test_init(); /* CAN2：PA430 MIT 测试（AUTO_START=1 自动启动） */
#endif

    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);
#if SRV_PA430_TORQUE_TEST_ENABLE
    drv_can_register_rx_callback(DRV_CAN_CH_2, can_rx_callback);
#endif

    const sw_timer_config_t cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = can_timer_cb,
    };
    sw_timer_init(&s_timer, &cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}

/* Private functions ---------------------------------------------------------*/

static void can_timer_cb(void* user_data)
{
    (void)user_data;

    drv_can_poll_status(DRV_CAN_CH_1); /* Bus-Off 恢复 + 错误状态告警（苇熠测试总线） */
#if SRV_PA430_TORQUE_TEST_ENABLE
    drv_can_poll_status(DRV_CAN_CH_2); /* Bus-Off 恢复 + 错误状态告警（PA430 伺服总线） */
#endif
    srv_can_process();
    HT_TEST_STEP(); /* CAN1：苇熠伺服执行器测试：扫描 + 循环驱动 */
#if SRV_PA430_TORQUE_TEST_ENABLE
    srv_pa430_torque_test_step(); /* CAN2：PA430 MIT 来回运动测试 */
#endif

    s_fb_tick++;
    s_status_tick++;

    // if (s_fb_tick >= (FB_INTERVAL_MS / TASK_PERIOD_MS)) {
    //     s_fb_tick = 0;
    //     srv_can_send_feedback();
    // }
    // if (s_status_tick >= (STATUS_INTERVAL_MS / TASK_PERIOD_MS)) {
    //     s_status_tick = 0;
    //     srv_can_send_status();
    // }
}

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    /* 按通道分发：CAN1 = 苇熠测试 + 旧 0x100 协议；CAN2 = PA430 反馈帧 */
    if (ch == DRV_CAN_CH_1) {
        srv_can_on_rx(msg);
    }
#if SRV_PA430_TORQUE_TEST_ENABLE
    else if (ch == DRV_CAN_CH_2) {
        srv_pa430_torque_test_on_rx(msg);
    }
#endif
}
