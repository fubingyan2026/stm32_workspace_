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
#include "srv_ht_can2_torque_test.h"
#include "srv_motor_test_select.h"
#include "srv_pa430_torque_test.h"
#include "srv_tongzhi_torque_test.h"
#include "sw_timer.h"

/* CAN1（FDCAN1）测试模块选择：由 service/srv_motor_test_select.h 的 SRV_MOTOR_TEST_SELECT
 * 统一决定（HT_TORQUE=苇熠位置往复 / HT_TEMP=苇熠速度 / TONGZHI=良志ODrive 位置往复）。
 *   - 苇熠模式：RX 经 srv_can_on_rx 路由（srv_can 内部再按选择转发给对应 HT 模块）；
 *   - 良志模式：CH_1 全部帧直连 srv_tongzhi_torque_test_on_rx，srv_can 不参与。
 * CAN2（FDCAN2）测试模块选择：苇熠 CAN2 版（srv_ht_can2_torque_test）优先，
 * PA430 (Motorevo) MIT 测试为后备；两者共用 FDCAN2 独立总线，同一时刻只激活一个，
 * 与 CAN1 上的测试并行运行、互不干扰 */
#if SRV_MOTOR_TEST_IS_TONGZHI
#define CAN1_TEST_INIT srv_tongzhi_torque_test_init
#define CAN1_TEST_STEP srv_tongzhi_torque_test_step
#define CAN1_TEST_ON_RX srv_tongzhi_torque_test_on_rx
#define CAN1_TEST_USE_SRV_CAN 0
#elif SRV_MOTOR_TEST_IS_HT_TORQUE
#include "srv_ht_torque_test.h"
#define CAN1_TEST_INIT srv_ht_torque_test_init
#define CAN1_TEST_STEP srv_ht_torque_test_step
#define CAN1_TEST_ON_RX srv_can_on_rx
#define CAN1_TEST_USE_SRV_CAN 1
#elif SRV_MOTOR_TEST_IS_HT_TEMP
#include "srv_ht_temp_test.h"
#define CAN1_TEST_INIT srv_ht_temp_test_init
#define CAN1_TEST_STEP srv_ht_temp_test_step
#define CAN1_TEST_ON_RX srv_can_on_rx
#define CAN1_TEST_USE_SRV_CAN 1
#else
#error "SRV_MOTOR_TEST_SELECT 值无效"
#endif

#if SRV_HT_CAN2_TORQUE_TEST_ENABLE
#define CAN2_TEST_INIT srv_ht_can2_torque_test_init
#define CAN2_TEST_STEP srv_ht_can2_torque_test_step
#define CAN2_TEST_ON_RX srv_ht_can2_torque_test_on_rx
#elif SRV_PA430_TORQUE_TEST_ENABLE
#define CAN2_TEST_INIT srv_pa430_torque_test_init
#define CAN2_TEST_STEP srv_pa430_torque_test_step
#define CAN2_TEST_ON_RX srv_pa430_torque_test_on_rx
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
#if CAN1_TEST_USE_SRV_CAN
    srv_can_init(); /* 旧 0x100 上位机协议：仅苇熠模式使用（良志接管时停用） */
#endif
    CAN1_TEST_INIT(); /* CAN1：苇熠伺服执行器测试或良志(ODrive)测试（见顶部接线宏） */
#if defined(CAN2_TEST_INIT)
    CAN2_TEST_INIT(); /* CAN2：苇熠 CAN2 版或 PA430 MIT 测试（见顶部接线宏） */
#endif

    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);
#if defined(CAN2_TEST_INIT)
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

    drv_can_poll_status(DRV_CAN_CH_1); /* Bus-Off 恢复 + 错误状态告警（苇熠/良志测试总线） */
#if defined(CAN2_TEST_INIT)
    drv_can_poll_status(DRV_CAN_CH_2); /* Bus-Off 恢复 + 错误状态告警（苇熠 CAN2 / PA430 伺服总线） */
#endif
#if CAN1_TEST_USE_SRV_CAN
    srv_can_process(); /* 旧 0x100 上位机协议：仅苇熠模式使用 */
#endif
    CAN1_TEST_STEP(); /* CAN1：苇熠/良志测试：扫描/往复驱动 */
#if defined(CAN2_TEST_INIT)
    CAN2_TEST_STEP(); /* CAN2：苇熠 CAN2 版/PA430：扫描/往复驱动 */
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
    /* 按通道分发：CAN1 = 良志(接管时)或苇熠测试 + 旧 0x100 协议；CAN2 = 苇熠 CAN2 版/PA430 反馈帧 */
    if (ch == DRV_CAN_CH_1) {
        CAN1_TEST_ON_RX(msg);
    }
#if defined(CAN2_TEST_INIT)
    else if (ch == DRV_CAN_CH_2) {
        CAN2_TEST_ON_RX(msg);
    }
#endif
}
