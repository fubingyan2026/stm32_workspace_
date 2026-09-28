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
#include "drv_systick.h"
#include "log.h"
#include "srv_can.h"
#include "srv_ht_can2_torque_test.h"
#include "srv_ht_temp_test.h"
#include "srv_ht_torque_test.h"
#include "srv_motor_test_select.h"
#include "srv_pa430_temp_test.h"
#include "srv_pa430_torque_test.h"
#include "srv_tongzhi_torque_test.h"
#include "srv_tz_temp_test.h"
#include "sw_timer.h"

/* CAN1/CAN2 测试模块选择：由 service/srv_motor_test_select.h 的**统一枚举**
 * 决定，两条通道各自从同一枚举取值（任意模块可放任意通道）：
 *   - SRV_MOTOR_TEST_SELECT      → CAN1(FDCAN1)
 *   - SRV_MOTOR_TEST_SELECT_CAN2 → CAN2(FDCAN2)
 * 接线约定：
 *   - 单例模块（HT_TORQUE/HT_TEMP/HT_CAN2/TONGZHI/PA430）内部收发走各自 XXX_CAN_CH
 *     宏（选在 CAN1→CH_1，否则→CH_2），RX 直连该模块 on_rx；
 *   - TZ_TEMP 为多实例服务：CAN1/CAN2 各自静态分配一个实例并设 config.can_ch，
 *     RX 走驱动 msg_fifo 队列（step() 内 drv_can_rx_pop 消费）；
 *   - 选 NONE 时该通道不运行任何测试（init/step/rx 均为空操作）。
 * 注：不再经 srv_can 转发旧 0x100 协议（旧协议保留在 srv_can 供他处使用）。 */

/* ---- CAN1(FDCAN1) 选择 ---- */
#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_TZ_TEMP)
static srv_tz_temp_test_inst_t s_tz_temp_inst1;
#define CAN1_TEST_INIT() do { \
    srv_tz_temp_test_config_default(&s_tz_temp_inst1.config); \
    s_tz_temp_inst1.config.can_ch = DRV_CAN_CH_1; \
    srv_tz_temp_test_init(&s_tz_temp_inst1, &s_tz_temp_inst1.config); \
} while (0)
#define CAN1_TEST_STEP() srv_tz_temp_test_step(&s_tz_temp_inst1)
#define CAN1_TEST_ON_RX(msg) ((void)(msg)) /* TZ_TEMP：RX 走驱动 msg_fifo，step() 内 drv_can_rx_pop 消费 */
#define CAN1_TEST_USE_SRV_CAN 0
#elif SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_TORQUE)
#define CAN1_TEST_INIT srv_ht_torque_test_init
#define CAN1_TEST_STEP srv_ht_torque_test_step
#define CAN1_TEST_ON_RX srv_ht_torque_test_on_rx
#define CAN1_TEST_USE_SRV_CAN 0
#elif SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_TEMP)
#define CAN1_TEST_INIT srv_ht_temp_test_init
#define CAN1_TEST_STEP srv_ht_temp_test_step
#define CAN1_TEST_ON_RX srv_ht_temp_test_on_rx
#define CAN1_TEST_USE_SRV_CAN 0
#elif SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_CAN2)
#define CAN1_TEST_INIT srv_ht_can2_torque_test_init
#define CAN1_TEST_STEP srv_ht_can2_torque_test_step
#define CAN1_TEST_ON_RX srv_ht_can2_torque_test_on_rx
#define CAN1_TEST_USE_SRV_CAN 0
#elif SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_TONGZHI)
#define CAN1_TEST_INIT srv_tongzhi_torque_test_init
#define CAN1_TEST_STEP srv_tongzhi_torque_test_step
#define CAN1_TEST_ON_RX srv_tongzhi_torque_test_on_rx
#define CAN1_TEST_USE_SRV_CAN 0
#elif SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_PA430)
#define CAN1_TEST_INIT srv_pa430_torque_test_init
#define CAN1_TEST_STEP srv_pa430_torque_test_step
#define CAN1_TEST_ON_RX srv_pa430_torque_test_on_rx
#define CAN1_TEST_USE_SRV_CAN 0
#elif SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_PA430_TEMP)
#define CAN1_TEST_INIT srv_pa430_temp_test_init
#define CAN1_TEST_STEP srv_pa430_temp_test_step
#define CAN1_TEST_ON_RX srv_pa430_temp_test_on_rx
#define CAN1_TEST_USE_SRV_CAN 0
#else /* SRV_MOTOR_TEST_NONE */
#define CAN1_TEST_INIT() ((void)0)
#define CAN1_TEST_STEP() ((void)0)
#define CAN1_TEST_ON_RX(msg) ((void)(msg))
#define CAN1_TEST_USE_SRV_CAN 0
#endif

/* ---- CAN2(FDCAN2) 选择 ---- */
#if SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_TZ_TEMP)
static srv_tz_temp_test_inst_t s_tz_temp_inst2;
#define CAN2_TEST_INIT() do { \
    srv_tz_temp_test_config_default(&s_tz_temp_inst2.config); \
    s_tz_temp_inst2.config.can_ch = DRV_CAN_CH_2; \
    s_tz_temp_inst2.config.pos_amp_turns = 8.5f; /* CAN2 实例往复半幅独立设置 */ \
    srv_tz_temp_test_init(&s_tz_temp_inst2, &s_tz_temp_inst2.config); \
} while (0)
#define CAN2_TEST_STEP() srv_tz_temp_test_step(&s_tz_temp_inst2)
#define CAN2_TEST_ON_RX(msg) ((void)(msg)) /* TZ_TEMP：RX 走驱动 msg_fifo，step() 内 drv_can_rx_pop 消费 */
#elif SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_HT_TORQUE)
#define CAN2_TEST_INIT srv_ht_torque_test_init
#define CAN2_TEST_STEP srv_ht_torque_test_step
#define CAN2_TEST_ON_RX srv_ht_torque_test_on_rx
#elif SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_HT_TEMP)
#define CAN2_TEST_INIT srv_ht_temp_test_init
#define CAN2_TEST_STEP srv_ht_temp_test_step
#define CAN2_TEST_ON_RX srv_ht_temp_test_on_rx
#elif SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_HT_CAN2)
#define CAN2_TEST_INIT srv_ht_can2_torque_test_init
#define CAN2_TEST_STEP srv_ht_can2_torque_test_step
#define CAN2_TEST_ON_RX srv_ht_can2_torque_test_on_rx
#elif SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_TONGZHI)
#define CAN2_TEST_INIT srv_tongzhi_torque_test_init
#define CAN2_TEST_STEP srv_tongzhi_torque_test_step
#define CAN2_TEST_ON_RX srv_tongzhi_torque_test_on_rx
#elif SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_PA430)
#define CAN2_TEST_INIT srv_pa430_torque_test_init
#define CAN2_TEST_STEP srv_pa430_torque_test_step
#define CAN2_TEST_ON_RX srv_pa430_torque_test_on_rx
#elif SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_PA430_TEMP)
#define CAN2_TEST_INIT srv_pa430_temp_test_init
#define CAN2_TEST_STEP srv_pa430_temp_test_step
#define CAN2_TEST_ON_RX srv_pa430_temp_test_on_rx
#else /* SRV_MOTOR_TEST_NONE */
#define CAN2_TEST_INIT() ((void)0)
#define CAN2_TEST_STEP() ((void)0)
#define CAN2_TEST_ON_RX(msg) ((void)(msg))
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
    delay_ms(1000);
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
