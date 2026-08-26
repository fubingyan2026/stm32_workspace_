/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.c
 * @brief   CAN 通信任务 — 1ms sw_timer 驱动电机/传感器/主机协议服务
 *
 * CAN1（FDCAN1）测试模块选择：由 service/srv_motor_test_select.h 的 SRV_MOTOR_TEST_SELECT
 * 统一决定（HT_TORQUE=苇熠位置往复 / HT_TEMP=苇熠速度 / TONGZHI=良志ODrive /
 * JUXIE=橘虾 CAN FD MIT，默认）。
 * CAN2（FDCAN2）测试模块选择：由 SRV_MOTOR_TEST_SELECT_CAN2 统一决定
 * （HT_CAN2=苇熠 CAN2 版 / PA430=Motorevo / MZ=Mz 扭矩传感器，默认）。
 *
 * 接收分发统一走 srv_can_bus：srv_can_bus_init() 在 drv_can 注册每通道唯一的
 * 分发回调，can_task 将各通道绑定到对应模块的 on_rx（JUXIE/MZ 直连服务，
 * 旧测试模块经适配器转发，语义不变）。换 CAN 通道只需改 srv_can_bus_bind 参数。
 */

#include "can_task.h"

#include "drv_can.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_can.h"
#include "srv_can_bus.h"
#include "srv_ht_can2_torque_test.h"
#include "srv_ht_temp_test.h"
#include "srv_ht_torque_test.h"
#include "srv_juxie_motor.h"
#include "srv_motor_test_select.h"
#include "srv_mz_sensor.h"
#include "srv_pa430_torque_test.h"
#include "srv_tongzhi_torque_test.h"
#include "srv_uart_host.h"
#include "sw_timer.h"

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;

/** @brief CAN1 总线句柄（JUXIE 模式绑定电机服务） */
static srv_can_bus_t s_can1_bus;

/** @brief CAN2 总线句柄（MZ 模式绑定扭矩传感器服务） */
static srv_can_bus_t s_can2_bus;

/* --- CAN1（FDCAN1）模块接线 -------------------------------------------------- */
#if SRV_MOTOR_TEST_IS_TONGZHI
#define CAN1_TEST_USE_SRV_CAN 0
static void can1_test_init(void)
{
    srv_tongzhi_torque_test_init();
}
static void can1_test_step(void)
{
    srv_tongzhi_torque_test_step();
}
static void can1_bus_rx(const drv_can_msg_t* msg, void* ctx)
{
    (void)ctx;
    (void)srv_tongzhi_torque_test_on_rx(msg);
}
#elif SRV_MOTOR_TEST_IS_HT_TORQUE
#define CAN1_TEST_USE_SRV_CAN 1
static void can1_test_init(void)
{
    srv_ht_torque_test_init();
}
static void can1_test_step(void)
{
    srv_ht_torque_test_step();
}
static void can1_bus_rx(const drv_can_msg_t* msg, void* ctx)
{
    (void)ctx;
    (void)srv_can_on_rx(msg); /* srv_can 内部再按选择转发给对应 HT 模块 */
}
#elif SRV_MOTOR_TEST_IS_HT_TEMP
#define CAN1_TEST_USE_SRV_CAN 1
static void can1_test_init(void)
{
    srv_ht_temp_test_init();
}
static void can1_test_step(void)
{
    srv_ht_temp_test_step();
}
static void can1_bus_rx(const drv_can_msg_t* msg, void* ctx)
{
    (void)ctx;
    (void)srv_can_on_rx(msg);
}
#elif SRV_MOTOR_TEST_IS_JUXIE
#define CAN1_TEST_USE_SRV_CAN 0
static void can1_test_init(void)
{
    srv_juxie_motor_init(&s_can1_bus);
}
static void can1_test_step(void)
{
    srv_juxie_motor_step();
}
static void can1_bus_rx(const drv_can_msg_t* msg, void* ctx)
{
    (void)ctx;
    srv_juxie_motor_on_rx(msg, NULL);
}
#else
#error "SRV_MOTOR_TEST_SELECT 值无效"
#endif

/* --- CAN2（FDCAN2）模块接线 -------------------------------------------------- */
#if SRV_MOTOR_TEST_IS_HT_CAN2
static void can2_test_init(void)
{
    srv_ht_can2_torque_test_init();
}
static void can2_test_step(void)
{
    srv_ht_can2_torque_test_step();
}
static void can2_bus_rx(const drv_can_msg_t* msg, void* ctx)
{
    (void)ctx;
    (void)srv_ht_can2_torque_test_on_rx(msg);
}
#elif SRV_MOTOR_TEST_IS_PA430
static void can2_test_init(void)
{
    srv_pa430_torque_test_init();
}
static void can2_test_step(void)
{
    srv_pa430_torque_test_step();
}
static void can2_bus_rx(const drv_can_msg_t* msg, void* ctx)
{
    (void)ctx;
    (void)srv_pa430_torque_test_on_rx(msg);
}
#elif SRV_MOTOR_TEST_IS_MZ
static void can2_test_init(void)
{
    srv_mz_sensor_init(&s_can2_bus);
}
static void can2_test_step(void)
{
    srv_mz_sensor_step();
}
static void can2_bus_rx(const drv_can_msg_t* msg, void* ctx)
{
    (void)ctx;
    srv_mz_sensor_on_rx(msg, NULL);
}
#else
#error "SRV_MOTOR_TEST_SELECT_CAN2 值无效"
#endif

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS 1U /* 1ms：总线状态轮询 + UART 主机协议（高速控制移到主循环 can_task_fast_step） */

/* Private function prototypes -----------------------------------------------*/

static void can_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void can_task_fast_step(void)
{
    /* 主循环全速调用：电机控制、传感器轮询与 UART 主机协议周期最大化。
     * 橘虾 MIT 发送受 TX FIFO 背压自限；Mz 轮询受应答门控自限；
     * UART 应答 TX 队列随 DMA 空闲逐帧发送（1M 下可达 ~5000 帧/s）。 */
    can1_test_step();
    can2_test_step();
    srv_uart_host_step();
}

void can_task_init(void)
{
    delay_ms(2000);
    drv_can_error_t err = drv_can_init();
    if (err != DRV_CAN_OK) {
        LOG_E("can_task", "drv_can_init failed: %d (FDCAN start error?)", (int)err);
        return; /* CAN 不可用，不启动周期任务 */
    }

    /* 统一接收分发：srv_can_bus 占用每通道唯一的 drv_can 回调槽位 */
    srv_can_bus_init();
    srv_can_bus_bind(&s_can1_bus, DRV_CAN_CH_1, can1_bus_rx, NULL);
    srv_can_bus_bind(&s_can2_bus, DRV_CAN_CH_2, can2_bus_rx, NULL);

#if CAN1_TEST_USE_SRV_CAN
    srv_can_init(); /* 旧 0x100 上位机协议：仅苇熠模式使用 */
#endif
    can1_test_init(); /* CAN1：苇熠/良志测试 或 橘虾 MIT 电机（见顶部接线宏） */
    can2_test_init(); /* CAN2：苇熠 CAN2 版/PA430 或 Mz 扭矩传感器（见顶部接线宏） */

    srv_uart_host_init(); /* USART1 主机二进制定长帧协议 */

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

    drv_can_poll_status(DRV_CAN_CH_1); /* Bus-Off 恢复 + 错误状态告警 */
    drv_can_poll_status(DRV_CAN_CH_2);

#if CAN1_TEST_USE_SRV_CAN
    srv_can_process(); /* 旧 0x100 上位机协议：仅苇熠模式使用 */
#endif
}
