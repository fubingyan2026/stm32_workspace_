/*
 * Copyright (c) 2022 HPMicro
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    app_main.c
 * @brief   G1_Hand 灵巧手主入口
 *
 * 初始化硬件后进入主循环，运行 RS-485 DMA 收发任务。
 */

#if defined(E1_BUILD_BOOT)
#include "boot_task.h" /* 位于 ../public_layer/task（Boot 目标 include path 提供） */
#endif
#include "buzzer_task.h"
#include "can_task.h"
#include "drv_revision.h"
#include "drv_systick.h"
#include "fan_task.h"
#include "flash_task.h"
#include "led_task.h"
#include "log.h"
#include "log_task.h"
#include "power_task.h"
#include "sample_task.h"
#include "sw_timer.h"
#include "ws2812_task.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define APP_MAIN_LOG_ENABLE 1

#if APP_MAIN_LOG_ENABLE
#define APP_MAIN_LOG_E(...) LOG_E("app_main", __VA_ARGS__)
#define APP_MAIN_LOG_W(...) LOG_W("app_main", __VA_ARGS__)
#define APP_MAIN_LOG_I(...) LOG_I("app_main", __VA_ARGS__)
#define APP_MAIN_LOG_D(...) LOG_D("app_main", __VA_ARGS__)
#else
#define APP_MAIN_LOG_E(...) ((void)0)
#define APP_MAIN_LOG_W(...) ((void)0)
#define APP_MAIN_LOG_I(...) ((void)0)
#define APP_MAIN_LOG_D(...) ((void)0)
#endif

#if defined(E1_BUILD_BOOT)
/* ===== Boot 镜像入口（E1_BUILD_BOOT 编译宏切换，见根 CMakeLists） ========== */

int app_main(void)
{
    /* 系统节拍（延时/时间戳）——先于 log_init()，此处不可打印 */
    delay_init();

    /* 日志输出（UART DMA） */
    log_task_init();

    /* 状态指示：蓝色 LED 按升级状态闪烁（等待/传输/校验/重启） */
    led_task_init();

    APP_MAIN_LOG_I("==== E1_Master_Power_Manage Bootloader ====");

    /* 启动决策：校验通过则跳转 App 分区，否则进入 CAN 升级接收 */
    if (!boot_task_try_boot_app()) {
        boot_task_init();
    }

    /* 主循环：所有周期性任务均由 sw_timer 驱动 */
    for (;;) {
        sw_timer_tick(millis());
        sw_timer_task();
    }

    return 0;
}

#else
/* ===== App 镜像入口（现有主控电源固件） =================================== */

int app_main(void)
{
    /* 系统节拍（延时/时间戳）——先于 log_init()，此处不可打印 */
    delay_init();

    /* 日志输出（UART DMA） */
    log_task_init();

    /* 启动横幅：日志链路已就绪，此后方可打印 */
    APP_MAIN_LOG_I("==== E1_Master_Power_Manage 系统启动 ====");
    APP_MAIN_LOG_I("硬件版本: rev=%u (%s)",
        (unsigned)drv_revision_read(), drv_revision_name());

    /* Flash 存储（参数分区 + boot metadata；需在 log 之后、可能读参数的任务之前） */
    flash_task_init();

    /* CAN 通信（需在 power_task 之前，注册 read_data 回调） */
    can_task_init();

    /* 风扇控制（温控调速 + 堵转检测） */
    fan_task_init();

    /* LED 状态指示 */
    led_task_init();

    /* 蜂鸣器（复用 srv_signal 实例；需在 led_task 之后） */
    buzzer_task_init();

    /* WS2812B 灯带（彗星流光演示） */
    ws2812_task_init();

    /* ADC 采样（TIM + DMA + VREFINT 校准） */
    sample_task_init();

    /* 电源管理（GPIO 控制 + 上电时序） */
    power_task_init();

    /* 主循环：所有周期性任务均由 sw_timer 驱动 */
    for (;;) {
        sw_timer_tick(millis());
        sw_timer_task();
    }

    return 0;
}
#endif /* E1_BUILD_BOOT */
