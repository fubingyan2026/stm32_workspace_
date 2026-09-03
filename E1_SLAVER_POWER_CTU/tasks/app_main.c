/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    app_main.c
 * @brief   E1_SLAVER_POWER_CTU 副电源模块主入口
 *
 * 初始化硬件后进入主循环，由 sw_timer 驱动全部周期任务。
 * 通信：RS485 主机查询应答式（USART3）；日志：USART1。
 */

#include "app_main.h"
#include "com_task.h"
#include "drv_systick.h"
#include "led_task.h"
#include "log.h"
#include "log_task.h"
#include "power_task.h"
#include "sample_task.h"
#include "sw_timer.h"

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

int app_main(void)
{
    /* 系统节拍（延时/时间戳）——先于 log_init()，此处不可打印 */
    delay_init();

    /* 日志输出（USART1 DMA） */
    log_task_init();

    /* 启动横幅：日志链路已就绪，此后方可打印 */
    APP_MAIN_LOG_I("==== E1_SLAVER_POWER_CTU 系统启动 ====");

    /* RS485 通信（dev_rs485 + 从机协议 + PWM/状态初始化） */
    com_task_init();

    /* LED 状态指示 */
    led_task_init();

    /* ADC 采样（DMA + VREFINT 校准） */
    sample_task_init();

    /* 电源管理（期望输出监督 FSM + 母线级故障保护策略） */
    power_task_init();

    /* 主循环：所有周期性任务均由 sw_timer 驱动 */
    for (;;) {
        sw_timer_tick(millis());
        sw_timer_task();
    }

    return 0;
}
