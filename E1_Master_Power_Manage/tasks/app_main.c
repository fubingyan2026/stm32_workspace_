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

#include "can_task.h"
#include "drv_systick.h"
#include "fan_task.h"
#include "led_task.h"
#include "log_task.h"
#include "power_task.h"
#include "sample_task.h"
#include "sw_timer.h"

int app_main(void)
{
    /* 系统节拍（延时/时间戳） */
    delay_init();

    /* 日志输出（UART DMA） */
    log_task_init();

    /* CAN 通信（需在 power_task 之前，注册 read_data 回调） */
    can_task_init();

    /* 风扇控制（温控调速 + 堵转检测） */
    fan_task_init();

    /* LED 状态指示 */
    led_task_init();

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
