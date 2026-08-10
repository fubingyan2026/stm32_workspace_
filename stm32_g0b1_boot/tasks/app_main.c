/*
 * Copyright (c) 2022 HPMicro
 * Copyright (c) 2026 E1 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    app_main.c
 * @brief   stm32_g0_boot 主入口
 *
 * 初始化硬件后进入主循环：启动决策（跳 App 或进 Boot）由 boot_task 驱动。
 */

#include "boot_task.h"
#include "drv_systick.h"
#include "log_task.h"
#include "sw_timer.h"

int app_main(void)
{
    /* 系统节拍（延时/时间戳） */
    delay_init();
    /* 日志输出（UART DMA） */
    log_task_init();

    /* 启动决策：有有效 App 则跳转，否则进入 bootloader */
    if (!boot_task_try_boot_app())
    {
        boot_task_init();
    }

    /* 主循环：所有周期性任务均由 sw_timer 驱动 */
    while (1) {
        sw_timer_tick(millis());
        sw_timer_task();
    }

    return 0;
}
