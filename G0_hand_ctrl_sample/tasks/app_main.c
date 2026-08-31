/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    app_main.c
 * @brief   G0 遥操作手套主入口
 *
 * 初始化硬件后进入主循环，运行 sw_timer 协作式调度。
 */

#include "app_main.h"

#include "app_rgb_status.h"
#include "app_uart_interact.h"
#include "can_task.h"
#include "efuse_task.h"
#include "drv_log_uart.h"
#include "drv_systick.h"
#include "drv_uart.h"
#include "key_task.h"
#include "led_task.h"
#include "log.h"
#include "log_task.h"
#include "sample_task.h"
#include "sw_timer.h"
#include "uart_cmd_task.h"

int app_main(void)
{
    /* 系统节拍（延时/时间戳） */
    delay_init();

    /* 通用串口（USART1，DMA + IDLE 接收） */
    drv_uart_init();

    /* 日志输出（USART2 DMA，log_task 内部完成 log_init + drv_log_uart_init） */
    log_task_init();

    /* CAN 通信 */
    can_task_init();

    /* LED 状态指示（TIM1_CH1 呼吸，依赖 drv_pwm） */
    led_task_init();

    /* ADC 采样（VIN/V_IMON） */
    sample_task_init();

    /* 按键轮询（KEY1/KEY2） */
    key_task_init();

    /* UART 命令收发（USART2，协议帧解析/打包） */
    uart_cmd_task_init();

    /* eFuse 24V 故障保护（周期扫描） */
    efuse_task_init();

    LOG_I("app_main", "==== G0_Hand 系统启动 ====");

    /* 主循环：sw_timer 驱动全部周期任务 */
    for (;;) {
        sw_timer_tick(millis());
        sw_timer_task();
    }

    return 0;
}
