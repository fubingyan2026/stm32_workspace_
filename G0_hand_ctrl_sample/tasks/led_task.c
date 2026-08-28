/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.c
 * @brief   LED 状态任务 — sw_timer 驱动 app_rgb_status 刷新
 * @note    LED 实例注册/输出映射在应用层 app_rgb_status 中，
 *          task 层仅提供周期刷新（10ms 调 srv_signal_task_refresh）。
 */

#include "led_task.h"

#include "app_rgb_status.h"
#include "sw_timer.h"

/** @brief LED 刷新周期 (ms) */
#define LED_TASK_REFRESH_PERIOD_MS (10U)

static sw_timer_t s_led_timer;

/* ── sw_timer 回调 ── */

static void led_timer_cb(void* user_data)
{
    (void)user_data;
    app_rgb_status_step();
}

/* Exported functions --------------------------------------------------------*/

void led_task_init(void)
{
    app_rgb_status_init();

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = led_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_led_timer, &timer_cfg);
    sw_timer_start(&s_led_timer, LED_TASK_REFRESH_PERIOD_MS, 0);
}
