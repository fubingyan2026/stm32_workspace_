/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.c
 * @brief   LED 状态指示任务 — 蓝+红双 LED，灯效策略由 app_status_indicator 驱动
 */

#include "led_task.h"
#include "app_status_indicator.h"
#include "drv_led.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_signal.h"
#include "sw_timer.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define LED_TASK_LOG_ENABLE 1

#if LED_TASK_LOG_ENABLE
#define LED_TASK_LOG_E(...) LOG_E("led_task", __VA_ARGS__)
#define LED_TASK_LOG_W(...) LOG_W("led_task", __VA_ARGS__)
#define LED_TASK_LOG_I(...) LOG_I("led_task", __VA_ARGS__)
#define LED_TASK_LOG_D(...) LOG_D("led_task", __VA_ARGS__)
#else
#define LED_TASK_LOG_E(...) ((void)0)
#define LED_TASK_LOG_W(...) ((void)0)
#define LED_TASK_LOG_I(...) ((void)0)
#define LED_TASK_LOG_D(...) ((void)0)
#endif

/** @brief LED 刷新周期 (ms) */
#define LED_TASK_REFRESH_PERIOD_MS (10U)

/** @brief 蓝色 LED 呼吸参数 */
#define LED_BLUE_BREATH_CYCLE_MS (1500U)
#define LED_BLUE_BREATH_MIN_DUTY (2U)
#define LED_BLUE_BREATH_MAX_DUTY (1023U)

/** @brief 红色 LED 呼吸参数 */
#define LED_RED_BREATH_CYCLE_MS (1500U)
#define LED_RED_BREATH_MIN_DUTY (2U)
#define LED_RED_BREATH_MAX_DUTY (1023U)

static srv_signal_handle_t s_led_blue;
static srv_signal_handle_t s_led_red;
static sw_timer_t s_led_timer;

/* ── 蓝色 LED write_output 回调 ── */

static void led_blue_write_output(uint16_t value)
{
    drv_led_set_duty(DRV_LED_CH_BLUE, 1024 - value);
}

/* ── 红色 LED write_output 回调 ── */

static void led_red_write_output(uint16_t value)
{
    drv_led_set_duty(DRV_LED_CH_RED, 1024 - value);
}

/* ── sw_timer 回调 ── */

static void led_timer_cb(void* user_data)
{
    (void)user_data;

    /* 状态评估（内部 100ms 节流，仅等级变化下发命令） */
    app_status_indicator_step(LED_TASK_REFRESH_PERIOD_MS);

    /* 驱动所有 LED FSM 平滑步进（闪烁/呼吸） */
    srv_signal_task_refresh();
}

/* Exported functions --------------------------------------------------------*/

void led_task_init(void)
{
    drv_led_init();
    srv_signal_init(millis);

    /* ── 蓝色 LED: 呼吸模式 ── */
    srv_signal_config_t cfg_blue = {
        .name = "blue",
        .init_state = SRV_SIGNAL_STATE_BREATHING,
        .write_output = led_blue_write_output,
        .breath_cycle_ms = LED_BLUE_BREATH_CYCLE_MS,
        .breath_min_duty = LED_BLUE_BREATH_MIN_DUTY,
        .breath_max_duty = LED_BLUE_BREATH_MAX_DUTY,
    };
    srv_signal_register_static(&cfg_blue, &s_led_blue);

    /* ── 红色 LED: 默认呼吸，灯效由 app_status_indicator 按状态驱动 ── */
    srv_signal_config_t cfg_red = {
        .name = "red",
        .init_state = SRV_SIGNAL_STATE_BREATHING,
        .write_output = led_red_write_output,
        .breath_cycle_ms = LED_RED_BREATH_CYCLE_MS,
        .breath_min_duty = LED_RED_BREATH_MIN_DUTY,
        .breath_max_duty = LED_RED_BREATH_MAX_DUTY,
    };
    srv_signal_register_static(&cfg_red, &s_led_red);

    /* 状态指示灯策略（显式注入 blue/red 句柄，需在注册之后） */
    app_status_indicator_init(&s_led_blue, &s_led_red);

    /* 启动 sw_timer 驱动 LED FSM */
    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = led_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_led_timer, &timer_cfg);
    sw_timer_start(&s_led_timer, LED_TASK_REFRESH_PERIOD_MS, 0);

    LED_TASK_LOG_I("LED 任务初始化完成: 蓝呼吸默认, 灯效策略已接 app_status_indicator (period=%ums)",
        (unsigned)LED_TASK_REFRESH_PERIOD_MS);
}
