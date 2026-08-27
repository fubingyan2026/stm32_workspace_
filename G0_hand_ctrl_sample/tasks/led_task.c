/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.c
 * @brief   LED 状态指示任务 — TIM1_CH1 呼吸模式
 * @note    使用共享 srv_signal（public_layer/service）驱动，输出走 drv_pwm。
 */

#include "led_task.h"

#include "drv_pwm.h"
#include "drv_systick.h"
#include "srv_signal.h"
#include "sw_timer.h"

/** @brief LED 刷新周期 (ms) */
#define LED_TASK_REFRESH_PERIOD_MS (10U)

/** @brief 蓝色 LED 呼吸参数 */
#define LED_BREATH_CYCLE_MS (2000U)
#define LED_BREATH_MIN_DUTY (0U)
#define LED_BREATH_MAX_DUTY (1023U)

static srv_signal_handle_t s_led;
static sw_timer_t s_led_timer;

/* ── LED write_output 回调 ── */

static void led_write_pin(uint16_t value)
{
    (void)drv_pwm_set_duty(DRV_PWM_TIM1_CH1, value);
}

/* ── sw_timer 回调 ── */

static void led_timer_cb(void* user_data)
{
    (void)user_data;
    srv_signal_task_refresh();
}

/* Exported functions --------------------------------------------------------*/

void led_task_init(void)
{
    drv_pwm_init();

    srv_signal_init(millis);

    srv_signal_config_t cfg = {
        .name = "led",
        .init_state = SRV_SIGNAL_STATE_BREATHING,
        .write_output = led_write_pin,
        .breath_cycle_ms = LED_BREATH_CYCLE_MS,
        .breath_min_duty = LED_BREATH_MIN_DUTY,
        .breath_max_duty = LED_BREATH_MAX_DUTY,
    };
    (void)srv_signal_register_static(&cfg, &s_led);

    /* 启动 sw_timer 驱动 LED FSM */
    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = led_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_led_timer, &timer_cfg);
    sw_timer_start(&s_led_timer, LED_TASK_REFRESH_PERIOD_MS, 0);
}
