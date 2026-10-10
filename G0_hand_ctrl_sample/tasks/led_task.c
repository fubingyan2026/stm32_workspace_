/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.c
 * @brief   LED 状态任务 — sw_timer 驱动 app_rgb_status 刷新
 * @note    周期性（10ms）根据电机状态/按键计算并刷新三色 LED：
 *            - 电机离线或报错       → 只显示红色
 *            - 上电自检按键仍按下   → 只显示红色
 *            - KEY1 按下            → 只显示绿色
 *            - 其余（正常/故障清除） → 白色（R+G+B 全亮）
 *          LED 实例注册/输出映射在应用层 app_rgb_status 中。
 */

#include "led_task.h"

#include "app_rgb_status.h"
#include "drv_key.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_dm4310_ctrl.h"
#include "sw_timer.h"

/** @brief LED 刷新周期 (ms) */
#define LED_TASK_REFRESH_PERIOD_MS (10U)

static sw_timer_t s_led_timer;

/** @brief 上电自检异常标志：按键被按下时 LED 常红；松开后自动恢复正常 */
static bool s_boot_fault;

/** @brief 各路 LED 最近一次下发的状态（变化检测，避免每周期重复入队） */
static srv_signal_state_t s_last_state[APP_RGB_CH_NUM] = {
    SRV_SIGNAL_STATE_MAX, SRV_SIGNAL_STATE_MAX, SRV_SIGNAL_STATE_MAX,
};

/* ── 输出辅助 ── */

/** @brief 仅在状态变化时下发（未设置用 SRV_SIGNAL_STATE_MAX 作为哨兵） */
static void led_apply(app_rgb_channel_t ch, srv_signal_state_t state)
{
    if (s_last_state[ch] == state) {
        return;
    }
    if (app_rgb_status_set_state(ch, state)) {
        s_last_state[ch] = state;
    }
}

/** @brief 红色：仅红灯亮 */
static void led_output_red(void)
{
    led_apply(APP_RGB_CH_BLUE, SRV_SIGNAL_STATE_OFF);
    led_apply(APP_RGB_CH_GREEN, SRV_SIGNAL_STATE_OFF);
    led_apply(APP_RGB_CH_RED, SRV_SIGNAL_STATE_ON);
}

/** @brief 绿色：仅绿灯亮 */
static void led_output_green(void)
{
    led_apply(APP_RGB_CH_BLUE, SRV_SIGNAL_STATE_OFF);
    led_apply(APP_RGB_CH_GREEN, SRV_SIGNAL_STATE_ON);
    led_apply(APP_RGB_CH_RED, SRV_SIGNAL_STATE_OFF);
}

/** @brief 白色：R+G+B 全亮 */
static void led_output_white(void)
{
    led_apply(APP_RGB_CH_BLUE, SRV_SIGNAL_STATE_ON);
    led_apply(APP_RGB_CH_GREEN, SRV_SIGNAL_STATE_ON);
    led_apply(APP_RGB_CH_RED, SRV_SIGNAL_STATE_ON);
}

/** @brief 全灭 */
static void led_output_off(void)
{
    led_apply(APP_RGB_CH_BLUE, SRV_SIGNAL_STATE_OFF);
    led_apply(APP_RGB_CH_GREEN, SRV_SIGNAL_STATE_OFF);
    led_apply(APP_RGB_CH_RED, SRV_SIGNAL_STATE_OFF);
}

/* ── 状态计算 ── */

static void led_update(void)
{
    /* 0) 上电按键自检异常：优先、立即亮红（不受电机自检门控影响）；
          按键松开后清除并恢复正常 */
    if (s_boot_fault) {
        if (drv_key_is_pressed(DRV_KEY_CH_1) || drv_key_is_pressed(DRV_KEY_CH_2)) {
            led_output_red();
            return;
        }
        s_boot_fault = false;
    }

    /* 1) 上电等待电机自检完成前：保持熄灭，不显示状态色 */
    if (!srv_dm4310_ctrl_is_selfcheck_done()) {
        led_output_off();
        return;
    }

    /* 2) 电机离线或报错：只显示红色 */
    if (srv_dm4310_ctrl_is_fault()) {
        led_output_red();
        return;
    }

    /* 3) 正常：KEY1 按下绿色，否则白色；故障/离线恢复后回白色 */
    if (drv_key_is_pressed(DRV_KEY_CH_1)) {
        led_output_green();
    } else {
        led_output_white();
    }
}

/* ── sw_timer 回调 ── */

static void led_timer_cb(void* user_data)
{
    (void)user_data;

    led_update();
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

void led_task_indicate_boot_fault(void)
{
    s_boot_fault = true;
    led_output_red();
}
