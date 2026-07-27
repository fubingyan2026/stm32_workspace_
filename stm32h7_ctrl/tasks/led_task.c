/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.c
 * @brief   LED 状态指示任务 — FreeRTOS 线程驱动蓝色+红色双 LED 呼吸 (TIM1 PWM)
 */

#include "led_task.h"

#include "cmsis_os2.h"
#include "drv_pwm.h"
#include "drv_systick.h"
#include "srv_led.h"

/** @brief LED 刷新周期 (ms) */
#define LED_TASK_REFRESH_PERIOD_MS (10U)

/** @brief 蓝色 LED 呼吸参数 */
#define LED_BLUE_BREATH_CYCLE_MS (2000U)
#define LED_BLUE_BREATH_MIN_DUTY (0U)
#define LED_BLUE_BREATH_MAX_DUTY (1023U)

/** @brief 红色 LED 呼吸参数 */
#define LED_RED_BREATH_CYCLE_MS (3000U)
#define LED_RED_BREATH_MIN_DUTY (0U)
#define LED_RED_BREATH_MAX_DUTY (1023U)

#define TASK_STACK_SIZE        128U
#define TASK_PRIORITY          osPriorityBelowNormal

static srv_led_handle_t s_led_blue;
static srv_led_handle_t s_led_red;
static osThreadId_t s_task_handle;

/* ── 蓝色 LED write_pin 回调 (TIM1_CH3, PE13) ── */

static void led_blue_write_pin(uint16_t value)
{
    // drv_pwm_set_duty(DRV_PWM_CH_TIM1_CH3, value);
}

/* ── 红色 LED write_pin 回调 (TIM12_CH2, PB15) ── */

static void led_red_write_pin(uint16_t value)
{
    // drv_pwm_set_duty(DRV_PWM_CH_TIM12_CH2, value);
}

/* ── FreeRTOS 任务入口 ── */

static void led_task_entry(void* argument)
{
    (void)argument;

    for (;;) {
        srv_led_task_refresh();
        osDelay(LED_TASK_REFRESH_PERIOD_MS);
    }
}

/* Exported functions --------------------------------------------------------*/

void led_task_init(void)
{
    srv_led_init(millis);

    /* ── 蓝色 LED: 呼吸模式 ── */
    srv_led_config_t cfg_blue = {
        .name = "blue",
        .init_state = SRV_LED_STATE_BREATHING,
        .write_pin = led_blue_write_pin,
        .breath_cycle_ms = LED_BLUE_BREATH_CYCLE_MS,
        .breath_min_duty = LED_BLUE_BREATH_MIN_DUTY,
        .breath_max_duty = LED_BLUE_BREATH_MAX_DUTY,
    };
    srv_led_register_static(&cfg_blue, &s_led_blue);

    /* ── 红色 LED: 呼吸模式 ── */
    srv_led_config_t cfg_red = {
        .name = "red",
        .init_state = SRV_LED_STATE_BREATHING,
        .write_pin = led_red_write_pin,
        .breath_cycle_ms = LED_RED_BREATH_CYCLE_MS,
        .breath_min_duty = LED_RED_BREATH_MIN_DUTY,
        .breath_max_duty = LED_RED_BREATH_MAX_DUTY,
    };
    srv_led_register_static(&cfg_red, &s_led_red);

    const osThreadAttr_t attr = {
        .name       = "led_task",
        .stack_size = TASK_STACK_SIZE * 4,
        .priority   = TASK_PRIORITY,
    };
    s_task_handle = osThreadNew(led_task_entry, NULL, &attr);
}
