/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    power_task.c
 * @brief   电源管理任务 — sw_timer 驱动，薄封装
 */

#include "power_task.h"

#include "srv_pwr_ctrl.h"
#include "sw_timer.h"

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS (10U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void power_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void power_task_init(void)
{
    srv_pwr_ctrl_init();

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = power_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}

void power_task_request_on(void)
{
    srv_pwr_ctrl_request_on();
}

void power_task_emergency_off(void)
{
    srv_pwr_ctrl_emergency_off();
}

bool power_task_is_powered_on(void)
{
    return srv_pwr_ctrl_is_powered_on();
}

/* Private functions ---------------------------------------------------------*/

static void power_timer_cb(void* user_data)
{
    (void)user_data;
    srv_pwr_ctrl_step(TASK_PERIOD_MS);
}
