/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    key_task.c
 * @brief   按键任务 — sw_timer 轮询 key_base 状态机
 *
 * key_base 需在主循环周期调用 key_base_task() 驱动消抖/连击/长按状态机。
 */

#include "key_task.h"

#include "drv_key.h"
#include "key_base.h"
#include "log.h"
#include "srv_key.h"
#include "sw_timer.h"

/* Private constants ---------------------------------------------------------*/

#define KEY_TASK_PERIOD_MS 10U

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void key_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void key_task_init(void)
{
    drv_key_init();
    srv_key_init();

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = key_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, KEY_TASK_PERIOD_MS, 0);

    LOG_I("key_task", "按键任务初始化完成 (period=%ums)", (unsigned)KEY_TASK_PERIOD_MS);
}

/* Private functions ---------------------------------------------------------*/

static void key_timer_cb(void* user_data)
{
    (void)user_data;
    key_base_task();
}
