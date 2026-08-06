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

#include "app_fault_policy.h"
#include "log.h"
#include "srv_pwr_ctrl.h"
#include "sw_timer.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define POWER_TASK_LOG_ENABLE 1

#if POWER_TASK_LOG_ENABLE
#define POWER_TASK_LOG_E(...) LOG_E("power_task", __VA_ARGS__)
#define POWER_TASK_LOG_W(...) LOG_W("power_task", __VA_ARGS__)
#define POWER_TASK_LOG_I(...) LOG_I("power_task", __VA_ARGS__)
#define POWER_TASK_LOG_D(...) LOG_D("power_task", __VA_ARGS__)
#else
#define POWER_TASK_LOG_E(...) ((void)0)
#define POWER_TASK_LOG_W(...) ((void)0)
#define POWER_TASK_LOG_I(...) ((void)0)
#define POWER_TASK_LOG_D(...) ((void)0)
#endif

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
    app_fault_policy_init();
    POWER_TASK_LOG_I("电源管理任务初始化完成 (period=%ums)", (unsigned)TASK_PERIOD_MS);

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = power_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}

/* Private functions ---------------------------------------------------------*/

static void power_timer_cb(void* user_data)
{
    (void)user_data;
    srv_pwr_ctrl_step(TASK_PERIOD_MS);
    app_fault_policy_step(TASK_PERIOD_MS);
}
