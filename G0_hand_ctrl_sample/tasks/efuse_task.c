/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    efuse_task.c
 * @brief   eFuse 故障保护任务 — sw_timer 周期扫描 srv_efuse_step
 */

#include "efuse_task.h"

#include "drv_efuse.h"
#include "log.h"
#include "srv_efuse.h"
#include "sw_timer.h"

/** @brief eFuse 状态扫描周期 (ms) */
#define EFUSE_TASK_PERIOD_MS (10U)

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void efuse_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void efuse_task_init(void)
{
    drv_efuse_init();
    srv_efuse_init();

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = efuse_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, EFUSE_TASK_PERIOD_MS, 0);

    LOG_I("efuse_task", "eFuse 故障保护任务初始化完成 (period=%ums)",
        (unsigned)EFUSE_TASK_PERIOD_MS);
}

/* Private functions ---------------------------------------------------------*/

static void efuse_timer_cb(void* user_data)
{
    (void)user_data;
    srv_efuse_step(EFUSE_TASK_PERIOD_MS);
}
