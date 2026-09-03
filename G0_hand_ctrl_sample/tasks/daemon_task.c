/*
 * Copyright (c) 2026 G0_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    daemon_task.c
 * @brief   守护进程任务 — sw_timer 周期驱动 daemon_task() 检测各服务在线状态
 */

#include "daemon_task.h"

#include "daemon.h"
#include "drv_systick.h"
#include "log.h"
#include "sw_timer.h"

/** @brief 守护进程扫描周期 (ms) */
#define DAEMON_TASK_PERIOD_MS (10U)

/** @brief 喂狗频率低频打印间隔 (ms) */
#define DAEMON_FREQ_PRINT_PERIOD_MS (1000U)

static sw_timer_t s_timer;
static uint32_t s_last_freq_print_ms;

/* Private function prototypes -----------------------------------------------*/

static void daemon_timer_cb(void* user_data);

static void daemon_print_frequencies(void);

/* Exported functions --------------------------------------------------------*/

void daemon_task_init(void)
{
    /* 初始化守护进程系统（全工程一次，各服务随后注册实例） */
    (void)daemon_init(millis);

    s_last_freq_print_ms = millis();

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = daemon_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, DAEMON_TASK_PERIOD_MS, 0);

    LOG_I("daemon_task", "守护进程任务初始化完成 (period=%ums)",
        (unsigned)DAEMON_TASK_PERIOD_MS);
}

/* Private functions ---------------------------------------------------------*/

static void daemon_timer_cb(void* user_data)
{
    (void)user_data;
    daemon_task();

    /* 低频打印各守护实例喂狗频率 */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_last_freq_print_ms) >= DAEMON_FREQ_PRINT_PERIOD_MS) {
        s_last_freq_print_ms = now_ms;
        daemon_print_frequencies();
    }
}

/**
 * @brief 打印各守护实例在线状态与喂狗频率（F1 禁浮点 printf，×10Hz 打印）
 */
static void daemon_print_frequencies(void)
{
    clist_head_t* head = daemon_get_head();
    if (head == NULL || clist_empty(head)) {
        return;
    }

    daemon_context_t* ctx;
    clist_for_each_entry(ctx, head, node)
    {
        const float freq_x10 = daemon_get_feed_frequency(ctx);
        LOG_D("daemon_task", "%s: %s (喂狗 %.3fHz)",
            daemon_get_name(ctx),
            daemon_is_online(ctx) ? "在线" : "掉线",
            freq_x10);
    }
}
