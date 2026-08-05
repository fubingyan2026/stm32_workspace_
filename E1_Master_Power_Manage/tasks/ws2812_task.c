/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    ws2812_task.c
 * @brief   WS2812B 灯带任务 — sw_timer 驱动，薄封装
 */

#include "ws2812_task.h"

#include "log.h"
#include "srv_ws2812b.h"
#include "sw_timer.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define WS2812_TASK_LOG_ENABLE 1

#if WS2812_TASK_LOG_ENABLE
#define WS2812_TASK_LOG_E(...) LOG_E("ws2812_task", __VA_ARGS__)
#define WS2812_TASK_LOG_W(...) LOG_W("ws2812_task", __VA_ARGS__)
#define WS2812_TASK_LOG_I(...) LOG_I("ws2812_task", __VA_ARGS__)
#define WS2812_TASK_LOG_D(...) LOG_D("ws2812_task", __VA_ARGS__)
#else
#define WS2812_TASK_LOG_E(...) ((void)0)
#define WS2812_TASK_LOG_W(...) ((void)0)
#define WS2812_TASK_LOG_I(...) ((void)0)
#define WS2812_TASK_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS (10U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void ws2812_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void ws2812_task_init(void)
{
    if (srv_ws2812b_init() != 0) {
        WS2812_TASK_LOG_E("WS2812B 灯带服务初始化失败");
        return;
    }

    WS2812_TASK_LOG_I("WS2812B 任务初始化完成 (period=%ums)", (unsigned)TASK_PERIOD_MS);

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = ws2812_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}

/* Private functions ---------------------------------------------------------*/

static void ws2812_timer_cb(void* user_data)
{
    (void)user_data;
    srv_ws2812b_step(TASK_PERIOD_MS);
}
