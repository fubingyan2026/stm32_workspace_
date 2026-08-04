/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    sample_task.c
 * @brief   ADC 采样任务 — sw_timer 驱动，薄封装
 */

#include "sample_task.h"

#include "log.h"
#include "srv_adc.h"
#include "sw_timer.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SAMPLE_TASK_LOG_ENABLE 1

#if SAMPLE_TASK_LOG_ENABLE
#define SAMPLE_TASK_LOG_E(...) LOG_E("sample_task", __VA_ARGS__)
#define SAMPLE_TASK_LOG_W(...) LOG_W("sample_task", __VA_ARGS__)
#define SAMPLE_TASK_LOG_I(...) LOG_I("sample_task", __VA_ARGS__)
#define SAMPLE_TASK_LOG_D(...) LOG_D("sample_task", __VA_ARGS__)
#else
#define SAMPLE_TASK_LOG_E(...) ((void)0)
#define SAMPLE_TASK_LOG_W(...) ((void)0)
#define SAMPLE_TASK_LOG_I(...) ((void)0)
#define SAMPLE_TASK_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define ADC_PERIOD_MS (10U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void sample_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void sample_task_init(void)
{
    srv_adc_init();
    SAMPLE_TASK_LOG_I("ADC 采样任务初始化完成 (period=%ums)", (unsigned)ADC_PERIOD_MS);

    /* NORMAL 优先级：回调在主循环 sw_timer_task() 中执行，避免采样换算占用 SysTick 中断 */
    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = sample_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, ADC_PERIOD_MS, 0);
}

bool sample_task_get_latest(srv_adc_data_t* sample)
{
    return srv_adc_get_latest(sample);
}

/* Private functions ---------------------------------------------------------*/

static void sample_timer_cb(void* user_data)
{
    (void)user_data;
    srv_adc_trigger(); /* 触发本周期 ADC 扫描 */
    srv_adc_step();    /* 换算上一周期原始快照（主循环上下文，含遥测日志） */
}
