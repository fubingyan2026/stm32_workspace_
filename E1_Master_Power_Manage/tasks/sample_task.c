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

#include "srv_adc.h"
#include "sw_timer.h"

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

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_HIGH,
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
    srv_adc_trigger();
}
