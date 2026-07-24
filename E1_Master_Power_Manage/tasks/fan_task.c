/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    fan_task.c
 * @brief   风扇控制任务 — sw_timer 驱动，薄封装
 */

#include "fan_task.h"

#include "srv_adc.h"
#include "srv_fan_ctrl.h"
#include "sw_timer.h"

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS (100U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;

/* Private function prototypes -----------------------------------------------*/

static void fan_timer_cb(void* user_data);
static int16_t fan_read_temp(uint8_t id);

/* Exported functions --------------------------------------------------------*/

void fan_task_init(void)
{
    srv_fan_ctrl_init(fan_read_temp);

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = fan_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}

/* Private functions ---------------------------------------------------------*/

static void fan_timer_cb(void* user_data)
{
    (void)user_data;
    srv_fan_ctrl_step(TASK_PERIOD_MS);
}

/**
 * @brief 温度读取回调：按风扇编号返回对应的 NTC 温度
 * @param id 风扇编号 (0=FAN0→NTC1, 1=FAN1→NTC2)
 * @return 温度 (0.01°C)，读取失败返回 0
 */
static int16_t fan_read_temp(uint8_t id)
{
    srv_adc_data_t s;
    if (!srv_adc_get_latest(&s)) {
        return 0;
    }
    return (id == 0) ? s.ntc1_temp_x100 : s.ntc2_temp_x100;
}
