/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    power_task.c
 * @brief   电源管理任务 — 1ms sw_timer 驱动输出监督 FSM + 母线级故障保护策略 + PGOOD 遥测
 */

#include "power_task.h"

#include "app_fault_policy.h"
#include "log.h"
#include "srv_adc.h"
#include "srv_pwr_ctrl.h"
#include "srv_pwr_det.h"
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

/** @brief 电源服务步进周期 (ms)：1ms 控制周期 */
#define TASK_PERIOD_MS (1U)

/** @brief 故障保护策略分频周期 (ms)：每 10 拍（10ms）执行一次 */
#define FAULT_POLICY_PERIOD_MS (10U)

/** @brief 分频系数 = FAULT_POLICY_PERIOD_MS / TASK_PERIOD_MS */
#define POWER_SUB_DIV (FAULT_POLICY_PERIOD_MS / TASK_PERIOD_MS)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static uint8_t s_sub_tick; /**< 故障策略分频计数 (0..POWER_SUB_DIV-1) */

/* Private function prototypes -----------------------------------------------*/

static void power_timer_cb(void* user_data);
static void power_read_voltage(srv_pwr_voltage_t* volt);

/* Exported functions --------------------------------------------------------*/

void power_task_init(void)
{
    /* srv_pwr_ctrl 自包含：内部完成 drv_power 初始化 + 输出监督 FSM */
    srv_pwr_ctrl_init();

    /* 母线电压读取回调：从 srv_adc 最新采样填充（AUX/MOTOR/LSD 门控与监控依赖） */
    srv_pwr_ctrl_set_voltage_cb(power_read_voltage);

    app_fault_policy_init();
    POWER_TASK_LOG_I("电源管理任务初始化完成 (步进周期=%ums, 故障策略分频=%ums)",
        (unsigned)TASK_PERIOD_MS, (unsigned)FAULT_POLICY_PERIOD_MS);

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

    /* 输出监督 FSM：每拍 1ms 步进 */
    srv_pwr_ctrl_step(TASK_PERIOD_MS);

    /* 母线级故障保护策略 + PGOOD 遥测：每 10 拍（10ms）执行一次 */
    if (++s_sub_tick >= POWER_SUB_DIV) {
        s_sub_tick = 0;
        srv_pwr_det_status_t det;
        srv_pwr_det_read(&det);
        app_fault_policy_step(FAULT_POLICY_PERIOD_MS);
    }
}

/**
 * @brief 母线电压读取接线（task 层聚合 srv_adc，service 层之间不直接互调）
 */
static void power_read_voltage(srv_pwr_voltage_t* volt)
{
    if (!volt) {
        return;
    }

    srv_adc_data_t adc;
    if (!srv_adc_get_latest(&adc)) {
        volt->valid = false;
        return;
    }

    volt->valid = true;
    volt->aux_mv = adc.aux_mv;
    volt->motor_mv = adc.motor_mv;
    volt->lsd1_mv = adc.lsd1_mv;
    volt->lsd2_mv = adc.lsd2_mv;
}
