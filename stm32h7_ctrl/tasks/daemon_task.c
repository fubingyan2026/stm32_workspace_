/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    daemon_task.c
 * @brief   守护进程监控任务 — FreeRTOS 线程驱动，9 电机反馈超时看门狗
 *
 * 喂狗策略（轮询式，零侵入 service 层）：
 * 1ms 周期检查每个电机反馈时间戳 (time_fb / time_status) 是否推进，
 * 推进则 daemon_reload() 喂狗；随后 daemon_task() 做超时判定。
 * 掉线/恢复仅打日志告警，不干预控制。
 *
 * 依赖：behavior_task_init() 已完成 srv_motor_init()（9 电机句柄已注册）。
 */

/* Includes ------------------------------------------------------------------*/
#include "daemon_task.h"

#include "cmsis_os2.h"
#include "daemon.h"
#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DAEMON_LOG_ENABLE 1

#if DAEMON_LOG_ENABLE
#define DAEMON_LOG_E(...) LOG_E("daemon", __VA_ARGS__)
#define DAEMON_LOG_W(...) LOG_W("daemon", __VA_ARGS__)
#define DAEMON_LOG_I(...) LOG_I("daemon", __VA_ARGS__)
#define DAEMON_LOG_D(...) LOG_D("daemon", __VA_ARGS__)
#else
#define DAEMON_LOG_E(...) ((void)0)
#define DAEMON_LOG_W(...) ((void)0)
#define DAEMON_LOG_I(...) ((void)0)
#define DAEMON_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 检查/派发周期 (FreeRTOS 1ms tick = 1 tick) */
#define DAEMON_TASK_PERIOD_MS (10U)

/** @brief 反馈超时阈值 */
#define MOTOR_FEED_TIMEOUT_MS (200U)

/** @brief 上电宁静期：覆盖 ENABLE + ~1s 编码器校准 */
#define MOTOR_INIT_WAIT_MS (3000U)

#define TASK_STACK_SIZE 256U
#define TASK_PRIORITY   osPriorityAboveNormal

/* Private variables ---------------------------------------------------------*/

static osThreadId_t s_task_handle;

/* Private function prototypes ------------------------------------------------*/

static void daemon_task_entry(void* argument);

/* Exported functions --------------------------------------------------------*/

void daemon_task_init(void)
{
    /* daemon 中间件首个使用者，负责初始化（ms 时基 = millis） */
    daemon_error_t err = daemon_init(millis);
    if (DAEMON_IS_ERR(err)) {
        DAEMON_LOG_E("daemon_init failed: %d", (int)err);
        return;
    }

    const osThreadAttr_t attr = {
        .name       = "daemon_task",
        .stack_size = TASK_STACK_SIZE * 4,
        .priority   = TASK_PRIORITY,
    };
    s_task_handle = osThreadNew(daemon_task_entry, NULL, &attr);

    DAEMON_LOG_I("daemon task init ok, %u motors monitored", (unsigned)daemon_get_count());
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 电机在线状态跳变回调（掉线与恢复共用，方向由 daemon_is_online 区分）
 * @note  预留接口，待注册到 daemon 中间件后启用
 */
__attribute__((unused))
static void motor_offline_cb(void* owner_ptr)
{
    (void)owner_ptr;
}

/**
 * @brief FreeRTOS 任务入口 — 1ms 周期看门狗派发
 */
static void daemon_task_entry(void* argument)
{
    (void)argument;

    for (;;) {
        daemon_task();
        osDelay(DAEMON_TASK_PERIOD_MS);
    }
}
