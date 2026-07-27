/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    can_task.c
 * @brief   CAN 通信任务 — FreeRTOS 线程驱动 srv_can 处理 + 反馈上报
 */

#include "can_task.h"

#include "cmsis_os2.h"
#include "drv_can.h"
#include "log.h"
#include "srv_can.h"

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS     10U
#define FB_INTERVAL_MS    100U
#define STATUS_INTERVAL_MS 500U

#define TASK_STACK_SIZE    256U
#define TASK_PRIORITY      osPriorityNormal

/* Private variables ---------------------------------------------------------*/

static osThreadId_t s_task_handle;
static uint8_t s_fb_tick;
static uint8_t s_status_tick;

/* Private function prototypes -----------------------------------------------*/

static void can_task_entry(void* argument);
static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions --------------------------------------------------------*/

void can_task_init(void)
{
    drv_can_error_t err = drv_can_init();
    if (err != DRV_CAN_OK) {
        LOG_E("can_task", "drv_can_init failed: %d (FDCAN start error?)", (int)err);
        return;
    }
    srv_can_init();

    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);

    const osThreadAttr_t attr = {
        .name       = "can_task",
        .stack_size = TASK_STACK_SIZE * 4,
        .priority   = TASK_PRIORITY,
    };
    s_task_handle = osThreadNew(can_task_entry, NULL, &attr);
}

/* Private functions ---------------------------------------------------------*/

static void can_task_entry(void* argument)
{
    (void)argument;

    for (;;) {
        drv_can_poll_status(DRV_CAN_CH_1);
        srv_can_process();

        s_fb_tick++;
        s_status_tick++;

        if (s_fb_tick >= (FB_INTERVAL_MS / TASK_PERIOD_MS)) {
            s_fb_tick = 0;
            srv_can_send_feedback();
        }
        if (s_status_tick >= (STATUS_INTERVAL_MS / TASK_PERIOD_MS)) {
            s_status_tick = 0;
            srv_can_send_status();
        }

        osDelay(TASK_PERIOD_MS);
    }
}

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    (void)ch;
    srv_can_on_rx(msg);
}
