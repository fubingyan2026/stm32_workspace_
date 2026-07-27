/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    app_main.c
 * @brief   E1_PRO 系统主入口 — 统一创建所有 FreeRTOS 任务
 *
 * 被 Core/Src/freertos.c 的 MX_FREERTOS_Init() 调用（调度器启动前）。
 * 各 xxx_task_init() 内部通过 osThreadNew 创建独立线程。
 *
 * 需硬件就绪的外设（ADC/OCTOSPI）在独立的后台线程中延迟初始化，
 * 避免阻塞主线程启动 FreeRTOS 调度器。
 */

#include "app_main.h"

#include "cmsis_os2.h"

#include "can_task.h"
#include "daemon_task.h"
#include "drv_adc.h"
#include "drv_i2c.h"
#include "drv_octospi.h"
#include "drv_pwm.h"
#include "drv_rng.h"
#include "hal_flash.h"
#include "drv_spi.h"
#include "drv_systick.h"
#include "drv_uart.h"
#include "led_task.h"
#include "log_task.h"
#include "storage_task.h"
#include "usb_device.h"

/* Private function prototypes -----------------------------------------------*/

static void deferred_hw_init_task(void* argument);

/* Exported functions --------------------------------------------------------*/

void app_main(void)
{
    /* 系统节拍（延时/时间戳） */
    delay_init();

    /* UART 驱动公共初始化（USART1/2/3） */
    drv_uart_init();

    /* ====== 外设驱动初始化（安全、无需硬件响应的） ====== */

    /* PWM 输出（TIM1/2/3/12，6 通道，纯寄存器操作） */
    drv_pwm_init();

    /* 硬件随机数生成器 */
    drv_rng_init();

    /* SPI 总线（只设标志，不操作硬件） */
    drv_spi_init();

    /* I2C 总线（只设标志，不操作硬件） */
    drv_i2c_init();

    /* Flash 抽象层（编译时选型 -DHAL_FLASH_CHIP_STM32H7） */
    hal_flash_init();

    /* ====== 需硬件响应的外设（推迟到调度器启动后） ====== */
    /* ADC/OSPI/USB 等可能存在硬件未就绪时的超时等待 */
    const osThreadAttr_t deferred_attr = {
        .name       = "deferred_hw",
        .stack_size = 256 * 4,
        .priority   = osPriorityLow,
    };
    osThreadNew(deferred_hw_init_task, NULL, &deferred_attr);

    /* ====== 任务层初始化 ====== */

    /* 日志输出（UART DMA / SEGGER RTT 线程） */
    log_task_init();

    /* LED 状态指示（双 LED 呼吸线程，依赖 drv_pwm） */
    led_task_init();

    /* 守护进程监控（9 电机反馈超时看门狗线程） */
    daemon_task_init();

    /* CAN 通信（控制帧/反馈/状态上报线程） */
    can_task_init();

    /* 参数存储（ring_storage + hal_flash，Flash 持久化） */
    storage_task_init();
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 延迟硬件初始化任务（调度器启动后执行）
 *
 * 将 ADC、OCTOSPI 等需要硬件确认的初始化放在低优先级线程中，
 * 避免在调度器启动前阻塞。
 */
static void deferred_hw_init_task(void* argument)
{
    (void)argument;

    /* 等待其他任务先跑稳 */
    osDelay(100);

    /* ADC1 DMA 循环转换 */
    drv_adc_init();

    /* OCTOSPI2 外部 Flash */
    drv_ospi_init();

    /* USB 虚拟串口 */
    // MX_USB_DEVICE_Init();

    /* 初始化完成，挂起本线程（FreeRTOS 任务严禁返回） */
    for (;;) {
        osDelay(10000);
    }
}
