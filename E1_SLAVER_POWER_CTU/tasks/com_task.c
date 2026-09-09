/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    com_task.c
 * @brief   RS485 通信任务（E1_SLAVER_POWER_CTU）
 *
 * 主机查询应答式 485 从站：任务层负责传输搬运，协议帧解析/打包全部由
 * service 层 srv_com_slv（protocol_parser/protocol_packer）完成。
 *  - 每周期：dev_rs485 读字节 → srv_com_slv_rx_feed 喂数据（内部解析并应答）
 *  - srv_com_slv_rx_tick() 驱动解析器空闲超时
 *  - dev_rs485_tx_flush() 排空底层发送队列
 * 控制帧（0x10）→ srv_pwr_ctrl 输出期望 + drv_pwm 补光亮度；
 * 清锁存帧（0x11）→ srv_pwr_ctrl_clear_latch；
 * 升级帧（0x1F）→ srv_boot_ctrl 写共享 metadata upgrade_flag=1 → 复位进 Boot。
 */

#include "com_task.h"

#include "app_status_report.h"
#include "dev_rs485.h"
#include "drv_pwm.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_boot_ctrl.h"
#include "srv_com_slv.h"
#include "srv_pwr_ctrl.h"
#include "srv_pwr_det.h"
#include "sw_timer.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define COM_TASK_LOG_ENABLE 1

#if COM_TASK_LOG_ENABLE
#define COM_TASK_LOG_E(...) LOG_E("com_task", __VA_ARGS__)
#define COM_TASK_LOG_W(...) LOG_W("com_task", __VA_ARGS__)
#define COM_TASK_LOG_I(...) LOG_I("com_task", __VA_ARGS__)
#define COM_TASK_LOG_D(...) LOG_D("com_task", __VA_ARGS__)
#else
#define COM_TASK_LOG_E(...) ((void)0)
#define COM_TASK_LOG_W(...) ((void)0)
#define COM_TASK_LOG_I(...) ((void)0)
#define COM_TASK_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define TASK_PERIOD_MS (10U)

/** @brief 单次从 dev_rs485 读取字节数 */
#define COM_READ_BUF_SIZE (32U)

/** @brief 补光亮度上限（千分比 0~1000） */
#define FILL_DUTY_MAX (1000U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static bool s_upgrade_pending; /**< 升级请求待处理（ACK 发出后执行跳转） */

/* Private function prototypes -----------------------------------------------*/

static void com_timer_cb(void* user_data);
static void com_send_frame(const uint8_t* data, uint32_t len);
static void com_apply_ctrl(const srv_com_slv_ctrl_t* ctrl);
static void com_reset_latch(void);
static void com_upgrade_request(void);

/* Exported functions --------------------------------------------------------*/

void com_task_init(void)
{
    /* RS485 驱动初始化（底层 drv_uart + DE/RE 方向控制，默认接收方向） */
    if (dev_rs485_init() != DEV_RS485_OK) {
        COM_TASK_LOG_E("RS485 驱动初始化失败");
    }

    /* PWM 驱动初始化（补光 DIM + 状态灯共用 TIM4；幂等，led_task 会再次调用） */
    drv_pwm_init();

    /* 电源状态检测服务（初始化 drv_status PGOOD 读取） */
    srv_pwr_det_init();

    /* 从机协议服务：read_data/ctrl/reset_latch/upgrade/send_frame 均由本任务接线 */
    const srv_com_slv_config_t cfg = {
        .read_data = app_status_report_fill,
        .ctrl = com_apply_ctrl,
        .reset_latch = com_reset_latch,
        .upgrade = com_upgrade_request,
        .send_frame = com_send_frame,
    };
    srv_com_slv_init(&cfg);

    s_upgrade_pending = false;

    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = com_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);

    COM_TASK_LOG_I("RS485 任务初始化完成 (period=%ums)", (unsigned)TASK_PERIOD_MS);
}

/* Private functions ---------------------------------------------------------*/

static void com_timer_cb(void* user_data)
{
    (void)user_data;

    /* 1. RX：读取字节喂入 srv_com_slv（内部 protocol_parser 解析 + 应答打包） */
    uint8_t buf[COM_READ_BUF_SIZE];
    uint32_t n = dev_rs485_rx_available();
    while (n > 0U) {
        const uint32_t chunk = (n > (uint32_t)sizeof(buf)) ? (uint32_t)sizeof(buf) : n;
        const uint32_t rd = dev_rs485_rx_read(buf, chunk);
        if (rd == 0U) {
            break;
        }
        srv_com_slv_rx_feed(buf, rd);
        n -= rd;
    }

    /* 2. 解析器空闲超时 tick */
    srv_com_slv_rx_tick();

    /* 3. TX：排空底层帧队列 */
    dev_rs485_tx_flush();

    /* 4. 升级请求：等 ACK 帧发完后写升级标志并复位（跳转 Boot 升级模式） */
    if (s_upgrade_pending && !dev_rs485_is_tx_busy()) {
        s_upgrade_pending = false;
        COM_TASK_LOG_I("升级请求：写 boot 标志并复位进入 Bootloader");
        if (srv_boot_ctrl_request_upgrade()) {
            drv_system_reset();
        } else {
            COM_TASK_LOG_E("升级标志写入失败，本次不复位");
        }
    }
}

/**
 * @brief 应答帧发送回调（srv_com_slv 打包完成后调用）
 */
static void com_send_frame(const uint8_t* data, uint32_t len)
{
    if (!data || len == 0U) {
        return;
    }

    const dev_rs485_error_t err = dev_rs485_send(data, len);
    if (err != DEV_RS485_OK) {
        COM_TASK_LOG_W("应答入队失败 (err=%d)", (int)err);
    }
}

/**
 * @brief 主机控制命令应用回调（0x10 输出控制帧）
 * @note  输出期望整帧覆盖 → srv_pwr_ctrl；补光亮度 → TIM4_CH3（PT4115 DIM）
 */
static void com_apply_ctrl(const srv_com_slv_ctrl_t* ctrl)
{
    if (!ctrl) {
        return;
    }

    COM_TASK_LOG_I("控制命令: output_mask=0x%02X fill_duty=%u",
        (unsigned)ctrl->output_mask, (unsigned)ctrl->fill_duty);

    srv_pwr_ctrl_request_outputs(ctrl->output_mask);

    uint16_t duty = ctrl->fill_duty;
    if (duty > FILL_DUTY_MAX) {
        duty = FILL_DUTY_MAX;
    }
    drv_pwm_set_duty(DRV_PWM_CH_FILL, duty);
}

/**
 * @brief 清除故障锁存回调（0x11 清除锁存命令）
 */
static void com_reset_latch(void)
{
    srv_pwr_ctrl_clear_latch();
}

/**
 * @brief 升级请求回调（0x1F 升级请求）
 * @note  先置标志等待 ACK 发出（见 com_timer_cb 第 4 步），再写 boot 标志并复位
 */
static void com_upgrade_request(void)
{
    s_upgrade_pending = true;
}
