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
 *  - RX 字节搬运/解析应答/TX 排空在 com_task_service()（主循环每轮迭代）高频执行，
 *    帧收齐后几乎立即应答，不受定时器周期限制（参考 E1_MASTER_POWER_CTU）
 *  - 定时器(1ms)仅驱动 parser 空闲超时 + 兜底排空 TX
 *  - 升级请求（0x06）：先发 ACK，等 TX 排空后写 boot 标志并复位
 * 控制帧（0x04）→ srv_pwr_ctrl 输出期望 + drv_pwm 补光亮度；
 * 清锁存帧（0x05）→ srv_pwr_ctrl_clear_latch。
 */

#include "com_task.h"

#include "app_status_report.h"
#include "boot_flash.h"
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

/** @brief 定时器周期：仅用于 parser 空闲超时 tick（RX/TX 已主循环高频服务） */
#define TASK_PERIOD_MS (1U)

/** @brief 单次从 dev_rs485 读取字节数 */
#define COM_READ_BUF_SIZE (32U)

/** @brief 升级会话空闲超时 (ms)：主机中途消失则自动退出会话，避免 App 卡在升级态不应答 */
#define COM_UPGRADE_IDLE_TIMEOUT_MS (10000U)

/** @brief 补光亮度上限（千分比 0~1000） */
#define FILL_DUTY_MAX (1000U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static uint32_t s_upgrade_last_ms; /**< 升级会话最近一次收到数据的时刻 (ms) */

/** @brief 本板 App 版本（0x07 查询上报，可按发布修改） */
#define COM_APP_FW_VERSION (0x001U)

/* Private function prototypes -----------------------------------------------*/

static void com_timer_cb(void* user_data);
static void com_send_frame(const uint8_t* data, uint32_t len);
static void com_apply_ctrl(const srv_com_slv_ctrl_t* ctrl);
static void com_reset_latch(void);
static void com_upgrade_request(void);
static void com_read_info(srv_com_slv_info_t* info);

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
        .info = com_read_info,
        .send_frame = com_send_frame,
    };
    srv_com_slv_init(&cfg);

    /* 升级控制：App 内直接下载到 B 槽（不跳转、不断电）；
       END 校验通过后写 metadata{flag=2}，下次重新上电由 Boot 提升 B→A 生效 */
    const srv_boot_ctrl_config_t boot_cfg = {
        .tx = com_send_frame,
        .dev_id = SRV_COM_SLV_DEV_ID,
    };
    srv_boot_ctrl_init(&boot_cfg);

    /* 定时器仅驱动 parser 空闲超时 + 兜底 TX 排空（低周期，不阻塞主循环 RX） */
    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = com_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_timer, &timer_cfg);
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);

    COM_TASK_LOG_I("RS485 任务初始化完成 (service 高频 + timer=%ums)",
        (unsigned)TASK_PERIOD_MS);
}

/**
 * @brief 主循环高频服务：搬运 RX → 解析应答 → 排空 TX
 * @note  由 app_main 每轮迭代调用；帧收齐后几乎立即应答。
 *        升级会话期间字节改由 srv_boot_ctrl（boot_session）解析；
 *        会话结束（END/ABORT）后自动恢复普通协议，App 不复位、继续运行。
 */
void com_task_service(void)
{
    /* 1. RX：读取字节喂入协议层（升级会话 → boot_session，否则 srv_com_slv） */
    uint8_t buf[COM_READ_BUF_SIZE];
    uint32_t n = dev_rs485_rx_available();
    while (n > 0U) {
        const uint32_t chunk = (n > (uint32_t)sizeof(buf)) ? (uint32_t)sizeof(buf) : n;
        const uint32_t rd = dev_rs485_rx_read(buf, chunk);
        if (rd == 0U) {
            break;
        }
        if (srv_boot_ctrl_is_upgrade_active()) {
            srv_boot_ctrl_feed(buf, rd); /* SELECT/START/DATA/END */
            s_upgrade_last_ms = millis();
        } else {
            srv_com_slv_rx_feed(buf, rd);
        }
        n -= rd;
    }

    /* 2. TX：排空底层帧队列 */
    dev_rs485_tx_flush();

    /* 3. 会话空闲超时：主机中途消失（如上位机崩溃/断线）时退出，恢复常规应答 */
    if (srv_boot_ctrl_is_upgrade_active()
        && (uint32_t)(millis() - s_upgrade_last_ms) > COM_UPGRADE_IDLE_TIMEOUT_MS) {
        COM_TASK_LOG_W("升级会话空闲超时，退出会话恢复常规运行");
        srv_boot_ctrl_abort_session();
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 周期回调：parser 空闲超时 tick + 兜底排空 TX
 * @note  RX 搬运/解析/应答由主循环 com_task_service() 高频执行，此处不做
 */
static void com_timer_cb(void* user_data)
{
    (void)user_data;

    /* 解析器空闲超时 tick */
    srv_com_slv_rx_tick();

    /* 兜底排空 TX（主循环 service 已每次迭代执行，此处仅保险） */
    dev_rs485_tx_flush();
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
 * @brief 主机控制命令应用回调（0x04 输出控制帧）
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
 * @brief 清除故障锁存回调（0x05 清除锁存命令）
 */
static void com_reset_latch(void)
{
    srv_pwr_ctrl_clear_latch();
}

/**
 * @brief 升级请求回调（0x06 升级请求）
 * @note  App 内直接进入升级会话：后续 485 字节由 srv_boot_ctrl 解析写入 B 槽，
 *        下载期间不跳转、不断电；END 通过后仅写 flag=2，重新上电才生效。
 */
static void com_upgrade_request(void)
{
    if (!srv_boot_ctrl_enter_upgrade()) {
        COM_TASK_LOG_E("进入 App 内升级会话失败");
        return;
    }
    s_upgrade_last_ms = millis();
}

/**
 * @brief 信息查询回调（0x07）：只读 metadata + 编译期 App 版本
 * @note  使用 srv_boot_ctrl_peek_metadata（不累加启动次数/不写 Flash）
 */
static void com_read_info(srv_com_slv_info_t* info)
{
    boot_metadata_t meta;

    info->app_version = COM_APP_FW_VERSION;
    info->flags = 0U;
    if (srv_boot_ctrl_peek_metadata(&meta)) {
        info->meta_version = meta.version;
        info->fw_size = meta.fw_size;
        info->fw_checksum = meta.fw_checksum;
        info->reboot_counts = (uint16_t)meta.reboot_counts;
        if (meta.magic == BOOT_METADATA_MAGIC) {
            info->flags |= 0x01U;
        }
        if (meta.upgrade_flag == 1U) {
            info->flags |= 0x02U;
        } else if (meta.upgrade_flag == 2U) {
            info->flags |= 0x04U;
        }
    }
}
