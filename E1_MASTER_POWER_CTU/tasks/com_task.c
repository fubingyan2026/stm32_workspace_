/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    com_task.c
 * @brief   RS485 通信任务（E1_MASTER_POWER_CTU）
 *
 * 主机查询应答式 485 从站：任务层负责传输搬运，协议帧解析/打包全部由
 * service 层 srv_com_mst（protocol_parser/protocol_packer）完成。
 *  - 每周期：dev_rs485 读字节 → srv_com_mst_rx_feed 喂数据（内部解析并应答）
 *  - srv_com_mst_rx_tick() 驱动解析器空闲超时
 *  - dev_rs485_tx_flush() 排空底层发送队列
 */

#include "com_task.h"

#include "app_fault_policy.h"
#include "app_status_report.h"
#include "drv_buzzer.h"
#include "dev_rs485.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_adc.h"
#include "boot_flash.h"
#include "srv_boot_ctrl.h"
#include "srv_com_mst.h"
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

#define TASK_PERIOD_MS (1U)

/** @brief 单次从 dev_rs485 读取字节数 */
#define COM_READ_BUF_SIZE (32U)

/* Private variables ---------------------------------------------------------*/

static sw_timer_t s_timer;
static bool s_upgrade_pending; /**< 升级请求待处理（ACK 发出后执行跳转） */

/** @brief 本板 App 版本（0x07 查询上报，可按发布修改） */
#define COM_APP_FW_VERSION (0x0100U)

/** @brief metadata 只读查询上下文（首次调用初始化；不写 Flash） */
static boot_flash_context_t s_boot_flash_ctx;

/* Private function prototypes -----------------------------------------------*/

static void com_timer_cb(void* user_data);
static void com_send_frame(const uint8_t* data, uint32_t len);
static void com_apply_ctrl(const srv_com_mst_ctrl_t* ctrl);
static void com_reset_latch(void);
static void com_upgrade_request(void);
static void com_read_info(srv_com_mst_info_t* info);
static void com_read_estop_redun(srv_pwr_det_estop_redun_t* redun);
static uint8_t com_rail_en_mask(void);

/* Exported functions --------------------------------------------------------*/

void com_task_init(void)
{
    /* RS485 驱动初始化（底层 drv_uart + DE/RE 方向控制，默认接收方向） */
    if (dev_rs485_init() != DEV_RS485_OK) {
        COM_TASK_LOG_E("RS485 驱动初始化失败");
    }

    /* 蜂鸣器 PWM 初始化（0x04 控制帧 buzzer_duty 由 ctrl 回调驱动） */
    drv_buzzer_init();

    /* 电源状态检测服务（初始化 drv_status；E-STOP 冗余/常开轨使能门控经回调注入，
     * 避免 service 层同层互引） */
    srv_pwr_det_init(com_read_estop_redun, com_rail_en_mask);

    /* 主机协议服务：read_data/ctrl/reset_latch/upgrade/send_frame 均由本任务接线 */
    const srv_com_mst_config_t cfg = {
        .read_data = app_status_report_fill,
        .ctrl = com_apply_ctrl,
        .reset_latch = com_reset_latch,
        .upgrade = com_upgrade_request,
        .info = com_read_info,
        .send_frame = com_send_frame,
    };
    srv_com_mst_init(&cfg);
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

    /* 解析器空闲超时 tick（RX 搬运/解析/应答已在主循环 com_task_service 高频执行） */
    srv_com_mst_rx_tick();

    /* 兜底排空 TX（主循环 service 已在每次迭代执行，此处仅保险） */
    dev_rs485_tx_flush();
}

/**
 * @brief 主循环高频服务：搬运 RX 字节 → 解析并应答 → 排空 TX 队列
 * @note  由 app_main 主循环每次迭代调用，应答延迟不再受 10ms 定时周期限制
 *        （帧收齐后几乎立即应答）
 */
void com_task_service(void)
{
    uint8_t buf[COM_READ_BUF_SIZE];
    uint32_t n = dev_rs485_rx_available();
    while (n > 0U) {
        const uint32_t chunk = (n > (uint32_t)sizeof(buf)) ? (uint32_t)sizeof(buf) : n;
        const uint32_t rd = dev_rs485_rx_read(buf, chunk);
        if (rd == 0U) {
            break;
        }
        srv_com_mst_rx_feed(buf, rd);
        n -= rd;
    }

    dev_rs485_tx_flush();

    /* 升级请求：等 ACK 帧发完后写升级标志并复位（跳转 Boot 升级模式） */
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
 * @brief 应答帧发送回调（srv_com_mst 打包完成后调用）
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
 * @brief E-STOP 冗余数据读取接线（供 srv_pwr_det 注入）
 * @note  task 层聚合 srv_adc（CD4051B 双冗余节点），保持 service 层之间无直接依赖
 */
static void com_read_estop_redun(srv_pwr_det_estop_redun_t* redun)
{
    redun->valid = srv_adc_estop_valid();
    redun->closed_mask = redun->valid ? srv_adc_estop_closed_mask() : 0U;
}

/**
 * @brief 常开轨使能掩码接线（供 srv_pwr_det 注入，按轨使能门控 PGD 异常监测）
 * @note  返回 bit0=LM5060/VIN_DC-DC、bit1=24V、bit2=AUX 的使能位
 */
static uint8_t com_rail_en_mask(void)
{
    const srv_pwr_ctrl_state_t st = srv_pwr_ctrl_get_state();

    uint8_t mask = 0;
    if (st.vin_en) {
        mask |= (uint8_t)(1U << 0);
    }
    if (st.dc24v_en) {
        mask |= (uint8_t)(1U << 1);
    }
    if (st.aux_en) {
        mask |= (uint8_t)(1U << 2);
    }
    return mask;
}

/**
 * @brief 主机控制命令应用回调（0x04 控制帧）
 */
static void com_apply_ctrl(const srv_com_mst_ctrl_t* ctrl)
{
    if (!ctrl) {
        return;
    }
    COM_TASK_LOG_I("控制命令: buzzer_duty=%u", (unsigned)ctrl->buzzer_duty);
    drv_buzzer_set(ctrl->buzzer_duty);
}

/**
 * @brief 清除故障锁存回调（0x05 清除故障锁存命令）
 * @note  等价于急停释放沿的自动解锁（app_fault_policy），清除后按当前条件自动重试使能
 */
static void com_reset_latch(void)
{
    app_fault_policy_reset();
}

/**
 * @brief 升级请求回调（0x06 升级请求）
 * @note  先置标志等待 ACK 发出（见 com_task_service 尾部），再写 boot 标志并复位
 */
static void com_upgrade_request(void)
{
    s_upgrade_pending = true;
}

/**
 * @brief 信息查询回调（0x07）：只读 metadata + 编译期 App 版本
 * @note  使用 boot_flash_peek_metadata（不累加启动次数/不写 Flash）
 */
static void com_read_info(srv_com_mst_info_t* info)
{
    boot_metadata_t meta;

    info->app_version = COM_APP_FW_VERSION;
    info->flags = 0U;
    if (boot_flash_peek_metadata(&s_boot_flash_ctx, &meta) == BOOT_FLASH_OK) {
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
