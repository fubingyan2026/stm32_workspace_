/**
 * @file    srv_can_mst.c
 * @author  maximillian
 * @version V1.2.0
 * @date    2026-07-2
 * @brief   P_CAN 主机上报服务实现（msg_fifo 队列 + pending 重试）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_can_mst.h"

#include <stddef.h>
#include <string.h>

#include "drv_systick.h"
#include "log.h"
#include "public.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_CAN_MST_LOG_ENABLE 0

#if SRV_CAN_MST_LOG_ENABLE
#define SRV_CAN_MST_LOG_E(...) LOG_E("srv_can_mst", __VA_ARGS__)
#define SRV_CAN_MST_LOG_W(...) LOG_W("srv_can_mst", __VA_ARGS__)
#define SRV_CAN_MST_LOG_I(...) LOG_I("srv_can_mst", __VA_ARGS__)
#define SRV_CAN_MST_LOG_D(...) LOG_D("srv_can_mst", __VA_ARGS__)
#else
#define SRV_CAN_MST_LOG_E(...) ((void)0)
#define SRV_CAN_MST_LOG_W(...) ((void)0)
#define SRV_CAN_MST_LOG_I(...) ((void)0)
#define SRV_CAN_MST_LOG_D(...) ((void)0)
#endif

/** @brief TX 失败/上报请求日志限频窗口 (ms)：10ms 调用下防止刷屏 */
#define SRV_CAN_MST_ERR_LOG_PERIOD_MS (1000U)

/* Private constants ---------------------------------------------------------*/

#define CM_FIFO_BUF_SIZE (256U)

/* Private types -------------------------------------------------------------*/

typedef struct {
    uint16_t id;
    uint8_t data[8];
    uint8_t len;
} cm_frame_t;

/* 0x001 帧布局编译期断言：status 首成员即 8 字节整帧，其后无填充字节 */
_Static_assert(sizeof(srv_can_mst_status_frame_t) == 8U, "0x001 状态帧必须为 8 字节");
_Static_assert(offsetof(srv_can_mst_data_t, bat1) == 8U, "0x001 帧后不得有填充字节");

/* Private variables ---------------------------------------------------------*/

static srv_can_mst_config_t s_config;
static bool s_initialized;
static msg_fifo_t s_fifo;
static uint8_t s_fifo_buf[CM_FIFO_BUF_SIZE];
static cm_frame_t s_pending;
static bool s_pending_active;

/** @brief 最近一次主机指令 */
static srv_can_mst_cmd_t s_last_cmd;
static bool s_cmd_pending; /**< 主机发了新指令，等待回复 */

/** @brief 日志限频时间戳 */
static uint32_t s_tx_fail_log_ts;
static uint32_t s_req_log_ts;

/* Private function prototypes -----------------------------------------------*/

static void cm_enqueue(uint16_t id, const uint8_t* data, uint8_t len);
static void cm_build_0x001(const srv_can_mst_data_t* d);
static void cm_build_battery(const srv_can_mst_data_t* d, uint8_t fb);
static bool cm_check_status_frame_layout(void);

/* Exported functions --------------------------------------------------------*/

srv_can_mst_error_t srv_can_mst_init(const srv_can_mst_config_t* config)
{
    if (!config || !config->read_data || !config->send_frame) {
        return SRV_CAN_MST_ERROR_NULL_PTR;
    }

    if (s_initialized) {
        srv_can_mst_deinit();
    }

    /* 位域布局自检：位域分配由编译器/ABI 决定，更换工具链可能改变协议字节位；
     * 启动时校验一次，失败则拒绝初始化，避免发出错乱帧。 */
    if (!cm_check_status_frame_layout()) {
        SRV_CAN_MST_LOG_E("0x001 帧位域布局与协议不符，禁止初始化");
        return SRV_CAN_MST_ERROR_LAYOUT;
    }

    s_config = *config;

    msg_fifo_init(&s_fifo, s_fifo_buf, CM_FIFO_BUF_SIZE, sizeof(cm_frame_t));
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_active = false;
    s_initialized = true;

    SRV_CAN_MST_LOG_I("主机上报服务初始化完成 (FIFO=%uB)", (unsigned)CM_FIFO_BUF_SIZE);

    return SRV_CAN_MST_OK;
}

void srv_can_mst_deinit(void)
{
    msg_fifo_deinit(&s_fifo);
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_active = false;
    memset(&s_config, 0, sizeof(s_config));
    s_initialized = false;
}

bool srv_can_mst_is_initialized(void)
{
    return s_initialized;
}

void srv_can_mst_request(uint8_t feedback_select)
{
    if (!s_initialized || !msg_fifo_empty(&s_fifo) || s_pending_active) {
        return;
    }

    /* 上报请求日志（限频 1s） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_req_log_ts) >= SRV_CAN_MST_ERR_LOG_PERIOD_MS) {
        s_req_log_ts = now_ms;
        SRV_CAN_MST_LOG_D("主机上报请求: feedback_select=0x%02X", (unsigned)feedback_select);
    }

    srv_can_mst_data_t data;
    memset(&data, 0, sizeof(data));

    s_config.read_data(&data);

    cm_build_0x001(&data);
    cm_build_battery(&data, feedback_select);
}

void srv_can_mst_task(void)
{
    if (!s_initialized) {
        return;
    }

    /* 没有 pending 帧则从 FIFO 取下一帧 */
    if (!s_pending_active) {
        if (!msg_fifo_pop(&s_fifo, &s_pending)) {
            return;
        }
        s_pending_active = true;
    }

    /* 发送 pending 帧（成功则清除，失败下次重试） */
    if (s_config.send_frame(s_pending.id, s_pending.data, s_pending.len)) {
        s_pending_active = false;
    } else {
        /* 发送失败告警（限频 1s，10ms 调用下 CAN 忙时防止刷屏） */
        const uint32_t now_ms = millis();
        if ((uint32_t)(now_ms - s_tx_fail_log_ts) >= SRV_CAN_MST_ERR_LOG_PERIOD_MS) {
            s_tx_fail_log_ts = now_ms;
            SRV_CAN_MST_LOG_W("主机帧发送失败待重试: id=0x%03X len=%u (CAN忙)",
                (unsigned)s_pending.id, (unsigned)s_pending.len);
        }
    }

    /* 主机请求的回复帧全部发完 → 标记已完成 */
    if (!s_pending_active && msg_fifo_empty(&s_fifo)) {
        s_cmd_pending = false;
    }
}

void srv_can_mst_process_rx(const uint8_t* data, uint8_t len)
{
    if (!s_initialized || !data || len < 7)
        return;

    memset(&s_last_cmd, 0, sizeof(s_last_cmd));

    s_last_cmd.feedback_select = data[0];
    s_last_cmd.rgb_mode = data[1];
    s_last_cmd.rgb_color = ((uint32_t)data[2] << 16)
        | ((uint32_t)data[3] << 8)
        | ((uint32_t)data[4]);
    s_last_cmd.buzzer_duty = (data[5] <= 100U) ? data[5] : 100U;

    /* byte6: 4对 valid+value 控制位 */
    if (data[6] & (1U << 7))
        s_last_cmd.hsd1_12v_on = (data[6] >> 6) & 1;
    if (data[6] & (1U << 5))
        s_last_cmd.hsd2_12v_on = (data[6] >> 4) & 1;
    if (data[6] & (1U << 3))
        s_last_cmd.lsd1_24v_on = (data[6] >> 2) & 1;
    if (data[6] & (1U << 1))
        s_last_cmd.lsd2_24v_on = (data[6] >> 0) & 1;

    s_cmd_pending = true;

    /* 主机指令日志（配置变更，I 级） */
    SRV_CAN_MST_LOG_D("收到主机指令: fb=0x%02X rgb_mode=%u rgb=0x%06lX buzzer=%u hsd1_12v=%d hsd2_12v=%d lsd1_24v=%d lsd2_24v=%d",
        (unsigned)s_last_cmd.feedback_select,
        (unsigned)s_last_cmd.rgb_mode,
        (unsigned long)s_last_cmd.rgb_color,
        (unsigned)s_last_cmd.buzzer_duty,
        (int)s_last_cmd.hsd1_12v_on,
        (int)s_last_cmd.hsd2_12v_on,
        (int)s_last_cmd.lsd1_24v_on,
        (int)s_last_cmd.lsd2_24v_on);

    /* 按主机请求立即触发回复 */
    srv_can_mst_request(s_last_cmd.feedback_select);
}

const srv_can_mst_cmd_t* srv_can_mst_get_cmd(void)
{
    return &s_last_cmd;
}

/* Private functions ---------------------------------------------------------*/

static void cm_enqueue(uint16_t id, const uint8_t* data, uint8_t len)
{
    cm_frame_t frame;
    frame.id = id;
    frame.len = len;
    memcpy(frame.data, data, len);
    memset(frame.data + len, 0, sizeof(frame.data) - len); /* 尾部补零 */

    msg_fifo_push(&s_fifo, &frame);
}

/* --- 0x001 帧构建 --- */

static void cm_build_0x001(const srv_can_mst_data_t* d)
{
    /* 0x001 帧 = status(Byte0-7) 完整 8 字节，偏移由文件顶部 offsetof 静态断言保障，
     * 整段 memcpy 无需逐位搬移。 */
    cm_enqueue(SRV_CAN_MST_ID_STATUS, (const uint8_t*)d, 8);
}

static bool cm_check_status_frame_layout(void)
{
    /* 对 8 个字节各选一个代表位/字节置位，比对期望值。
     * 期望值源自协议文档位定义，与 GCC ARM (little-endian) 实测一致。 */
    srv_can_mst_status_frame_t f;
    memset(&f, 0, sizeof(f));
    f.bits.stop_key_state = 1U; /* byte0 bit0 */
    f.bits.device_online_bat1 = 1U; /* byte0 bit6 */
    f.bits.err_24v_user = 1U; /* byte1 bit7 */
    f.bits.err_dbr = 1U; /* byte2 bit7 */
    f.bits.byte3_fixed1 = 1U; /* byte3 bit7 */
    f.bits.err_ntc7 = 1U; /* byte4 bit7 */
    f.bits.seq_motor_fault = 1U; /* byte5 bit5 */
    f.bits.bat1_soc = 0xA5U; /* byte6 */
    f.bits.bat2_soc = 0x5AU; /* byte7 */
    const uint8_t expect[8] = { 0x41U, 0x80U, 0x80U, 0x80U, 0x80U, 0x20U, 0xA5U, 0x5AU };
    return memcmp(f.bytes, expect, sizeof(expect)) == 0;
}

/* --- 电池数据帧构建 (0x011-0x015) --- */

static void cm_build_battery(const srv_can_mst_data_t* d, uint8_t fb)
{
    uint8_t frame[8];

    /* 0x011 — 双电池容量 + 循环次数 + 充电标志（soc 已在 0x001 上报，不重复） */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_BASE) {
        frame[0] = (uint8_t)(d->bat1.capacity_mah >> 8U);
        frame[1] = (uint8_t)(d->bat1.capacity_mah >> 16U);
        frame[2] = (uint8_t)(d->bat2.capacity_mah >> 8U);
        frame[3] = (uint8_t)(d->bat2.capacity_mah >> 16U);
        frame[4] = (uint8_t)(d->bat1.cycle_count >> 0U);
        frame[5] = (uint8_t)(d->bat2.cycle_count >> 0U);
        frame[6] = (uint8_t)(d->bat1.charging ? (1U << 0) : 0)
            | (uint8_t)(d->bat2.charging ? (1U << 1) : 0);
        frame[7] = 0; /* 保留 */
        cm_enqueue(SRV_CAN_MST_ID_BAT_BASE, frame, 8);
    }

    /* 0x012 — 双电池电压 + 电流 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_VOLTAGE) {
        frame[0] = (uint8_t)(d->bat1.voltage_dv >> 0U);
        frame[1] = (uint8_t)(d->bat1.voltage_dv >> 8U);
        frame[2] = (uint8_t)(d->bat2.voltage_dv >> 0U);
        frame[3] = (uint8_t)(d->bat2.voltage_dv >> 8U);
        frame[4] = (uint8_t)(d->bat1.current_da >> 0U);
        frame[5] = (uint8_t)(d->bat1.current_da >> 8U);
        frame[6] = (uint8_t)(d->bat2.current_da >> 0U);
        frame[7] = (uint8_t)(d->bat2.current_da >> 8U);
        cm_enqueue(SRV_CAN_MST_ID_BAT_VOLT, frame, 8);
    }

    /* 0x013 — 双电池版本信息 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_STATUS) {
        frame[0] = (uint8_t)(d->bat1.hw_version >> 0U);
        frame[1] = (uint8_t)(d->bat1.hw_version >> 8U);
        frame[2] = (uint8_t)(d->bat1.sw_version >> 0U);
        frame[3] = (uint8_t)(d->bat1.sw_version >> 8U);
        frame[4] = (uint8_t)(d->bat2.hw_version >> 0U);
        frame[5] = (uint8_t)(d->bat2.hw_version >> 8U);
        frame[6] = (uint8_t)(d->bat2.sw_version >> 0U);
        frame[7] = (uint8_t)(d->bat2.sw_version >> 8U);
        cm_enqueue(SRV_CAN_MST_ID_BAT_STATUS, frame, 8);
    }

    /* 0x014 — 双电池故障码 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_ERROR) {
        frame[0] = d->bat1_fault.volt.raw;
        frame[1] = d->bat1_fault.curr.raw;
        frame[2] = d->bat1_fault.temp.raw;
        frame[3] = d->bat1_fault.hw.raw;
        frame[4] = d->bat2_fault.temp.raw;
        frame[5] = d->bat2_fault.volt.raw;
        frame[6] = d->bat2_fault.curr.raw;
        frame[7] = d->bat2_fault.fet.raw;
        cm_enqueue(SRV_CAN_MST_ID_BAT_ERROR, frame, 8);
    }

    /* 0x015 — 故障等级 + 预警 + 温度 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_COUNTER) {
        frame[0] = (uint8_t)(d->bat1_fault.warnings >> 0U);
        frame[1] = (uint8_t)(d->bat1_fault.warnings >> 8U);
        frame[2] = (uint8_t)d->bat1_fault.level;
        frame[3] = (uint8_t)d->bat2_fault.level;
        frame[4] = (uint8_t)(d->bat2_fault.extra_warnings >> 0U);
        frame[5] = (uint8_t)(d->bat2_fault.extra_warnings >> 8U);
        frame[6] = (uint8_t)d->bat1.temp_c;
        frame[7] = (uint8_t)d->bat2.temp_c;
        cm_enqueue(SRV_CAN_MST_ID_BAT_CNT, frame, 8);
    }
}
