/**
 * @file    srv_can_mst.c
 * @author  maximillian
 * @version V1.2.0
 * @date    2026-07-2
 * @brief   P_CAN 主机上报服务实现（msg_fifo 队列 + pending 重试）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_can_mst.h"

#include <string.h>

#include "drv_systick.h"
#include "log.h"
#include "public.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_CAN_MST_LOG_ENABLE 1

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
static uint8_t pack_byte0(const srv_can_mst_data_t* d);
static uint8_t pack_byte1(const srv_can_mst_data_t* d);
static uint8_t pack_byte2(const srv_can_mst_data_t* d);
static uint8_t pack_byte3(const srv_can_mst_data_t* d);
static uint8_t pack_byte4(const srv_can_mst_data_t* d);
static uint8_t pack_byte5(const srv_can_mst_data_t* d);

/* Exported functions --------------------------------------------------------*/

srv_can_mst_error_t srv_can_mst_init(const srv_can_mst_config_t* config)
{
    if (!config || !config->read_data || !config->send_frame) {
        return SRV_CAN_MST_ERROR_NULL_PTR;
    }

    if (s_initialized) {
        srv_can_mst_deinit();
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
    SRV_CAN_MST_LOG_I("收到主机指令: fb=0x%02X rgb_mode=%u rgb=0x%06lX buzzer=%u hsd1_12v=%d hsd2_12v=%d lsd1_24v=%d lsd2_24v=%d",
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
    uint8_t frame[8] = {
        pack_byte0(d),
        pack_byte1(d),
        pack_byte2(d),
        pack_byte3(d),
        pack_byte4(d),
        pack_byte5(d),
        d->bat1_soc,
        d->bat2_soc,
    };
    cm_enqueue(SRV_CAN_MST_ID_STATUS, frame, 8);
}

static uint8_t pack_byte0(const srv_can_mst_data_t* d)
{
    uint8_t b = 0;
    if (d->stop_key_state)
        b |= (1U << 0);
    if (d->battery_key_state)
        b |= (1U << 1);
    if (d->battery_charging)
        b |= (1U << 2);
    if (d->battery_temp_error)
        b |= (1U << 3);
    if (d->bat1_online)
        b |= (1U << 4);
    if (d->bat_has_error)
        b |= (1U << 5);
    if (d->bat2_online)
        b |= (1U << 6);
    return b;
}

static uint8_t pack_byte1(const srv_can_mst_data_t* d)
{
    uint8_t b = 0;
    if (d->err_vin)
        b |= (1U << 0);
    if (d->err_vin_dcdc)
        b |= (1U << 1);
    if (d->err_12v_int)
        b |= (1U << 2);
    if (d->err_5v_int)
        b |= (1U << 3);
    if (d->err_12v_ext)
        b |= (1U << 4);
    if (d->err_24v_ext)
        b |= (1U << 5);
    if (d->err_12v_user)
        b |= (1U << 6);
    if (d->err_24v_user)
        b |= (1U << 7);
    return b;
}

static uint8_t pack_byte2(const srv_can_mst_data_t* d)
{
    uint8_t b = 0;
    if (d->err_24v_comp)
        b |= (1U << 0);
    if (d->err_power)
        b |= (1U << 1);
    if (d->err_motor)
        b |= (1U << 2);
    if (d->err_chg_out)
        b |= (1U << 3);
    if (d->err_hsd1_12v)
        b |= (1U << 4);
    if (d->err_hsd2_12v)
        b |= (1U << 5);
    if (d->err_hsd3_12v)
        b |= (1U << 6);
    if (d->err_dbr)
        b |= (1U << 7);
    return b;
}

static uint8_t pack_byte3(const srv_can_mst_data_t* d)
{
    uint8_t b = 0;
    if (d->err_hsd1_24v)
        b |= (1U << 0);
    if (d->err_hsd2_24v)
        b |= (1U << 1);
    if (d->err_hsd3_24v)
        b |= (1U << 2);
    if (d->err_lsd1_24v)
        b |= (1U << 3);
    if (d->err_lsd2_24v)
        b |= (1U << 4);
    if (d->err_fan[0])
        b |= (1U << 5);
    if (d->err_fan[1])
        b |= (1U << 6);
    if (d->err_fan[2])
        b |= (1U << 7);
    return b;
}

static uint8_t pack_byte4(const srv_can_mst_data_t* d)
{
    uint8_t b = 0;
    for (uint32_t i = 0; i < 8; i++) {
        if (d->err_ntc[i]) {
            b |= (1U << i);
        }
    }
    return b;
}

static uint8_t pack_byte5(const srv_can_mst_data_t* d)
{
    uint8_t b = 0;
    if (d->a_in1_io)
        b |= (1U << 0);
    if (d->a_in2_io)
        b |= (1U << 1);
    if (d->a_in3_io)
        b |= (1U << 2);
    if (d->seq_vin_fault)
        b |= (1U << 3);
    if (d->seq_chg_fault)
        b |= (1U << 4);
    if (d->seq_motor_fault)
        b |= (1U << 5);
    return b;
}

/* --- 电池数据帧构建 (0x011-0x0B1) --- */

static void cm_build_battery(const srv_can_mst_data_t* d, uint8_t fb)
{
    uint8_t frame[8];

    /* 0x011 — 双电池 SOC + 双电池容量 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_BASE) {
        frame[0] = d->bat1_soc;
        frame[1] = d->bat2_soc;
        frame[2] = (uint8_t)(d->bat1_capacity_mah >> 8U);
        frame[3] = (uint8_t)(d->bat1_capacity_mah >> 16U);
        frame[4] = (uint8_t)(d->bat2_capacity_mah >> 8U);
        frame[5] = (uint8_t)(d->bat2_capacity_mah >> 16U);
        frame[6] = (uint8_t)(d->bat1_cycle_count >> 0U);
        frame[7] = (uint8_t)(d->bat2_cycle_count >> 0U);
        cm_enqueue(SRV_CAN_MST_ID_BAT_BASE, frame, 8);
    }

    /* 0x021 — 双电池电压 + 充电标志 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_VOLTAGE) {
        frame[0] = (uint8_t)(d->bat1_voltage_dv >> 0U);
        frame[1] = (uint8_t)(d->bat1_voltage_dv >> 8U);
        frame[2] = (uint8_t)(d->bat2_voltage_dv >> 0U);
        frame[3] = (uint8_t)(d->bat2_voltage_dv >> 8U);
        frame[4] = (uint8_t)(d->bat1_charging ? (1U << 0) : 0)
            | (uint8_t)(d->bat2_charging ? (1U << 1) : 0);
        frame[5] = 0;
        frame[6] = 0;
        frame[7] = 0;
        cm_enqueue(SRV_CAN_MST_ID_BAT_VOLT, frame, 8);
    }

    /* 0x041 — 双电池温度 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_TEMPERATURE) {
        frame[0] = (uint8_t)d->bat1_temp_c;
        frame[1] = (uint8_t)d->bat2_temp_c;
        frame[2] = 0;
        frame[3] = 0;
        frame[4] = 0;
        frame[5] = 0;
        frame[6] = 0;
        frame[7] = 0;
        cm_enqueue(SRV_CAN_MST_ID_BAT_TEMP14, frame, 8);
    }

    /* 0x031 — 双电池电流 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_CURRENT) {
        frame[0] = (uint8_t)(d->bat1_current_da >> 0U);
        frame[1] = (uint8_t)(d->bat1_current_da >> 8U);
        frame[2] = (uint8_t)(d->bat2_current_da >> 0U);
        frame[3] = (uint8_t)(d->bat2_current_da >> 8U);
        frame[4] = 0;
        frame[5] = 0;
        frame[6] = 0;
        frame[7] = 0;
        cm_enqueue(SRV_CAN_MST_ID_BAT_CURR, frame, 8);
    }

    /* 0x051 — 双电池版本信息 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_STATUS) {
        frame[0] = (uint8_t)(d->bat1_hw_version >> 0U);
        frame[1] = (uint8_t)(d->bat1_hw_version >> 8U);
        frame[2] = (uint8_t)(d->bat1_sw_version >> 0U);
        frame[3] = (uint8_t)(d->bat1_sw_version >> 8U);
        frame[4] = (uint8_t)(d->bat2_hw_version >> 0U);
        frame[5] = (uint8_t)(d->bat2_hw_version >> 8U);
        frame[6] = (uint8_t)(d->bat2_sw_version >> 0U);
        frame[7] = (uint8_t)(d->bat2_sw_version >> 8U);
        cm_enqueue(SRV_CAN_MST_ID_BAT_STATUS, frame, 8);
    }

    /* 0x061 — 双电池故障码 */
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

    /* 0x071 — 故障等级 + 预警 */
    if (fb & SRV_CAN_MST_FEEDBACK_BAT_COUNTER) {
        frame[0] = (uint8_t)(d->bat1_fault.warnings >> 0U);
        frame[1] = (uint8_t)(d->bat1_fault.warnings >> 8U);
        frame[2] = (uint8_t)d->bat1_fault.level;
        frame[3] = (uint8_t)d->bat2_fault.level;
        frame[4] = (uint8_t)(d->bat2_fault.extra_warnings >> 0U);
        frame[5] = (uint8_t)(d->bat2_fault.extra_warnings >> 8U);
        frame[6] = 0;
        frame[7] = 0;
        cm_enqueue(SRV_CAN_MST_ID_BAT_CNT, frame, 8);
    }
}
