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
static void cm_build_volt_temp(const srv_can_mst_data_t* d);
static void cm_build_power_fault(const srv_can_mst_data_t* d);
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
    cm_build_volt_temp(&data);
    cm_build_power_fault(&data);
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
    if (!s_initialized || !data || len < 6)
        return;

    memset(&s_last_cmd, 0, sizeof(s_last_cmd));

    s_last_cmd.buzzer_duty = (data[0] <= 50U) ? data[0] : 50U;

    /* byte1: 3对 valid+value 控制位 */
    if (data[1] & (1U << 5))
        s_last_cmd.hsd1_12v_on = (data[1] >> 4) & 1;
    if (data[1] & (1U << 3))
        s_last_cmd.hsd1_24v_on = (data[1] >> 2) & 1;
    if (data[1] & (1U << 1))
        s_last_cmd.hsd2_24v_on = (data[1] >> 0) & 1;

    /* byte2-5: LED RGB 控制（led_index 选通道，0-31=通道1, 32-63=通道2） */
    s_last_cmd.led_index = data[2];
    s_last_cmd.led_r = data[3];
    s_last_cmd.led_g = data[4];
    s_last_cmd.led_b = data[5];

    s_cmd_pending = true;

    /* 主机指令日志（配置变更，I 级） */
    SRV_CAN_MST_LOG_D("收到主机指令: buzzer=%u hsd1_12v=%d hsd1_24v=%d hsd2_24v=%d led=%u #%02X%02X%02X",
        (unsigned)s_last_cmd.buzzer_duty,
        (int)s_last_cmd.hsd1_12v_on,
        (int)s_last_cmd.hsd1_24v_on,
        (int)s_last_cmd.hsd2_24v_on,
        (unsigned)s_last_cmd.led_index,
        (unsigned)s_last_cmd.led_r,
        (unsigned)s_last_cmd.led_g,
        (unsigned)s_last_cmd.led_b);

    /* 收到主机指令后立即触发一轮上报回复 */
    srv_can_mst_request(0);
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
    /* 0x001 帧 = status 完整字节（2 字节状态帧），整段 memcpy 无需逐位搬移 */
    cm_enqueue(SRV_CAN_MST_ID_STATUS, d->status.bytes, sizeof(d->status.bytes));
}

/* --- 0x016 温度帧构建 --- */

static void cm_build_volt_temp(const srv_can_mst_data_t* d)
{
    /* 0x016 帧 = volt_temp(Byte0-7) 完整 8 字节 */
    cm_enqueue(SRV_CAN_MST_ID_VOLT_TEMP, d->volt_temp.bytes, sizeof(d->volt_temp.bytes));
}

/* --- 0x017 电源电压+预充故障帧构建 --- */

static void cm_build_power_fault(const srv_can_mst_data_t* d)
{
    /* 0x017 帧 = power_fault(Byte0-7) 完整 8 字节 */
    cm_enqueue(SRV_CAN_MST_ID_POWER_FAULT, d->power_fault.bytes, sizeof(d->power_fault.bytes));
}

static bool cm_check_status_frame_layout(void)
{
    /* 对 2 个有效字节各选代表位置位，比对期望值。
     * 期望值源自协议文档位定义，与 GCC ARM (little-endian) 实测一致。 */
    srv_can_mst_status_frame_t f;
    memset(&f, 0, sizeof(f));
    f.bits.stop_key_state = 1U; /* byte0 bit0 */
    f.bits.err_hsd_fault = 1U; /* byte0 bit7 */
    f.bits.err_dbr = 1U; /* byte1 bit0 */
    f.bits.err_fan1 = 1U; /* byte1 bit5 */
    f.bits.err_ntc2 = 1U; /* byte1 bit7 */
    const uint8_t expect[2] = { 0x81U, 0xA1U };
    return memcmp(f.bytes, expect, sizeof(expect)) == 0;
}
