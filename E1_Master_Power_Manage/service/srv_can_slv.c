/**
 * @file    srv_can_slv.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   从电源板控制服务实现 — 0x002 协议 + ACK 重试
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_can_slv.h"

#include <string.h>

#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_CAN_SLV_LOG_ENABLE 0

#if SRV_CAN_SLV_LOG_ENABLE
#define SRV_CAN_SLV_LOG_E(...) LOG_E("srv_can_slv", __VA_ARGS__)
#define SRV_CAN_SLV_LOG_W(...) LOG_W("srv_can_slv", __VA_ARGS__)
#define SRV_CAN_SLV_LOG_I(...) LOG_I("srv_can_slv", __VA_ARGS__)
#define SRV_CAN_SLV_LOG_D(...) LOG_D("srv_can_slv", __VA_ARGS__)
#else
#define SRV_CAN_SLV_LOG_E(...) ((void)0)
#define SRV_CAN_SLV_LOG_W(...) ((void)0)
#define SRV_CAN_SLV_LOG_I(...) ((void)0)
#define SRV_CAN_SLV_LOG_D(...) ((void)0)
#endif

/** @brief ACK 超时重发日志限频窗口 (ms)：无 ACK 时约 20 行/s，必须限频 */
#define SRV_CAN_SLV_ERR_LOG_PERIOD_MS (1000U)

/* Private types -------------------------------------------------------------*/

typedef enum {
    SLAVE_IDLE,
    SLAVE_PENDING,
    SLAVE_ACKED,
} slave_state_t;

/* Private variables ---------------------------------------------------------*/

static srv_can_slv_config_t s_config;
static bool s_initialized;
static slave_state_t s_state;
static srv_can_slv_ctrl_t s_pending_ctrl; /**< 当前待发送的控制数据 */
static uint16_t s_retry_ms; /**< 上次发送以来的时间 (ms) */
static uint32_t s_retry_log_ts; /**< 上次打印超时告警的时间戳 (ms) */

/* Private function prototypes -----------------------------------------------*/

static void slaver_send(void);

/* Exported functions --------------------------------------------------------*/

srv_can_slv_error_t srv_can_slv_init(const srv_can_slv_config_t* config)
{
    if (!config || !config->send_frame || !config->get_ctrl) {
        return SRV_CAN_SLV_ERROR_NULL_PTR;
    }

    if (s_initialized) {
        srv_can_slv_deinit();
    }

    s_config = *config;
    s_state = SLAVE_IDLE;
    memset(&s_pending_ctrl, 0, sizeof(s_pending_ctrl));
    s_retry_ms = 0;
    s_initialized = true;

    SRV_CAN_SLV_LOG_I("从板控制服务初始化完成 (重试间隔=%ums)", (unsigned)SRV_CAN_SLV_RETRY_MS);

    return SRV_CAN_SLV_OK;
}

void srv_can_slv_deinit(void)
{
    memset(&s_config, 0, sizeof(s_config));
    memset(&s_pending_ctrl, 0, sizeof(s_pending_ctrl));
    s_state = SLAVE_IDLE;
    s_retry_ms = 0;
    s_initialized = false;
}

bool srv_can_slv_is_initialized(void)
{
    return s_initialized;
}

void srv_can_slv_request(void)
{
    if (!s_initialized) {
        return;
    }

    /* 如果当前在等待 ACK，忽略新请求 */
    if (s_state != SLAVE_IDLE) {
        return;
    }

    /* 读取当前控制状态 */
    s_config.get_ctrl(&s_pending_ctrl);

    SRV_CAN_SLV_LOG_D("从板控制请求: hsd1_12v=%d reserved=%d",
        (int)s_pending_ctrl.hsd1_12v_on, (int)s_pending_ctrl.reserved_channel);

    /* 立即发送首次帧 */
    slaver_send();
}

void srv_can_slv_task(void)
{
    if (!s_initialized) {
        return;
    }

    /* 一轮请求/ACK 握手完成：回到 IDLE，允许下轮周期存活探测 */
    if (s_state == SLAVE_ACKED) {
        s_state = SLAVE_IDLE;
        return;
    }

    if (s_state != SLAVE_PENDING) {
        return;
    }

    s_retry_ms += 10; /* 假定每 10ms 调用一次 */

    if (s_retry_ms >= SRV_CAN_SLV_RETRY_MS) {
        s_retry_ms = 0;

        /* ACK 超时重发告警（限频 1s） */
        const uint32_t now_ms = millis();
        if ((uint32_t)(now_ms - s_retry_log_ts) >= SRV_CAN_SLV_ERR_LOG_PERIOD_MS) {
            s_retry_log_ts = now_ms;
            SRV_CAN_SLV_LOG_W("从板 ACK 超时重发 0x002 (retry=%ums)",
                (unsigned)SRV_CAN_SLV_RETRY_MS);
        }

        slaver_send();
    }
}

void srv_can_slv_process_rx(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    (void)data;

    if (!s_initialized || s_state != SLAVE_PENDING) {
        return;
    }

    /* ACK：ID=0x002, len=8 */
    if (can_id == SRV_CAN_SLV_ID_CTRL && len == SRV_CAN_SLV_ACK_LEN) {
        s_state = SLAVE_ACKED;
        s_retry_ms = 0;
        SRV_CAN_SLV_LOG_I("从板 ACK 已收到 (0x002 len=%u): PENDING -> ACKED", (unsigned)len);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 构建并发送 0x002 控制帧
 *
 * 帧格式（6 字节）：
 *   byte0: 保留
 *   byte1~4: 保留
 *   byte5: bit4=HSD1_12V, bit5=reserved, bit6=valid mask
 */
static void slaver_send(void)
{
    uint8_t frame[6];
    memset(frame, 0, sizeof(frame));

    /* bit4=HSD1_12V, bit5=reserved, bit6=valid mask */
    frame[5] = (1U << 6); /* valid mask 始终置位 */
    if (s_pending_ctrl.hsd1_12v_on)
        frame[5] |= (1U << 4);
    if (s_pending_ctrl.reserved_channel)
        frame[5] |= (1U << 5);

    if (s_config.send_frame(SRV_CAN_SLV_ID_CTRL, frame, sizeof(frame))) {
        s_state = SLAVE_PENDING;
        s_retry_ms = 0;
    }
    /* 发送失败则下次重试（状态保持 SLAVE_PENDING 或 SLAVE_IDLE） */
}
