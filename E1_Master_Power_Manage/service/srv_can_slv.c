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

    /* 立即发送首次帧 */
    slaver_send();
}

void srv_can_slv_task(void)
{
    if (!s_initialized || s_state != SLAVE_PENDING) {
        return;
    }

    s_retry_ms += 10; /* 假定每 10ms 调用一次 */

    if (s_retry_ms >= SRV_CAN_SLV_RETRY_MS) {
        s_retry_ms = 0;
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
