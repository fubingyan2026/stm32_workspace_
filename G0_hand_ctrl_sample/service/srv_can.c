/**
 * @file    srv_can.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   CAN 协议服务实现 — 协议骨架占位
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_can.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_CAN_LOG_ENABLE 1

#if SRV_CAN_LOG_ENABLE
#define SRV_CAN_LOG_E(...) LOG_E("srv_can", __VA_ARGS__)
#define SRV_CAN_LOG_W(...) LOG_W("srv_can", __VA_ARGS__)
#define SRV_CAN_LOG_I(...) LOG_I("srv_can", __VA_ARGS__)
#define SRV_CAN_LOG_D(...) LOG_D("srv_can", __VA_ARGS__)
#else
#define SRV_CAN_LOG_E(...) ((void)0)
#define SRV_CAN_LOG_W(...) ((void)0)
#define SRV_CAN_LOG_I(...) ((void)0)
#define SRV_CAN_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief RX 日志限频窗口 (ms)：总线繁忙时防止刷屏 */
#define SRV_CAN_RX_LOG_PERIOD_MS (1000U)

/** @brief 心跳数据载荷长度 */
#define SRV_CAN_HEARTBEAT_LEN (2U)

/* Private variables ---------------------------------------------------------*/

/** @brief 初始化标志 */
static bool s_initialized;

/** @brief 心跳计数（每发送一次 +1） */
static uint16_t s_heartbeat_tick;

/** @brief 已收帧计数 */
static uint32_t s_rx_count;

/** @brief 上次 RX 日志时间戳 (ms) */
static uint32_t s_rx_log_ts;

/* Exported functions --------------------------------------------------------*/

void srv_can_init(void)
{
    s_heartbeat_tick = 0;
    s_rx_count = 0;
    s_rx_log_ts = 0;
    s_initialized = true;

    SRV_CAN_LOG_I("CAN 协议服务初始化完成 (骨架占位, ID 0x%03X/0x%03X)",
        (unsigned)SRV_CAN_ID_HEARTBEAT, (unsigned)SRV_CAN_ID_ACK);
}

void srv_can_on_rx(const drv_can_msg_t* msg)
{
    if (!s_initialized || !msg) {
        return;
    }

    s_rx_count++;

    /* RX 日志限频（ISR 上下文，仅计数 + 时间戳比较） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_rx_log_ts) >= SRV_CAN_RX_LOG_PERIOD_MS) {
        s_rx_log_ts = now_ms;
        SRV_CAN_LOG_D("CAN RX: id=0x%03X dlc=%u total=%lu",
            (unsigned)msg->id, (unsigned)msg->dlc, (unsigned long)s_rx_count);
    }

    /* TODO: 协议文档定稿后，在此按 CAN ID 分发各业务帧 */
}

void srv_can_process(void)
{
    if (!s_initialized) {
        return;
    }

    /* TODO: 协议文档定稿后，在此处理缓存的控制帧并下发 */
}

void srv_can_send_heartbeat(void)
{
    if (!s_initialized || !drv_can_tx_ready(DRV_CAN_CH_1)) {
        return;
    }

    drv_can_msg_t msg;
    msg.id = SRV_CAN_ID_HEARTBEAT;
    msg.is_extended = false;
    msg.dlc = SRV_CAN_HEARTBEAT_LEN;
    msg.data[0] = (uint8_t)(s_heartbeat_tick & 0xFFU);
    msg.data[1] = (uint8_t)((s_heartbeat_tick >> 8) & 0xFFU);
    memset(&msg.data[SRV_CAN_HEARTBEAT_LEN], 0,
        (uint8_t)sizeof(msg.data) - SRV_CAN_HEARTBEAT_LEN);

    if (drv_can_send(DRV_CAN_CH_1, &msg) == DRV_CAN_OK) {
        s_heartbeat_tick++;
    }
}
