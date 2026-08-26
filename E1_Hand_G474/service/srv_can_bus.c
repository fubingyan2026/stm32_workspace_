/**
 * @file    srv_can_bus.c
 * @brief   CAN 总线抽象实现 — 通道级接收分发 + 发送封装
 *
 * 内部维护每通道唯一的 `rx_handler` 槽位表（s_bus[ch]）。
 * srv_can_bus_init() 将 drv_can 的通道接收回调统一指向 bus_dispatch，
 * 收到帧后按通道查表调用对应服务注册的 rx_handler（ISR 上下文）。
 */

#include "srv_can_bus.h"

#include <stddef.h>

/* Private variables ---------------------------------------------------------*/

/** @brief 每通道总线槽位表（存放绑定信息，分发用） */
static srv_can_bus_t s_bus[DRV_CAN_CH_NUM];

/* Private function prototypes -----------------------------------------------*/

static void srv_can_bus_dispatch(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions --------------------------------------------------------*/

void srv_can_bus_init(void)
{
    for (uint32_t ch = 0; ch < DRV_CAN_CH_NUM; ch++) {
        s_bus[ch].channel = (drv_can_channel_t)ch;
        s_bus[ch].rx_handler = NULL;
        s_bus[ch].rx_user_data = NULL;
        s_bus[ch].tx_cnt = 0;
        s_bus[ch].rx_cnt = 0;
        (void)drv_can_register_rx_callback((drv_can_channel_t)ch, srv_can_bus_dispatch);
    }
}

drv_can_error_t srv_can_bus_bind(srv_can_bus_t* bus,
    drv_can_channel_t ch,
    srv_can_bus_rx_handler_t rx,
    void* user_data)
{
    if (!bus || (ch >= DRV_CAN_CH_NUM)) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }
    if (!drv_can_is_initialized(ch)) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }

    bus->channel = ch;
    bus->rx_handler = rx;
    bus->rx_user_data = user_data;
    bus->tx_cnt = 0;
    bus->rx_cnt = 0;

    s_bus[ch].channel = ch;
    s_bus[ch].rx_handler = rx;
    s_bus[ch].rx_user_data = user_data;

    return DRV_CAN_OK;
}

drv_can_error_t srv_can_bus_send(const srv_can_bus_t* bus, const drv_can_msg_t* msg)
{
    if (!bus) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }
    drv_can_error_t err = drv_can_send(bus->channel, msg);
    if (err == DRV_CAN_OK) {
        s_bus[bus->channel].tx_cnt++;
    }
    return err;
}

bool srv_can_bus_tx_ready(const srv_can_bus_t* bus)
{
    if (!bus) {
        return false;
    }
    return drv_can_tx_ready(bus->channel);
}

void srv_can_bus_poll_status(const srv_can_bus_t* bus)
{
    if (!bus) {
        return;
    }
    drv_can_poll_status(bus->channel);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 通道级接收分发回调（由 drv_can 在 ISR 中调用）
 * @note   只做查表转发，不打日志，转发目标服务须保持简短
 */
static void srv_can_bus_dispatch(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    if ((ch >= DRV_CAN_CH_NUM) || !msg) {
        return;
    }
    srv_can_bus_t* bus = &s_bus[ch];
    bus->rx_cnt++;
    if (bus->rx_handler) {
        bus->rx_handler(msg, bus->rx_user_data);
    }
}
