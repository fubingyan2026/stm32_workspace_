/**
 * @file    app_uart_interact.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-31
 * @brief   应用层 — UART 交互实现（收发集中管理）
 */

/* Includes ------------------------------------------------------------------*/
#include "app_uart_interact.h"

#include "drv_systick.h"
#include "key_base.h"
#include "log.h"
#include "srv_dm4310_ctrl.h"
#include "srv_key.h"
#include "srv_uart_rx_cmd.h"
#include "srv_uart_tx_cmd.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define APP_UART_INTERACT_LOG_ENABLE 1

#if APP_UART_INTERACT_LOG_ENABLE
#define APP_UART_INTERACT_LOG_E(...) LOG_E("app_uart_interact", __VA_ARGS__)
#define APP_UART_INTERACT_LOG_W(...) LOG_W("app_uart_interact", __VA_ARGS__)
#define APP_UART_INTERACT_LOG_I(...) LOG_I("app_uart_interact", __VA_ARGS__)
#define APP_UART_INTERACT_LOG_D(...) LOG_D("app_uart_interact", __VA_ARGS__)
#else
#define APP_UART_INTERACT_LOG_E(...) ((void)0)
#define APP_UART_INTERACT_LOG_W(...) ((void)0)
#define APP_UART_INTERACT_LOG_I(...) ((void)0)
#define APP_UART_INTERACT_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 按键事件上报负载长度：key_index(1) + event(1) */
#define APP_UART_KEY_EVENT_PAYLOAD_LEN (2U)

/** @brief 心跳发送间隔 (ms) */
#define APP_UART_HEARTBEAT_INTERVAL_MS (1000U)

/** @brief 心跳载荷长度（2 字节递增计数） */
#define APP_UART_HEARTBEAT_PAYLOAD_LEN (2U)

/** @brief 电机 MIT 目标载荷长度：pos/vel/kp/kd/tor × 4B float = 20B */
#define APP_UART_MOTOR_TARGET_PAYLOAD_LEN (20U)

/** @brief 电机反馈载荷长度：pos/vel/tor/Tmos/Tcoil × 4B float = 20B */
#define APP_UART_MOTOR_FEEDBACK_PAYLOAD_LEN (20U)

/* Private variables ---------------------------------------------------------*/

static uint32_t s_last_heartbeat_ms;
static uint16_t s_heartbeat_tick;

/* Private function prototypes -----------------------------------------------*/

static void app_uart_key_event_cb(const char* name, key_base_event_t event);

static uint8_t app_uart_key_index(const char* name);

static void app_uart_rx_cmd_cb(uint8_t cmd, const uint8_t* data,
    uint8_t data_len);

static void app_uart_send_heartbeat(void);

static void app_uart_motor_handle_cmd(uint8_t cmd, const uint8_t* data,
    uint8_t data_len);

static void app_uart_motor_send_feedback(void);

/* Exported functions --------------------------------------------------------*/

void app_uart_interact_init(void)
{
    /* UART 命令发送服务（protocol_packer 打包 + drv_uart 队列发送） */
    srv_uart_tx_cmd_init();

    /* 注册 RX 命令回调（srv_uart_rx_cmd 解析出完整帧后触发） */
    srv_uart_rx_cmd_init(app_uart_rx_cmd_cb);

    /* 注册按键事件回调 → 经 srv_uart_tx_cmd 上报 */
    srv_key_register_event_cb(app_uart_key_event_cb);

    s_last_heartbeat_ms = millis();
    s_heartbeat_tick = 0;

    APP_UART_INTERACT_LOG_I("UART 交互应用初始化完成 "
        "(心跳 cmd=0x%02X, 按键事件 cmd=0x%02X)",
        (unsigned)APP_UART_CMD_HEARTBEAT, (unsigned)APP_UART_CMD_KEY_EVENT);
}

void app_uart_interact_step(void)
{
    /* 周期心跳（链路验证） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_last_heartbeat_ms) >= APP_UART_HEARTBEAT_INTERVAL_MS) {
        s_last_heartbeat_ms = now_ms;
        app_uart_send_heartbeat();
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 按键事件回调（key_task 主循环上下文）：打包并发送上报帧
 */
static void app_uart_key_event_cb(const char* name, key_base_event_t event)
{
    /* 过滤低层过程事件（PRESS/RELEASE/LONG_WAIT_PRESS/DOWN），
       仅处理点击/长按完成事件（CLICK/ONE_CLICK/DOUBLE_CLICK 等） */
    // if (event == KEY_BASE_EVENT_PRESS || event == KEY_BASE_EVENT_RELEASE
    //     || event == KEY_BASE_EVENT_LONG_WAIT_PRESS || event == KEY_BASE_EVENT_DOWN) {
    //     return;
    // }

    /* 短按（单击）：KEY1 位置增加，KEY2 位置减小 */
    if (event == KEY_BASE_EVENT_PRESS) {
        const uint8_t idx = app_uart_key_index(name);
        if (idx == 0U) {
            srv_dm4310_ctrl_pos_step(SRV_DM4310_POS_STEP_RAD);
        } else if (idx == 1U) {
            srv_dm4310_ctrl_pos_step(-SRV_DM4310_POS_STEP_RAD);
        }
    }

    uint8_t payload[APP_UART_KEY_EVENT_PAYLOAD_LEN];

    payload[0] = app_uart_key_index(name);
    payload[1] = (uint8_t)event;

    const srv_uart_tx_cmd_error_t err = srv_uart_tx_cmd_send(APP_UART_CMD_KEY_EVENT, payload,
        APP_UART_KEY_EVENT_PAYLOAD_LEN);

    if (err != SRV_UART_TX_CMD_OK) {
        APP_UART_INTERACT_LOG_W("按键事件上报失败: %d (key=%s event=%u)",
            (int)err, name ? name : "?", (unsigned)event);
    }
}

/**
 * @brief 按键名 → 上报索引（0=KEY1, 1=KEY2）
 */
static uint8_t app_uart_key_index(const char* name)
{
    if (name != NULL && strcmp(name, "key2") == 0) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief RX 命令回调（uart_cmd_task 主循环上下文）：按 cmd 分发
 */
static void app_uart_rx_cmd_cb(uint8_t cmd, const uint8_t* data,
    uint8_t data_len)
{
    switch (cmd) {
    case APP_UART_CMD_MOTOR_ENABLE:
    case APP_UART_CMD_MOTOR_DISABLE:
    case APP_UART_CMD_MOTOR_SET_TARGET:
    case APP_UART_CMD_MOTOR_CTRL_SEND:
    case APP_UART_CMD_MOTOR_REQ_FEEDBACK:
        app_uart_motor_handle_cmd(cmd, data, data_len);
        break;

    default:
        APP_UART_INTERACT_LOG_D("收到未处理命令 cmd=0x%02X data_len=%u",
            (unsigned)cmd, (unsigned)data_len);
        break;
    }
}

/**
 * @brief 发送心跳帧（协议格式，载荷=2字节递增计数）
 */
static void app_uart_send_heartbeat(void)
{
    uint8_t payload[APP_UART_HEARTBEAT_PAYLOAD_LEN];

    payload[0] = (uint8_t)(s_heartbeat_tick & 0xFFU);
    payload[1] = (uint8_t)((s_heartbeat_tick >> 8) & 0xFFU);

    if (srv_uart_tx_cmd_send(APP_UART_CMD_HEARTBEAT, payload,
            APP_UART_HEARTBEAT_PAYLOAD_LEN) == SRV_UART_TX_CMD_OK) {
        s_heartbeat_tick++;
    }
}

/* ===== DM4310 电机命令 ===== */

/**
 * @brief 处理电机控制/查询命令
 * @param cmd      命令字节
 * @param data     负载数据
 * @param data_len 负载长度
 */
static void app_uart_motor_handle_cmd(uint8_t cmd, const uint8_t* data,
    uint8_t data_len)
{
    switch (cmd) {
    case APP_UART_CMD_MOTOR_ENABLE:
        srv_dm4310_ctrl_enable();
        break;

    case APP_UART_CMD_MOTOR_DISABLE:
        srv_dm4310_ctrl_disable();
        break;

    case APP_UART_CMD_MOTOR_SET_TARGET: {
        if (data == NULL || data_len < APP_UART_MOTOR_TARGET_PAYLOAD_LEN) {
            APP_UART_INTERACT_LOG_W("电机设置目标命令负载不足 (%u)", (unsigned)data_len);
            break;
        }

        float pos, vel, kp, kd, tor;
        memcpy(&pos, &data[0], 4);
        memcpy(&vel, &data[4], 4);
        memcpy(&kp, &data[8], 4);
        memcpy(&kd, &data[12], 4);
        memcpy(&tor, &data[16], 4);

        srv_dm4310_ctrl_set_target(pos, vel, kp, kd, tor);
        break;
    }

    case APP_UART_CMD_MOTOR_CTRL_SEND:
        srv_dm4310_ctrl_send();
        break;

    case APP_UART_CMD_MOTOR_REQ_FEEDBACK:
        app_uart_motor_send_feedback();
        break;

    default:
        break;
    }
}

/**
 * @brief 上报电机最新反馈（pos/vel/tor/Tmos/Tcoil，各 4B float LE）
 */
static void app_uart_motor_send_feedback(void)
{
    motor_t* motor = srv_dm4310_ctrl_get_motor();
    if (motor == NULL) {
        return;
    }

    uint8_t payload[APP_UART_MOTOR_FEEDBACK_PAYLOAD_LEN];

    memcpy(&payload[0], &motor->para.pos, 4);
    memcpy(&payload[4], &motor->para.vel, 4);
    memcpy(&payload[8], &motor->para.tor, 4);
    memcpy(&payload[12], &motor->para.Tmos, 4);
    memcpy(&payload[16], &motor->para.Tcoil, 4);

    const srv_uart_tx_cmd_error_t err = srv_uart_tx_cmd_send(
        APP_UART_CMD_MOTOR_FEEDBACK_REPORT, payload,
        APP_UART_MOTOR_FEEDBACK_PAYLOAD_LEN);

    if (err != SRV_UART_TX_CMD_OK) {
        APP_UART_INTERACT_LOG_W("电机反馈上报失败: %d", (int)err);
    }
}
