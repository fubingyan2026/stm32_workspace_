/**
 * @file    app_uart_interact.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   应用层 — UART 交互实现：按键事件经 srv_uart_tx_cmd 协议帧上报
 */

/* Includes ------------------------------------------------------------------*/
#include "app_uart_interact.h"

#include "key_base.h"
#include "log.h"
#include "srv_key.h"
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

/** @brief 上报负载长度：key_index(1) + event(1) */
#define APP_UART_INTERACT_PAYLOAD_LEN (2U)

/* Private function prototypes -----------------------------------------------*/

static void app_uart_key_event_cb(const char* name, key_base_event_t event);

static uint8_t app_uart_key_index(const char* name);

/* Exported functions --------------------------------------------------------*/

void app_uart_interact_init(void)
{
    /* 注册按键事件回调 → 经 srv_uart_tx_cmd 上报 */
    srv_key_register_event_cb(app_uart_key_event_cb);

    APP_UART_INTERACT_LOG_I("UART 交互应用初始化完成 (按键事件上报 cmd=0x%02X)",
        (unsigned)APP_UART_CMD_KEY_EVENT);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 按键事件回调（key_task 主循环上下文）：打包并发送上报帧
 */
static void app_uart_key_event_cb(const char* name, key_base_event_t event)
{
    if (event >= KEY_BASE_EVENT_LONG_WAIT_PRESS) {
        return;
    }

    uint8_t payload[APP_UART_INTERACT_PAYLOAD_LEN];

    payload[0] = app_uart_key_index(name);
    payload[1] = (uint8_t)event;

    const srv_uart_tx_cmd_error_t err = srv_uart_tx_cmd_send(APP_UART_CMD_KEY_EVENT, payload,
        APP_UART_INTERACT_PAYLOAD_LEN);

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
