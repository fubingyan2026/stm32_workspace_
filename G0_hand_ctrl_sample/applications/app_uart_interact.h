/**
 * @file    app_uart_interact.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   应用层 — UART 交互：将按键事件经 srv_uart_tx_cmd 协议帧上报
 * @attention
 *
 * 上报帧（UART 命令协议）：
 *   [z][cmd=0x01 按键事件][data_len=2][key_index][event][crc][\n]
 *   key_index: 0=KEY1, 1=KEY2
 *   event:     key_base_event_t 值
 */

#ifndef APP_UART_INTERACT_H
#define APP_UART_INTERACT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/** @brief 按键事件上报命令字节 */
#define APP_UART_CMD_KEY_EVENT (0x01U)

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 UART 交互应用
 * @note  需在 key_task_init() 与 uart_cmd_task_init() 之后调用
 *        （内部注册 srv_key 事件回调 → 经 srv_uart_tx_cmd 上报）
 */
void app_uart_interact_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UART_INTERACT_H */
