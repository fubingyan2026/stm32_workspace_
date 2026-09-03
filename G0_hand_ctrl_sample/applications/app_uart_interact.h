/**
 * @file    app_uart_interact.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-31
 * @brief   应用层 — UART 交互（收发集中管理：按键上报 / 心跳 / RX 命令处理）
 * @attention
 *
 * 串口协议帧（收发一致）：
 *   [z][cmd][data_len][payload...][crc][\n]
 *
 * 本模块统一负责：
 *   - 按键事件上报（cmd=0x01）
 *   - 周期心跳（cmd=0x00）
 *   - RX 命令接收处理（srv_uart_rx_cmd 回调）
 * 底层驱动（drv_uart 轮询/恢复、srv_uart_rx_cmd step/tick）由 uart_cmd_task 驱动。
 */

#ifndef APP_UART_INTERACT_H
#define APP_UART_INTERACT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/** @brief 心跳命令字节（周期上报，载荷=2字节递增计数） */
#define APP_UART_CMD_HEARTBEAT (0x00U)

/** @brief 按键事件上报命令字节 */
#define APP_UART_CMD_KEY_EVENT (0x01U)

/** @brief 电机设置目标命令（载荷=20B：pos/vel/kp/kd/tor 各 4B float LE） */
#define APP_UART_CMD_MOTOR_SET_TARGET (0x02U)

/** @brief 请求电机反馈命令（载荷=0，G0 回发 MOTOR_FEEDBACK_REPORT） */
#define APP_UART_CMD_MOTOR_REQ_FEEDBACK (0x03U)

/** @brief 电机反馈上报帧（载荷=21B：state(1) + pos/vel/tor/Tmos/Tcoil 各 4B float LE） */
#define APP_UART_CMD_MOTOR_FEEDBACK_REPORT (0x04U)

/** @brief LED 控制命令（载荷=2+：ch(1) + action(1) [+ 参数]，见 app_rgb_status.h） */
#define APP_UART_CMD_LED_CTRL (0x05U)

/* LED 控制 action */
#define APP_UART_LED_ACTION_OFF (0x00U) /**< 关闭 */
#define APP_UART_LED_ACTION_ON (0x01U) /**< 常亮 */
#define APP_UART_LED_ACTION_BLINK (0x02U) /**< 闪烁（后随 2B 间隔 ms 小端） */
#define APP_UART_LED_ACTION_BREATH (0x03U) /**< 呼吸 */

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 UART 交互应用
 * @note  需在 uart_cmd_task_init() 之后调用
 *        （注册 RX 命令回调 + 按键事件回调 → 经 srv_uart_tx_cmd 上报）
 */
void app_uart_interact_init(void);

/**
 * @brief UART 交互步进：周期心跳发送（由 uart_cmd_task 的 sw_timer 调用）
 */
void app_uart_interact_step(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UART_INTERACT_H */
