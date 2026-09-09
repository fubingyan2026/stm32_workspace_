/**
 * @file    app_status_indicator.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   应用层 — 状态指示灯策略（错误/警告 → 单颗状态灯灯效映射）
 * @attention
 *
 * 遵循 app 层规则：只调 service，不拥有 sw_timer，不触碰 HAL/驱动。
 *
 * ## 灯效映射（优先级高者胜，单颗状态 LED，以闪烁节奏区分）
 *   P0 急停 / 故障锁存   → 快闪 (100ms)
 *   P1 关键电源轨故障     → 慢闪 (500ms)
 *   P2 警告（12V/风扇/NTC）→ 更慢闪 (1000ms)
 *   P3 正常               → 呼吸（常亮呼吸）
 *
 * ## 用法
 * @code
 *   app_status_indicator_init(&led_status); // led_task_init 内、srv_signal 注册之后
 *   app_status_indicator_step(10);          // led_task 10ms sw_timer 内
 * @endcode
 */

#ifndef __APP_STATUS_INDICATOR_H
#define __APP_STATUS_INDICATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "srv_signal.h"

/**
 * @brief 初始化状态指示灯策略
 * @param status_led 状态灯信号实例（由 led_task 注入，需在 srv_signal 注册之后）
 */
void app_status_indicator_init(srv_signal_handle_t* status_led);

/**
 * @brief 周期评估系统状态并驱动 LED 灯效
 * @param elapsed_ms 距上次调用的毫秒数（由 led_task sw_timer 传入）
 * @note 内部 100ms 节流评估；仅状态等级变化时下发 srv_signal 命令
 */
void app_status_indicator_step(uint16_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* __APP_STATUS_INDICATOR_H */
