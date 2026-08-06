/**
 * @file    app_status_indicator.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   应用层 — 状态指示灯策略（错误/警告 → LED 灯效映射）
 * @attention
 *
 * 遵循 app 层规则：只调 service，不拥有 sw_timer，不触碰 HAL/驱动。
 *
 * ## 职责
 * - 聚合系统状态（0x001 故障位域 + 故障锁存），按优先级映射为蓝/红双 LED 灯效
 * - 仅状态等级变化时下发 srv_signal 命令，避免刷爆异步命令队列
 *
 * ## 灯效映射（优先级高者胜）
 *   P0 急停 / 故障锁存        → 红灯快闪 (100ms) + 蓝灯熄灭
 *   P1 关键电源轨故障          → 红灯慢闪 (300ms) + 蓝灯熄灭
 *   P2 子设备离线 (slaver/dual) → 红灯中速闪烁 (500ms) + 蓝灯呼吸
 *   P3 警告（次级轨/HSD-LSD/风扇/NTC/电池温度） → 红灯呼吸 + 蓝灯呼吸
 *   P4 正常                    → 蓝灯呼吸 + 红灯熄灭
 *
 * ## 用法
 * @code
 *   app_status_indicator_init(&led_blue, &led_red); // led_task_init 内、srv_signal 注册之后
 *   app_status_indicator_step(10);        // led_task 10ms sw_timer 内
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
 * @param blue 蓝色 LED 实例（由 led_task 注入，需在 srv_signal 注册之后）
 * @param red  红色 LED 实例
 * @note 句柄显式注入，避免 app 按字符串名反查注册表造成名字/时序耦合
 */
void app_status_indicator_init(srv_signal_handle_t* blue, srv_signal_handle_t* red);

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
