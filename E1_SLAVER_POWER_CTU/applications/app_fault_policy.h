/**
 * @file    app_fault_policy.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   应用层 — 故障保护策略（母线级监控 + 锁存日志联动）
 *
 * 副电源模块使能期门控/运行期 PGOOD/节点丢失去抖由 srv_pwr_ctrl FSM 内联处理。
 * 本策略负责 srv_pwr_ctrl 覆盖不到的母线级判定：
 *   - DC24V 实际运行中 AUX 母线跌落后持续缺失（≥100ms 防误关断）→ 锁存 DC24V；
 *   - 锁存状态变化日志与 tripped 查询（供状态灯/上报聚合使用）。
 * 只调用 service，不拥有 sw_timer（由 power_task 驱动），不触碰 HAL/驱动。
 */

#ifndef __APP_FAULT_POLICY_H
#define __APP_FAULT_POLICY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** @brief 初始化故障保护策略 */
void app_fault_policy_init(void);

/**
 * @brief 周期评估保护条件（由 power_task sw_timer 调用）
 * @param elapsed_ms 距上次调用的毫秒数
 */
void app_fault_policy_step(uint16_t elapsed_ms);

/** @brief 是否处于故障锁存（映射 srv_pwr_ctrl 锁存掩码） */
bool app_fault_policy_is_tripped(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FAULT_POLICY_H */
