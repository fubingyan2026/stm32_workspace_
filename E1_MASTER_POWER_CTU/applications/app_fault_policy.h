/**
 * @file    app_fault_policy.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-05
 * @brief   应用层 — 故障保护策略（断电/联动响应）
 *
 * 用例：评估电源故障条件（E-STOP / 关键电源轨丢失 / 过流），
 * 触发后联动 srv_pwr_ctrl 紧急断电 + srv_fan_ctrl 风扇满速散热，并锁存。
 * 只调用 service，不拥有 sw_timer（由 power_task 驱动），不触碰 HAL/驱动。
 *
 * 锁存语义：保护触发后保持断电，需操作员显式调用 app_fault_policy_reset()
 * 确认后才解除（急停安全惯例）。auto 恢复策略暂未实现。
 */

#ifndef __APP_FAULT_POLICY_H
#define __APP_FAULT_POLICY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** @brief 初始化故障保护策略（复位锁存状态） */
void app_fault_policy_init(void);

/**
 * @brief 周期评估保护条件（由 power_task sw_timer 调用）
 * @param elapsed_ms 距上次调用的毫秒数
 */
void app_fault_policy_step(uint16_t elapsed_ms);

/** @brief 是否处于保护锁存（触发后置位，需显式复位） */
bool app_fault_policy_is_tripped(void);

/**
 * @brief 显式复位保护锁存（操作员确认故障排除后调用）
 * @note  仅解除锁存标志，不自动重新上电；上电需另行调用 request_on
 */
void app_fault_policy_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FAULT_POLICY_H */
