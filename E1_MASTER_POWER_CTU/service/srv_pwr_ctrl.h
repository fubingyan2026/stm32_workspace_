/**
 * @file    srv_pwr_ctrl.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   电源控制服务 — 顺序上电 FSM（E1_MASTER_POWER_CTU）
 *
 * 电源 FSM 按序使能四路电源轨并 PGOOD 门控，直接驱动 drv_power / drv_status。
 * 不管理 sw_timer，由 task 层定期调用 srv_pwr_ctrl_step() 推进状态。
 *
 * 上电顺序：VIN_DC-DC(LM5060) → DC-DC 24V(MP9931N) → AUX(LM5069) → MOTOR(LM5069)。
 * 无电机预充电 FET/HSD/DBR（CTU 板无此硬件）。
 */

#ifndef __SRV_PWR_CTRL_H
#define __SRV_PWR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化电源控制服务（drv_power + 状态机复位，全部轨关闭） */
void srv_pwr_ctrl_init(void);

/**
 * @brief 推进 FSM 状态机一步
 * @param elapsed_ms 距离上次调用经过的毫秒数
 * @note  由 task 层 sw_timer 周期调用（1ms）
 */
void srv_pwr_ctrl_step(uint16_t elapsed_ms);

/** @brief 请求上电（异步，FSM 自动推进） */
void srv_pwr_ctrl_request_on(void);

/** @brief 紧急断电（同步，立即关闭所有输出并回到待机） */
void srv_pwr_ctrl_emergency_off(void);

/** @brief 是否已完成上电流程（四路全部使能且 PGD 就绪） */
bool srv_pwr_ctrl_is_powered_on(void);

/** @brief 各电源轨是否已被 FSM 驱动使能（供故障策略门控 PGD 判定） */
bool srv_pwr_ctrl_is_vin_enabled(void);
bool srv_pwr_ctrl_is_dc24v_enabled(void);
bool srv_pwr_ctrl_is_aux_enabled(void);
bool srv_pwr_ctrl_is_motor_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_CTRL_H */
