/**
 * @file    srv_pwr_ctrl.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-2
 * @brief   电源控制服务 — FSM 上电时序（含电机预充电软启动）+ 状态采集
 *
 * 包含电源 FSM 与预充电 FSM 双状态机及电源轨使能逻辑，直接驱动
 * drv_power / drv_pwm / drv_status / srv_adc（Style B，自包含）。
 * 不管理 sw_timer，由 task 层定期调用 srv_pwr_ctrl_step() 推进状态。
 */

#ifndef __SRV_PWR_CTRL_H
#define __SRV_PWR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化电源控制服务（drv_power + drv_pwm + 双 FSM） */
void srv_pwr_ctrl_init(void);

/**
 * @brief 推进 FSM 状态机一步
 * @param elapsed_ms 距离上次调用经过的毫秒数
 * @note  由 task 层 sw_timer 周期调用（1ms）
 */
void srv_pwr_ctrl_step(uint16_t elapsed_ms);

/** @brief 请求上电（异步，FSM 自动推进） */
void srv_pwr_ctrl_request_on(void);

/** @brief 紧急断电（同步，立即关闭所有输出并复位 FSM） */
void srv_pwr_ctrl_emergency_off(void);

/** @brief 是否已完成上电流程 */
bool srv_pwr_ctrl_is_powered_on(void);

/**
 * @brief 辅助电源使能是否已驱动（AUX_POWER_EN 状态）
 * @note  供故障策略门控 PGD 判定：AUX_EN=0 时 AUX_PGD 低为正常，不应判故障
 */
bool srv_pwr_ctrl_is_aux_enabled(void);

/**
 * @brief 电机电源使能是否已驱动（MOTOR_POWER_EN 状态）
 * @note  供故障策略门控 PGD 判定：MOTOR_EN=0 时 MOTOR_PGD 低为正常，不应判故障
 */
bool srv_pwr_ctrl_is_motor_enabled(void);

/**
 * @brief 读取电机预充电故障码
 * @return 0=无故障, 1=后级短路 (SHORT_CIRCUIT), 2=未接负载/上电故障 (NO_LOAD)
 */
uint8_t srv_pwr_ctrl_get_precharge_fault(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_CTRL_H */
