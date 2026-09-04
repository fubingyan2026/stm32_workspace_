/**
 * @file    srv_pwr_ctrl.h
 * @author  maximillian
 * @version V3.0.0
 * @date    2026-09-04
 * @brief   电源控制服务 — 三路默认常开 + MOTOR 独立受控 (E1_MASTER_POWER_CTU)
 *
 * 电源语义（量产板）：
 * - VIN_DC-DC(LM5060) / DC_DC_24V(MP9931N) / AUX(LM5069) 三路**默认常开**：
 *   初始化即直接使能，不随 FSM/急停开关；其 PGOOD 仅作状态监测与上报（见 srv_pwr_det），
 *   不作为 MOTOR 使能的门控，其中任何一路异常不再阻塞/影响 MOTOR 的受控。
 * - MOTOR_POWER(LM5069) **独立受控**：request_on() 使能，emergency_off() 无条件强制
 *   关断（任何状态下都直接拉低 MOTOR_EN，不受三路健康影响）。
 *
 * 内部为 fsm 库两态机：IDLE(motor 关) ↔ POWERED(motor 开)。不管理 sw_timer，
 * 由 task 层定期调用 srv_pwr_ctrl_step() 推进。
 */

#ifndef __SRV_PWR_CTRL_H
#define __SRV_PWR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化电源控制服务
 * @note  内部完成 drv_power 初始化并**立即使能 VIN_DC-DC / DC24V / AUX 三路**，
 *        MOTOR 保持关闭，FSM 进入 IDLE
 */
void srv_pwr_ctrl_init(void);

/**
 * @brief 推进 FSM 状态机一步
 * @param elapsed_ms 距离上次调用经过的毫秒数（当前两态机下仅作参数保留）
 * @note  由 task 层 sw_timer 周期调用
 */
void srv_pwr_ctrl_step(uint16_t elapsed_ms);

/**
 * @brief 使能 MOTOR（异步，下一拍由 FSM 进入 POWERED 时拉高 MOTOR_EN）
 * @note  不依赖三路常开轨的 PGD 状态
 */
void srv_pwr_ctrl_request_on(void);

/**
 * @brief 紧急断电：无条件直接强制 MOTOR_EN=0 并回到 IDLE
 * @note  MOTOR 关断与 FSM 当前状态/三路健康无关；三路常开轨保持使能
 */
void srv_pwr_ctrl_emergency_off(void);

/** @brief MOTOR 是否已使能（FSM 处于 POWERED） */
bool srv_pwr_ctrl_is_powered_on(void);

/** @brief 各轨当前使能状态查询（三路常开轨初始化后恒为 true，MOTOR 视 FSM） */
bool srv_pwr_ctrl_is_vin_enabled(void);
bool srv_pwr_ctrl_is_dc24v_enabled(void);
bool srv_pwr_ctrl_is_aux_enabled(void);
bool srv_pwr_ctrl_is_motor_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_CTRL_H */
