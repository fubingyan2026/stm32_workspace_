/**
 * @file    srv_pwr_ctrl.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   电源控制服务 — FSM 上电时序 + 状态采集
 *
 * 包含完整的 FSM 状态机和电源轨使能逻辑。
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

/** @brief 初始化电源控制服务（drv_power + FSM） */
void srv_pwr_ctrl_init(void);

/**
 * @brief 推进 FSM 状态机一步
 * @param elapsed_ms 距离上次调用经过的毫秒数
 * @note  由 task 层 sw_timer 周期调用
 */
void srv_pwr_ctrl_step(uint16_t elapsed_ms);

/** @brief 请求上电（异步，FSM 自动推进） */
void srv_pwr_ctrl_request_on(void);

/** @brief 紧急断电（同步，立即关闭所有输出并复位 FSM） */
void srv_pwr_ctrl_emergency_off(void);

/** @brief 是否已完成上电流程 */
bool srv_pwr_ctrl_is_powered_on(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_CTRL_H */
