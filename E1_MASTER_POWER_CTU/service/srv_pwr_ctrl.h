/**
 * @file    srv_pwr_ctrl.h
 * @author  maximillian
 * @version V5.0.0
 * @date    2026-09-05
 * @brief   电源控制服务 — 默认 5 步上电流程 + MOTOR 独立急停控制 (E1_MASTER_POWER_CTU)
 *
 * ## 上电（程序默认启动，初始化即自动执行，无需请求）
 *   1. 采样 VIN：满足 36~58V
 *   2. 检测 VIN_DC-DC ≥ 90%·VIN
 *   3. 使能 VIN_DC-DC_EN，延迟 100ms
 *   4. 使能 DC_DC_24V_EN
 *   5. DC_DC_24V_PGOOD 与 LM5060_PGOOD 均高 → 上电成功(POWERED)
 *
 * ## 电源轨语义
 * - AUX：**初始化即直接使能**，不等待其他轨上电成功（直通电源）。
 * - VIN_DC-DC / 24V：按上述流程门控；上电成功后持续供电。
 * - **急停/异常不关断 VIN/24V/AUX**（仅 MOTOR 受控）。
 * - MOTOR_EN 为受控负载轨：由上层（app_fault_policy）按「急停/故障状态」调用
 *   srv_pwr_ctrl_motor_set() 开/关，急停只控制 MOTOR。
 *
 * 电压数据经注入回调读取（task 层聚合 srv_adc），避免 service 同层互引。
 */

#ifndef __SRV_PWR_CTRL_H
#define __SRV_PWR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/** @brief 电压读取回调（task 层接线，通常读 srv_adc 最新采样） */
typedef void (*srv_pwr_ctrl_volt_cb_t)(uint32_t* vin_mv, uint32_t* vin_dcdc_mv);

/** @brief 服务配置 */
typedef struct {
    srv_pwr_ctrl_volt_cb_t read_voltage; /**< 电压读取回调（必填；缺失时跳过电压门控直接执行后续使能） */
} srv_pwr_ctrl_config_t;

/** @brief 电源状态快照（一次调用取全部） */
typedef struct {
    bool powered_on; /**< 上电流程成功 (POWERED) */
    bool vin_en;     /**< VIN_DC-DC_EN 已使能 */
    bool dc24v_en;   /**< DC_DC_24V_EN 已使能 */
    bool aux_en;     /**< AUX_EN 已使能 */
    bool motor_en;   /**< MOTOR_EN 已使能 */
} srv_pwr_ctrl_state_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化并自动启动默认上电流程（drv_power 复位后 FSM 进入 CHECK_VIN）
 * @param config 配置（read_voltage 由 power_task 注入）
 */
void srv_pwr_ctrl_init(const srv_pwr_ctrl_config_t* config);

/**
 * @brief 推进上电 FSM 一步
 * @param elapsed_ms 距上次调用经过的毫秒数（1ms）
 */
void srv_pwr_ctrl_step(uint16_t elapsed_ms);

/**
 * @brief MOTOR_EN 开/关（受控负载轨，由上层故障策略按急停状态驱动）
 * @note  仅操作 MOTOR；VIN/24V/AUX 常供电源轨不受影响。
 *        延后设定：on=true 且尚未上电成功(POWERED，即 VIN_DC-DC/24V/AUX 启动完成)时，
 *        仅记录请求，POWERED 后自动生效；off=false 任何时刻立即生效。
 */
void srv_pwr_ctrl_motor_set(bool on);

/** @brief 获取电源状态快照（上电完成标志 + 各轨使能标志） */
srv_pwr_ctrl_state_t srv_pwr_ctrl_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_CTRL_H */
