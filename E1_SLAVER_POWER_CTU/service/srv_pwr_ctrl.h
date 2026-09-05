/**
 * @file    srv_pwr_ctrl.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-05
 * @brief   电源输出监督服务 — 期望输出 + 门控使能 + 故障锁存 FSM (E1_SLAVER_POWER_CTU)
 *
 * 远程指令 + 门控保护模型：外部（485 控制帧）写入期望输出掩码，本服务按电源
 * 依赖逐步使能（DC-DC 24V → 12V_ISO / LSD1 / LSD2），PGOOD/节点电压门控 + 超时 +
 * 运行期丢失去抖（默认 100ms，防误关断）→ 关断并故障锁存；故障位与锁存经状态
 * 接口上报，主机发清除锁存命令后自动按当前期望重试。
 *
 * 状态机基于 fsm 库实现（与 E1_MASTER_POWER_CTU 同框架）：4 路输出各持一个独立
 * fsm_t 实例，状态 OFF/ENABLING/ON/LATCHED，handler 在 task 周期内按拍推进。
 *
 * 母线电压（AUX/MOTOR/LSD1/LSD2 mV）经 task 层注入的读取回调获取（读 srv_adc），
 * service 层之间不直接互调。
 *
 * 不管理 sw_timer，由 task 层定期调用 srv_pwr_ctrl_step() 推进状态。
 */

#ifndef __SRV_PWR_CTRL_H
#define __SRV_PWR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief 输出掩码位（bit 位置 = drv_power_rail_t 顺序 = 协议 ctrl/status 位顺序） */
#define SRV_PWR_OUT_MASK_DC24V  (0x01U) /**< bit0 — DC-DC 24V 使能 */
#define SRV_PWR_OUT_MASK_ISO12V (0x02U) /**< bit1 — 12V_ISO 使能 */
#define SRV_PWR_OUT_MASK_LSD1   (0x04U) /**< bit2 — LSD1 使能 */
#define SRV_PWR_OUT_MASK_LSD2   (0x08U) /**< bit3 — LSD2 使能 */
#define SRV_PWR_OUT_MASK_ALL    (0x0FU)

/* 阈值/时序（待实机确认，均集中于本处便于调整） */
#define SRV_PWR_AUX_PRESENT_MV      (20000UL) /**< AUX 输入存在判定阈值 (mV) */
#define SRV_PWR_MOTOR_PRESENT_MV    (20000UL) /**< MOTOR 输入存在判定阈值 (mV) */
#define SRV_PWR_LSD_NODE_ON_MAX_MV  (3000UL)  /**< LSD 使能后输出节点最大电压 (mV) */
#define SRV_PWR_READY_DEBOUNCE_MS   (10U)     /**< PGOOD/节点就绪稳定判定时间 (ms) */
#define SRV_PWR_LOSS_DEBOUNCE_MS    (100U)    /**< 运行期好状态丢失去抖时间 (ms, 防误关断) */
#define SRV_PWR_EN_TIMEOUT_DC24_MS  (800U)    /**< 24V 使能 PGOOD 超时 (ms) */
#define SRV_PWR_EN_TIMEOUT_ISO_MS   (500U)    /**< 12V_ISO 使能 PGOOD 超时 (ms) */
#define SRV_PWR_EN_TIMEOUT_LSD_MS   (100U)    /**< LSD 使能节点就绪超时 (ms) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 母线电压快照（task 层从 srv_adc 读取后经回调注入）
 */
typedef struct {
    bool valid; /**< 本次快照有效（ADC 已完成首轮采样） */
    uint32_t aux_mv;   /**< AUX 输入电压 (mV) */
    uint32_t motor_mv; /**< MOTOR 输入电压 (mV) */
    uint32_t lsd1_mv;  /**< LSD1 输出节点电压 (mV) */
    uint32_t lsd2_mv;  /**< LSD2 输出节点电压 (mV) */
} srv_pwr_voltage_t;

/** @brief 母线电压读取回调（task 层实现） */
typedef void (*srv_pwr_voltage_cb_t)(srv_pwr_voltage_t* volt);

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化电源输出监督服务（初始化 drv_power，全部输出关闭，状态复位） */
void srv_pwr_ctrl_init(void);

/** @brief 设置母线电压读取回调（task 层接线到 srv_adc；未设置时视 AUX/LSD 门控不满足） */
void srv_pwr_ctrl_set_voltage_cb(srv_pwr_voltage_cb_t cb);

/**
 * @brief 推进监督 FSM 一步
 * @param elapsed_ms 距离上次调用经过的毫秒数
 * @note  由 task 层 sw_timer 周期调用（1ms）
 */
void srv_pwr_ctrl_step(uint16_t elapsed_ms);

/**
 * @brief 写入期望输出掩码（整帧覆盖，来自 485 0x10 控制帧）
 * @param mask 期望使能位（SRV_PWR_OUT_MASK_*）
 * @note  被置 0 的锁存轨顺带清除该路锁存（视为人工关断确认）
 */
void srv_pwr_ctrl_request_outputs(uint8_t mask);

/** @brief 读取当前期望输出掩码 */
uint8_t srv_pwr_ctrl_get_desired(void);

/** @brief 读取当前实际处于 ON 的输出掩码（供状态上报） */
uint8_t srv_pwr_ctrl_get_on_outputs(void);

/**
 * @brief 强制锁存关断指定输出（app_fault_policy 运行期监控调用）
 * @param mask 需锁存关断的输出位（SRV_PWR_OUT_MASK_*）
 */
void srv_pwr_ctrl_latch_outputs(uint8_t mask);

/** @brief 读取当前故障锁存掩码（供状态上报） */
uint8_t srv_pwr_ctrl_get_latch_mask(void);

/** @brief 是否存在故障锁存 */
bool srv_pwr_ctrl_is_any_latched(void);

/**
 * @brief 清除全部故障锁存（来自 485 0x11 清除锁存命令）
 * @note  清除后处于 LATCHED 的输出回到 OFF，若期望仍为 ON 则自动重新使能
 */
void srv_pwr_ctrl_clear_latch(void);

/** @brief 紧急关断：清空期望并立即关闭全部输出（锁存保留，需 clear_latch 恢复） */
void srv_pwr_ctrl_emergency_off(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_CTRL_H */
