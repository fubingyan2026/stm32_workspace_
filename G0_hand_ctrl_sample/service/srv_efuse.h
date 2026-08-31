/**
 * @file    srv_efuse.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-31
 * @brief   24V eFuse 故障保护服务（状态机）
 * @attention
 *
 * 状态机：
 *   OFF     — SHDN=0，24V 关断
 *   STARTUP — SHDN=1，等待 PGOOD 拉高（上电中/输出未达门限）
 *   RUN     — PGOOD=1 且 FLT_N=1，24V 正常输出
 *   FAULT   — FLT_N=0 故障锁存（SHDN=0 自动关断），需 disable+enable 复位
 *
 * 保护策略：
 *   - STARTUP 超时（PGOOD 未就绪）→ FAULT 锁存
 *   - RUN 中 FLT_N=0 → 立即关断 24V 并锁存 FAULT
 *   - FAULT 状态不自动重试（安全侧），由上层确认后重新使能
 */

#ifndef SRV_EFUSE_H
#define SRV_EFUSE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief eFuse 工作状态
 */
typedef enum {
    SRV_EFUSE_STATE_OFF = 0, /**< 关断（SHDN=0） */
    SRV_EFUSE_STATE_STARTUP, /**< 上电中（等待 PGOOD） */
    SRV_EFUSE_STATE_RUN,     /**< 正常输出 24V */
    SRV_EFUSE_STATE_FAULT,   /**< 故障锁存（SHDN=0） */
    SRV_EFUSE_STATE_MAX,
} srv_efuse_state_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 eFuse 故障保护服务
 * @note  需在 drv_efuse_init() 之后调用；初始状态 OFF（SHDN=0 安全侧）
 */
void srv_efuse_init(void);

/**
 * @brief 周期步进：状态机扫描 + 故障检测（task 层 sw_timer 周期调用）
 * @param elapsed_ms 距上次调用经过的毫秒数
 */
void srv_efuse_step(uint16_t elapsed_ms);

/** @brief 请求开启 24V 输出（OFF → STARTUP） */
void srv_efuse_enable(void);

/** @brief 请求关断 24V 输出（任意状态 → OFF，同时清除故障锁存） */
void srv_efuse_disable(void);

/** @brief 获取当前 eFuse 状态 */
srv_efuse_state_t srv_efuse_get_state(void);

/** @brief 24V 输出是否正常（RUN） */
bool srv_efuse_is_power_ok(void);

/** @brief 是否处于故障锁存 */
bool srv_efuse_is_fault(void);

/** @brief 累计故障次数（自上次 disable 复位以来） */
uint32_t srv_efuse_get_fault_count(void);

#ifdef __cplusplus
}
#endif

#endif /* SRV_EFUSE_H */
