/**
 * @file    drv_power.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   电源轨使能控制驱动（E1_MASTER_POWER_CTU 量产主控板）
 * @attention
 *
 * 配置表内置在 drv_power.c 中，外部只需调用 init。
 * CTU 板使能轨为 4 路（均高有效）：
 *   - VIN_DC-DC_EN   (LM5060  电源前端使能, PA11)
 *   - DC_DC_24V_EN   (MP9931N 24V/20A 降压使能, PB7)
 *   - AUX_POWER_EN   (LM5069  辅助电源热插拔使能, PD2)
 *   - MOTOR_POWER_EN (LM5069  电机电源热插拔使能, PC11)
 */

#ifndef __DRV_POWER_H
#define __DRV_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    DRV_POWER_RAIL_VIN_DCDC,     /**< VIN_DC-DC_EN — DC-DC 电源前端使能 (LM5060) */
    DRV_POWER_RAIL_DC24V,        /**< DC_DC_24V_EN — 24V/20A 降压电源使能 (MP9931N) */
    DRV_POWER_RAIL_AUX,          /**< AUX_POWER_EN — 辅助电源热插拔使能 (LM5069) */
    DRV_POWER_RAIL_MOTOR,        /**< MOTOR_POWER_EN — 电机电源热插拔使能 (LM5069) */

    DRV_POWER_RAIL_NUM,
} drv_power_rail_t;

/* Exported functions prototypes ---------------------------------------------*/

void drv_power_init(void);
void drv_power_deinit(void);

void drv_power_set(drv_power_rail_t rail, bool on);

void drv_power_toggle(drv_power_rail_t rail);

/** @brief 获取电源轨名称字符串 */
const char* drv_power_rail_name(drv_power_rail_t rail);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_POWER_H */
