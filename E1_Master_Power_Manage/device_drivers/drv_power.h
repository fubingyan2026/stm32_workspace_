/**
 * @file    drv_power.h
 * @author  maximillian
 * @version V2.2.0
 * @date    2026-07-2
 * @brief   电源轨使能控制驱动
 * @attention
 *
 * 配置表内置在 drv_power.c 中，外部只需调用 init。
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
    DRV_POWER_RAIL_HSD1_12V,     /**< 12V_HSD1_IN — 12V HSD 通道控制 */
    DRV_POWER_RAIL_HSD1_24V,     /**< 24V_HSD1_IN — 24V HSD1 通道控制 */
    DRV_POWER_RAIL_HSD2_24V,     /**< 24V_HSD2_IN — 24V HSD2 通道控制 */
    DRV_POWER_RAIL_HSD1_12V_DIAG,/**< 12V_HSD1_DIAG_EN — 12V HSD1 诊断使能 */
    DRV_POWER_RAIL_HSD1_24V_DIAG,/**< 24V_HSD1_DIAG_EN — 24V HSD1 诊断使能 */
    DRV_POWER_RAIL_HSD2_24V_DIAG,/**< 24V_HSD2_DIAG_EN — 24V HSD2 诊断使能 */
    DRV_POWER_RAIL_AUX_EN,       /**< AUX_POWER_EN — 辅助(副电源控制板)电源使能 */
    DRV_POWER_RAIL_MOTOR_EN,     /**< MOTOR_POWER_EN — 电机电源使能 */
    DRV_POWER_RAIL_MOTOR_CHG_EN, /**< MOTOR_POWER_CHG_EN — 电机预充电使能 */
    DRV_POWER_RAIL_DBR_LSD_EN,   /**< DBR_LSD_EN — 动态制动低边驱动使能 */
    DRV_POWER_RAIL_DC_DC_EN,     /**< DC_DC_EN — DC-DC电源转换器使能 */

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
