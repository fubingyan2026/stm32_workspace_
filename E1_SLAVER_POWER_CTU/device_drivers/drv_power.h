/**
 * @file    drv_power.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   电源输出使能控制驱动（E1_SLAVER_POWER_CTU 副电源模块）
 * @attention
 *
 * 配置表内置在 drv_power.c 中，外部只需调用 init。
 * 副电源模块使能输出为 4 路（均高有效）：
 *   - DC_DC_EN    (LM5146  24V/6A 降压电源使能, PA0)
 *   - ISO_EN_12V  (URB2412S 隔离 12V 模块使能, PA4)
 *   - LSD1_IN     (ZXMS6004FF 低边开关 1 驱动, PA2)
 *   - LSD2_IN     (ZXMS6004FF 低边开关 2 驱动, PA3)
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
    DRV_POWER_RAIL_DC24V,        /**< DC_DC_EN — 24V/6A 降压电源使能 (LM5146, PA0) */
    DRV_POWER_RAIL_ISO12V,       /**< ISO_EN_12V — 隔离 12V 模块使能 (PA4) */
    DRV_POWER_RAIL_LSD1,         /**< LSD1_IN — 低边开关 1 驱动 (PA2) */
    DRV_POWER_RAIL_LSD2,         /**< LSD2_IN — 低边开关 2 驱动 (PA3) */

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
