/**
 * @file    drv_stm32f1_flash.h
 * @brief   STM32F103 Flash 底层驱动 — 适配 hal_flash 抽象层
 *
 * 导出 f1_sectors (扇区表) 和 f1_dev 供 HAL 层引用。
 * 仅在定义了 HAL_FLASH_CHIP_STM32F1 时内容可见。
 */

#ifndef __DRV_STM32F1_FLASH_H
#define __DRV_STM32F1_FLASH_H

#include "hal_flash_base.h"

#ifdef HAL_FLASH_CHIP_STM32F1

#ifdef __cplusplus
extern "C" {
#endif

/* ====== 扇区描述类型 (内部使用) ============================================*/

typedef struct {
    uint32_t base;
    uint32_t size;
} f1_sector_desc_t;

/* ====== 导出符号 ===========================================================*/

extern const f1_sector_desc_t f1_sectors[];
extern hal_flash_dev_t f1_dev;

#ifdef __cplusplus
}
#endif

#endif /* HAL_FLASH_CHIP_STM32F1 */

#endif /* __DRV_STM32F1_FLASH_H */
