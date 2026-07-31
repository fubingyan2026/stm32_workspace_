/**
 * @file    drv_stm32h7_flash.h
 * @brief   STM32H723 Flash 底层驱动 — 适配 hal_flash 抽象层
 *
 * 导出 h7_ops (VTable) 和 h7_sectors (扇区表) 供 HAL 层引用。
 * 仅在定义了 HAL_FLASH_CHIP_STM32H7 时内容可见。
 *
 * H723VG: 1MB Flash, 8 × 128KB 均匀扇区, 单 Bank, 32-byte FLASHWORD 编程
 */

#ifndef __DRV_STM32H7_FLASH_H
#define __DRV_STM32H7_FLASH_H

#include "hal_flash_base.h"

#ifdef HAL_FLASH_CHIP_STM32H7

#ifdef __cplusplus
extern "C" {
#endif

/* ====== 扇区描述类型 (内部使用) ============================================*/

typedef struct {
    uint32_t base;
    uint32_t size;
} h7_sector_desc_t;

/* ====== 导出符号 ===========================================================*/

extern const h7_sector_desc_t h7_sectors[];
extern hal_flash_dev_t h7_dev;

#ifdef __cplusplus
}
#endif

#endif /* HAL_FLASH_CHIP_STM32H7 */

#endif /* __DRV_STM32H7_FLASH_H */
