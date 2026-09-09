/**
 * @file    drv_stm32f1_flash.h
 * @brief   STM32F103 Flash 底层驱动 — 适配 hal_flash 抽象层
 *
 * 导出 f1_ops (VTable) 和 f1_priv (私有数据) 供 HAL 层引用。
 * 仅在定义了 HAL_FLASH_CHIP_STM32F1 时内容可见。
 *
 * 注意：F103xE 全片 Flash 页均匀 2KB（FLASH_PAGE_SIZE=0x800），
 *       编程粒度为 32-bit WORD（HAL 支持 FLASH_TYPEPROGRAM_WORD）。
 */

#ifndef __DRV_STM32F1_FLASH_H
#define __DRV_STM32F1_FLASH_H

#include "hal_flash_base.h"

#ifdef HAL_FLASH_CHIP_STM32F1

#ifdef __cplusplus
extern "C" {
#endif

/* ====== F1 私有数据类型 ====================================================*/

typedef struct {
    uint32_t page_size;  /**< 页大小（2KB） */
    uint32_t total_size; /**< Flash 总大小（字节） */
} f1_priv_data_t;

/* ====== 导出符号 ===========================================================*/

extern f1_priv_data_t f1_priv;
extern hal_flash_dev_t f1_dev;

#ifdef __cplusplus
}
#endif

#endif /* HAL_FLASH_CHIP_STM32F1 */

#endif /* __DRV_STM32F1_FLASH_H */
