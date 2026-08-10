/**
 * @file    drv_stm32g0_flash.h
 * @brief   STM32G0B1 Flash 底层驱动 — 适配 hal_flash 抽象层
 *
 * 导出 g0_ops (VTable) 和 g0_priv (私有数据) 供 HAL 层引用。
 * 仅在定义了 HAL_FLASH_CHIP_STM32G0 时内容可见。
 */

#ifndef __DRV_STM32G0_FLASH_H
#define __DRV_STM32G0_FLASH_H

#include "hal_flash_base.h"

#ifdef HAL_FLASH_CHIP_STM32G0

#ifdef __cplusplus
extern "C" {
#endif

/* ====== G0 私有数据类型 ====================================================*/

typedef struct {
    uint32_t page_size;
    uint32_t bank_size;
} g0_priv_data_t;

/* ====== 导出符号 ===========================================================*/

extern g0_priv_data_t g0_priv;
extern hal_flash_dev_t g0_dev;

#ifdef __cplusplus
}
#endif

#endif /* HAL_FLASH_CHIP_STM32G0 */

#endif /* __DRV_STM32G0_FLASH_H */
