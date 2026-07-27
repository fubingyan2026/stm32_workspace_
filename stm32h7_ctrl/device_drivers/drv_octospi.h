/**
 * @file    drv_octospi.h
 * @brief   OCTOSPI2 Quad SPI Flash 驱动 — Micron 8MB, 80MHz
 *
 * 支持间接读写 + 内存映射模式 (AXI 0x90000000)。
 * 使用 Micron 标准四路 SPI 命令集。
 */

#ifndef __DRV_OSPI_H
#define __DRV_OSPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ====== 错误码 =============================================================*/

typedef enum {
    DRV_OSPI_OK = 0,
    DRV_OSPI_ERR_PARAM      = -1,
    DRV_OSPI_ERR_INIT       = -2,
    DRV_OSPI_ERR_TIMEOUT    = -3,
    DRV_OSPI_ERR_ERASE      = -4,
    DRV_OSPI_ERR_WRITE      = -5,
    DRV_OSPI_ERR_READ       = -6,
    DRV_OSPI_ERR_NOT_FOUND  = -7,
} drv_ospi_error_t;

/* ====== API ================================================================*/

drv_ospi_error_t drv_ospi_init(void);

drv_ospi_error_t drv_ospi_read(uint32_t address, uint8_t* data, uint32_t len);
drv_ospi_error_t drv_ospi_write(uint32_t address, const uint8_t* data, uint32_t len);
drv_ospi_error_t drv_ospi_erase_sector(uint32_t address);
drv_ospi_error_t drv_ospi_erase_chip(void);

drv_ospi_error_t drv_ospi_enter_memory_mapped_mode(void);
drv_ospi_error_t drv_ospi_exit_memory_mapped_mode(void);
const uint8_t*   drv_ospi_get_mmap_ptr(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_OSPI_H */
