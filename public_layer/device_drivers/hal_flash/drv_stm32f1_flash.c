/**
 * @file    drv_stm32f1_flash.c
 * @brief   STM32F103 Flash 底层驱动 — 适配 hal_flash 抽象层
 *
 * F103C8Tx 共 64KB Flash（中容量），以 1KB 页为最小擦除单元：
 *   页 0-63：各 1KB
 *
 * 实现 hal_flash_ops_t 接口，导出 f1_ops 和 f1_sectors 供 HAL 层引用。
 * 仅在定义了 HAL_FLASH_CHIP_STM32F1 时编译，否则本文件为空。
 */

#include "hal_flash.h"

#ifdef HAL_FLASH_CHIP_STM32F1

/* Includes ------------------------------------------------------------------*/
#include "drv_stm32f1_flash.h"
#include "log.h"

#include "stm32f1xx_hal.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define FLASH_LOG_ENABLE 1

#if FLASH_LOG_ENABLE
#define FLASH_LOG_E(...) LOG_E("flash", __VA_ARGS__)
#define FLASH_LOG_W(...) LOG_W("flash", __VA_ARGS__)
#define FLASH_LOG_I(...) LOG_I("flash", __VA_ARGS__)
#define FLASH_LOG_D(...) LOG_D("flash", __VA_ARGS__)
#else
#define FLASH_LOG_E(...) ((void)0)
#define FLASH_LOG_W(...) ((void)0)
#define FLASH_LOG_I(...) ((void)0)
#define FLASH_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief F1 错误标志组合（F1 无 FLASH_FLAG_ALL_ERRORS 宏） */
#define DRV_FLASH_FLAG_ALL_ERRORS (FLASH_FLAG_BSY | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_EOP)

/* Private variables ---------------------------------------------------------*/

static uint32_t s_write_buf; /* Flash 写入临时缓冲区 (32-bit 对齐) */
static uint32_t s_read_buf; /* Flash 读取校验缓冲区 (32-bit 对齐) */

/* Private const -------------------------------------------------------------*/

static const uint32_t FLASH_PROGRAM_SIZE = 4; /* 编程粒度: 32-bit 字 */
static const uint32_t FLASH_ERASED_VAL = (~0U); /* Flash 擦除后默认值 */
static const uint32_t FLASH_BASE_ADDR = 0x08000000U; /* F1 Flash 基地址 */
static const uint32_t FLASH_TOTAL_SIZE = 64 * 1024U; /* 64 KB (F103C8Tx) */
static const uint32_t F1_FLASH_PAGE_SIZE = 0x400U; /* 1 KB 页 */

/** @brief STM32F103 页表（64KB Flash，64 页 × 1KB，中容量） */
const f1_sector_desc_t f1_sectors[64] = {
    { 0x08000000U + 0x400U * 0U, 0x400U },
    { 0x08000000U + 0x400U * 1U, 0x400U },
    { 0x08000000U + 0x400U * 2U, 0x400U },
    { 0x08000000U + 0x400U * 3U, 0x400U },
    { 0x08000000U + 0x400U * 4U, 0x400U },
    { 0x08000000U + 0x400U * 5U, 0x400U },
    { 0x08000000U + 0x400U * 6U, 0x400U },
    { 0x08000000U + 0x400U * 7U, 0x400U },
    { 0x08000000U + 0x400U * 8U, 0x400U },
    { 0x08000000U + 0x400U * 9U, 0x400U },
    { 0x08000000U + 0x400U * 10U, 0x400U },
    { 0x08000000U + 0x400U * 11U, 0x400U },
    { 0x08000000U + 0x400U * 12U, 0x400U },
    { 0x08000000U + 0x400U * 13U, 0x400U },
    { 0x08000000U + 0x400U * 14U, 0x400U },
    { 0x08000000U + 0x400U * 15U, 0x400U },
    { 0x08000000U + 0x400U * 16U, 0x400U },
    { 0x08000000U + 0x400U * 17U, 0x400U },
    { 0x08000000U + 0x400U * 18U, 0x400U },
    { 0x08000000U + 0x400U * 19U, 0x400U },
    { 0x08000000U + 0x400U * 20U, 0x400U },
    { 0x08000000U + 0x400U * 21U, 0x400U },
    { 0x08000000U + 0x400U * 22U, 0x400U },
    { 0x08000000U + 0x400U * 23U, 0x400U },
    { 0x08000000U + 0x400U * 24U, 0x400U },
    { 0x08000000U + 0x400U * 25U, 0x400U },
    { 0x08000000U + 0x400U * 26U, 0x400U },
    { 0x08000000U + 0x400U * 27U, 0x400U },
    { 0x08000000U + 0x400U * 28U, 0x400U },
    { 0x08000000U + 0x400U * 29U, 0x400U },
    { 0x08000000U + 0x400U * 30U, 0x400U },
    { 0x08000000U + 0x400U * 31U, 0x400U },
    { 0x08000000U + 0x400U * 32U, 0x400U },
    { 0x08000000U + 0x400U * 33U, 0x400U },
    { 0x08000000U + 0x400U * 34U, 0x400U },
    { 0x08000000U + 0x400U * 35U, 0x400U },
    { 0x08000000U + 0x400U * 36U, 0x400U },
    { 0x08000000U + 0x400U * 37U, 0x400U },
    { 0x08000000U + 0x400U * 38U, 0x400U },
    { 0x08000000U + 0x400U * 39U, 0x400U },
    { 0x08000000U + 0x400U * 40U, 0x400U },
    { 0x08000000U + 0x400U * 41U, 0x400U },
    { 0x08000000U + 0x400U * 42U, 0x400U },
    { 0x08000000U + 0x400U * 43U, 0x400U },
    { 0x08000000U + 0x400U * 44U, 0x400U },
    { 0x08000000U + 0x400U * 45U, 0x400U },
    { 0x08000000U + 0x400U * 46U, 0x400U },
    { 0x08000000U + 0x400U * 47U, 0x400U },
    { 0x08000000U + 0x400U * 48U, 0x400U },
    { 0x08000000U + 0x400U * 49U, 0x400U },
    { 0x08000000U + 0x400U * 50U, 0x400U },
    { 0x08000000U + 0x400U * 51U, 0x400U },
    { 0x08000000U + 0x400U * 52U, 0x400U },
    { 0x08000000U + 0x400U * 53U, 0x400U },
    { 0x08000000U + 0x400U * 54U, 0x400U },
    { 0x08000000U + 0x400U * 55U, 0x400U },
    { 0x08000000U + 0x400U * 56U, 0x400U },
    { 0x08000000U + 0x400U * 57U, 0x400U },
    { 0x08000000U + 0x400U * 58U, 0x400U },
    { 0x08000000U + 0x400U * 59U, 0x400U },
    { 0x08000000U + 0x400U * 60U, 0x400U },
    { 0x08000000U + 0x400U * 61U, 0x400U },
    { 0x08000000U + 0x400U * 62U, 0x400U },
    { 0x08000000U + 0x400U * 63U, 0x400U },
};

static const uint32_t FLASH_SECTOR_COUNT = sizeof(f1_sectors) / sizeof(f1_sectors[0]);

/* Private function prototypes -----------------------------------------------*/

static uint32_t find_sector(uint32_t addr)
{
    for (uint32_t i = 0; i < FLASH_SECTOR_COUNT; i++) {
        if (addr >= f1_sectors[i].base && addr < f1_sectors[i].base + f1_sectors[i].size) {
            return i;
        }
    }
    return FLASH_SECTOR_COUNT;
}

/* ====== hal_flash_ops_t 接口实现 ========================================== */

static hal_flash_err_t f1_init(void)
{
    FLASH_LOG_I("Init: F103 Flash, %lu pages, total=%luKB",
        (unsigned long)FLASH_SECTOR_COUNT,
        (unsigned long)(FLASH_TOTAL_SIZE >> 10));
    return HAL_FLASH_OK;
}

static hal_flash_err_t f1_read(uint32_t offset, uint8_t* buf, size_t size)
{
    uint32_t addr = FLASH_BASE_ADDR + offset;
    uint8_t* dst = buf;

    for (size_t i = 0; i < size; i++, addr++, dst++) {
        *dst = *(volatile uint8_t*)addr;
    }

    return HAL_FLASH_OK;
}

static hal_flash_err_t f1_erase(uint32_t offset, size_t size)
{
    hal_flash_err_t result = HAL_FLASH_OK;
    uint32_t error = 0;

    uint32_t addr = FLASH_BASE_ADDR + offset;
    uint32_t start_sector = find_sector(addr);
    uint32_t end_addr = addr + size;
    uint32_t end_sector = (size > 0) ? find_sector(end_addr - 1) : FLASH_SECTOR_COUNT;

    if (start_sector >= FLASH_SECTOR_COUNT || end_sector >= FLASH_SECTOR_COUNT) {
        FLASH_LOG_E("Erase: offset out of range, addr=0x%08lX, end=0x%08lX",
            (unsigned long)addr, (unsigned long)end_addr);
        return HAL_FLASH_OFFSET_ERR;
    }
    if (addr != f1_sectors[start_sector].base) {
        FLASH_LOG_E("Erase: start not sector-aligned, addr=0x%08lX, sector=%lu base=0x%08lX",
            (unsigned long)addr, (unsigned long)start_sector,
            (unsigned long)f1_sectors[start_sector].base);
        return HAL_FLASH_ALIGN_ERR;
    }
    if (end_addr != f1_sectors[end_sector].base + f1_sectors[end_sector].size) {
        FLASH_LOG_E("Erase: end not sector-aligned, end=0x%08lX, sector=%lu end=0x%08lX",
            (unsigned long)end_addr, (unsigned long)end_sector,
            (unsigned long)(f1_sectors[end_sector].base + f1_sectors[end_sector].size));
        return HAL_FLASH_ALIGN_ERR;
    }

    uint32_t nb_sectors = end_sector - start_sector + 1;

    FLASH_LOG_I("Erase: addr=0x%08lX, size=%lu, pages=%lu..%lu (%lu)",
        (unsigned long)addr, (unsigned long)size,
        (unsigned long)start_sector, (unsigned long)end_sector,
        (unsigned long)nb_sectors);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(DRV_FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = f1_sectors[start_sector].base,
        .NbPages = nb_sectors,
    };

    if (HAL_FLASHEx_Erase(&erase_init, &error) != HAL_OK) {
        FLASH_LOG_E("Erase error: addr=0x%08lX, page=%lu, HAL_Err=0x%08lX",
            (unsigned long)addr, (unsigned long)error,
            (unsigned long)HAL_FLASH_GetError());
        result = HAL_FLASH_ERASE_ERR;
    }

    HAL_FLASH_Lock();
    return result;
}

static hal_flash_err_t f1_write(uint32_t offset, const uint8_t* buf, size_t size)
{
    hal_flash_err_t result = HAL_FLASH_OK;
    uint32_t addr = FLASH_BASE_ADDR + offset;
    const uint8_t* src = buf;

    FLASH_LOG_D("Write: addr=0x%08lX, size=%lu",
        (unsigned long)addr, (unsigned long)size);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(DRV_FLASH_FLAG_ALL_ERRORS);

    for (size_t i = 0; i < size; i += FLASH_PROGRAM_SIZE) {
        size_t copy_len = (size - i >= FLASH_PROGRAM_SIZE) ? FLASH_PROGRAM_SIZE : (size - i);

        if (copy_len < FLASH_PROGRAM_SIZE) {
            s_write_buf = FLASH_ERASED_VAL;
            memcpy(&s_write_buf, src, copy_len);
        } else {
            memcpy(&s_write_buf, src, FLASH_PROGRAM_SIZE);
        }

        if (s_write_buf != FLASH_ERASED_VAL) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, s_write_buf) != HAL_OK) {
                FLASH_LOG_E("Flash program error: i=%u, addr=0x%08lX, HAL_Error=0x%08lX",
                    (unsigned)i, (unsigned long)addr,
                    (unsigned long)HAL_FLASH_GetError());
                result = HAL_FLASH_WRITE_ERR;
                goto exit_write;
            }
        }

        s_read_buf = *(volatile uint32_t*)addr;
        if (s_read_buf != s_write_buf) {
            FLASH_LOG_E("Flash readback mismatch: i=%u, addr=0x%08lX, "
                        "written=0x%08lX, readback=0x%08lX",
                (unsigned)i, (unsigned long)addr,
                (unsigned long)s_write_buf,
                (unsigned long)s_read_buf);
            result = HAL_FLASH_WRITE_ERR;
            goto exit_write;
        }

        addr += FLASH_PROGRAM_SIZE;
        src += copy_len;
    }

exit_write:
    HAL_FLASH_Lock();
    return result;
}

static void f1_cache_invalidate(void)
{
    /* F103 Cortex-M3 无 D-Cache，无需操作 */
}

static uint32_t f1_erase_size_at(uint32_t offset)
{
    uint32_t addr = FLASH_BASE_ADDR + offset;
    uint32_t idx = find_sector(addr);

    if (idx < FLASH_SECTOR_COUNT) {
        return f1_sectors[idx].size;
    }
    return 0;
}

/* ====== 设备实例导出 ======================================================== */

hal_flash_dev_t f1_dev = {
    .name = "stm32f1",
    .ops = {
        .init = f1_init,
        .read = f1_read,
        .write = f1_write,
        .erase = f1_erase,
        .erase_size_at = f1_erase_size_at,
        .cache_invalidate = f1_cache_invalidate,
    },
    .caps = {
        .addr = 0x08000000U,
        .total_size = FLASH_TOTAL_SIZE,
        .erase_size = F1_FLASH_PAGE_SIZE,
        .write_gran = HAL_FLASH_WRITE_GRAN_32,
        .erase_size_uniform = true,
        .has_ecc = false,
        .has_write_protect = false,
        .has_crc = false,
    },
    .priv = (void*)f1_sectors,
    .initialized = false,
    .lock_cb = NULL,
    .unlock_cb = NULL,
    .lock_depth = 0,
    .event_cb = NULL,
    .event_arg = NULL,
};

#endif /* HAL_FLASH_CHIP_STM32F1 */
