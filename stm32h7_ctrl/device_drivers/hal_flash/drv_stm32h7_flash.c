/**
 * @file    drv_stm32h7_flash.c
 * @brief   STM32H723 Flash 底层驱动 — 适配 hal_flash 抽象层
 *
 * H723VG: 1MB Flash, 8 × 128KB 均匀扇区, 单 Bank,
 *         FLASHWORD (256-bit = 32 bytes) 编程, 支持 ECC.
 *
 * 实现 hal_flash_ops_t 接口，导出 h7_ops 和 h7_sectors 供 HAL 层引用。
 * 仅在定义了 HAL_FLASH_CHIP_STM32H7 时编译，否则本文件为空。
 */

#include "hal_flash.h"

#ifdef HAL_FLASH_CHIP_STM32H7

/* Includes ------------------------------------------------------------------*/
#include "drv_stm32h7_flash.h"
#include "log.h"

#include "stm32h7xx_hal.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

#define FLASH_LOG_ENABLE 0

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

/** @brief H7 编程粒度: 256-bit FLASHWORD (32 bytes) */
#define H7_FLASHWORD_SIZE         32U

/** @brief Flash 擦除后默认值 (256-bit all 0xFF) */
#define H7_FLASHWORD_ERASED_VAL   0xFFFFFFFFFFFFFFFFULL

/** @brief Flash 基地址 */
#define FLASH_BASE_ADDR           0x08000000U
/** @brief Flash 总容量 (1 MB) */
#define FLASH_TOTAL_SIZE          0x00100000U
/** @brief 均匀扇区大小 (128 KB) */
#define H7_SECTOR_SIZE            0x00020000U

/** @brief STM32H723VG 扇区表（1MB Flash，8 扇区 × 128KB，均匀） */
const h7_sector_desc_t h7_sectors[] = {
    { FLASH_BASE_ADDR + 0x00000U, H7_SECTOR_SIZE }, /* 扇区 0: 128 KB */
    { FLASH_BASE_ADDR + 0x20000U, H7_SECTOR_SIZE }, /* 扇区 1: 128 KB */
    { FLASH_BASE_ADDR + 0x40000U, H7_SECTOR_SIZE }, /* 扇区 2: 128 KB */
    { FLASH_BASE_ADDR + 0x60000U, H7_SECTOR_SIZE }, /* 扇区 3: 128 KB */
    { FLASH_BASE_ADDR + 0x80000U, H7_SECTOR_SIZE }, /* 扇区 4: 128 KB */
    { FLASH_BASE_ADDR + 0xA0000U, H7_SECTOR_SIZE }, /* 扇区 5: 128 KB */
    { FLASH_BASE_ADDR + 0xC0000U, H7_SECTOR_SIZE }, /* 扇区 6: 128 KB */
    { FLASH_BASE_ADDR + 0xE0000U, H7_SECTOR_SIZE }, /* 扇区 7: 128 KB */
};

static const uint32_t FLASH_SECTOR_COUNT = sizeof(h7_sectors) / sizeof(h7_sectors[0]);

/* Private variables ---------------------------------------------------------*/

/* FLASHWORD 写入临时缓冲区 (32-byte aligned for H7 FLASHWORD) */
static __ALIGNED(32) uint8_t s_write_buf[H7_FLASHWORD_SIZE];
static uint64_t s_read_buf; /* 读回校验缓冲区 */

/* Private function prototypes -----------------------------------------------*/

static uint32_t find_sector(uint32_t addr)
{
    for (uint32_t i = 0; i < FLASH_SECTOR_COUNT; i++) {
        if (addr >= h7_sectors[i].base && addr < h7_sectors[i].base + h7_sectors[i].size) {
            return i;
        }
    }
    return FLASH_SECTOR_COUNT;
}

/**
 * @brief 擦除/写入后刷新 CPU 缓存，确保读取到最新数据
 */
static void h7_cache_invalidate_procedure(void)
{
    SCB_DisableDCache();
    SCB_InvalidateDCache();
    SCB_EnableDCache();

    SCB_InvalidateICache();
}

/* ====== hal_flash_ops_t 接口实现 ========================================== */

static hal_flash_err_t h7_init(void)
{
    FLASH_LOG_I("初始化: H723 Flash, %lu 个扇区, 总容量=%luKB",
        (unsigned long)FLASH_SECTOR_COUNT,
        (unsigned long)(FLASH_TOTAL_SIZE >> 10));
    return HAL_FLASH_OK;
}

static hal_flash_err_t h7_read(uint32_t offset, uint8_t* buf, size_t size)
{
    uint32_t addr = FLASH_BASE_ADDR + offset;
    uint8_t* dst = buf;

    for (size_t i = 0; i < size; i++, addr++, dst++) {
        *dst = *(volatile uint8_t*)addr;
    }

    return HAL_FLASH_OK;
}

static hal_flash_err_t h7_erase(uint32_t offset, size_t size)
{
    hal_flash_err_t result = HAL_FLASH_OK;
    uint32_t sector_error = 0;

    uint32_t addr = FLASH_BASE_ADDR + offset;
    uint32_t start_sector = find_sector(addr);
    uint32_t end_addr = addr + size;
    uint32_t end_sector = (size > 0) ? find_sector(end_addr - 1) : FLASH_SECTOR_COUNT;

    if (start_sector >= FLASH_SECTOR_COUNT || end_sector >= FLASH_SECTOR_COUNT) {
        FLASH_LOG_E("擦除: 偏移越界, addr=0x%08lX, end=0x%08lX",
            (unsigned long)addr, (unsigned long)end_addr);
        return HAL_FLASH_OFFSET_ERR;
    }
    if (addr != h7_sectors[start_sector].base) {
        FLASH_LOG_E("擦除: 起始地址未扇区对齐, addr=0x%08lX, sector=%lu base=0x%08lX",
            (unsigned long)addr, (unsigned long)start_sector,
            (unsigned long)h7_sectors[start_sector].base);
        return HAL_FLASH_ALIGN_ERR;
    }
    if (end_addr != h7_sectors[end_sector].base + h7_sectors[end_sector].size) {
        FLASH_LOG_E("擦除: 结束地址未扇区对齐, end=0x%08lX, sector=%lu end=0x%08lX",
            (unsigned long)end_addr, (unsigned long)end_sector,
            (unsigned long)(h7_sectors[end_sector].base + h7_sectors[end_sector].size));
        return HAL_FLASH_ALIGN_ERR;
    }

    uint32_t nb_sectors = end_sector - start_sector + 1;

    FLASH_LOG_I("擦除: addr=0x%08lX, size=%lu, 扇区=%lu..%lu (%lu)",
        (unsigned long)addr, (unsigned long)size,
        (unsigned long)start_sector, (unsigned long)end_sector,
        (unsigned long)nb_sectors);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase   = FLASH_TYPEERASE_SECTORS,
        .Banks       = FLASH_BANK_1,
        .Sector      = start_sector,
        .NbSectors   = nb_sectors,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };

    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        FLASH_LOG_E("擦除错误: addr=0x%08lX, sector=%lu, HAL_Err=0x%08lX",
            (unsigned long)addr, (unsigned long)sector_error,
            (unsigned long)HAL_FLASH_GetError());
        result = HAL_FLASH_ERASE_ERR;
    }

    HAL_FLASH_Lock();

    h7_cache_invalidate_procedure();

    return result;
}

static hal_flash_err_t h7_write(uint32_t offset, const uint8_t* buf, size_t size)
{
    hal_flash_err_t result = HAL_FLASH_OK;
    uint32_t addr = FLASH_BASE_ADDR + offset;
    const uint8_t* src = buf;

    FLASH_LOG_I("写入: addr=0x%08lX, size=%lu",
        (unsigned long)addr, (unsigned long)size);

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    for (size_t i = 0; i < size; i += H7_FLASHWORD_SIZE) {
        size_t copy_len = (size - i >= H7_FLASHWORD_SIZE) ? H7_FLASHWORD_SIZE : (size - i);

        /* 不足 FLASHWORD 的部分用 0xFF 填充 */
        memset(s_write_buf, 0xFF, H7_FLASHWORD_SIZE);
        memcpy(s_write_buf, src, copy_len);

        /* 检查是否为全 0xFF → 跳过编程（已擦除状态无需写入） */
        bool all_erased = true;
        for (uint32_t j = 0; j < H7_FLASHWORD_SIZE; j += sizeof(uint64_t)) {
            if (*(uint64_t*)(s_write_buf + j) != H7_FLASHWORD_ERASED_VAL) {
                all_erased = false;
                break;
            }
        }

        if (!all_erased) {
            /* H7 FLASHWORD 编程: 参数为 256-bit (32 bytes) 数据的地址 */
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)(uintptr_t)s_write_buf) != HAL_OK) {
                FLASH_LOG_E("Flash 编程错误: i=%u, addr=0x%08lX, HAL_Error=0x%08lX",
                    (unsigned)i, (unsigned long)addr,
                    (unsigned long)HAL_FLASH_GetError());
                result = HAL_FLASH_WRITE_ERR;
                goto exit_write;
            }
        }

        /* 读回校验：逐 64-bit 比较 */
        for (uint32_t j = 0; j < H7_FLASHWORD_SIZE; j += sizeof(uint64_t)) {
            s_read_buf = *(volatile uint64_t*)(addr + j);
            uint64_t expect = *(uint64_t*)(s_write_buf + j);
            if (s_read_buf != expect) {
                FLASH_LOG_E("Flash 读回不匹配: i=%u, addr=0x%08lX+%u, "
                            "written=0x%016llX, readback=0x%016llX",
                    (unsigned)i, (unsigned long)addr, (unsigned)j,
                    (unsigned long long)expect,
                    (unsigned long long)s_read_buf);
                result = HAL_FLASH_WRITE_ERR;
                goto exit_write;
            }
        }

        addr += H7_FLASHWORD_SIZE;
        src  += copy_len;
    }

exit_write:
    HAL_FLASH_Lock();

    h7_cache_invalidate_procedure();

    return result;
}

static void h7_cache_invalidate(void)
{
    h7_cache_invalidate_procedure();
}

/* ====== 设备实例导出 ======================================================== */

hal_flash_dev_t h7_dev = {
    .name = "stm32h7",
    .ops = {
        .init             = h7_init,
        .read             = h7_read,
        .write            = h7_write,
        .erase            = h7_erase,
        .cache_invalidate = h7_cache_invalidate,
        /* .erase_size_at = NULL — 均匀扇区，使用 caps.erase_size */
    },
    .caps = {
        .addr                = FLASH_BASE_ADDR,
        .total_size          = FLASH_TOTAL_SIZE,
        .erase_size          = H7_SECTOR_SIZE,
        .write_gran          = HAL_FLASH_WRITE_GRAN_256, /* 256-bit FLASHWORD */
        .erase_size_uniform  = true,
        .has_ecc             = true,
        .has_write_protect   = true,
        .has_crc             = false,
    },
    .priv               = (void*)h7_sectors,
    .initialized        = false,
    .lock_cb            = NULL,
    .unlock_cb          = NULL,
    .lock_depth         = 0,
    .event_cb           = NULL,
    .event_arg          = NULL,
};

#endif /* HAL_FLASH_CHIP_STM32H7 */
