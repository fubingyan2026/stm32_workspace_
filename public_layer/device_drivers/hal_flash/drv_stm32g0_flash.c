/**
 * @file    drv_stm32g0_flash.c
 * @brief   STM32G0B1 Flash 底层驱动 — 适配 hal_flash 抽象层
 *
 * 实现 hal_flash_ops_t 接口，导出 g0_ops 和 g0_priv 供 HAL 层引用。
 * G0B1CB：128 KB Flash，2 KB 页，64-bit 双字编程。
 * 支持单/双 Bank 自动检测（OPTR.DUAL_BANK），跨 Bank 擦除自动拆分。
 * 仅在定义了 HAL_FLASH_CHIP_STM32G0 时编译，否则本文件为空。
 */

#include "hal_flash.h"

#ifdef HAL_FLASH_CHIP_STM32G0

/* Includes ------------------------------------------------------------------*/
#include "drv_stm32g0_flash.h"
#include "log.h"

#include "stm32g0xx_hal.h"

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

static const uint32_t FLASH_PROGRAM_SIZE = 8;
static const uint64_t FLASH_ERASED_VAL = (~0ULL);
static const uint32_t FLASH_BASE_ADDR = 0x08000000U;
static const uint32_t FLASH_TOTAL_SIZE = 128 * 1024U; /* 128 KB */

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint64_t s_write_buf;
static uint64_t s_read_buf;

/* G0 私有数据实例 (HAL 层 s_dev.priv 引用) */
g0_priv_data_t g0_priv = {
    .page_size = 0, /* init 中动态检测 */
    .bank_size = 0,
};

/* Private function prototypes -----------------------------------------------*/

static uint32_t get_page_size(const g0_priv_data_t* priv)
{
    return priv->page_size;
}

static uint32_t get_bank_size(const g0_priv_data_t* priv)
{
    return priv->bank_size;
}

static uint32_t get_bank_number(uint32_t addr)
{
    /* 单 Bank 时无跨 Bank 需求（bank_size == FLASH_SIZE） */
    if (get_bank_size(&g0_priv) >= FLASH_TOTAL_SIZE) {
        return FLASH_BANK_1;
    }
    if (addr < (FLASH_BASE_ADDR + get_bank_size(&g0_priv))) {
        return FLASH_BANK_1;
    }
    return FLASH_BANK_2;
}

/* ====== hal_flash_ops_t 接口实现 ========================================== */

static hal_flash_err_t g0_init(void)
{
    if (g0_priv.page_size != 0U) {
        return HAL_FLASH_OK;
    }

    hal_flash_dev_t* dev = hal_flash_dev();

    /* G0B1 页大小固定 2 KB；Bank 大小视 OPTR.DUAL_BANK 而定：
     * 单 Bank = 128 KB（双字编程到整个 Flash），双 Bank = 各 64 KB。 */
    g0_priv.page_size = 0x800U;

#if defined(FLASH_DBANK_SUPPORT)
    if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DUAL_BANK) != 0U) {
        g0_priv.bank_size = FLASH_TOTAL_SIZE / 2U;
        FLASH_LOG_I("Init: dual-bank mode, page=%luKB, bank=%luKB",
            (unsigned long)(g0_priv.page_size >> 10),
            (unsigned long)(g0_priv.bank_size >> 10));
    } else
#endif /* FLASH_DBANK_SUPPORT */
    {
        g0_priv.bank_size = FLASH_TOTAL_SIZE;
        FLASH_LOG_I("Init: single-bank mode, page=%luKB, bank=%luKB",
            (unsigned long)(g0_priv.page_size >> 10),
            (unsigned long)(g0_priv.bank_size >> 10));
    }

    /* 初始化后更新 caps 中的 erase_size */
    dev->caps.erase_size = g0_priv.page_size;

    return HAL_FLASH_OK;
}

static hal_flash_err_t g0_read(uint32_t offset, uint8_t* buf, size_t size)
{
    uint32_t addr = FLASH_BASE_ADDR + offset;
    uint8_t* dst = buf;

    for (size_t i = 0; i < size; i++, addr++, dst++) {
        *dst = *(volatile uint8_t*)addr;
    }

    return HAL_FLASH_OK;
}

static hal_flash_err_t g0_erase(uint32_t offset, size_t size)
{
    hal_flash_err_t result = HAL_FLASH_OK;
    uint32_t error = 0;
    uint32_t page_size = get_page_size(&g0_priv);
    uint32_t bank_size = get_bank_size(&g0_priv);

    uint32_t addr = FLASH_BASE_ADDR + offset;

    if (addr % page_size != 0) {
        FLASH_LOG_E("Erase: address not page-aligned, addr=0x%08lX, page_size=%lu",
            (unsigned long)addr, (unsigned long)page_size);
        return HAL_FLASH_ALIGN_ERR;
    }
    if (size % page_size != 0) {
        FLASH_LOG_E("Erase: size not page-aligned, size=%lu, page_size=%lu",
            (unsigned long)size, (unsigned long)page_size);
        return HAL_FLASH_ALIGN_ERR;
    }

    FLASH_LOG_I("Erase: addr=0x%08lX, size=%lu, page_size=%lu",
        (unsigned long)addr, (unsigned long)size,
        (unsigned long)page_size);

    HAL_FLASH_Unlock();
    /* G0 无 FLASH_FLAG_ALL_ERRORS 宏：直接写 SR 清除全部错误位 */
    FLASH->SR = FLASH_SR_CLEAR;

    uint32_t current_addr = addr;
    size_t remaining = size;

    while (remaining > 0) {
        uint32_t bank = get_bank_number(current_addr);
        uint32_t bank_base = (bank == FLASH_BANK_1)
            ? FLASH_BASE_ADDR
            : (FLASH_BASE_ADDR + bank_size);
        uint32_t bank_end = bank_base + bank_size;

        size_t chunk = bank_end - current_addr;
        if (chunk > remaining) {
            chunk = remaining;
        }
        uint32_t chunk_pages = (uint32_t)(chunk / page_size);
        uint32_t page = (current_addr - bank_base) / page_size;

        FLASH_LOG_I("Erase chunk: bank=%lu, page=%lu, nb=%lu",
            (unsigned long)bank, (unsigned long)page,
            (unsigned long)chunk_pages);

        FLASH_EraseInitTypeDef erase_init = {
            .TypeErase = FLASH_TYPEERASE_PAGES,
            .Banks = bank,
            .Page = page,
            .NbPages = chunk_pages,
        };

        if (HAL_FLASHEx_Erase(&erase_init, &error) != HAL_OK) {
            FLASH_LOG_E("Erase error: addr=0x%08lX, bank=%lu, page=%lu, HAL_Err=0x%08lX",
                (unsigned long)current_addr, (unsigned long)bank,
                (unsigned long)page, (unsigned long)HAL_FLASH_GetError());
            result = HAL_FLASH_ERASE_ERR;
            goto exit_erase;
        }

        current_addr += chunk;
        remaining -= chunk;
    }

exit_erase:
    HAL_FLASH_Lock();
    return result;
}

static hal_flash_err_t g0_write(uint32_t offset, const uint8_t* buf, size_t size)
{
    hal_flash_err_t result = HAL_FLASH_OK;
    uint32_t addr = FLASH_BASE_ADDR + offset;
    const uint8_t* src = buf;

    FLASH_LOG_I("Write: addr=0x%08lX, size=%lu",
        (unsigned long)addr, (unsigned long)size);

    HAL_FLASH_Unlock();
    /* G0 无 FLASH_FLAG_ALL_ERRORS 宏：直接写 SR 清除全部错误位 */
    FLASH->SR = FLASH_SR_CLEAR;

    for (size_t i = 0; i < size; i += FLASH_PROGRAM_SIZE) {
        size_t copy_len = (size - i >= FLASH_PROGRAM_SIZE) ? FLASH_PROGRAM_SIZE : (size - i);

        if (copy_len < FLASH_PROGRAM_SIZE) {
            s_write_buf = FLASH_ERASED_VAL;
            memcpy(&s_write_buf, src, copy_len);
        } else {
            memcpy(&s_write_buf, src, FLASH_PROGRAM_SIZE);
        }

        if (s_write_buf != FLASH_ERASED_VAL) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, s_write_buf) != HAL_OK) {
                uint32_t error = HAL_FLASH_GetError();
                (void)error;
                FLASH_LOG_E("Flash program error: i=%u, addr=0x%08lX, HAL_Error=0x%08lX",
                    (unsigned)i, addr, (unsigned long)error);
                result = HAL_FLASH_WRITE_ERR;
                goto exit_write;
            }
        }

        s_read_buf = *(volatile uint64_t*)addr;
        if (s_read_buf != s_write_buf) {
            FLASH_LOG_E("Flash readback mismatch: i=%u, addr=0x%08lX, "
                        "written=0x%016llX, readback=0x%016llX",
                (unsigned)i, addr,
                (unsigned long long)s_write_buf,
                (unsigned long long)s_read_buf);
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

/* G0 Cortex-M0+ 无 D-Cache，无需 cache_invalidate 实现（HAL 层允许 NULL） */

/* ====== 设备实例导出 ======================================================== */

hal_flash_dev_t g0_dev = {
    .name = "stm32g0",
    .ops = {
        .init = g0_init,
        .read = g0_read,
        .write = g0_write,
        .erase = g0_erase,
        .cache_invalidate = NULL,
    },
    .caps = {
        .addr = 0x08000000U,
        .total_size = FLASH_TOTAL_SIZE,
        .erase_size = 0x800U,
        .write_gran = HAL_FLASH_WRITE_GRAN_64,
        .erase_size_uniform = true,
        .has_ecc = false,
        .has_write_protect = false,
        .has_crc = false,
    },
    .priv = &g0_priv,
    .initialized = false,
    .lock_cb = NULL,
    .unlock_cb = NULL,
    .lock_depth = 0,
    .event_cb = NULL,
    .event_arg = NULL,
};

#endif /* HAL_FLASH_CHIP_STM32G0 */
