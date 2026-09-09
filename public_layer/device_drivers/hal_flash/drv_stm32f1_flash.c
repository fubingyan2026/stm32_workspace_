/**
 * @file    drv_stm32f1_flash.c
 * @brief   STM32F103 Flash 底层驱动 — 适配 hal_flash 抽象层
 *
 * 实现 hal_flash_ops_t 接口，导出 f1_ops 和 f1_priv 供 HAL 层引用。
 *
 * F103 高密度（256KB，CubeMX 宏 STM32F103xE）Flash 特性：
 *   - 页大小均匀 2KB（FLASH_PAGE_SIZE = 0x800）
 *   - 编程支持 HALFLWORD/WORD（使用 32-bit WORD + 读回校验）
 *   - 无 D/I-Cache，无需 cache 维护
 *
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

#define F1_PROGRAM_SIZE    (4U)       /**< 32-bit WORD 编程 */
#define F1_PAGE_SIZE       (0x800U)   /**< 2KB 页 */
#define F1_FLASH_BASE_ADDR (0x08000000U)
#define F1_FLASH_TOTAL_SIZE (256U * 1024U)  /**< 256KB（F103RCTx） */

/* Private variables ---------------------------------------------------------*/

/* F1 私有数据实例 (HAL 层 f1_dev.priv 引用) */
f1_priv_data_t f1_priv = {
    .page_size = F1_PAGE_SIZE,
    .total_size = F1_FLASH_TOTAL_SIZE,
};

/* ====== hal_flash_ops_t 接口实现 ========================================== */

static hal_flash_err_t f1_init(void)
{
    hal_flash_dev_t* dev = hal_flash_dev();

    if (f1_priv.page_size == 0U) {
        f1_priv.page_size = F1_PAGE_SIZE;
    }
    if (f1_priv.total_size == 0U) {
        f1_priv.total_size = F1_FLASH_TOTAL_SIZE;
    }

    /* 初始化后同步 caps 中的 erase_size（与 boot_flash 默认 meta sector 取值一致） */
    dev->caps.erase_size = f1_priv.page_size;

    return HAL_FLASH_OK;
}

static hal_flash_err_t f1_read(uint32_t offset, uint8_t* buf, size_t size)
{
    const volatile uint8_t* p = (const volatile uint8_t*)(F1_FLASH_BASE_ADDR + offset);
    for (size_t i = 0; i < size; i++) {
        buf[i] = p[i];
    }
    return HAL_FLASH_OK;
}

static hal_flash_err_t f1_erase(uint32_t offset, size_t size)
{
    const uint32_t addr = F1_FLASH_BASE_ADDR + offset;
    const uint32_t page_size = f1_priv.page_size;

    if ((addr % page_size) != 0U) {
        FLASH_LOG_E("Erase: address not page-aligned, addr=0x%08lX", (unsigned long)addr);
        return HAL_FLASH_ALIGN_ERR;
    }
    if ((size % page_size) != 0U) {
        FLASH_LOG_E("Erase: size not page-aligned, size=%lu", (unsigned long)size);
        return HAL_FLASH_ALIGN_ERR;
    }

    FLASH_LOG_I("Erase: addr=0x%08lX, size=%lu, page_size=%lu",
        (unsigned long)addr, (unsigned long)size, (unsigned long)page_size);

    HAL_FLASH_Unlock();

    uint32_t page_error = 0U;
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = addr,
        .NbPages = (uint32_t)(size / page_size),
    };

    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) {
        FLASH_LOG_E("Erase error: addr=0x%08lX, HAL_Err=0x%08lX, PageError=0x%08lX",
            (unsigned long)addr,
            (unsigned long)HAL_FLASH_GetError(),
            (unsigned long)page_error);
        HAL_FLASH_Lock();
        return HAL_FLASH_ERASE_ERR;
    }

    HAL_FLASH_Lock();
    return HAL_FLASH_OK;
}

static hal_flash_err_t f1_write(uint32_t offset, const uint8_t* buf, size_t size)
{
    hal_flash_err_t result = HAL_FLASH_OK;
    uint32_t addr = F1_FLASH_BASE_ADDR + offset;
    const uint8_t* src = buf;
    uint32_t read_back = 0U;

    if ((offset & 3U) != 0U) {
        FLASH_LOG_E("Write: offset not word-aligned, offset=0x%08lX", (unsigned long)offset);
        return HAL_FLASH_ALIGN_ERR;
    }

    FLASH_LOG_I("Write: addr=0x%08lX, size=%lu", (unsigned long)addr, (unsigned long)size);

    HAL_FLASH_Unlock();

    size_t i = 0U;
    while (i < size) {
        /* 拼装待写 32-bit WORD：不足 4B 的尾部用 0xFF 填充 */
        uint32_t word = 0xFFFFFFFFU;
        const size_t copy_len = ((size - i) >= F1_PROGRAM_SIZE) ? F1_PROGRAM_SIZE : (size - i);
        memcpy(&word, src, copy_len);

        /* 与当前内容合并（编程只能 1→0；正常仅写已擦除区，合并为空操作） */
        word &= *(volatile uint32_t*)addr;

        if (word != 0xFFFFFFFFU) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word) != HAL_OK) {
                FLASH_LOG_E("Flash program error: offset=0x%08lX, HAL_Error=0x%08lX",
                    (unsigned long)addr,
                    (unsigned long)HAL_FLASH_GetError());
                result = HAL_FLASH_WRITE_ERR;
                goto exit_write;
            }

            read_back = *(volatile uint32_t*)addr;
            if (read_back != word) {
                FLASH_LOG_E("Flash readback mismatch: addr=0x%08lX, "
                            "written=0x%08lX, readback=0x%08lX",
                    (unsigned long)addr,
                    (unsigned long)word,
                    (unsigned long)read_back);
                result = HAL_FLASH_WRITE_ERR;
                goto exit_write;
            }
        }

        addr += F1_PROGRAM_SIZE;
        src += copy_len;
        i += copy_len;
    }

exit_write:
    HAL_FLASH_Lock();
    return result;
}

static void f1_cache_invalidate(void)
{
    /* F103 无 I/D-Cache，无需处理 */
}

/* ====== 设备实例导出 ======================================================== */

hal_flash_dev_t f1_dev = {
    .name = "stm32f1",
    .ops = {
        .init = f1_init,
        .read = f1_read,
        .write = f1_write,
        .erase = f1_erase,
        .cache_invalidate = f1_cache_invalidate,
    },
    .caps = {
        .addr = F1_FLASH_BASE_ADDR,
        .total_size = F1_FLASH_TOTAL_SIZE,
        .erase_size = F1_PAGE_SIZE,
        .write_gran = HAL_FLASH_WRITE_GRAN_32,
        .erase_size_uniform = true,
        .has_ecc = false,
        .has_write_protect = false,
        .has_crc = false,
    },
    .priv = &f1_priv,
    .initialized = false,
    .lock_cb = NULL,
    .unlock_cb = NULL,
    .lock_depth = 0,
    .event_cb = NULL,
    .event_arg = NULL,
};

#endif /* HAL_FLASH_CHIP_STM32F1 */
