/**
 * @file    drv_octospi.c
 * @brief   OCTOSPI2 Quad SPI Flash 驱动实现
 *
 * Micron/NOR Flash, 8MB (2^23), QSPI Quad 4-line 模式.
 * 间接模式下发送命令进行读写擦除，内存映射模式用于快速读取。
 *
 * 内存映射基址: 0x90000000 (STM32H7 OCTOSPI AXI 地址空间)
 */

#include "drv_octospi.h"

#include "octospi.h"

#include <stdbool.h>
#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

#define OSPI_LOG_ENABLE 0

#if OSPI_LOG_ENABLE
#include "log.h"
#define OSPI_LOG_E(...) LOG_E("ospi", __VA_ARGS__)
#define OSPI_LOG_I(...) LOG_I("ospi", __VA_ARGS__)
#else
#define OSPI_LOG_E(...) ((void)0)
#define OSPI_LOG_I(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define DRV_OSPI_PAGE_SIZE      256U
#define DRV_OSPI_SECTOR_SIZE    4096U
#define DRV_OSPI_BLOCK_SIZE     65536U
#define DRV_OSPI_MMAP_BASE      0x90000000U

/* Micron/JEDEC 标准 SPI Flash 命令 */
#define CMD_WRITE_ENABLE        0x06U
#define CMD_VOLATILE_SR_WRITE   0x50U
#define CMD_WRITE_STATUS_REG    0x01U
#define CMD_READ_STATUS_REG     0x05U
#define CMD_READ_JEDEC_ID       0x9FU
#define CMD_SECTOR_ERASE_4K     0x20U
#define CMD_BLOCK_ERASE_64K     0xD8U
#define CMD_CHIP_ERASE          0xC7U
#define CMD_PAGE_PROGRAM        0x02U
#define CMD_QUAD_PAGE_PROGRAM   0x38U
#define CMD_READ_DATA           0x03U
#define CMD_QUAD_READ           0x6BU
#define CMD_RESET_ENABLE        0x66U
#define CMD_RESET               0x99U

/* Status register bits */
#define SR_BUSY                 0x01U
#define SR_WEL                  0x02U
#define SR_QE                   0x40U /* Quad Enable (top speed) */

/* Private variables ---------------------------------------------------------*/

static bool s_init = false;
static bool s_mmap_active = false;

/* Private function prototypes -----------------------------------------------*/

static drv_ospi_error_t write_enable(void);
static drv_ospi_error_t wait_busy(uint32_t timeout_ms);
static drv_ospi_error_t command_addr_data(uint32_t instruction, uint32_t address,
    const void* data, uint32_t len, bool is_write);
static drv_ospi_error_t command_no_addr(uint32_t instruction, uint32_t dummy_cycles,
    void* data, uint32_t len, bool is_read);

/* ====== 内部辅助 ===========================================================*/

static drv_ospi_error_t write_enable(void)
{
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType   = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.Instruction     = CMD_WRITE_ENABLE;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode     = HAL_OSPI_ADDRESS_NONE;
    cmd.DataMode        = HAL_OSPI_DATA_NONE;
    cmd.DummyCycles     = 0;

    if (HAL_OSPI_Command(&hospi2, &cmd, 100) != HAL_OK) {
        return DRV_OSPI_ERR_INIT;
    }
    return DRV_OSPI_OK;
}

static drv_ospi_error_t wait_busy(uint32_t timeout_ms)
{
    uint32_t tick = HAL_GetTick();
    uint8_t sr = SR_BUSY;

    while (sr & SR_BUSY) {
        OSPI_RegularCmdTypeDef cmd = {0};
        cmd.OperationType   = HAL_OSPI_OPTYPE_COMMON_CFG;
        cmd.Instruction     = CMD_READ_STATUS_REG;
        cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
        cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
        cmd.AddressMode     = HAL_OSPI_ADDRESS_NONE;
        cmd.DataMode        = HAL_OSPI_DATA_1_LINE;
        cmd.NbData          = 1;
        cmd.DummyCycles     = 0;

        if (HAL_OSPI_Command(&hospi2, &cmd, 10) != HAL_OK) {
            return DRV_OSPI_ERR_TIMEOUT;
        }
        if (HAL_OSPI_Receive(&hospi2, &sr, 10) != HAL_OK) {
            return DRV_OSPI_ERR_TIMEOUT;
        }

        if (HAL_GetTick() - tick > timeout_ms) {
            return DRV_OSPI_ERR_TIMEOUT;
        }
    }
    return DRV_OSPI_OK;
}

static drv_ospi_error_t command_addr_data(uint32_t instruction, uint32_t address,
    const void* data, uint32_t len, bool is_write)
{
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.Instruction        = instruction;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.Address            = address;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    cmd.DataMode           = (len > 0) ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_NONE;
    cmd.NbData             = len;
    cmd.DummyCycles        = 0;

    if (HAL_OSPI_Command(&hospi2, &cmd, 1000) != HAL_OK) {
        return DRV_OSPI_ERR_PARAM;
    }

    if (len > 0) {
        if (is_write) {
            if (HAL_OSPI_Transmit(&hospi2, (uint8_t*)data, 1000) != HAL_OK) {
                return DRV_OSPI_ERR_WRITE;
            }
        } else {
            if (HAL_OSPI_Receive(&hospi2, (uint8_t*)data, 1000) != HAL_OK) {
                return DRV_OSPI_ERR_READ;
            }
        }
    }

    return DRV_OSPI_OK;
}

static drv_ospi_error_t command_no_addr(uint32_t instruction, uint32_t dummy_cycles,
    void* data, uint32_t len, bool is_read)
{
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.Instruction        = instruction;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_NONE;
    cmd.DataMode           = (len > 0) ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_NONE;
    cmd.NbData             = len;
    cmd.DummyCycles        = dummy_cycles;

    if (HAL_OSPI_Command(&hospi2, &cmd, 100) != HAL_OK) {
        return DRV_OSPI_ERR_PARAM;
    }

    if (len > 0) {
        if (is_read) {
            if (HAL_OSPI_Receive(&hospi2, (uint8_t*)data, 100) != HAL_OK) {
                return DRV_OSPI_ERR_READ;
            }
        } else {
            if (HAL_OSPI_Transmit(&hospi2, (uint8_t*)data, 100) != HAL_OK) {
                return DRV_OSPI_ERR_WRITE;
            }
        }
    }

    return DRV_OSPI_OK;
}

/* ====== API 实现 ===========================================================*/

drv_ospi_error_t drv_ospi_init(void)
{
    if (s_init) return DRV_OSPI_OK;

    /* 复位 Flash (Micron 重置序列) */
    command_no_addr(CMD_RESET_ENABLE, 0, NULL, 0, false);
    HAL_Delay(1);
    command_no_addr(CMD_RESET, 0, NULL, 0, false);
    HAL_Delay(10);

    /* 读取 JEDEC ID 确认连接 */
    uint8_t jedec[3] = {0};
    command_no_addr(CMD_READ_JEDEC_ID, 0, jedec, 3, true);
    OSPI_LOG_I("JEDEC ID: 0x%02X 0x%02X 0x%02X",
        jedec[0], jedec[1], jedec[2]);

    if (jedec[0] == 0xFF && jedec[1] == 0xFF && jedec[2] == 0xFF) {
        OSPI_LOG_E("OSPI device not responding (all 0xFF)");
        return DRV_OSPI_ERR_NOT_FOUND;
    }

    /* 启用 Quad 模式 (写状态寄存器) */
    write_enable();
    command_addr_data(CMD_WRITE_STATUS_REG, 0, NULL, 0, false);

    /* 内存映射默认未激活 */
    s_mmap_active = false;
    s_init = true;

    OSPI_LOG_I("OSPI init ok, 8MB Micron Flash at 80MHz");
    return DRV_OSPI_OK;
}

drv_ospi_error_t drv_ospi_read(uint32_t address, uint8_t* data, uint32_t len)
{
    if (!s_init) return DRV_OSPI_ERR_INIT;
    if (!data || len == 0) return DRV_OSPI_ERR_PARAM;
    if (s_mmap_active) return DRV_OSPI_ERR_PARAM;

    /* 退出内存映射模式（如果在其中） */
    if (s_mmap_active) {
        drv_ospi_exit_memory_mapped_mode();
    }

    drv_ospi_error_t err = command_addr_data(CMD_READ_DATA, address, data, len, false);
    if (err != DRV_OSPI_OK) {
        OSPI_LOG_E("OSPI read failed at 0x%08lX len=%lu", (unsigned long)address, (unsigned long)len);
    }
    return err;
}

drv_ospi_error_t drv_ospi_write(uint32_t address, const uint8_t* data, uint32_t len)
{
    if (!s_init) return DRV_OSPI_ERR_INIT;
    if (!data || len == 0) return DRV_OSPI_ERR_PARAM;
    if (s_mmap_active) return DRV_OSPI_ERR_PARAM;

    uint32_t remaining = len;
    uint32_t addr = address;
    const uint8_t* src = data;

    while (remaining > 0) {
        /* 页内偏移 */
        uint32_t page_offset = addr % DRV_OSPI_PAGE_SIZE;
        uint32_t chunk = DRV_OSPI_PAGE_SIZE - page_offset;
        if (chunk > remaining) chunk = remaining;

        drv_ospi_error_t err = write_enable();
        if (err != DRV_OSPI_OK) return err;

        err = command_addr_data(CMD_PAGE_PROGRAM, addr, src, chunk, true);
        if (err != DRV_OSPI_OK) {
            OSPI_LOG_E("OSPI write failed at 0x%08lX", (unsigned long)addr);
            return err;
        }

        err = wait_busy(1000);
        if (err != DRV_OSPI_OK) return err;

        addr += chunk;
        src  += chunk;
        remaining -= chunk;
    }

    return DRV_OSPI_OK;
}

drv_ospi_error_t drv_ospi_erase_sector(uint32_t address)
{
    if (!s_init) return DRV_OSPI_ERR_INIT;
    if (s_mmap_active) return DRV_OSPI_ERR_PARAM;

    /* 地址对齐到扇区边界 */
    address &= ~(DRV_OSPI_SECTOR_SIZE - 1);

    drv_ospi_error_t err = write_enable();
    if (err != DRV_OSPI_OK) return err;

    err = command_addr_data(CMD_SECTOR_ERASE_4K, address, NULL, 0, false);
    if (err != DRV_OSPI_OK) return err;

    err = wait_busy(5000);
    if (err != DRV_OSPI_OK) {
        OSPI_LOG_E("OSPI sector erase timeout at 0x%08lX", (unsigned long)address);
        return DRV_OSPI_ERR_ERASE;
    }

    return DRV_OSPI_OK;
}

drv_ospi_error_t drv_ospi_erase_chip(void)
{
    if (!s_init) return DRV_OSPI_ERR_INIT;
    if (s_mmap_active) return DRV_OSPI_ERR_PARAM;

    drv_ospi_error_t err = write_enable();
    if (err != DRV_OSPI_OK) return err;

    err = command_no_addr(CMD_CHIP_ERASE, 0, NULL, 0, false);
    if (err != DRV_OSPI_OK) return err;

    err = wait_busy(60000);
    if (err != DRV_OSPI_OK) {
        OSPI_LOG_E("OSPI chip erase timeout");
        return DRV_OSPI_ERR_ERASE;
    }

    return DRV_OSPI_OK;
}

drv_ospi_error_t drv_ospi_enter_memory_mapped_mode(void)
{
    if (!s_init) return DRV_OSPI_ERR_INIT;
    if (s_mmap_active) return DRV_OSPI_OK;

    /* 配置内存映射命令 */
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.Instruction        = CMD_QUAD_READ;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
    cmd.DataMode           = HAL_OSPI_DATA_4_LINES;
    cmd.DummyCycles        = 8; /* Quad Read 需要 8 个 dummy cycle */
    cmd.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

    if (HAL_OSPI_Command(&hospi2, &cmd, 1000) != HAL_OK) {
        return DRV_OSPI_ERR_INIT;
    }

    /* 进入内存映射模式 */
    OSPI_MemoryMappedTypeDef mmap_cfg = {0};
    mmap_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

    if (HAL_OSPI_MemoryMapped(&hospi2, &mmap_cfg) != HAL_OK) {
        OSPI_LOG_E("OSPI MemoryMapped failed");
        return DRV_OSPI_ERR_INIT;
    }

    s_mmap_active = true;
    OSPI_LOG_I("OSPI memory-mapped mode active @ 0x%08X", DRV_OSPI_MMAP_BASE);
    return DRV_OSPI_OK;
}

drv_ospi_error_t drv_ospi_exit_memory_mapped_mode(void)
{
    if (!s_mmap_active) return DRV_OSPI_OK;

    HAL_OSPI_Abort(&hospi2);
    s_mmap_active = false;
    return DRV_OSPI_OK;
}

const uint8_t* drv_ospi_get_mmap_ptr(void)
{
    return (const uint8_t*)DRV_OSPI_MMAP_BASE;
}
