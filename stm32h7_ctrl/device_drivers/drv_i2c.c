/**
 * @file    drv_i2c.c
 * @brief   I2C 设备驱动实现
 *
 * I2C2: DMA TX (DMA2 Stream0) 用于大块发送，PIO 轮询用于接收和小块传输。
 * 支持 7-bit 地址、Mem 读写、设备探测、总线恢复。
 */

#include "drv_i2c.h"

#include "i2c.h"

#include <stdbool.h>
#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

#define I2C_LOG_ENABLE 0

#if I2C_LOG_ENABLE
#include "log.h"
#define I2C_LOG_E(...) LOG_E("i2c", __VA_ARGS__)
#define I2C_LOG_I(...) LOG_I("i2c", __VA_ARGS__)
#else
#define I2C_LOG_E(...) ((void)0)
#define I2C_LOG_I(...) ((void)0)
#endif

/* Private types -------------------------------------------------------------*/

typedef struct {
    bool tx_busy;
} drv_i2c_ctx_t;

/* Private constants ---------------------------------------------------------*/

#define DRV_I2C_TX_BUF_SIZE 256U

static I2C_HandleTypeDef* const s_hi2c[DRV_I2C_CH_NUM] = {
    [DRV_I2C_CH_2] = &hi2c2,
};

/* Private variables ---------------------------------------------------------*/

static bool s_init = false;
static drv_i2c_ctx_t s_ctx[DRV_I2C_CH_NUM];

/* ====== HAL 回调 ===========================================================*/

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    for (uint32_t i = 0; i < DRV_I2C_CH_NUM; i++) {
        if (s_hi2c[i] == hi2c) {
            s_ctx[i].tx_busy = false;
            break;
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
    for (uint32_t i = 0; i < DRV_I2C_CH_NUM; i++) {
        if (s_hi2c[i] == hi2c) {
            I2C_LOG_E("I2C ch=%lu error=0x%08lX",
                (unsigned long)i, (unsigned long)HAL_I2C_GetError(hi2c));
            s_ctx[i].tx_busy = false;
            break;
        }
    }
}

/* ====== API =================================================================*/

drv_i2c_error_t drv_i2c_init(void)
{
    if (s_init) return DRV_I2C_OK;

    for (uint32_t i = 0; i < DRV_I2C_CH_NUM; i++) {
        if (s_hi2c[i] == NULL) {
            I2C_LOG_E("I2C ch=%lu handle is NULL", (unsigned long)i);
            return DRV_I2C_ERR_INIT;
        }
        s_ctx[i].tx_busy = false;
    }

    s_init = true;
    I2C_LOG_I("I2C init ok, %lu channels", (unsigned long)DRV_I2C_CH_NUM);
    return DRV_I2C_OK;
}

drv_i2c_error_t drv_i2c_master_transmit(drv_i2c_channel_t ch, uint16_t dev_addr,
    const uint8_t* data, uint32_t len, uint32_t timeout_ms)
{
    if (!s_init) return DRV_I2C_ERR_INIT;
    if (ch >= DRV_I2C_CH_NUM || !data || len == 0) return DRV_I2C_ERR_PARAM;

    /* 使用轮询传输（DMA 未配置完成中断链，PIO 更可靠） */
    if (HAL_I2C_Master_Transmit(s_hi2c[ch], (uint16_t)(dev_addr << 1),
            (uint8_t*)data, len, timeout_ms) != HAL_OK) {
        I2C_LOG_E("I2C ch=%lu Master_Transmit timeout addr=0x%02X",
            (unsigned long)ch, (unsigned)dev_addr);
        return (HAL_I2C_GetError(s_hi2c[ch]) == HAL_I2C_ERROR_AF)
            ? DRV_I2C_ERR_NACK : DRV_I2C_ERR_TIMEOUT;
    }

    return DRV_I2C_OK;
}

drv_i2c_error_t drv_i2c_master_receive(drv_i2c_channel_t ch, uint16_t dev_addr,
    uint8_t* data, uint32_t len, uint32_t timeout_ms)
{
    if (!s_init) return DRV_I2C_ERR_INIT;
    if (ch >= DRV_I2C_CH_NUM || !data || len == 0) return DRV_I2C_ERR_PARAM;

    if (HAL_I2C_Master_Receive(s_hi2c[ch], (uint16_t)(dev_addr << 1),
            data, len, timeout_ms) != HAL_OK) {
        I2C_LOG_E("I2C ch=%lu Master_Receive timeout addr=0x%02X",
            (unsigned long)ch, (unsigned)dev_addr);
        return (HAL_I2C_GetError(s_hi2c[ch]) == HAL_I2C_ERROR_AF)
            ? DRV_I2C_ERR_NACK : DRV_I2C_ERR_TIMEOUT;
    }

    return DRV_I2C_OK;
}

drv_i2c_error_t drv_i2c_mem_write(drv_i2c_channel_t ch, uint16_t dev_addr,
    uint32_t mem_addr, uint32_t mem_addr_size,
    const uint8_t* data, uint32_t len, uint32_t timeout_ms)
{
    if (!s_init) return DRV_I2C_ERR_INIT;
    if (ch >= DRV_I2C_CH_NUM || !data || len == 0) return DRV_I2C_ERR_PARAM;

    if (HAL_I2C_Mem_Write(s_hi2c[ch], (uint16_t)(dev_addr << 1),
            mem_addr, mem_addr_size,
            (uint8_t*)data, len, timeout_ms) != HAL_OK) {
        I2C_LOG_E("I2C ch=%lu Mem_Write timeout addr=0x%02X reg=0x%04lX",
            (unsigned long)ch, (unsigned)dev_addr, (unsigned long)mem_addr);
        return DRV_I2C_ERR_TIMEOUT;
    }

    return DRV_I2C_OK;
}

drv_i2c_error_t drv_i2c_mem_read(drv_i2c_channel_t ch, uint16_t dev_addr,
    uint32_t mem_addr, uint32_t mem_addr_size,
    uint8_t* data, uint32_t len, uint32_t timeout_ms)
{
    if (!s_init) return DRV_I2C_ERR_INIT;
    if (ch >= DRV_I2C_CH_NUM || !data || len == 0) return DRV_I2C_ERR_PARAM;

    if (HAL_I2C_Mem_Read(s_hi2c[ch], (uint16_t)(dev_addr << 1),
            mem_addr, mem_addr_size,
            data, len, timeout_ms) != HAL_OK) {
        I2C_LOG_E("I2C ch=%lu Mem_Read timeout addr=0x%02X reg=0x%04lX",
            (unsigned long)ch, (unsigned)dev_addr, (unsigned long)mem_addr);
        return DRV_I2C_ERR_TIMEOUT;
    }

    return DRV_I2C_OK;
}

bool drv_i2c_is_device_ready(drv_i2c_channel_t ch, uint16_t dev_addr,
    uint32_t trials, uint32_t timeout_ms)
{
    if (ch >= DRV_I2C_CH_NUM) return false;

    return (HAL_I2C_IsDeviceReady(s_hi2c[ch], (uint16_t)(dev_addr << 1),
        trials, timeout_ms) == HAL_OK);
}

void drv_i2c_recover_bus(drv_i2c_channel_t ch)
{
    if (ch >= DRV_I2C_CH_NUM) return;

    /* 尝试复位 I2C 外设以释放总线 */
    HAL_I2C_DeInit(s_hi2c[ch]);
    HAL_Delay(10);
    HAL_I2C_Init(s_hi2c[ch]);

    I2C_LOG_I("I2C ch=%lu bus recovered", (unsigned long)ch);
}
