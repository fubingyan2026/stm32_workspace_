/**
 * @file    drv_i2c.h
 * @brief   I2C 设备驱动 — DMA TX + PIO RX
 *
 * I2C2: PB10(SCL), PB11(SDA), Fast Mode Plus, DMA2 Stream0 TX
 */

#ifndef __DRV_I2C_H
#define __DRV_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ====== 错误码 =============================================================*/

typedef enum {
    DRV_I2C_OK = 0,
    DRV_I2C_ERR_PARAM   = -1,
    DRV_I2C_ERR_INIT    = -2,
    DRV_I2C_ERR_BUSY    = -3,
    DRV_I2C_ERR_TIMEOUT = -4,
    DRV_I2C_ERR_NACK    = -5,
} drv_i2c_error_t;

/* ====== 通道枚举 ===========================================================*/

typedef enum {
    DRV_I2C_CH_2 = 0, /**< I2C2 */
    DRV_I2C_CH_NUM,
} drv_i2c_channel_t;

/* ====== API ================================================================*/

drv_i2c_error_t drv_i2c_init(void);

drv_i2c_error_t drv_i2c_master_transmit(drv_i2c_channel_t ch, uint16_t dev_addr,
    const uint8_t* data, uint32_t len, uint32_t timeout_ms);

drv_i2c_error_t drv_i2c_master_receive(drv_i2c_channel_t ch, uint16_t dev_addr,
    uint8_t* data, uint32_t len, uint32_t timeout_ms);

drv_i2c_error_t drv_i2c_mem_write(drv_i2c_channel_t ch, uint16_t dev_addr,
    uint32_t mem_addr, uint32_t mem_addr_size,
    const uint8_t* data, uint32_t len, uint32_t timeout_ms);

drv_i2c_error_t drv_i2c_mem_read(drv_i2c_channel_t ch, uint16_t dev_addr,
    uint32_t mem_addr, uint32_t mem_addr_size,
    uint8_t* data, uint32_t len, uint32_t timeout_ms);

bool drv_i2c_is_device_ready(drv_i2c_channel_t ch, uint16_t dev_addr,
    uint32_t trials, uint32_t timeout_ms);

void drv_i2c_recover_bus(drv_i2c_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_I2C_H */
