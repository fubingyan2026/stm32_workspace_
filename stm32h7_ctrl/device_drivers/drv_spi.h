/**
 * @file    drv_spi.h
 * @brief   SPI 设备驱动 — 3 通道，混合 DMA/PIO
 *
 * SPI1: DMA1 Stream7 TX + PIO RX（全双工）
 * SPI2: PIO only（轮询）
 * SPI6: BDMA Channel0 TX only（无 MISO）
 */

#ifndef __DRV_SPI_H
#define __DRV_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ====== 错误码 =============================================================*/

typedef enum {
    DRV_SPI_OK = 0,
    DRV_SPI_ERR_PARAM   = -1,
    DRV_SPI_ERR_INIT    = -2,
    DRV_SPI_ERR_BUSY    = -3,
    DRV_SPI_ERR_TIMEOUT = -4,
    DRV_SPI_ERR_TXONLY  = -5, /**< TXONLY 通道不支持接收 */
} drv_spi_error_t;

/* ====== 通道枚举 ===========================================================*/

typedef enum {
    DRV_SPI_CH_1 = 0, /**< SPI1: DMA TX + PIO RX, 全双工 */
    DRV_SPI_CH_2,     /**< SPI2: PIO only, 全双工 */
    DRV_SPI_CH_6,     /**< SPI6: BDMA TX only, 无 RX */
    DRV_SPI_CH_NUM,
} drv_spi_channel_t;

/* ====== API ================================================================*/

drv_spi_error_t drv_spi_init(void);
drv_spi_error_t drv_spi_transmit(drv_spi_channel_t ch, const uint8_t* tx_data, uint32_t len);
drv_spi_error_t drv_spi_transmit_receive(drv_spi_channel_t ch, const uint8_t* tx_data, uint8_t* rx_data, uint32_t len);
drv_spi_error_t drv_spi_receive(drv_spi_channel_t ch, uint8_t* rx_data, uint32_t len);
bool            drv_spi_is_tx_busy(drv_spi_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SPI_H */
