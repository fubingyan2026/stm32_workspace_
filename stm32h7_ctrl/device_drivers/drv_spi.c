/**
 * @file    drv_spi.c
 * @brief   SPI 设备驱动实现
 *
 * SPI1: DMA TX (DMA1 Stream7) + PIO RX
 *   - 纯 TX (>16B) 使用 HAL_SPI_Transmit_DMA
 *   - 全双工/小 TX 使用 HAL_SPI_TransmitReceive 轮询
 * SPI2: PIO 轮询全双工
 * SPI6: BDMA Ch0 TX only (HAL_SPI_Transmit_DMA)
 *
 * DMA TX 完成回调清除 busy 标志，调用者需轮询 drv_spi_is_tx_busy 或保证时序。
 */

#include "drv_spi.h"

#include "spi.h"

#include <stdbool.h>
#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

#define SPI_LOG_ENABLE 0

#if SPI_LOG_ENABLE
#include "log.h"
#define SPI_LOG_E(...) LOG_E("spi", __VA_ARGS__)
#define SPI_LOG_I(...) LOG_I("spi", __VA_ARGS__)
#else
#define SPI_LOG_E(...) ((void)0)
#define SPI_LOG_I(...) ((void)0)
#endif

#define DRV_SPI_TX_BUF_SIZE 256U

/* Private types -------------------------------------------------------------*/

typedef struct {
    SPI_HandleTypeDef* hspi;
    bool use_dma_tx;
    bool is_txonly;
} drv_spi_hw_t;

typedef struct {
    bool tx_busy;
} drv_spi_ctx_t;

/* Private constants ---------------------------------------------------------*/

static const drv_spi_hw_t s_hw[DRV_SPI_CH_NUM] = {
    [DRV_SPI_CH_1] = { .hspi = &hspi1, .use_dma_tx = true,  .is_txonly = false },
    [DRV_SPI_CH_2] = { .hspi = &hspi2, .use_dma_tx = false, .is_txonly = false },
    [DRV_SPI_CH_6] = { .hspi = &hspi6, .use_dma_tx = true,  .is_txonly = true },
};

/* Private variables ---------------------------------------------------------*/

static bool s_init = false;
static drv_spi_ctx_t s_ctx[DRV_SPI_CH_NUM];
static uint8_t s_tx_buf[DRV_SPI_CH_NUM][DRV_SPI_TX_BUF_SIZE];

/* ====== HAL 回调（在驱动 .c 中重写） ======================================*/

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    for (uint32_t i = 0; i < DRV_SPI_CH_NUM; i++) {
        if (s_hw[i].hspi == hspi) {
            s_ctx[i].tx_busy = false;
            break;
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    for (uint32_t i = 0; i < DRV_SPI_CH_NUM; i++) {
        if (s_hw[i].hspi == hspi) {
            SPI_LOG_E("SPI ch=%lu error=0x%08lX",
                (unsigned long)i, (unsigned long)HAL_SPI_GetError(hspi));
            s_ctx[i].tx_busy = false;
            break;
        }
    }
}

/* ====== API 实现 ===========================================================*/

drv_spi_error_t drv_spi_init(void)
{
    if (s_init) return DRV_SPI_OK;

    for (uint32_t i = 0; i < DRV_SPI_CH_NUM; i++) {
        s_ctx[i].tx_busy = false;
    }

    s_init = true;
    SPI_LOG_I("SPI init ok, %lu channels", (unsigned long)DRV_SPI_CH_NUM);
    return DRV_SPI_OK;
}

drv_spi_error_t drv_spi_transmit(drv_spi_channel_t ch, const uint8_t* tx_data, uint32_t len)
{
    if (!s_init) return DRV_SPI_ERR_INIT;
    if (ch >= DRV_SPI_CH_NUM) return DRV_SPI_ERR_PARAM;
    if (!tx_data || len == 0) return DRV_SPI_ERR_PARAM;

    if (s_hw[ch].use_dma_tx) {
        if (s_ctx[ch].tx_busy) return DRV_SPI_ERR_BUSY;
        if (len > DRV_SPI_TX_BUF_SIZE) return DRV_SPI_ERR_PARAM;

        memcpy(s_tx_buf[ch], tx_data, len);
        s_ctx[ch].tx_busy = true;

        if (HAL_SPI_Transmit_DMA(s_hw[ch].hspi, s_tx_buf[ch], len) != HAL_OK) {
            s_ctx[ch].tx_busy = false;
            SPI_LOG_E("SPI ch=%lu Transmit_DMA failed", (unsigned long)ch);
            return DRV_SPI_ERR_INIT;
        }
    } else {
        /* SPI2: PIO 轮询 */
        if (HAL_SPI_Transmit(s_hw[ch].hspi, (uint8_t*)tx_data, len, 1000) != HAL_OK) {
            SPI_LOG_E("SPI ch=%lu Transmit timeout", (unsigned long)ch);
            return DRV_SPI_ERR_TIMEOUT;
        }
    }

    return DRV_SPI_OK;
}

drv_spi_error_t drv_spi_transmit_receive(drv_spi_channel_t ch, const uint8_t* tx_data, uint8_t* rx_data, uint32_t len)
{
    if (!s_init) return DRV_SPI_ERR_INIT;
    if (ch >= DRV_SPI_CH_NUM) return DRV_SPI_ERR_PARAM;
    if (!tx_data || !rx_data || len == 0) return DRV_SPI_ERR_PARAM;
    if (s_hw[ch].is_txonly) return DRV_SPI_ERR_TXONLY;

    /* 全双工统一使用 PIO 轮询（CubeMX 未配置 RX DMA） */
    if (HAL_SPI_TransmitReceive(s_hw[ch].hspi, (uint8_t*)tx_data, rx_data, len, 1000) != HAL_OK) {
        SPI_LOG_E("SPI ch=%lu TransmitReceive timeout", (unsigned long)ch);
        return DRV_SPI_ERR_TIMEOUT;
    }

    return DRV_SPI_OK;
}

drv_spi_error_t drv_spi_receive(drv_spi_channel_t ch, uint8_t* rx_data, uint32_t len)
{
    if (!s_init) return DRV_SPI_ERR_INIT;
    if (ch >= DRV_SPI_CH_NUM) return DRV_SPI_ERR_PARAM;
    if (!rx_data || len == 0) return DRV_SPI_ERR_PARAM;
    if (s_hw[ch].is_txonly) return DRV_SPI_ERR_TXONLY;

    /* 发送 0xFF 虚拟字节同时接收 */
    uint8_t dummy = 0xFF;
    if (HAL_SPI_TransmitReceive(s_hw[ch].hspi, &dummy, rx_data, len, 1000) != HAL_OK) {
        SPI_LOG_E("SPI ch=%lu Receive timeout", (unsigned long)ch);
        return DRV_SPI_ERR_TIMEOUT;
    }

    return DRV_SPI_OK;
}

bool drv_spi_is_tx_busy(drv_spi_channel_t ch)
{
    if (ch >= DRV_SPI_CH_NUM) return false;
    return s_ctx[ch].tx_busy;
}
