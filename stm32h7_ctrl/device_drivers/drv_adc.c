/**
 * @file    drv_adc.c
 * @brief   ADC1 DMA 循环转换驱动实现
 *
 * 使用 DMA1 Stream0 半字循环模式自动填充缓冲区。
 * 半传输完成和传输完成回调中更新最新样本值。
 * VREF = 3300mV, 16 位分辨率。
 */

#include "drv_adc.h"

#include "adc.h"

#include <stdbool.h>

/* 模块日志开关 ----------------------------------------------------------------*/

#define ADC_LOG_ENABLE 0

#if ADC_LOG_ENABLE
#include "log.h"
#define ADC_LOG_E(...) LOG_E("adc", __VA_ARGS__)
#define ADC_LOG_I(...) LOG_I("adc", __VA_ARGS__)
#else
#define ADC_LOG_E(...) ((void)0)
#define ADC_LOG_I(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define DRV_ADC_DMA_BUF_LEN 32U
#define DRV_ADC_VREF_MV     3300U
#define DRV_ADC_RESOLUTION  65535U /* 16-bit: 2^16 - 1 */

/* Private variables ---------------------------------------------------------*/

static bool s_init = false;

/** @brief DMA 循环缓冲区 */
static volatile uint16_t s_dma_buf[DRV_ADC_DMA_BUF_LEN];

/** @brief 最新采样值 (由 DMA 回调更新) */
static volatile uint16_t s_last_raw;

/** @brief 写索引追踪 (DMA 当前写到哪了) */
static volatile uint32_t s_write_idx;

/* ====== HAL 回调 ===========================================================*/

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    /* 下半部分填满 (0 .. BUF_LEN/2-1) */
    s_last_raw = s_dma_buf[DRV_ADC_DMA_BUF_LEN / 2 - 1];
    s_write_idx = DRV_ADC_DMA_BUF_LEN / 2;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;

    /* 上半部分填满 (BUF_LEN/2 .. BUF_LEN-1) */
    s_last_raw = s_dma_buf[DRV_ADC_DMA_BUF_LEN - 1];
    s_write_idx = 0;
}

/* ====== API 实现 ===========================================================*/

drv_adc_error_t drv_adc_init(void)
{
    if (s_init) return DRV_ADC_OK;

    /* 启动 DMA 循环转换 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)s_dma_buf, DRV_ADC_DMA_BUF_LEN) != HAL_OK) {
        ADC_LOG_E("ADC1 Start_DMA failed");
        return DRV_ADC_ERR_INIT;
    }

    s_init = true;
    ADC_LOG_I("ADC1 init ok (16-bit, DMA circular, VREF=%u mV)", DRV_ADC_VREF_MV);
    return DRV_ADC_OK;
}

uint32_t drv_adc_get_raw(void)
{
    if (!s_init) return 0;
    return (uint32_t)s_last_raw;
}

uint32_t drv_adc_get_voltage(void)
{
    if (!s_init) return 0;
    return (uint32_t)((uint32_t)s_last_raw * DRV_ADC_VREF_MV / DRV_ADC_RESOLUTION);
}
