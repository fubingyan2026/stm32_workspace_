/**
 * @file    drv_adc.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   ADC 设备驱动实现（STM32F103 单实例 ADC1，DMA normal 采样）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_adc.h"

#include "adc.h"
#include "drv_systick.h"
#include "log.h"
#include "main.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_ADC_LOG_ENABLE 0

#if DRV_ADC_LOG_ENABLE
#define DRV_ADC_LOG_E(...) LOG_E("drv_adc", __VA_ARGS__)
#define DRV_ADC_LOG_W(...) LOG_W("drv_adc", __VA_ARGS__)
#define DRV_ADC_LOG_I(...) LOG_I("drv_adc", __VA_ARGS__)
#define DRV_ADC_LOG_D(...) LOG_D("drv_adc", __VA_ARGS__)
#else
#define DRV_ADC_LOG_E(...) ((void)0)
#define DRV_ADC_LOG_W(...) ((void)0)
#define DRV_ADC_LOG_I(...) ((void)0)
#define DRV_ADC_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief ADC 触发失败日志限频窗口 (ms) */
#define DRV_ADC_ERR_LOG_PERIOD_MS (1000U)

/* Private types -------------------------------------------------------------*/

typedef struct {
    ADC_HandleTypeDef* hadc;
    uint32_t dma_buf[DRV_ADC_CH_NUM]; /**< 与 CubeMX Rank 顺序对应（Rank1→idx0） */
    uint8_t channel_count;
    volatile bool busy;
    bool initialized;
} drv_adc_ctx_t;

/* Private variables ---------------------------------------------------------*/

static drv_adc_ctx_t s_ctx = {
    .hadc = NULL,
    .channel_count = 0,
    .busy = false,
    .initialized = false,
};

/** @brief 触发失败日志时间戳 (ms) */
static uint32_t s_trig_err_log_ts;

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

void drv_adc_init(void)
{
    memset(s_ctx.dma_buf, 0, sizeof(s_ctx.dma_buf));
    s_ctx.hadc = &hadc1;
    s_ctx.channel_count = (uint8_t)hadc1.Init.NbrOfConversion;
    s_ctx.busy = false;
    s_ctx.initialized = true;

    DRV_ADC_LOG_I("ADC1 初始化完成: 通道数=%u", (unsigned)s_ctx.channel_count);
}

void drv_adc_deinit_all(void)
{
    if (s_ctx.hadc) {
        HAL_ADC_Stop_DMA(s_ctx.hadc);
    }
    s_ctx.initialized = false;
    s_ctx.busy = false;
}

bool drv_adc_is_initialized(void)
{
    return s_ctx.initialized;
}

/* --- 采样 --- */

drv_adc_error_t drv_adc_trigger(void)
{
    if (!s_ctx.initialized || s_ctx.hadc == NULL) {
        return DRV_ADC_ERROR_UNINITIALIZED;
    }
    if (s_ctx.busy) {
        return DRV_ADC_ERROR_BUSY;
    }

    s_ctx.busy = true;

    if (HAL_ADC_Start_DMA(s_ctx.hadc, s_ctx.dma_buf, s_ctx.channel_count) != HAL_OK) {
        s_ctx.busy = false;

        const uint32_t now_ms = millis();
        if ((uint32_t)(now_ms - s_trig_err_log_ts) >= DRV_ADC_ERR_LOG_PERIOD_MS) {
            s_trig_err_log_ts = now_ms;
            DRV_ADC_LOG_E("ADC1 启动 DMA 采样失败");
        }
        return DRV_ADC_ERROR_BUSY;
    }

    return DRV_ADC_OK;
}

bool drv_adc_is_busy(void)
{
    return s_ctx.busy;
}

/* --- 读取 --- */

uint32_t drv_adc_read_raw(drv_adc_channel_t ch)
{
    if (ch >= DRV_ADC_CH_NUM || !s_ctx.initialized) {
        return 0;
    }
    if (ch >= s_ctx.channel_count) {
        return 0;
    }
    return s_ctx.dma_buf[ch];
}

/* ===== HAL 回调 ===== */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == s_ctx.hadc) {
        s_ctx.busy = false;
    }
}
