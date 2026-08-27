/**
 * @file    drv_adc.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   ADC 设备驱动（STM32F103 单实例 ADC1，DMA 采样）
 * @attention
 *
 * CubeMX 配置 ADC1 两路常规通道（DMA normal）：
 *   Rank 1: PA7  = ADC1_IN7  VIN_ADC
 *   Rank 2: PB0  = ADC1_IN8  V_IMON
 * 句柄表内置在模块中，drv_adc_init() 无需传参。
 */

#ifndef __DRV_ADC_H
#define __DRV_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ADC 逻辑通道枚举（与 CubeMX Rank 顺序对应）
 */
typedef enum {
    DRV_ADC_CH_VIN = 0, /**< VIN_ADC — PA7 (ADC1_IN7, Rank1) */
    DRV_ADC_CH_V_IMON,  /**< V_IMON — PB0 (ADC1_IN8, Rank2) */
    DRV_ADC_CH_NUM,     /**< 通道总数 */
} drv_adc_channel_t;

/**
 * @brief ADC 驱动错误码
 */
typedef enum {
    DRV_ADC_OK = 0,
    DRV_ADC_ERROR_UNINITIALIZED,
    DRV_ADC_ERROR_BUSY,
    DRV_ADC_ERROR_INVALID_PARAM,
} drv_adc_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化 ADC1（内部句柄表，无需传参） */
void drv_adc_init(void);

/** @brief 反初始化 ADC1 */
void drv_adc_deinit_all(void);

bool drv_adc_is_initialized(void);

/* --- 采样 --- */

/**
 * @brief 触发一次 ADC 扫描（DMA normal 单次，完成回调清 busy）
 * @return DRV_ADC_OK 启动成功；DRV_ADC_ERROR_BUSY 上次扫描未完成
 */
drv_adc_error_t drv_adc_trigger(void);

/** @brief 查询是否忙（上次扫描未完成） */
bool drv_adc_is_busy(void);

/* --- 读取 --- */

/**
 * @brief 读取指定通道最新一次采样的原始值（12-bit）
 * @param ch 逻辑通道
 * @return 原始采样值；未初始化/参数非法返回 0
 */
uint32_t drv_adc_read_raw(drv_adc_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ADC_H */
