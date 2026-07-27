/**
 * @file    drv_adc.h
 * @brief   ADC1 DMA 循环转换驱动 — 16-bit, 连续转换, DMA Circular
 *
 * ADC1 Ch4 (PC4), 单端, 采样时间 32.5 cycles.
 * DMA1 Stream0 半字循环模式。
 */

#ifndef __DRV_ADC_H
#define __DRV_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ====== 错误码 =============================================================*/

typedef enum {
    DRV_ADC_OK = 0,
    DRV_ADC_ERR_PARAM = -1,
    DRV_ADC_ERR_INIT  = -2,
} drv_adc_error_t;

/* ====== API ================================================================*/

drv_adc_error_t drv_adc_init(void);
uint32_t        drv_adc_get_raw(void);
uint32_t        drv_adc_get_voltage(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ADC_H */
