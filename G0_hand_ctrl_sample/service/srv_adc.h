/**
 * @file    srv_adc.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   ADC 采样服务 — 原始值到物理量换算
 *
 * service 层仅提供数据处理管道，不管理 sw_timer（由 task 层负责）。
 * G0 仅 ADC1 两路：VIN_ADC（PA7）与 V_IMON（PB0），12-bit 原始值。
 * F1 无 VREFINT 校准，电压按 3.3V 满量程换算（参考电压偏差待实测修正）。
 */

#ifndef __SRV_ADC_H
#define __SRV_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief ADC 采样数据
 */
typedef struct {
    uint32_t timestamp_ms; /**< 时间戳 (ms) */
    uint32_t vin_mv; /**< VIN_ADC — 输入电压 (mV, raw*3300/4095) */
    uint16_t vin_raw; /**< VIN_ADC — 12-bit 原始值 */
    uint16_t v_imon_raw; /**< V_IMON — 12-bit 原始值（电流监控，换算待定） */
    bool valid; /**< 快照有效标志 */
} srv_adc_data_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化 ADC 采样管道 */
void srv_adc_init(void);

/** @brief 触发一次 ADC 扫描（由 task 层的 sw_timer 调用） */
void srv_adc_trigger(void);

/**
 * @brief ADC 处理步进（由 task 层 sw_timer 在主循环上下文调用）
 * @note  完成物理量换算并缓存最新快照
 */
void srv_adc_step(void);

/** @brief 获取最新采样数据（非阻塞） */
bool srv_adc_get_latest(srv_adc_data_t* sample);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_ADC_H */
