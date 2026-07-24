/**
 * @file    drv_adc.h
 * @author  maximillian
 * @version V2.1.0
 * @date    2026-07-8
 * @brief   ADC 设备驱动（3 实例 DMA + 内置通道路由表）
 * @attention
 *
 * 硬件配置（句柄表 + 路由表）内置在 drv_adc.c 中，上层无需传参。
 * drv_adc_init() 自动初始化全部 3 个 ADC 实例。
 *
 * ## ADC 实例分配
 * - ADC1: E-STOP 通道1 冗余采样 (PA0/PA2/PA4/PA6) + 内部通道 (TEMPSENSOR/VREFINT/VBAT)
 * - ADC2: E-STOP 通道2 冗余采样 (PA1/PA3/PA5/PA7) + 电压采样 (VIN/MOTOR/AUX: PC2/PC3/PC4)
 * - ADC3: NTC 温度 (PC0, PC1)
 *
 * ## 用法
 * @code
 *   drv_adc_init();
 *   drv_adc_register_callback(my_callback);
 *   drv_adc_trigger_all();
 *   uint32_t raw = drv_adc_read_raw(DRV_ADC_CH_VIN);
 * @endcode
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
 * @brief ADC 实例（物理 ADC 外设）
 */
typedef enum {
    DRV_ADC_INST_1 = 0, /**< ADC1 — E-STOP_CH1 + 内部通道 */
    DRV_ADC_INST_2,     /**< ADC2 — E-STOP_CH2 + 电压采样 */
    DRV_ADC_INST_3,     /**< ADC3 — NTC 温度 */
    DRV_ADC_INST_NUM,
} drv_adc_inst_t;

/**
 * @brief 逻辑通道（与 Core/Src/adc.c MX_ADCx_Init 的 Rank 顺序严格对应）
 *
 * DMA 缓冲区索引 = Rank - 1
 */
typedef enum {
    /* ── ADC2: 电压采样 (Rank 5/6/7) ── */
    DRV_ADC_CH_VIN = 0,         /**< VIN_ADC — 主输入总线电压 (PC2, ADC2_IN12) */
    DRV_ADC_CH_MOTOR_POWER,     /**< MOTOR_POWER_ADC — 电机供电电压 (PC3, ADC2_IN13) */
    DRV_ADC_CH_AUX_POWER,       /**< AUX_POWER_ADC — 辅助电源电压 (PC4, ADC2_IN14) */

    /* ── ADC1: E-STOP 通道1 冗余 (Rank 1/2/3/4) ── */
    DRV_ADC_CH_E_STOP1_ADC1,    /**< 急停1 通道1 (PA0, ADC1_IN0) */
    DRV_ADC_CH_E_STOP1_ADC2,    /**< 急停1 通道2 (PA1, ADC2_IN1) */
    DRV_ADC_CH_E_STOP2_ADC1,    /**< 急停2 通道1 (PA2, ADC1_IN2) */
    DRV_ADC_CH_E_STOP2_ADC2,    /**< 急停2 通道2 (PA3, ADC2_IN3) */
    DRV_ADC_CH_E_STOP3_ADC1,    /**< 急停3 通道1 (PA4, ADC1_IN4) */
    DRV_ADC_CH_E_STOP3_ADC2,    /**< 急停3 通道2 (PA5, ADC2_IN5) */
    DRV_ADC_CH_E_STOP4_ADC1,    /**< 急停4 通道1 (PA6, ADC1_IN6) */
    DRV_ADC_CH_E_STOP4_ADC2,    /**< 急停4 通道2 (PA7, ADC2_IN7) */

    /* ── ADC3: NTC 温度 (Rank 1) ── */
    DRV_NTC1_ADC,               /**< NTC1 温度 (PC0, ADC3_IN10) */
    DRV_NTC2_ADC,               /**< NTC2 温度 (PC1, ADC3_IN11) — 待 CubeMX 配置 */

    /* ── ADC1: 内部通道 (Rank 5/6/7) ── */
    DRV_ADC_CH_TEMPSENSOR,      /**< MCU 内部温度传感器 (ADC1_IN16) */
    DRV_ADC_CH_VREFINT,         /**< 内部参考电压 ~1.21V (ADC1_IN17) */
    DRV_ADC_CH_VBAT,            /**< 备份电池电压 (ADC1_IN18) */

    DRV_ADC_CH_MAX,
} drv_adc_channel_t;

/** @brief 通道路由项：逻辑通道 → (ADC实例, DMA缓冲区索引) */
typedef struct {
    drv_adc_inst_t inst;
    uint8_t dma_idx;
} drv_adc_route_t;

/** @brief ADC 转换完成回调（中断上下文） */
typedef void (*drv_adc_callback_t)(drv_adc_inst_t inst);

typedef enum {
    DRV_ADC_OK = 0,
    DRV_ADC_ERROR_NULL_PTR,
    DRV_ADC_ERROR_UNINITIALIZED,
    DRV_ADC_ERROR_BUSY,
    DRV_ADC_ERROR_INVALID_PARAM,
} drv_adc_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化全部 ADC 实例（内部句柄表 + 路由表，无需传参） */
void drv_adc_init(void);

/** @brief 反初始化全部 ADC 实例（停止 DMA） */
void drv_adc_deinit_all(void);

/* --- 触发 --- */

/** @brief 触发单个 ADC 实例 DMA 采样 */
drv_adc_error_t drv_adc_trigger(drv_adc_inst_t inst);

/** @brief 触发全部已初始化 ADC 实例 */
void drv_adc_trigger_all(void);

/** @brief 查询 ADC 实例是否正在 DMA 传输 */
bool drv_adc_is_busy(drv_adc_inst_t inst);

/* --- 读取 --- */

/** @brief 读取逻辑通道最近一次 DMA 采样值（12-bit 原始值） */
uint32_t drv_adc_read_raw(drv_adc_channel_t ch);

/* --- 回调 --- */

/** @brief 注册 ADC 转换完成回调（所有实例共享，通过 inst 参数区分） */
drv_adc_error_t drv_adc_register_callback(drv_adc_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_ADC_H */
