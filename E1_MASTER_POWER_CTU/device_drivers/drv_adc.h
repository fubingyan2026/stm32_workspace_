/**
 * @file    drv_adc.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   ADC 设备驱动（单 ADC1 DMA + 内置通道路由表，E1_MASTER_POWER_CTU）
 * @attention
 *
 * 硬件配置（句柄表 + 路由表）内置在 drv_adc.c 中，上层无需传参。
 * drv_adc_init() 自动初始化 ADC1。
 *
 * ## ADC 实例分配
 * - ADC1: PC0/PC1/PC2/PC3/PC4 (IN10~IN14) + 内部通道 (TEMPSENSOR/VREFINT)
 *   Rank1..7 = NTC1, NTC2, VIN_DC-DC, VIN, CD4051B(预留), TEMPSENSOR, VREFINT
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
    DRV_ADC_INST_1 = 0, /**< ADC1 — 外部通道 + 内部通道 */
    DRV_ADC_INST_NUM,
} drv_adc_inst_t;

/**
 * @brief 逻辑通道（与 Core/Src/adc.c MX_ADC1_Init 的 Rank 顺序严格对应）
 *
 * DMA 缓冲区索引 = Rank - 1
 */
typedef enum {
    DRV_ADC_CH_NTC1 = 0,        /**< NTC1_ADC — 外部 NTC 测温 (PC0, ADC1_IN10, Rank1) */
    DRV_ADC_CH_NTC2,            /**< NTC2_ADC — DC-DC MOS 测温 (PC1, ADC1_IN11, Rank2) */
    DRV_ADC_CH_VIN_DC_DC,       /**< VIN_DC_DC_ADC — DC-DC 输入电压 (PC2, ADC1_IN12, Rank3) */
    DRV_ADC_CH_VIN,             /**< VIN_ADC — 主母线输入电压 (PC3, ADC1_IN13, Rank4) */
    DRV_ADC_CH_CD4051B,         /**< CD4051B_ADC — 多路选择器输出 (PC4, ADC1_IN14, Rank5, 预留) */
    DRV_ADC_CH_TEMPSENSOR,      /**< MCU 内部温度传感器 (ADC1_IN16, Rank6) */
    DRV_ADC_CH_VREFINT,         /**< 内部参考电压 ~1.20V (ADC1_IN17, Rank7) */

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

/**
 * @brief 恢复 ADC 链路：中止残留 DMA、清 busy（供“长时间无采样快照”看门狗调用）
 * @param inst ADC 实例
 * @return 操作结果错误码
 */
drv_adc_error_t drv_adc_recover(drv_adc_inst_t inst);

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
