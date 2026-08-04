/**
 * @file    srv_adc.h
 * @author  maximillian
 * @version V2.1.0
 * @date    2026-07-8
 * @brief   ADC 采样服务 — 物理量换算 + VREFINT 校准 + msg_fifo
 *
 * service 层仅提供数据处理管道，不管理 sw_timer（由 task 层负责）。
 * 通道路由和 ADC 实例管理内置在 drv_adc 中。
 */

#ifndef __SRV_ADC_H
#define __SRV_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief ADC 物理量换算返回状态
 *
 * 由 calc_vdda_mv / calc_mcu_temp / ntc_raw_to_temp 返回，真实数值经出参指针输出，
 * 状态码区分计算失败原因（校准无效 / 短路 / 开路 / 采样过低），并存入
 * srv_adc_data_t 对应字段供上层观测。
 */
typedef enum {
    SRV_ADC_CALC_OK = 0,      /**< 计算成功 */
    SRV_ADC_CALC_ERR_PARAM,   /**< 参数非法（空指针等） */
    SRV_ADC_CALC_ERR_CAL,     /**< 校准值无效 */
    SRV_ADC_CALC_ERR_SHORT,   /**< 传感器短路 */
    SRV_ADC_CALC_ERR_OPEN,    /**< 传感器开路 */
    SRV_ADC_CALC_ERR_LOW_RAW, /**< 采样值过低 */
} srv_adc_calc_status_t;

/**
 * @brief ADC 采样数据
 */
typedef struct {
    uint32_t timestamp_ms; /**< 时间戳 (ms) */

    /* ── 外部电压 (mV, VREFINT 校准后) ── */
    uint32_t vin_mv; /**< VIN_ADC — 主输入总线电压 (mV) */
    uint32_t motor_power_mv; /**< MOTOR_POWER_ADC — 电机供电电压 (mV) */
    uint32_t aux_power_mv; /**< AUX_POWER_ADC — 辅助电源电压 (mV) */

    /* ── 内部校准 ── */
    uint32_t vdda_mv; /**< VDDA 实际值 (mV)，由 VREFINT 反推 */

    /* ── 急停双通道冗余采样 (12-bit 原始值) ── */
    uint16_t e_stop1_adc1;
    uint16_t e_stop1_adc2;
    uint16_t e_stop2_adc1;
    uint16_t e_stop2_adc2;
    uint16_t e_stop3_adc1;
    uint16_t e_stop3_adc2;
    uint16_t e_stop4_adc1;
    uint16_t e_stop4_adc2;

    /* ── 温度 ── */
    int16_t ntc1_temp_x100; /**< NTC1 外部温度 (°C × 100) */
    int16_t ntc2_temp_x100; /**< NTC2 外部温度 (°C × 100) */
    int16_t mcu_temp_x100; /**< MCU 内部温度 (°C × 100) */

    /* ── 备份电池 ── */
    uint32_t vbat_mv; /**< VBAT 备份电池电压 (mV) */

    /* ── CD4051B 多路选择 A_INx_IO (12-bit 原始值) ──
     * A_IN1_IO/2_IO/3_IO 模拟采样输入经 CD4051B (Y1/Y2/Y3) 多路选择 → PC5 采样，
     * 每 3 个采样周期轮转更新一路（30ms 全量刷新一次）。 */
    uint16_t a_in1_io_raw; /**< A_IN1_IO 原始采样值（CD4051B Y1 → PC5, ADC2_IN15） */
    uint16_t a_in2_io_raw; /**< A_IN2_IO 原始采样值（CD4051B Y2 → PC5, ADC2_IN15） */
    uint16_t a_in3_io_raw; /**< A_IN3_IO 原始采样值（CD4051B Y3 → PC5, ADC2_IN15） */

    /* ── 换算状态（供上层观测各物理量计算是否异常） ── */
    srv_adc_calc_status_t vdda_status; /**< VDDA 校准状态 */
    srv_adc_calc_status_t ntc1_status; /**< NTC1 温度换算状态 */
    srv_adc_calc_status_t ntc2_status; /**< NTC2 温度换算状态 */
    srv_adc_calc_status_t mcu_temp_status; /**< MCU 温度换算状态 */
} srv_adc_data_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化 ADC 采样管道（ADC + 路由 + 回调 + FIFO） */
void srv_adc_init(void);

/** @brief 触发一次 ADC 扫描（由 task 层的 sw_timer 调用） */
void srv_adc_trigger(void);

/**
 * @brief ADC 处理步进（由 task 层 sw_timer 在主循环上下文调用）
 *
 * 从原始快照 FIFO 取最新一帧，完成 PT1 滤波、VREFINT 校准、温度/NTC 换算，
 * 并写入采样 FIFO、输出遥测日志。与 srv_adc_trigger() 配对使用：
 * trigger 启动扫描 → DMA 中断回调只做原始快照 → 本函数负责全部换算与日志
 * （不占用中断上下文，避免 float 计算与 printf 阻塞低优先级中断）。
 */
void srv_adc_step(void);

/** @brief 获取最新采样数据（非阻塞） */
bool srv_adc_get_latest(srv_adc_data_t* sample);

/**
 * @brief 读取模拟输入逻辑状态（A_IN1_IO/2_IO/3_IO）
 *
 * 基于最新快照的 a_in1_io_raw/2_io_raw/3_io_raw 按阈值判定逻辑高低。
 * @return 位掩码：bit0=A_IN1_IO，bit1=A_IN2_IO，bit2=A_IN3_IO；srv_adc 未初始化或尚无快照时返回 0
 */
uint8_t srv_adc_read_ain(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_ADC_H */
