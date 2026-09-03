/**
 * @file    srv_adc.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   ADC 采样服务 — 物理量换算 + VREFINT 校准 + msg_fifo (E1_MASTER_POWER_CTU)
 *
 * service 层仅提供数据处理管道，不管理 sw_timer（由 task 层负责）。
 * 通道路由和 ADC 实例管理内置在 drv_adc 中。
 *
 * 本期采样集：NTC1/NTC2 温度、VIN/VIN_DC-DC 电压、MCU 内部温度、VDDA 校准。
 * CD4051B 通道采样值保留但不参与业务（MUX 未启用）。
 */

#ifndef __SRV_ADC_H
#define __SRV_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ADC 物理量换算返回状态
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
    uint32_t vin_mv; /**< VIN_ADC — 主母线输入电压 (mV, 1/23 分压) */
    uint32_t vin_dcdc_mv; /**< VIN_DC-DC_ADC — DC-DC 输入电压 (mV, 1/23 分压) */

    /* ── 内部校准 ── */
    uint32_t vdda_mv; /**< VDDA 实际值 (mV)，由 VREFINT 反推 */

    /* ── 温度 ── */
    int16_t ntc1_temp_x100; /**< NTC1 外部温度 (°C × 100) */
    int16_t ntc2_temp_x100; /**< NTC2 DC-DC MOS 温度 (°C × 100) */
    int16_t mcu_temp_x100; /**< MCU 内部温度 (°C × 100) */

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
 * 并写入采样 FIFO。与 srv_adc_trigger() 配对使用：
 * trigger 启动扫描 → DMA 中断回调只做原始快照 → 本函数负责全部换算与日志
 * （不占用中断上下文，避免 float 计算与 printf 阻塞低优先级中断）。
 */
void srv_adc_step(void);

/** @brief 获取最新采样数据（非阻塞） */
bool srv_adc_get_latest(srv_adc_data_t* sample);

/* --- CD4051B 多路 E-STOP 双冗余采样（srv_adc 按周期轮转选通） --- */

/** @brief 读取某 mux 通道最近一次采样值（12-bit；未采到过返回 0） */
uint16_t srv_adc_mux_raw(uint8_t ch);

/**
 * @brief E-STOP 冗余闭合位掩码（bit i = E_STOP(i+1) 双路冗余均正常闭合）
 *
 * CD4051B 通道对：Y(2i) = E_STOP(i+1)_ADC1 节点、Y(2i+1) = E_STOP(i+1)_ADC2 节点。
 * 判定（沿用 F407 双通道冗余）：
 *   - 两节点采样偏差 ≤ 容差（电平一致，触点闭合）→ 该回路 bit=1（闭合）
 *   - 偏差远离 0（冗余互补，急停按下断开）或处于中间区间（线缆异常）→ bit=0
 * @return 4 位闭合掩码（E_STOP1=bit0 ... E_STOP4=bit3；全闭合常量定义于消费方
 *         srv_pwr_det：SRV_PWR_DET_ESTOP_ALL_CLOSED_MASK）
 */
uint8_t srv_adc_estop_closed_mask(void);

/**
 * @brief E-STOP 冗余采样有效标志
 * @note  全部 8 个 mux 通道均至少完成一轮采样才为 true（首个完整轮转约 80ms）。
 *        （与数字 E_STOP_ON 组成双判据前必须确认此标志，避免轮转未就绪误判）
 */
bool srv_adc_estop_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_ADC_H */
