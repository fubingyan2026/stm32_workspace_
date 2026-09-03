/**
 * @file    srv_adc.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   ADC 采样服务 — 物理量换算 + VREFINT 校准 + msg_fifo (E1_SLAVER_POWER_CTU)
 *
 * service 层仅提供数据处理管道，不管理 sw_timer（由 task 层负责）。
 * 通道路由和 ADC 实例管理内置在 drv_adc 中。
 *
 * 本期采样集：AUX/MOTOR/LSD1/LSD2 电压、MCU 内部温度、VDDA 校准。
 * 分压比：AUX/MOTOR 220k/10k(×23)，LSD1/LSD2 100k/10k(×11)。
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
    SRV_ADC_CALC_ERR_LOW_RAW, /**< 采样值过低（输入缺失/未上电） */
} srv_adc_calc_status_t;

/**
 * @brief ADC 采样数据
 */
typedef struct {
    uint32_t timestamp_ms; /**< 时间戳 (ms) */

    /* ── 外部电压 (mV, VREFINT 校准后) ── */
    uint32_t aux_mv;    /**< AUX_POWER_ADC — 副电源输入电压 (mV, ×23 分压) */
    uint32_t motor_mv;  /**< MOTOR_POWER_ADC — 电机电源输入电压 (mV, ×23 分压) */
    uint32_t lsd1_mv;   /**< LSD1_ADC — 低边开关 1 输出节点电压 (mV, ×11 分压) */
    uint32_t lsd2_mv;   /**< LSD2_ADC — 低边开关 2 输出节点电压 (mV, ×11 分压) */

    /* ── 内部校准 ── */
    uint32_t vdda_mv;   /**< VDDA 实际值 (mV)，由 VREFINT 反推 */

    /* ── 温度 ── */
    int16_t mcu_temp_x100; /**< MCU 内部温度 (°C × 100) */

    /* ── 换算状态（供上层观测各物理量计算是否异常） ── */
    srv_adc_calc_status_t vdda_status;   /**< VDDA 校准状态 */
    srv_adc_calc_status_t mcu_temp_status; /**< MCU 温度换算状态 */
    srv_adc_calc_status_t aux_status;    /**< AUX 电压换算状态 */
    srv_adc_calc_status_t motor_status;  /**< MOTOR 电压换算状态 */
    srv_adc_calc_status_t lsd1_status;   /**< LSD1 电压换算状态 */
    srv_adc_calc_status_t lsd2_status;   /**< LSD2 电压换算状态 */
} srv_adc_data_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化 ADC 采样管道（ADC + 路由 + 回调 + FIFO） */
void srv_adc_init(void);

/** @brief 触发一次 ADC 扫描（由 task 层的 sw_timer 调用） */
void srv_adc_trigger(void);

/**
 * @brief ADC 处理步进（由 task 层 sw_timer 在主循环上下文调用）
 *
 * 从原始快照 FIFO 取最新一帧，完成 PT1 滤波、VREFINT 校准、电压/温度换算，
 * 并写入采样 FIFO。与 srv_adc_trigger() 配对使用：
 * trigger 启动扫描 → DMA 中断回调只做原始快照 → 本函数负责全部换算与日志
 * （不占用中断上下文，避免 float 计算与 printf 阻塞低优先级中断）。
 */
void srv_adc_step(void);

/** @brief 获取最新采样数据（非阻塞） */
bool srv_adc_get_latest(srv_adc_data_t* sample);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_ADC_H */
