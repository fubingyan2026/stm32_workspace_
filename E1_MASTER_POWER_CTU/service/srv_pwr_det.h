/**
 * @file    srv_pwr_det.h
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-09-03
 * @brief   电源状态监控服务 — 封装 drv_status + CD4051B 急停冗余判据 (E1_MASTER_POWER_CTU)
 *
 * 提供语义化的电源状态查询接口。PGOOD/E-STOP 经 drv_status 读取。
 *
 * ## 急停双判据
 * - 数字判据：E_STOP_ON（PC9，低电平=按下）
 * - 冗余判据：CD4051B 轮询 4 组 E-STOP 双冗余节点（偶数 mux 通道=ADC1、奇数=ADC2），
 *   两节点电平一致（偏差 ≤ 容差）判该回路闭合；任一回路口径不一致（互补断开）即冗余侧检出断开
 * - 有效急停 = 数字按下 且 冗余闭合掩码非全闭合（双判据 AND）；
 *   仅数字按下而冗余仍全部闭合 → estop_inconsistent（线缆/按钮异常，不触发断电，仅告警）
 */

#ifndef __SRV_PWR_DET_H
#define __SRV_PWR_DET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/** @brief 全部 E-STOP 回路闭合掩码（4 路：E_STOP1=bit0 ... E_STOP4=bit3） */
#define SRV_PWR_DET_ESTOP_ALL_CLOSED_MASK (0x0FU)

/**
 * @brief E-STOP 冗余采样结果（由接线层回调填充，通常读 srv_adc 的 CD4051B 数据）
 */
typedef struct {
    bool valid; /**< 冗余采样有效（首轮完整轮转完成） */
    uint8_t closed_mask; /**< 各回路闭合位掩码（E_STOP1=bit0 ... E_STOP4=bit3） */
} srv_pwr_det_estop_redun_t;

/** @brief E-STOP 冗余数据读取回调（task 层接线实现，禁止为 NULL 时按冗余无效处理） */
typedef void (*srv_pwr_det_estop_redun_cb_t)(srv_pwr_det_estop_redun_t* redun);

/** @brief 常开轨使能掩码读取回调（返回 3bit：bit0=LM5060/VIN_DC-DC、bit1=24V、bit2=AUX；
 *         返回 NULL 时按“全部已使能”处理，即保留纯常开语义） */
typedef uint8_t (*srv_pwr_det_rail_en_cb_t)(void);

/**
 * @brief 电源状态汇总
 */
typedef struct {
    bool lm5060_ok; /**< VIN_DC-DC 前端正常（LM5060_PGOOD） */
    bool dc24v_ok; /**< 24V 降压电源正常（DC_DC_24V_PGOOD） */
    bool p12v_ok; /**< 12V 降压电源正常（PGOOD_12V） */
    bool aux_power_ok; /**< 辅助电源正常（AUX PGD） */
    bool motor_power_ok; /**< 电机电源正常（MOTOR PGD） */
    bool estop_on; /**< 有效急停触发状态（数字 E_STOP_ON 按下 且 冗余回路检出断开） */
    bool estop_inconsistent; /**< 双判据不一致（数字按下 但 冗余侧未断开），仅诊断不触发断电 */
} srv_pwr_det_status_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化电源监控服务
 * @param estop_redun_cb E-STOP 冗余数据读取回调（可为 NULL：冗余判据视为无效，
 *                       有效急停恒为 false，交由硬件/其他安全链路兜底）
 * @param rail_en_cb     常开轨使能掩码回调（可为 NULL：视为全部使能，恒监测）
 */
void srv_pwr_det_init(srv_pwr_det_estop_redun_cb_t estop_redun_cb,
    srv_pwr_det_rail_en_cb_t rail_en_cb);

/** @brief 读取电源状态（批量，推荐用于 RS485 上报打包） */
void srv_pwr_det_read(srv_pwr_det_status_t* status);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_DET_H */
