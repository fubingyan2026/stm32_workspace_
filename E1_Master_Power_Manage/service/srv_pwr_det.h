/**
 * @file    srv_pwr_det.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   电源状态监控服务 — 封装 drv_status 层
 *
 * 提供语义化的电源状态查询接口。PGOOD/E-STOP/故障标志经 drv_status 读取。
 * 模拟输入 A_IN1_IO/2_IO/3_IO 统一由 srv_adc 管理（见 srv_adc_read_ain）。
 */

#ifndef __SRV_PWR_DET_H
#define __SRV_PWR_DET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电源状态汇总
 */
typedef struct {
    bool ext_12v_ok; /**< 外部 12V 电源正常 */
    bool ext_24v_ok; /**< 外部 24V 电源正常 */
    bool comp_24v_ok; /**< 工控机 24V 电源正常 */
    bool aux_power_ok; /**< 辅助电源正常 */
    bool motor_power_ok; /**< 电机电源正常 */
    bool hsd_fault; /**< 高边驱动故障汇总 */
    bool dbr_ocp; /**< 制动电阻过流标志 */
    bool motor_chg_ocp; /**< 电机充电过流标志 */
    bool estop_on; /**< 急停触发状态 */
} srv_pwr_det_status_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化电源监控服务
 * @note  仅记录初始化状态，无额外配置
 */
void srv_pwr_det_init(void);

/** @brief 读取电源状态（批量，推荐用于 CAN 上报打包） */
void srv_pwr_det_read(srv_pwr_det_status_t* status);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_DET_H */
