/**
 * @file    drv_status.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   系统状态与故障反馈驱动（GPIO 电平读取）
 * @attention
 *
 * 配置表内置在 drv_status.c 中，外部只需调用 init。
 */

#ifndef __DRV_STATUS_H
#define __DRV_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    DRV_STATUS_HSD_FAULT,       /**< HSD_FAULT — 高边驱动器故障汇总 */
    DRV_STATUS_12V_PGOOD,       /**< 12V_EXT_PGOOD — 12V 电源正常 */
    DRV_STATUS_24V_PGOOD,       /**< 24V_EXT_PGOOD — 24V 电源正常 */
    DRV_STATUS_24V_COMP_PGD,    /**< 24V_COMP_PGOOD — 工控机电源正常 */
    DRV_STATUS_AUX_PGD,         /**< AUX_POWER_PGD — 辅电电源正常 */
    DRV_STATUS_MOTOR_PGD,       /**< MOTOR_POWER_PGD — 电机电源正常 */
    DRV_STATUS_DBR_OCP_FLAG,    /**< DBR_LSD_OCP_FLAG — 制动电阻过流标志 */
    DRV_STATUS_MOTOR_CHG_OCP,   /**< MOTOR_POWER_CHG_OCP_FLAG — 电机充电过流标志 */
    DRV_STATUS_E_STOP_ON,       /**< E_STOP_ON — 急停触发状态 */

    DRV_STATUS_NUM,
} drv_status_signal_t;

/* Exported functions prototypes ---------------------------------------------*/

void drv_status_init(void);
void drv_status_deinit(void);

/** @brief 读取单个信号 */
bool drv_status_read(drv_status_signal_t sig);

/** @brief 获取信号名称字符串 */
const char* drv_status_name(drv_status_signal_t sig);

/**
 * @brief 读取所有信号为位掩码
 *        bit N = drv_status_read(signal at index N)
 */
uint32_t drv_status_read_all(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_STATUS_H */
