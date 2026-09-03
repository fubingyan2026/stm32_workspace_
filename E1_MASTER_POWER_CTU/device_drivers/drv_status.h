/**
 * @file    drv_status.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   系统状态与故障反馈驱动（GPIO 电平读取，E1_MASTER_POWER_CTU）
 * @attention
 *
 * 配置表内置在 drv_status.c 中，外部只需调用 init。
 * CTU 板状态输入 6 路：
 *   - LM5060_PGOOD   (VIN_DC-DC 前端就绪, PA8)
 *   - DC_DC_24V_PGOOD(24V/20A 降压 PGOOD, PB5)
 *   - PGOOD_12V      (12V Buck PGOOD, PA12)
 *   - AUX_PWER_PGD   (辅助电源热插拔 PGD, PC12)
 *   - MOTOR_POWER_PGD(电机电源热插拔 PGD, PC10)
 *   - E_STOP_ON      (急停触发状态, PC9, 低电平有效)
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
    DRV_STATUS_LM5060_PGD,      /**< LM5060_PGOOD — VIN_DC-DC 电源前端就绪 */
    DRV_STATUS_DC24V_PGD,       /**< DC_DC_24V_PGOOD — 24V 降压电源正常 */
    DRV_STATUS_12V_PGD,         /**< PGOOD_12V — 12V 降压电源正常 */
    DRV_STATUS_AUX_PGD,         /**< AUX_PWER_PGD — 辅助电源正常 */
    DRV_STATUS_MOTOR_PGD,       /**< MOTOR_POWER_PGD — 电机电源正常 */
    DRV_STATUS_E_STOP_ON,       /**< E_STOP_ON — 急停触发状态 */

    DRV_STATUS_NUM,
} drv_status_signal_t;

/* Exported functions prototypes ---------------------------------------------*/

void drv_status_init(void);
void drv_status_deinit(void);

/** @brief 读取单个信号（已按 active_low 归一到「有效 = true」） */
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
