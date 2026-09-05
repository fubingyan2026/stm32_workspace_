/**
 * @file    drv_status.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   系统状态与故障反馈驱动（GPIO 电平读取，E1_SLAVER_POWER_CTU）
 * @attention
 *
 * 配置表内置在 drv_status.c 中，外部只需调用 init。
 * 副电源模块状态输入 2 路：
 *   - EXT_PCOOG_24V (LM5146 24V 输出 PGOOD, PA1, 高=好)
 *   - ISO_PGOOD_12V (12V_ISO 窗口比较器 11.4~12.6V, PA5, 高=好)
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
    DRV_STATUS_24V_PGD,         /**< EXT_PCOOG_24V — 24V 降压输出正常 (PA1, 高有效) */
    DRV_STATUS_ISO12V_PGD,      /**< ISO_PGOOD_12V — 12V_ISO 输出正常 (PA5, 高有效) */

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
