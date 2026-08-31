/**
 * @file    drv_efuse.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-31
 * @brief   24V eFuse 电源开关驱动（枚举信号表 + GPIO 读写）
 * @attention
 *
 * 参考 E1_Master_Power_Manage 的 drv_status 风格：信号枚举 + 引脚配置表。
 * G0 手套 eFuse（CubeMX 已配置）：
 *   SHDN  — PB2  输出：0=关断 24V 输出，1=开启 24V 输出
 *   PGOOD — PB10 输入：1=输出正常（达门限）
 *   FLT_N — PB11 输入：0=故障（过流/过温等，低电平有效）
 */

#ifndef __DRV_EFUSE_H
#define __DRV_EFUSE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief eFuse 信号枚举
 */
typedef enum {
    DRV_EFUSE_SHDN = 0,  /**< SHDN — PB2 输出：0=关断 24V, 1=开启 24V */
    DRV_EFUSE_PGOOD,     /**< PGOOD — PB10 输入：1=输出正常 */
    DRV_EFUSE_FLT_N,     /**< FLT_N — PB11 输入：0=故障（低有效，读返回 1=故障） */

    DRV_EFUSE_NUM,
} drv_efuse_signal_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化 eFuse 驱动（引脚由 CubeMX 配置，此处仅登记信号表） */
void drv_efuse_init(void);

/** @brief 反初始化 eFuse 驱动 */
void drv_efuse_deinit(void);

bool drv_efuse_is_initialized(void);

/* --- 信号读取 --- */

/**
 * @brief 读取单个 eFuse 信号
 * @param sig 信号枚举
 * @return 逻辑有效电平（已按 active_low 归一化）：
 *         SHDN→当前输出状态；PGOOD→1=正常；FLT_N→1=故障
 */
bool drv_efuse_read(drv_efuse_signal_t sig);

/** @brief 获取信号名称字符串 */
const char* drv_efuse_name(drv_efuse_signal_t sig);

/**
 * @brief 读取所有信号为位掩码
 *        bit N = drv_efuse_read(signal at index N)
 */
uint32_t drv_efuse_read_all(void);

/* --- 输出控制（SHDN） --- */

/**
 * @brief 设置 24V 输出开关
 * @param on true=开启（SHDN=1），false=关断（SHDN=0）
 */
void drv_efuse_set_shdn(bool on);

/** @brief 查询当前 24V 输出使能状态（SHDN 电平） */
bool drv_efuse_is_shdn(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_EFUSE_H */
