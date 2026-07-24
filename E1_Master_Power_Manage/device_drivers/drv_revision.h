/**
 * @file    drv_revision.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   硬件版本识别驱动（3-bit 版本编码）
 * @attention
 *
 * 通过 REV_PD0/PD1/PD2 三引脚电平读取硬件版本号。
 * 编码规则：version = PD2×4 + PD1×2 + PD0×1（0-7）
 * 引脚配置由 CubeMX main.h 定义，驱动层直接使用。
 */

#ifndef __DRV_REVISION_H
#define __DRV_REVISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 读取硬件版本号 (0-7) */
uint8_t drv_revision_read(void);

/** @brief 获取版本字符串，如 "V2" */
const char* drv_revision_name(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_REVISION_H */
