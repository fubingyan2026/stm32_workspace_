/**
 * @file    drv_cd4051b.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-04
 * @brief   CD4051B 8 通道模拟多路选择器驱动
 * @attention
 *
 * 选择引脚 A/B/C（PD3/PD4/PD5）由 CubeMX 配置为 OUTPUT_PP + PULLDOWN。
 * 公共输出 COM 接 PC5 (ADC2_IN15)，由 drv_adc 的 DRV_ADC_CH_CD4051B 通道采样。
 * A_IN1_IO/2_IO/3_IO 模拟采样输入经多路选择后接入 ADC 采样，见 srv_adc 的轮转逻辑。
 */

#ifndef __DRV_CD4051B_H
#define __DRV_CD4051B_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    DRV_CD4051B_OK = 0, /**< 成功 */
    DRV_CD4051B_ERROR_INVALID_CH, /**< 通道号越界（有效 0-7） */
} drv_cd4051b_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化多路选择器（默认选择通道 Y0，全部引脚拉低） */
void drv_cd4051b_init(void);

/**
 * @brief 选择输入通道
 * @param channel 通道号 0-7，对应 Y0-Y7（选择码 = C<<2 | B<<1 | A）
 * @return DRV_CD4051B_OK 成功；通道号越界返回 DRV_CD4051B_ERROR_INVALID_CH
 */
drv_cd4051b_error_t drv_cd4051b_select(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CD4051B_H */
