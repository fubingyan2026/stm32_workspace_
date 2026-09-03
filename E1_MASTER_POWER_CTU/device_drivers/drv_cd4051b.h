/**
 * @file    drv_cd4051b.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   CD4051B 8 通道模拟多路选择器驱动（E1_MASTER_POWER_CTU）
 * @attention
 *
 * 选择引脚 A/B/C（PA4/PA5/PA6）由 CubeMX 配置为 OUTPUT_PP。
 * 公共输出 COM 接 PC4 (ADC1_IN14)，由 drv_adc 的 DRV_ADC_CH_CD4051B 通道采样。
 * 板上用途：轮询各急停回路节点电平，作为 PC9 数字急停的冗余判据（见 srv_adc/srv_pwr_det）。
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

/** @brief 初始化多路选择器（默认选择通道 Y0，选择码全 0） */
void drv_cd4051b_init(void);

/**
 * @brief 选择输入通道
 * @param channel 通道号 0-7，对应 Y0-Y7（选择码 = C<<2 | B<<1 | A，PA6=C, PA5=B, PA4=A）
 * @return DRV_CD4051B_OK 成功；通道号越界返回 DRV_CD4051B_ERROR_INVALID_CH
 */
drv_cd4051b_error_t drv_cd4051b_select(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CD4051B_H */
