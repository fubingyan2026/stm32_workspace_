/**
 * @file    drv_cd4051b.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-04
 * @brief   CD4051B 8 通道模拟多路选择器驱动实现
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_cd4051b.h"

#include "log.h"
#include "main.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_CD4051B_LOG_ENABLE 1

#if DRV_CD4051B_LOG_ENABLE
#define DRV_CD4051B_LOG_E(...) LOG_E("drv_cd4051b", __VA_ARGS__)
#define DRV_CD4051B_LOG_W(...) LOG_W("drv_cd4051b", __VA_ARGS__)
#define DRV_CD4051B_LOG_I(...) LOG_I("drv_cd4051b", __VA_ARGS__)
#define DRV_CD4051B_LOG_D(...) LOG_D("drv_cd4051b", __VA_ARGS__)
#else
#define DRV_CD4051B_LOG_E(...) ((void)0)
#define DRV_CD4051B_LOG_W(...) ((void)0)
#define DRV_CD4051B_LOG_I(...) ((void)0)
#define DRV_CD4051B_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define DRV_CD4051B_CH_MAX (8U) /**< 通道数 Y0-Y7 */

/* Exported functions --------------------------------------------------------*/

void drv_cd4051b_init(void)
{
    HAL_GPIO_WritePin(CD4051B_A_GPIO_Port, CD4051B_A_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CD4051B_B_GPIO_Port, CD4051B_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CD4051B_C_GPIO_Port, CD4051B_C_Pin, GPIO_PIN_RESET);

    DRV_CD4051B_LOG_I("CD4051B 多路选择器初始化完成 (默认通道 Y1)");
}

drv_cd4051b_error_t drv_cd4051b_select(uint8_t channel)
{
    if (channel >= DRV_CD4051B_CH_MAX) {
        DRV_CD4051B_LOG_E("通道号越界: %u (有效 0-7)", (unsigned)channel);
        return DRV_CD4051B_ERROR_INVALID_CH;
    }

    HAL_GPIO_WritePin(CD4051B_A_GPIO_Port, CD4051B_A_Pin,
        (channel & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CD4051B_B_GPIO_Port, CD4051B_B_Pin,
        (channel & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CD4051B_C_GPIO_Port, CD4051B_C_Pin,
        (channel & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    return DRV_CD4051B_OK;
}
