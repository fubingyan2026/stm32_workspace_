/**
 * @file    drv_cd4051b.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   CD4051B 8 通道模拟多路选择器驱动实现（E1_MASTER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_cd4051b.h"

#include "log.h"
#include "main.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_CD4051B_LOG_ENABLE 0

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

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void mux_write_select_code(uint8_t code);

/* Exported functions --------------------------------------------------------*/

void drv_cd4051b_init(void)
{
    mux_write_select_code(0); /* 默认选择 Y0，选择码全 0 */
    s_initialized = true;

    DRV_CD4051B_LOG_I("CD4051B 多路选择器初始化完成 (默认 Y0)");
}

drv_cd4051b_error_t drv_cd4051b_select(uint8_t channel)
{
    if (channel > 7U) {
        return DRV_CD4051B_ERROR_INVALID_CH;
    }

    mux_write_select_code(channel);

    return DRV_CD4051B_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 按选择码驱动 A/B/C 引脚（高电平输出）
 * @param code 0-7；code bit0=A(PA4), bit1=B(PA5), bit2=C(PA6)
 */
static void mux_write_select_code(uint8_t code)
{
    HAL_GPIO_WritePin(CD4051B_A_GPIO_Port, CD4051B_A_Pin,
        (code & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CD4051B_B_GPIO_Port, CD4051B_B_Pin,
        (code & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CD4051B_C_GPIO_Port, CD4051B_C_Pin,
        (code & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
