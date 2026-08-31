/**
 * @file    drv_efuse.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-31
 * @brief   24V eFuse 电源开关驱动实现（枚举信号表 + GPIO 读写）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_efuse.h"

#include "log.h"
#include "main.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_EFUSE_LOG_ENABLE 1

#if DRV_EFUSE_LOG_ENABLE
#define DRV_EFUSE_LOG_E(...) LOG_E("drv_efuse", __VA_ARGS__)
#define DRV_EFUSE_LOG_W(...) LOG_W("drv_efuse", __VA_ARGS__)
#define DRV_EFUSE_LOG_I(...) LOG_I("drv_efuse", __VA_ARGS__)
#define DRV_EFUSE_LOG_D(...) LOG_D("drv_efuse", __VA_ARGS__)
#else
#define DRV_EFUSE_LOG_E(...) ((void)0)
#define DRV_EFUSE_LOG_W(...) ((void)0)
#define DRV_EFUSE_LOG_I(...) ((void)0)
#define DRV_EFUSE_LOG_D(...) ((void)0)
#endif

/* Private types -------------------------------------------------------------*/

typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    bool          is_output; /**< true=输出引脚（SHDN） */
    bool          active_low; /**< true=低电平有效（FLT_N） */
    const char*   name;       /**< 信号名称（调试用） */
} drv_efuse_pin_t;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief eFuse 信号引脚配置表（引脚宏来自 Core/Inc/main.h，CubeMX 已配置）
 */
static const drv_efuse_pin_t s_pins[DRV_EFUSE_NUM] = {
    [DRV_EFUSE_SHDN]  = { EFUSE_SHDN_GPIO_Port,  EFUSE_SHDN_Pin,  true,  false, "SHDN" },
    [DRV_EFUSE_PGOOD] = { EFUSE_PGOOD_GPIO_Port, EFUSE_PGOOD_Pin, false, false, "PGOOD" },
    [DRV_EFUSE_FLT_N] = { EFUSE_FLT_N_GPIO_Port, EFUSE_FLT_N_Pin, false, true,  "FLT_N" },
};

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void drv_efuse_init(void)
{
    s_initialized = true;
    DRV_EFUSE_LOG_I("eFuse 驱动初始化完成 (%u 路信号)", (unsigned)DRV_EFUSE_NUM);
}

void drv_efuse_deinit(void)
{
    s_initialized = false;
    DRV_EFUSE_LOG_I("eFuse 驱动反初始化完成");
}

bool drv_efuse_is_initialized(void)
{
    return s_initialized;
}

bool drv_efuse_read(drv_efuse_signal_t sig)
{
    if (!s_initialized || sig >= DRV_EFUSE_NUM) {
        return false;
    }

    const drv_efuse_pin_t* pin = &s_pins[sig];
    if (!pin->port || !pin->pin) {
        return false;
    }

    const bool pin_high = (HAL_GPIO_ReadPin(pin->port, pin->pin) == GPIO_PIN_SET);

    /* 输出引脚（SHDN）回读当前驱动电平；输入引脚按 active_low 归一化 */
    return pin->active_low ? !pin_high : pin_high;
}

const char* drv_efuse_name(drv_efuse_signal_t sig)
{
    if (sig >= DRV_EFUSE_NUM) {
        return "INVALID";
    }
    return s_pins[sig].name ? s_pins[sig].name : "UNNAMED";
}

uint32_t drv_efuse_read_all(void)
{
    uint32_t mask = 0;

    for (uint32_t i = 0; i < DRV_EFUSE_NUM; i++) {
        if (drv_efuse_read((drv_efuse_signal_t)i)) {
            mask |= (1U << i);
        }
    }

    return mask;
}

void drv_efuse_set_shdn(bool on)
{
    if (!s_initialized) {
        return;
    }
    HAL_GPIO_WritePin(EFUSE_SHDN_GPIO_Port, EFUSE_SHDN_Pin,
        on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool drv_efuse_is_shdn(void)
{
    if (!s_initialized) {
        return false;
    }
    return HAL_GPIO_ReadPin(EFUSE_SHDN_GPIO_Port, EFUSE_SHDN_Pin) == GPIO_PIN_SET;
}
