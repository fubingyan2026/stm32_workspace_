/**
 * @file    drv_power.c
 * @author  maximillian
 * @version V2.2.0
 * @date    2026-07-2
 * @brief   电源轨使能控制驱动实现
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_power.h"

#include "main.h"

/* Private types -------------------------------------------------------------*/

typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    const char*   name;
} drv_power_rail_pin_t;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 电源轨引脚配置表
 *
 * 引脚宏定义来自 Core/Inc/main.h，每路对应 CubeMX 已配置的 GPIO 输出。
 */
static const drv_power_rail_pin_t s_pins[DRV_POWER_RAIL_NUM] = {
    [DRV_POWER_RAIL_HSD1_12V]      = { HSD1_IN_12V_GPIO_Port,      HSD1_IN_12V_Pin,      "12V_HSD1" },
    [DRV_POWER_RAIL_HSD1_24V]      = { HSD1_IN_24V_GPIO_Port,      HSD1_IN_24V_Pin,      "24V_HSD1" },
    [DRV_POWER_RAIL_HSD2_24V]      = { HSD2_IN_24V_GPIO_Port,      HSD2_IN_24V_Pin,      "24V_HSD2" },
    [DRV_POWER_RAIL_HSD1_12V_DIAG] = { HSD1_DIAG_EN_12V_GPIO_Port, HSD1_DIAG_EN_12V_Pin, "12V_HSD1_DIAG" },
    [DRV_POWER_RAIL_HSD1_24V_DIAG] = { HSD1_DIAG_EN_24V_GPIO_Port, HSD1_DIAG_EN_24V_Pin, "24V_HSD1_DIAG" },
    [DRV_POWER_RAIL_HSD2_24V_DIAG] = { HSD2_DIAG_EN_24V_GPIO_Port, HSD2_DIAG_EN_24V_Pin, "24V_HSD2_DIAG" },
    [DRV_POWER_RAIL_AUX_EN]        = { AUX_POWER_EN_GPIO_Port,     AUX_POWER_EN_Pin,     "AUX_EN" },
    [DRV_POWER_RAIL_MOTOR_EN]      = { MOTOR_POWER_EN_GPIO_Port,   MOTOR_POWER_EN_Pin,   "MOTOR_EN" },
    [DRV_POWER_RAIL_MOTOR_CHG_EN]  = { MOTOR_POWER_CHG_EN_GPIO_Port, MOTOR_POWER_CHG_EN_Pin, "MOTOR_CHG" },
    [DRV_POWER_RAIL_DBR_LSD_EN]    = { DBR_LSD_EN_GPIO_Port,      DBR_LSD_EN_Pin,       "DBR_LSD" },
    [DRV_POWER_RAIL_MOTOR_CHG_IN]  = { MOTOR_POWER_CHG_IN_GPIO_Port, MOTOR_POWER_CHG_IN_Pin, "MOTOR_CHG_IN" },
    [DRV_POWER_RAIL_DC_DC_EN]      = { DC_DC_EN_GPIO_Port,        DC_DC_EN_Pin,         "DC_DC_EN" },
};

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void drv_power_init(void)
{
    if (s_initialized) {
        drv_power_deinit();
    }

    for (uint32_t i = 0; i < DRV_POWER_RAIL_NUM; i++) {
        if (s_pins[i].port && s_pins[i].pin) {
            HAL_GPIO_WritePin(s_pins[i].port, s_pins[i].pin, GPIO_PIN_RESET);
        }
    }

    s_initialized = true;
}

void drv_power_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    for (uint32_t i = 0; i < DRV_POWER_RAIL_NUM; i++) {
        if (s_pins[i].port && s_pins[i].pin) {
            HAL_GPIO_WritePin(s_pins[i].port, s_pins[i].pin, GPIO_PIN_RESET);
        }
    }

    s_initialized = false;
}

void drv_power_set(drv_power_rail_t rail, bool on)
{
    if (!s_initialized || rail >= DRV_POWER_RAIL_NUM) {
        return;
    }

    if (s_pins[rail].port && s_pins[rail].pin) {
        HAL_GPIO_WritePin(s_pins[rail].port, s_pins[rail].pin,
            on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

const char* drv_power_rail_name(drv_power_rail_t rail)
{
    if (rail >= DRV_POWER_RAIL_NUM) {
        return "INVALID";
    }
    return s_pins[rail].name ? s_pins[rail].name : "UNNAMED";
}
