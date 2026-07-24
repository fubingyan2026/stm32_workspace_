/**
 * @file    drv_status.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   系统状态与故障反馈驱动实现
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_status.h"

#include "main.h"

/* Private types -------------------------------------------------------------*/

typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    bool          active_low;
    const char*   name;       /**< 信号名称（调试用） */
} drv_status_pin_t;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 状态信号引脚配置表
 *
 * 引脚宏定义来自 Core/Inc/main.h，每路对应 CubeMX 已配置的 GPIO 输入。
 */
static const drv_status_pin_t s_pins[DRV_STATUS_NUM] = {
    [DRV_STATUS_HSD_FAULT]       = { HSD_FAULT_GPIO_Port,         HSD_FAULT_Pin,         false, "HSD_FAULT" },
    [DRV_STATUS_12V_PGOOD]       = { EXT_PGOOD_12V_GPIO_Port,     EXT_PGOOD_12V_Pin,     false, "12V_PGOOD" },
    [DRV_STATUS_24V_PGOOD]       = { EXT_PGOOD_24V_GPIO_Port,     EXT_PGOOD_24V_Pin,     false, "24V_PGOOD" },
    [DRV_STATUS_24V_COMP_PGD]    = { COMP_PGOOD_24V_GPIO_Port,    COMP_PGOOD_24V_Pin,    false, "24V_COMP_PGD" },
    [DRV_STATUS_AUX_PGD]         = { AUX_POWER_PGD_GPIO_Port,     AUX_POWER_PGD_Pin,     false, "AUX_PGD" },
    [DRV_STATUS_MOTOR_PGD]       = { MOTOR_POWER_PGD_GPIO_Port,   MOTOR_POWER_PGD_Pin,   false, "MOTOR_PGD" },
    [DRV_STATUS_DBR_OCP_FLAG]    = { DBR_LSD_OCP_FLAG_GPIO_Port,  DBR_LSD_OCP_FLAG_Pin,  true,  "DBR_OCP" },
    [DRV_STATUS_MOTOR_CHG_OCP]   = { MOTOR_POWER_CHG_OCP_FLAG_GPIO_Port, MOTOR_POWER_CHG_OCP_FLAG_Pin, true, "MOTOR_CHG_OCP" },
    [DRV_STATUS_E_STOP_ON]       = { E_STOP_ON_GPIO_Port,         E_STOP_ON_Pin,         false, "E_STOP_ON" },
    [DRV_STATUS_DIN1]            = { D_IN1_IO_GPIO_Port,          D_IN1_IO_Pin,          false, "DIN1" },
    [DRV_STATUS_DIN2]            = { D_IN2_IO_GPIO_Port,          D_IN2_IO_Pin,          false, "DIN2" },
    [DRV_STATUS_DIN3]            = { D_IN3_IO_GPIO_Port,          D_IN3_IO_Pin,          false, "DIN3" },
};

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void drv_status_init(void)
{
    s_initialized = true;
}

void drv_status_deinit(void)
{
    s_initialized = false;
}

bool drv_status_read(drv_status_signal_t sig)
{
    if (!s_initialized || sig >= DRV_STATUS_NUM) {
        return false;
    }

    const drv_status_pin_t* pin = &s_pins[sig];
    if (!pin->port || !pin->pin) {
        return false;
    }

    bool pin_high = (HAL_GPIO_ReadPin(pin->port, pin->pin) == GPIO_PIN_SET);
    return pin->active_low ? !pin_high : pin_high;
}

const char* drv_status_name(drv_status_signal_t sig)
{
    if (sig >= DRV_STATUS_NUM) {
        return "INVALID";
    }
    return s_pins[sig].name ? s_pins[sig].name : "UNNAMED";
}

uint32_t drv_status_read_all(void)
{
    uint32_t mask = 0;

    for (uint32_t i = 0; i < DRV_STATUS_NUM; i++) {
        if (drv_status_read((drv_status_signal_t)i)) {
            mask |= (1U << i);
        }
    }

    return mask;
}
