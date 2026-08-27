/**
 * @file    drv_key.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   按键设备驱动实现（GPIO 读取，KEY1/KEY2）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_key.h"

#include "log.h"
#include "main.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_KEY_LOG_ENABLE 1

#if DRV_KEY_LOG_ENABLE
#define DRV_KEY_LOG_E(...) LOG_E("drv_key", __VA_ARGS__)
#define DRV_KEY_LOG_W(...) LOG_W("drv_key", __VA_ARGS__)
#define DRV_KEY_LOG_I(...) LOG_I("drv_key", __VA_ARGS__)
#define DRV_KEY_LOG_D(...) LOG_D("drv_key", __VA_ARGS__)
#else
#define DRV_KEY_LOG_E(...) ((void)0)
#define DRV_KEY_LOG_W(...) ((void)0)
#define DRV_KEY_LOG_I(...) ((void)0)
#define DRV_KEY_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 按键按下判定：低电平有效（外部上拉，按下接地） */
#define DRV_KEY_PRESS_LEVEL (0U)

/* Private types -------------------------------------------------------------*/

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} drv_key_hw_t;

/* Private constants ---------------------------------------------------------*/

static const drv_key_hw_t s_key_hw[DRV_KEY_CH_NUM] = {
    [DRV_KEY_CH_1] = { .port = KEY1_GPIO_Port, .pin = KEY1_Pin },
    [DRV_KEY_CH_2] = { .port = KEY2_GPIO_Port, .pin = KEY2_Pin },
};

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void drv_key_init(void)
{
    s_initialized = true;
    DRV_KEY_LOG_I("按键驱动初始化完成 (KEY1=PB9, KEY2=PB8)");
}

bool drv_key_is_initialized(void)
{
    return s_initialized;
}

uint8_t drv_key_read_level(drv_key_channel_t ch)
{
    if (ch >= DRV_KEY_CH_NUM || !s_initialized) {
        return 0;
    }
    return (uint8_t)HAL_GPIO_ReadPin(s_key_hw[ch].port, s_key_hw[ch].pin);
}

bool drv_key_is_pressed(drv_key_channel_t ch)
{
    if (ch >= DRV_KEY_CH_NUM || !s_initialized) {
        return false;
    }
    return (uint8_t)HAL_GPIO_ReadPin(s_key_hw[ch].port, s_key_hw[ch].pin)
        == DRV_KEY_PRESS_LEVEL;
}
