/**
 * @file    drv_gpio.h
 * @brief   板级 GPIO 抽象 — 纯 static inline 封装，零运行时开销
 *
 * 封装 main.h 中 CubeMX 生成的 GPIO 宏，提供按功能分组的 API。
 * 不创建 .c 文件（MX_GPIO_Init 已完成全部初始化）。
 * 所有函数为 static inline，编译器内联为单条 STR 指令。
 */

#ifndef __DRV_GPIO_H
#define __DRV_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* ====== 电源控制 ===========================================================*/

/** @brief 电源输出位掩码 */
typedef enum {
    DRV_GPIO_POWER_24V_1 = (1U << 0), /**< PC14 */
    DRV_GPIO_POWER_24V_2 = (1U << 1), /**< PC13 */
    DRV_GPIO_POWER_5V    = (1U << 2), /**< PC15 */
    DRV_GPIO_POWER_ALL   = (1U << 0) | (1U << 1) | (1U << 2),
} drv_gpio_power_mask_t;

/** @brief 控制电源输出 */
static inline void drv_gpio_power_set(drv_gpio_power_mask_t mask, bool on)
{
    GPIO_PinState state = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (mask & DRV_GPIO_POWER_24V_1)
        HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, state);
    if (mask & DRV_GPIO_POWER_24V_2)
        HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, state);
    if (mask & DRV_GPIO_POWER_5V)
        HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, state);
}

static inline void drv_gpio_power_all_on(void)  { drv_gpio_power_set(DRV_GPIO_POWER_ALL, true); }
static inline void drv_gpio_power_all_off(void) { drv_gpio_power_set(DRV_GPIO_POWER_ALL, false); }

/* ====== SPI 片选 ===========================================================*/

/** @brief SPI 片选通道 */
typedef enum {
    DRV_GPIO_CS_ACC  = 0, /**< 加速度计 PC0 */
    DRV_GPIO_CS_GYRO,     /**< 陀螺仪 PC3 */
    DRV_GPIO_CS_NUM,
} drv_gpio_cs_t;

/** @brief 选中 SPI 设备（低电平有效，自动释放其他 CS） */
static inline void drv_gpio_cs_select(drv_gpio_cs_t cs)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
    switch (cs) {
    case DRV_GPIO_CS_ACC:  HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET); break;
    case DRV_GPIO_CS_GYRO: HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET); break;
    default: break;
    }
}

/** @brief 释放所有 SPI 片选 */
static inline void drv_gpio_cs_deselect_all(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

/* ====== LCD 控制 ===========================================================*/

/** @brief LCD 硬件复位（低电平有效） */
static inline void drv_gpio_lcd_reset(bool assert)
{
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin,
        assert ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/** @brief LCD 片选（低电平有效） */
static inline void drv_gpio_lcd_cs(bool select)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin,
        select ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/** @brief LCD 数据/命令选择（高=数据，低=命令） */
static inline void drv_gpio_lcd_dc(bool is_data)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin,
        is_data ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief LCD 背光控制 */
static inline void drv_gpio_lcd_backlight(bool on)
{
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin,
        on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ====== DCMI 摄像头控制 ====================================================*/

/** @brief DCMI 电源关断（高有效） */
static inline void drv_gpio_dcmi_power_down(bool assert)
{
    HAL_GPIO_WritePin(DCMI_PWDN_GPIO_Port, DCMI_PWDN_Pin,
        assert ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief DCMI 复位（低电平有效） */
static inline void drv_gpio_dcmi_reset(bool assert)
{
    HAL_GPIO_WritePin(DCMI_REST_GPIO_Port, DCMI_REST_Pin,
        assert ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

#ifdef __cplusplus
}
#endif

#endif /* __DRV_GPIO_H */
