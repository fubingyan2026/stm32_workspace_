/**
 * @file    drv_led.h
 * @brief   运行 LED 设备驱动（GPIO）
 */

#ifndef __DRV_LED_H
#define __DRV_LED_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 LED
 * @note  引脚 PA2 已由 CubeMX MX_GPIO_Init() 配置为推挽输出，此处无需重复初始化
 */
void drv_led_init(void);

/** @brief 点亮 LED */
void drv_led_on(void);

/** @brief 熄灭 LED */
void drv_led_off(void);

/** @brief 翻转 LED 电平 */
void drv_led_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LED_H */
