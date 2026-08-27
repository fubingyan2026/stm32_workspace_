/**
 * @file    drv_key.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   按键设备驱动（GPIO 读取，KEY1/KEY2）
 * @attention
 *
 * G0 手套按键（CubeMX 配置，输入无上拉/下拉，外部上拉 → 按下为低电平）：
 *   KEY1 — PB9
 *   KEY2 — PB8
 */

#ifndef __DRV_KEY_H
#define __DRV_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 按键通道枚举
 */
typedef enum {
    DRV_KEY_CH_1 = 0, /**< KEY1 — PB9 */
    DRV_KEY_CH_2,     /**< KEY2 — PB8 */
    DRV_KEY_CH_NUM,   /**< 通道总数 */
} drv_key_channel_t;

/**
 * @brief 按键驱动错误码
 */
typedef enum {
    DRV_KEY_OK = 0,
    DRV_KEY_ERROR_INVALID_PARAM,
} drv_key_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化按键驱动（内部引脚表，无需传参） */
void drv_key_init(void);

bool drv_key_is_initialized(void);

/* --- 读取 --- */

/**
 * @brief 读取按键原始引脚电平
 * @param ch 通道号
 * @return 0=低电平, 1=高电平；参数非法返回 0
 */
uint8_t drv_key_read_level(drv_key_channel_t ch);

/**
 * @brief 判断按键是否按下（按下=低电平，外部上拉）
 * @param ch 通道号
 * @return true=按下
 */
bool drv_key_is_pressed(drv_key_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_KEY_H */
