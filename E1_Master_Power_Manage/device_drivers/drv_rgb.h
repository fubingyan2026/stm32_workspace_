/**
 * @file    drv_rgb.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-8
 * @brief   WS2812B RGB LED 灯带驱动（SPI DMA，句柄自包含）
 * @attention
 *
 * CubeMX 配置 SPI1(RGB1) / SPI3(RGB2)。句柄与 LED 数量内置在 drv_rgb.c 中。
 * 仅负责 SPI DMA 位流编码和输出，不包含灯效逻辑。
 */

#ifndef __DRV_RGB_H
#define __DRV_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    DRV_RGB_INST_1 = 0, /**< 灯带通道 1 (SPI1) */
    DRV_RGB_INST_2,     /**< 灯带通道 2 (SPI3) */
    DRV_RGB_INST_NUM,   /**< 实例总数 */
} drv_rgb_inst_t;

typedef enum {
    DRV_RGB_OK = 0,
    DRV_RGB_ERROR_NULL_PTR,
    DRV_RGB_ERROR_INVALID_PARAM,
    DRV_RGB_ERROR_BUSY,
    DRV_RGB_ERROR_UNINITIALIZED,
} drv_rgb_error_t;

/* Exported constants --------------------------------------------------------*/

#define DRV_RGB_MAX_INST  ((uint32_t)DRV_RGB_INST_NUM)
#define DRV_RGB_MAX_LEDS  (64U) /**< 单路最大 LED 数 */

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化全部灯带（内部句柄表，无需传参） */
drv_rgb_error_t drv_rgb_init(void);

/** @brief 反初始化全部灯带 */
void drv_rgb_deinit_all(void);

bool drv_rgb_is_initialized(drv_rgb_inst_t inst);

/* --- 像素控制 --- */

void drv_rgb_set(drv_rgb_inst_t inst, uint16_t pos, uint8_t r, uint8_t g, uint8_t b);
void drv_rgb_set_all(drv_rgb_inst_t inst, uint32_t color);
void drv_rgb_clear(drv_rgb_inst_t inst);

/* --- 输出 --- */

/** @brief SPI DMA 发送（非阻塞），忙时返回 DRV_RGB_ERROR_BUSY */
drv_rgb_error_t drv_rgb_update(drv_rgb_inst_t inst);
bool drv_rgb_is_busy(drv_rgb_inst_t inst);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_RGB_H */
