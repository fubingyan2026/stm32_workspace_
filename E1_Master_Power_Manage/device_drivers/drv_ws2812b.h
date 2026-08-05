/**
 * @file    drv_ws2812b.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-8
 * @brief   WS2812B RGB LED 灯带驱动（SPI DMA，句柄自包含）
 * @attention
 *
 * CubeMX 配置 SPI1(RGB1) / SPI3(RGB2)。句柄与 LED 数量内置在 drv_ws2812b.c 中。
 * 仅负责 SPI DMA 位流编码和输出，不包含灯效逻辑。
 */

#ifndef __DRV_WS2812B_H
#define __DRV_WS2812B_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

typedef enum {
    DRV_WS2812B_INST_1 = 0, /**< 灯带通道 1 (SPI1) */
    DRV_WS2812B_INST_2, /**< 灯带通道 2 (SPI3) */
    DRV_WS2812B_INST_NUM, /**< 实例总数 */
} drv_ws2812b_inst_t;

typedef enum {
    DRV_WS2812B_OK = 0,
    DRV_WS2812B_ERROR_NULL_PTR,
    DRV_WS2812B_ERROR_INVALID_PARAM,
    DRV_WS2812B_ERROR_BUSY,
    DRV_WS2812B_ERROR_UNINITIALIZED,
} drv_ws2812b_error_t;

/* Exported constants --------------------------------------------------------*/

#define DRV_WS2812B_MAX_INST ((uint32_t)DRV_WS2812B_INST_NUM)
#define DRV_WS2812B_MAX_LEDS (32U) /**< 单路最大 LED 数 */

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化全部灯带（内部句柄表，无需传参） */
drv_ws2812b_error_t drv_ws2812b_init(void);

/** @brief 反初始化全部灯带 */
void drv_ws2812b_deinit_all(void);

bool drv_ws2812b_is_initialized(drv_ws2812b_inst_t inst);

/** @brief 获取指定灯带通道的 LED 数量（非法实例/未初始化返回 0） */
uint16_t drv_ws2812b_get_led_count(drv_ws2812b_inst_t inst);

/* --- 像素控制 --- */

void drv_ws2812b_set(drv_ws2812b_inst_t inst, uint16_t pos, uint8_t r, uint8_t g, uint8_t b);
void drv_ws2812b_set_all(drv_ws2812b_inst_t inst, uint32_t color);
void drv_ws2812b_clear(drv_ws2812b_inst_t inst);

/* --- 输出 --- */

/** @brief SPI DMA 发送（非阻塞），忙时返回 DRV_WS2812B_ERROR_BUSY */
drv_ws2812b_error_t drv_ws2812b_update(drv_ws2812b_inst_t inst);
bool drv_ws2812b_is_busy(drv_ws2812b_inst_t inst);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_WS2812B_H */
