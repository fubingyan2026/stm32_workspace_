/**
 * @file    drv_rgb.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-8
 * @brief   WS2812B RGB LED 灯带驱动实现（SPI DMA，句柄自包含）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_rgb.h"

#include "main.h"
#include "spi.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

#define WS2812_BIT_0  (0xE0U)
#define WS2812_BIT_1  (0xFCU)
#define LED_BITS      (24U)
#define RESET_BYTES   (64U)
#define BYTES_PER_LED (LED_BITS)

/** @brief 各路灯带 LED 数量（按实际灯带长度调整，≤ DRV_RGB_MAX_LEDS） */
#define DRV_RGB1_LED_COUNT (1U)
#define DRV_RGB2_LED_COUNT (1U)

/* Private types -------------------------------------------------------------*/

typedef struct { uint8_t r, g, b; } pixel_t;

/** @brief 灯带硬件配置 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    uint16_t           led_count;
} drv_rgb_hw_t;

typedef struct {
    const drv_rgb_hw_t* hw;
    pixel_t             leds[DRV_RGB_MAX_LEDS];
    uint8_t             spi_buf[DRV_RGB_MAX_LEDS * BYTES_PER_LED + RESET_BYTES];
    volatile bool       busy;
    bool                initialized;
} drv_rgb_ctx_t;

/* Private constants ---------------------------------------------------------*/

/** @brief 灯带硬件表（来自 CubeMX spi.c: RGB1→SPI1, RGB2→SPI3） */
static const drv_rgb_hw_t s_hw[DRV_RGB_INST_NUM] = {
    [DRV_RGB_INST_1] = { &hspi1, DRV_RGB1_LED_COUNT },
    [DRV_RGB_INST_2] = { &hspi3, DRV_RGB2_LED_COUNT },
};

/* Private variables ---------------------------------------------------------*/

static drv_rgb_ctx_t s_ctx[DRV_RGB_INST_NUM];

/* Private function prototypes -----------------------------------------------*/

static void drv_rgb_encode(drv_rgb_ctx_t* ctx);
static int  find_by_spi(SPI_HandleTypeDef* hspi);

/* Exported functions --------------------------------------------------------*/

drv_rgb_error_t drv_rgb_init(void)
{
    for (uint32_t inst = 0; inst < DRV_RGB_INST_NUM; inst++) {
        drv_rgb_ctx_t* ctx = &s_ctx[inst];
        memset(ctx, 0, sizeof(*ctx));
        ctx->hw = &s_hw[inst];

        if (ctx->hw->led_count == 0 || ctx->hw->led_count > DRV_RGB_MAX_LEDS) {
            return DRV_RGB_ERROR_INVALID_PARAM;
        }

        /* 全灭帧发送（阻塞等待完成），leds 已被 memset 清零 */
        drv_rgb_encode(ctx);
        ctx->busy = true;
        HAL_SPI_Transmit_DMA(ctx->hw->hspi, ctx->spi_buf,
            ctx->hw->led_count * BYTES_PER_LED + RESET_BYTES);
        while (ctx->busy) { }

        ctx->initialized = true;
    }

    return DRV_RGB_OK;
}

void drv_rgb_deinit_all(void)
{
    for (uint32_t inst = 0; inst < DRV_RGB_INST_NUM; inst++) {
        drv_rgb_ctx_t* ctx = &s_ctx[inst];
        if (!ctx->initialized) {
            continue;
        }
        while (ctx->busy) { }
        memset(ctx->leds, 0, sizeof(ctx->leds));
        drv_rgb_encode(ctx);
        ctx->busy = true;
        HAL_SPI_Transmit_DMA(ctx->hw->hspi, ctx->spi_buf,
            ctx->hw->led_count * BYTES_PER_LED + RESET_BYTES);
        while (ctx->busy) { }
        memset(ctx, 0, sizeof(*ctx));
    }
}

bool drv_rgb_is_initialized(drv_rgb_inst_t inst)
{
    if (inst >= DRV_RGB_INST_NUM) {
        return false;
    }
    return s_ctx[inst].initialized;
}

/* --- 像素控制 --- */

void drv_rgb_set(drv_rgb_inst_t inst, uint16_t pos, uint8_t r, uint8_t g, uint8_t b)
{
    if (inst >= DRV_RGB_INST_NUM || !s_ctx[inst].initialized
        || pos >= s_ctx[inst].hw->led_count) {
        return;
    }

    s_ctx[inst].leds[pos].r = r;
    s_ctx[inst].leds[pos].g = g;
    s_ctx[inst].leds[pos].b = b;
}

void drv_rgb_set_all(drv_rgb_inst_t inst, uint32_t color)
{
    if (inst >= DRV_RGB_INST_NUM || !s_ctx[inst].initialized) {
        return;
    }

    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    for (uint16_t i = 0; i < s_ctx[inst].hw->led_count; i++) {
        s_ctx[inst].leds[i].r = r;
        s_ctx[inst].leds[i].g = g;
        s_ctx[inst].leds[i].b = b;
    }
}

void drv_rgb_clear(drv_rgb_inst_t inst)
{
    drv_rgb_set_all(inst, 0x000000);
}

/* --- 输出 --- */

drv_rgb_error_t drv_rgb_update(drv_rgb_inst_t inst)
{
    if (inst >= DRV_RGB_INST_NUM || !s_ctx[inst].initialized) {
        return DRV_RGB_ERROR_UNINITIALIZED;
    }

    drv_rgb_ctx_t* ctx = &s_ctx[inst];
    if (ctx->busy) {
        return DRV_RGB_ERROR_BUSY;
    }

    drv_rgb_encode(ctx);
    ctx->busy = true;

    uint32_t total = ctx->hw->led_count * BYTES_PER_LED + RESET_BYTES;
    if (HAL_SPI_Transmit_DMA(ctx->hw->hspi, ctx->spi_buf, total) != HAL_OK) {
        ctx->busy = false;
        return DRV_RGB_ERROR_BUSY;
    }

    return DRV_RGB_OK;
}

bool drv_rgb_is_busy(drv_rgb_inst_t inst)
{
    if (inst >= DRV_RGB_INST_NUM) {
        return false;
    }
    return s_ctx[inst].busy;
}

/* ===== HAL 回调 ===== */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    int idx = find_by_spi(hspi);
    if (idx >= 0) {
        s_ctx[idx].busy = false;
    }
}

/* Private functions ---------------------------------------------------------*/

static void drv_rgb_encode(drv_rgb_ctx_t* ctx)
{
    if (!ctx || !ctx->hw) {
        return;
    }

    for (uint16_t i = 0; i < ctx->hw->led_count; i++) {
        uint8_t* p = &ctx->spi_buf[i * BYTES_PER_LED];
        uint8_t grb[3] = { ctx->leds[i].g, ctx->leds[i].r, ctx->leds[i].b };

        for (uint8_t c = 0; c < 3; c++) {
            for (int8_t bit = 7; bit >= 0; bit--) {
                p[c * 8 + (7 - bit)] = (grb[c] & (1 << bit)) ? WS2812_BIT_1 : WS2812_BIT_0;
            }
        }
    }
}

static int find_by_spi(SPI_HandleTypeDef* hspi)
{
    for (uint32_t i = 0; i < DRV_RGB_INST_NUM; i++) {
        if (s_ctx[i].initialized && s_ctx[i].hw->hspi == hspi) {
            return (int)i;
        }
    }
    return -1;
}
