/**
 * @file    drv_ws2812b.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-8
 * @brief   WS2812B RGB LED 灯带驱动实现（SPI DMA，句柄自包含）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_ws2812b.h"

#include "log.h"
#include "main.h"
#include "spi.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_WS2812B_LOG_ENABLE 1

#if DRV_WS2812B_LOG_ENABLE
#define DRV_WS2812B_LOG_E(...) LOG_E("drv_ws2812b", __VA_ARGS__)
#define DRV_WS2812B_LOG_W(...) LOG_W("drv_ws2812b", __VA_ARGS__)
#define DRV_WS2812B_LOG_I(...) LOG_I("drv_ws2812b", __VA_ARGS__)
#define DRV_WS2812B_LOG_D(...) LOG_D("drv_ws2812b", __VA_ARGS__)
#else
#define DRV_WS2812B_LOG_E(...) ((void)0)
#define DRV_WS2812B_LOG_W(...) ((void)0)
#define DRV_WS2812B_LOG_I(...) ((void)0)
#define DRV_WS2812B_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/* 5.25MHz SPI(每位置190ns): 0xC0=2高(381ns)"0", 0xF0=4高(762ns)"1"，
   均落在 WS2812B 判定窗口(200~500ns / 550~850ns)正中。原 0xE0(571ns)超出"0"上限
   会被误读为"1"，导致全亮白。 */
#define WS2812_BIT_0 (0xC0U)
#define WS2812_BIT_1 (0xF0U)
#define LED_BITS (24U)
#define RESET_BYTES (64U)
#define BYTES_PER_LED (LED_BITS)

/** @brief 各路灯带 LED 数量（按实际灯带长度调整，≤ DRV_WS2812B_MAX_LEDS） */
#define DRV_WS2812B1_LED_COUNT (24U)
#define DRV_WS2812B2_LED_COUNT (24U)

/* Private types -------------------------------------------------------------*/

typedef struct {
    uint8_t r, g, b;
} pixel_t;

/** @brief 灯带硬件配置 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    uint16_t led_count;
} drv_ws2812b_hw_t;

typedef struct {
    const drv_ws2812b_hw_t* hw;
    pixel_t leds[DRV_WS2812B_MAX_LEDS];
    uint8_t spi_buf[DRV_WS2812B_MAX_LEDS * BYTES_PER_LED + RESET_BYTES];
    volatile bool busy;
    bool initialized;
} drv_ws2812b_ctx_t;

/* Private constants ---------------------------------------------------------*/

/** @brief 灯带硬件表（来自 CubeMX spi.c: RGB1→SPI1, RGB2→SPI3） */
static const drv_ws2812b_hw_t s_hw[DRV_WS2812B_INST_NUM] = {
    [DRV_WS2812B_INST_1] = { &hspi1, DRV_WS2812B1_LED_COUNT },
    [DRV_WS2812B_INST_2] = { &hspi3, DRV_WS2812B2_LED_COUNT },
};

/* Private variables ---------------------------------------------------------*/

static drv_ws2812b_ctx_t s_ctx[DRV_WS2812B_INST_NUM];

/** @brief 编码查找表: 颜色字节值 → 8 个 SPI 位型字节 (MSB 在前) */
static uint8_t s_encode_lut[256][8];

/* Private function prototypes -----------------------------------------------*/

static void drv_ws2812b_encode(drv_ws2812b_ctx_t* ctx);
static void drv_ws2812b_encode_lut_init(void);
static int find_by_spi(SPI_HandleTypeDef* hspi);

/* Exported functions --------------------------------------------------------*/

drv_ws2812b_error_t drv_ws2812b_init(void)
{
    drv_ws2812b_encode_lut_init();

    for (uint32_t inst = 0; inst < DRV_WS2812B_INST_NUM; inst++) {
        drv_ws2812b_ctx_t* ctx = &s_ctx[inst];
        memset(ctx, 0, sizeof(*ctx));
        ctx->hw = &s_hw[inst];

        if (ctx->hw->led_count == 0 || ctx->hw->led_count > DRV_WS2812B_MAX_LEDS) {
            DRV_WS2812B_LOG_E("RGB%u LED 数量非法: %u (上限%u)",
                (unsigned)inst + 1U, (unsigned)ctx->hw->led_count, (unsigned)DRV_WS2812B_MAX_LEDS);
            return DRV_WS2812B_ERROR_INVALID_PARAM;
        }

        /* 全灭帧发送（阻塞等待完成），leds 已被 memset 清零 */
        drv_ws2812b_encode(ctx);
        ctx->busy = true;
        HAL_SPI_Transmit_DMA(ctx->hw->hspi, ctx->spi_buf,
            ctx->hw->led_count * BYTES_PER_LED + RESET_BYTES);
        while (ctx->busy) { }

        ctx->initialized = true;
        DRV_WS2812B_LOG_I("RGB%u 初始化完成 (led=%u, 全灭帧已发送)",
            (unsigned)inst + 1U, (unsigned)ctx->hw->led_count);
    }

    return DRV_WS2812B_OK;
}

void drv_ws2812b_deinit_all(void)
{
    for (uint32_t inst = 0; inst < DRV_WS2812B_INST_NUM; inst++) {
        drv_ws2812b_ctx_t* ctx = &s_ctx[inst];
        if (!ctx->initialized) {
            continue;
        }
        while (ctx->busy) { }
        memset(ctx->leds, 0, sizeof(ctx->leds));
        drv_ws2812b_encode(ctx);
        ctx->busy = true;
        HAL_SPI_Transmit_DMA(ctx->hw->hspi, ctx->spi_buf,
            ctx->hw->led_count * BYTES_PER_LED + RESET_BYTES);
        while (ctx->busy) {
            HAL_Delay(10);
        }
        memset(ctx, 0, sizeof(*ctx));
    }
}

bool drv_ws2812b_is_initialized(drv_ws2812b_inst_t inst)
{
    if (inst >= DRV_WS2812B_INST_NUM) {
        return false;
    }
    return s_ctx[inst].initialized;
}

uint16_t drv_ws2812b_get_led_count(drv_ws2812b_inst_t inst)
{
    if (inst >= DRV_WS2812B_INST_NUM || !s_ctx[inst].initialized) {
        return 0;
    }
    return s_ctx[inst].hw->led_count;
}

/* --- 像素控制 --- */

void drv_ws2812b_set(drv_ws2812b_inst_t inst, uint16_t pos, uint8_t r, uint8_t g, uint8_t b)
{
    if (inst >= DRV_WS2812B_INST_NUM || !s_ctx[inst].initialized
        || pos >= s_ctx[inst].hw->led_count) {
        return;
    }

    s_ctx[inst].leds[pos].r = r;
    s_ctx[inst].leds[pos].g = g;
    s_ctx[inst].leds[pos].b = b;
}

void drv_ws2812b_set_all(drv_ws2812b_inst_t inst, uint32_t color)
{
    if (inst >= DRV_WS2812B_INST_NUM || !s_ctx[inst].initialized) {
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

void drv_ws2812b_clear(drv_ws2812b_inst_t inst)
{
    drv_ws2812b_set_all(inst, 0x000000);
}

/* --- 输出 --- */

drv_ws2812b_error_t drv_ws2812b_update(drv_ws2812b_inst_t inst)
{
    if (inst >= DRV_WS2812B_INST_NUM || !s_ctx[inst].initialized) {
        return DRV_WS2812B_ERROR_UNINITIALIZED;
    }

    drv_ws2812b_ctx_t* ctx = &s_ctx[inst];
    if (ctx->busy) {
        return DRV_WS2812B_ERROR_BUSY;
    }

    drv_ws2812b_encode(ctx);
    ctx->busy = true;

    uint32_t total = ctx->hw->led_count * BYTES_PER_LED + RESET_BYTES;
    if (HAL_SPI_Transmit_DMA(ctx->hw->hspi, ctx->spi_buf, total) != HAL_OK) {
        ctx->busy = false;
        DRV_WS2812B_LOG_W("RGB%u SPI DMA 发送失败 (busy 已复位)", (unsigned)inst + 1U);
        return DRV_WS2812B_ERROR_BUSY;
    }

    return DRV_WS2812B_OK;
}

bool drv_ws2812b_is_busy(drv_ws2812b_inst_t inst)
{
    if (inst >= DRV_WS2812B_INST_NUM) {
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

/** @brief 构建编码查找表（init 时调用一次，占 2KB RAM） */
static void drv_ws2812b_encode_lut_init(void)
{
    for (uint16_t v = 0; v < 256; v++) {
        for (uint8_t j = 0; j < 8; j++) {
            /* 输出第 0 字节对应最高位，与 SPI MSB 先发一致 */
            s_encode_lut[v][j] = (v & (0x80U >> j)) ? WS2812_BIT_1 : WS2812_BIT_0;
        }
    }
}

static void drv_ws2812b_encode(drv_ws2812b_ctx_t* ctx)
{
    if (!ctx || !ctx->hw) {
        return;
    }

    for (uint16_t i = 0; i < ctx->hw->led_count; i++) {
        const pixel_t* led = &ctx->leds[i];
        uint8_t* p = &ctx->spi_buf[i * BYTES_PER_LED];

        /* 查表整块复制 8 字节，替代原来的逐位判断（每 LED 24 次分支 → 3 次复制） */
        memcpy(p, s_encode_lut[led->g], 8U);
        memcpy(p + 8U, s_encode_lut[led->r], 8U);
        memcpy(p + 16U, s_encode_lut[led->b], 8U);
    }
}

static int find_by_spi(SPI_HandleTypeDef* hspi)
{
    for (uint32_t i = 0; i < DRV_WS2812B_INST_NUM; i++) {
        /* 不依赖 initialized：init 阶段发全灭帧时该标志尚未置位，
           否则回调找不到实例、busy 永不解除，会死锁在 while(ctx->busy) */
        if (s_ctx[i].hw->hspi == hspi) {
            return (int)i;
        }
    }
    return -1;
}
