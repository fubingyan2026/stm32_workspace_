/**
 * @file    srv_ws2812b.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-05
 * @brief   WS2812B 灯带效果服务实现 — 彗星流光演示
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_ws2812b.h"

#include "drv_ws2812b.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_WS2812B_LOG_ENABLE 1

#if SRV_WS2812B_LOG_ENABLE
#define SRV_WS2812B_LOG_E(...) LOG_E("srv_ws2812b", __VA_ARGS__)
#define SRV_WS2812B_LOG_W(...) LOG_W("srv_ws2812b", __VA_ARGS__)
#define SRV_WS2812B_LOG_I(...) LOG_I("srv_ws2812b", __VA_ARGS__)
#define SRV_WS2812B_LOG_D(...) LOG_D("srv_ws2812b", __VA_ARGS__)
#else
#define SRV_WS2812B_LOG_E(...) ((void)0)
#define SRV_WS2812B_LOG_W(...) ((void)0)
#define SRV_WS2812B_LOG_I(...) ((void)0)
#define SRV_WS2812B_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define SRV_WS2812B_HUE_STEP_MS (40U) /**< 色相每 40ms 步进 1 */
#define SRV_WS2812B_CHASE_INTERVAL_MS (30U) /**< 彗头每 30ms 移动一格 */
#define SRV_WS2812B_HEAD_BRIGHT (255U) /**< 彗头亮度，尾部线性衰减 */

/* Private variables ---------------------------------------------------------*/

static uint32_t s_phase_ms; /**< 动画累计相位 (ms) */
static bool s_auto; /**< 自动动画开关（true=彗星动画，false=手动/CAN 控制） */

/* Private function prototypes -----------------------------------------------*/

static void ws2812b_hue_to_rgb(uint8_t hue, uint8_t* r, uint8_t* g, uint8_t* b);

/* Exported functions --------------------------------------------------------*/

int srv_ws2812b_init(void)
{
    int err = drv_ws2812b_init();
    if (err != DRV_WS2812B_OK) {
        SRV_WS2812B_LOG_E("WS2812B 灯带初始化失败 (err=%d)", err);
        return err;
    }

    s_auto = true;

    SRV_WS2812B_LOG_I("WS2812B 灯带初始化完成 (通道1=%u, 通道2=%u)",
        (unsigned)drv_ws2812b_get_led_count(DRV_WS2812B_INST_1),
        (unsigned)drv_ws2812b_get_led_count(DRV_WS2812B_INST_2));
    return 0;
}

void srv_ws2812b_step(uint16_t elapsed_ms)
{
    /* 手动模式（CAN RGB 控制）下停止彗星动画，避免覆盖主机设置的灯 */
    if (!s_auto) {
        return;
    }

    s_phase_ms += elapsed_ms;

    for (uint8_t inst = 0; inst < DRV_WS2812B_INST_NUM; inst++) {
        const uint16_t count = drv_ws2812b_get_led_count((drv_ws2812b_inst_t)inst);

        /* DMA 发送未完成则跳过本帧，避免覆盖正在传输的缓冲 */
        if (count == 0U || drv_ws2812b_is_busy((drv_ws2812b_inst_t)inst)) {
            continue;
        }

        /* 彗头位置: 每 30ms 移动一格 */
        const uint16_t head = (s_phase_ms / SRV_WS2812B_CHASE_INTERVAL_MS) % count;

        for (uint16_t i = 0; i < count; i++) {
            uint8_t r, g, b;
            uint8_t hue = (uint8_t)(s_phase_ms / SRV_WS2812B_HUE_STEP_MS + i * (256U / count));
            ws2812b_hue_to_rgb(hue, &r, &g, &b);

            /* 与彗头的环向距离(0=头部)，亮度线性衰减成拖尾 */
            uint16_t dist = (i + count - head) % count;
            uint8_t bright = (uint8_t)((uint32_t)SRV_WS2812B_HEAD_BRIGHT * (count - dist) / count);

            drv_ws2812b_set((drv_ws2812b_inst_t)inst, i,
                (uint8_t)((uint16_t)r * bright / 255U),
                (uint8_t)((uint16_t)g * bright / 255U),
                (uint8_t)((uint16_t)b * bright / 255U));
        }
        drv_ws2812b_update((drv_ws2812b_inst_t)inst);
    }
}

void srv_ws2812b_set_auto(bool on)
{
    s_auto = on;
}

int srv_ws2812b_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    /* 索引 0-31 → 通道1(RGB1/SPI1)，32-63 → 通道2(RGB2/SPI3) */
    const drv_ws2812b_inst_t inst = (index >= 32U) ? DRV_WS2812B_INST_2 : DRV_WS2812B_INST_1;
    const uint16_t pos = index & 0x1FU;

    if (pos >= drv_ws2812b_get_led_count(inst)) {
        SRV_WS2812B_LOG_W("RGB 控制索引越界: idx=%u (通道%u 灯数=%u)",
            (unsigned)index, (unsigned)inst, (unsigned)drv_ws2812b_get_led_count(inst));
        return -1;
    }

    /* 进入手动模式：停止彗星动画，避免覆盖 CAN 控制 */
    s_auto = false;

    drv_ws2812b_set(inst, pos, r, g, b);
    drv_ws2812b_update(inst);

    SRV_WS2812B_LOG_D("RGB 控制: idx=%u -> 通道%u pos=%u (#%02X%02X%02X)",
        (unsigned)index, (unsigned)inst, (unsigned)pos,
        (unsigned)r, (unsigned)g, (unsigned)b);
    return 0;
}

/* Private functions ---------------------------------------------------------*/

/** @brief 色相(0~255) → RGB888，六段整数色环 */
static void ws2812b_hue_to_rgb(uint8_t hue, uint8_t* r, uint8_t* g, uint8_t* b)
{
    uint8_t region = hue / 43U;
    uint8_t x = (uint8_t)((hue % 43U) * 6U);

    switch (region) {
    case 0:
        *r = 255U;
        *g = x;
        *b = 0U;
        break;
    case 1:
        *r = 255U - x;
        *g = 255U;
        *b = 0U;
        break;
    case 2:
        *r = 0U;
        *g = 255U;
        *b = x;
        break;
    case 3:
        *r = 0U;
        *g = 255U - x;
        *b = 255U;
        break;
    case 4:
        *r = x;
        *g = 0U;
        *b = 255U;
        break;
    default:
        *r = 255U;
        *g = 0U;
        *b = 255U - x;
        break;
    }
}
