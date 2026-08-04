/**
 * @file    drv_adc.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-2
 * @brief   ADC 设备驱动实现（DMA + 通道路由表）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_adc.h"

#include "adc.h"
#include "drv_systick.h"
#include "log.h"
#include "main.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_ADC_LOG_ENABLE 1

#if DRV_ADC_LOG_ENABLE
#define DRV_ADC_LOG_E(...) LOG_E("drv_adc", __VA_ARGS__)
#define DRV_ADC_LOG_W(...) LOG_W("drv_adc", __VA_ARGS__)
#define DRV_ADC_LOG_I(...) LOG_I("drv_adc", __VA_ARGS__)
#define DRV_ADC_LOG_D(...) LOG_D("drv_adc", __VA_ARGS__)
#else
#define DRV_ADC_LOG_E(...) ((void)0)
#define DRV_ADC_LOG_W(...) ((void)0)
#define DRV_ADC_LOG_I(...) ((void)0)
#define DRV_ADC_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define DRV_ADC_DMA_BUF_SIZE (16U) /**< 单 ADC 最大 DMA 槽位 */

/** @brief ADC 触发失败日志限频窗口 (ms)：10ms 周期触发下 Start_DMA 连续失败时防刷屏 */
#define DRV_ADC_ERR_LOG_PERIOD_MS (1000U)

/**
 * @brief 内置通道路由表（与 Core/Src/adc.c MX_ADCx_Init 的 Rank 顺序严格对应）
 *
 * DMA 缓冲区索引 = Rank - 1
 *
 * ADC1 (INST_1): PA0/PA2/PA4/PA6 + TEMPSENSOR + VREFINT + VBAT (7ch)
 * ADC2 (INST_2): PA1/PA3/PA5/PA7/PC2/PC3/PC4/PC5 (8ch)
 * ADC3 (INST_3): PC0/PC1 (2ch) — NTC1/NTC2
 */
static const drv_adc_route_t s_routes[DRV_ADC_CH_MAX] = {
    /* ADC2 (INST_2): 电压采样 — Rank 5/6/7 */
    [DRV_ADC_CH_VIN]          = { DRV_ADC_INST_2, 4 }, /**< PC2, ADC2_IN12 */
    [DRV_ADC_CH_MOTOR_POWER]  = { DRV_ADC_INST_2, 5 }, /**< PC3, ADC2_IN13 */
    [DRV_ADC_CH_AUX_POWER]    = { DRV_ADC_INST_2, 6 }, /**< PC4, ADC2_IN14 */

    /* ADC2 (INST_2): CD4051B 多路选择器输出 — Rank 8 */
    [DRV_ADC_CH_CD4051B]      = { DRV_ADC_INST_2, 7 }, /**< PC5, ADC2_IN15 — A_IN1_IO/2_IO/3_IO 经多路选择 */

    /* ADC1 (INST_1): E-STOP 通道1 冗余 — Rank 1/2/3/4 */
    [DRV_ADC_CH_E_STOP1_ADC1] = { DRV_ADC_INST_1, 0 }, /**< PA0, ADC1_IN0 */
    [DRV_ADC_CH_E_STOP2_ADC1] = { DRV_ADC_INST_1, 1 }, /**< PA2, ADC1_IN2 */
    [DRV_ADC_CH_E_STOP3_ADC1] = { DRV_ADC_INST_1, 2 }, /**< PA4, ADC1_IN4 */
    [DRV_ADC_CH_E_STOP4_ADC1] = { DRV_ADC_INST_1, 3 }, /**< PA6, ADC1_IN6 */

    /* ADC2 (INST_2): E-STOP 通道2 冗余 — Rank 1/2/3/4 */
    [DRV_ADC_CH_E_STOP1_ADC2] = { DRV_ADC_INST_2, 0 }, /**< PA1, ADC2_IN1 */
    [DRV_ADC_CH_E_STOP2_ADC2] = { DRV_ADC_INST_2, 1 }, /**< PA3, ADC2_IN3 */
    [DRV_ADC_CH_E_STOP3_ADC2] = { DRV_ADC_INST_2, 2 }, /**< PA5, ADC2_IN5 */
    [DRV_ADC_CH_E_STOP4_ADC2] = { DRV_ADC_INST_2, 3 }, /**< PA7, ADC2_IN7 */

    /* ADC3 (INST_3): NTC 温度 — Rank 1/2 */
    [DRV_NTC1_ADC] = { DRV_ADC_INST_3, 0 }, /**< PC0, ADC3_IN10 (Rank1) */
    [DRV_NTC2_ADC] = { DRV_ADC_INST_3, 1 }, /**< PC1, ADC3_IN11 (Rank2) */

    /* ADC1 (INST_1): 内部通道 — Rank 5/6/7 */
    [DRV_ADC_CH_TEMPSENSOR]   = { DRV_ADC_INST_1, 4 }, /**< 内部温度传感器, ADC1_IN16 */
    [DRV_ADC_CH_VREFINT]      = { DRV_ADC_INST_1, 5 }, /**< 内部参考电压, ADC1_IN17 */
    [DRV_ADC_CH_VBAT]         = { DRV_ADC_INST_1, 6 }, /**< 备份电池, ADC1_IN18 */
};

/* Private types -------------------------------------------------------------*/

typedef struct {
    ADC_HandleTypeDef*  hadc;
    uint32_t            dma_buf[DRV_ADC_DMA_BUF_SIZE];
    uint8_t             channel_count; /**< CubeMX 配置的 NbrOfConversion */
    volatile bool       busy;
    bool                initialized;
} drv_adc_ctx_t;

/* Private variables ---------------------------------------------------------*/

/** @brief 内置 ADC 实例 → HAL 句柄映射（基于 CubeMX adc.c） */
static ADC_HandleTypeDef* const s_adc_handles[DRV_ADC_INST_NUM] = {
    [DRV_ADC_INST_1] = &hadc1,
    [DRV_ADC_INST_2] = &hadc2,
    [DRV_ADC_INST_3] = &hadc3,
};

static drv_adc_ctx_t     s_ctx[DRV_ADC_INST_NUM];
static drv_adc_callback_t s_callback;

/** @brief 各实例触发失败日志时间戳 (ms)，独立限频 */
static uint32_t s_trig_err_log_ts[DRV_ADC_INST_NUM];

#define HADC(p) ((ADC_HandleTypeDef*)(p))

/* Private function prototypes -----------------------------------------------*/

static drv_adc_inst_t hadc_to_inst(ADC_HandleTypeDef* hadc);

/* Exported functions --------------------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

void drv_adc_init(void)
{
    for (uint32_t i = 0; i < DRV_ADC_INST_NUM; i++) {
        drv_adc_ctx_t* ctx = &s_ctx[i];
        memset(ctx, 0, sizeof(*ctx));
        ctx->hadc = s_adc_handles[i];

        /* 从 CubeMX 配置读取通道数 */
        ctx->channel_count = ctx->hadc->Init.NbrOfConversion;
        ctx->initialized = true;
    }

    DRV_ADC_LOG_I("ADC 初始化完成: 实例数=%u, 通道数 ADC1=%u ADC2=%u ADC3=%u",
        (unsigned)DRV_ADC_INST_NUM,
        (unsigned)s_ctx[0].channel_count,
        (unsigned)s_ctx[1].channel_count,
        (unsigned)s_ctx[2].channel_count);
}

void drv_adc_deinit_all(void)
{
    for (uint32_t i = 0; i < DRV_ADC_INST_NUM; i++) {
        if (s_ctx[i].hadc) {
            HAL_ADC_Stop_DMA(s_ctx[i].hadc);
        }
        memset(&s_ctx[i], 0, sizeof(s_ctx[i]));
    }
}

/* --- 读取（通道路由） --- */

drv_adc_error_t drv_adc_trigger(drv_adc_inst_t inst)
{
    if (inst >= DRV_ADC_INST_NUM) {
        return DRV_ADC_ERROR_NULL_PTR;
    }

    drv_adc_ctx_t* ctx = &s_ctx[inst];
    if (!ctx->initialized) {
        return DRV_ADC_ERROR_UNINITIALIZED;
    }
    if (ctx->busy) {
        return DRV_ADC_ERROR_BUSY;
    }

    ctx->busy = true;

    if (HAL_ADC_Start_DMA(ctx->hadc, ctx->dma_buf, ctx->channel_count) != HAL_OK) {
        ctx->busy = false;

        /* 限频 1s：Start_DMA 每次 10ms 触发都会失败（circular DMA 常驻 BUSY），防刷屏 */
        const uint32_t now_ms = millis();
        if ((uint32_t)(now_ms - s_trig_err_log_ts[inst]) >= DRV_ADC_ERR_LOG_PERIOD_MS) {
            s_trig_err_log_ts[inst] = now_ms;
            DRV_ADC_LOG_E("ADC%u 启动 DMA 采样失败 (inst=%u)",
                (unsigned)inst + 1U, (unsigned)inst);
        }
        return DRV_ADC_ERROR_BUSY;
    }

    return DRV_ADC_OK;
}

void drv_adc_trigger_all(void)
{
    for (uint32_t i = 0; i < DRV_ADC_INST_NUM; i++) {
        if (s_ctx[i].initialized) {
            drv_adc_trigger((drv_adc_inst_t)i);
        }
    }
}

bool drv_adc_is_busy(drv_adc_inst_t inst)
{
    if (inst >= DRV_ADC_INST_NUM) {
        return false;
    }
    return s_ctx[inst].busy;
}

/* --- 读取（通道路由） --- */

uint32_t drv_adc_read_raw(drv_adc_channel_t ch)
{
    if (ch >= DRV_ADC_CH_MAX) {
        return 0;
    }

    drv_adc_route_t route = s_routes[ch];

    /* 通道未映射（inst 或 dma_idx 不合法） */
    if (route.inst >= DRV_ADC_INST_NUM
        || route.dma_idx >= s_ctx[route.inst].channel_count) {
        return 0;
    }

    return s_ctx[route.inst].dma_buf[route.dma_idx];
}

/* --- 回调 --- */

drv_adc_error_t drv_adc_register_callback(drv_adc_callback_t callback)
{
    s_callback = callback;
    DRV_ADC_LOG_I("ADC 采样回调已注册");
    return DRV_ADC_OK;
}

/* ===== HAL 回调 ===== */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    drv_adc_inst_t inst = hadc_to_inst(hadc);
    if (inst >= DRV_ADC_INST_NUM) {
        return;
    }

    s_ctx[inst].busy = false;

    if (s_callback) {
        s_callback(inst);
    }
}

/* Private functions ---------------------------------------------------------*/

static drv_adc_inst_t hadc_to_inst(ADC_HandleTypeDef* hadc)
{
    for (uint32_t i = 0; i < DRV_ADC_INST_NUM; i++) {
        if (s_ctx[i].initialized && s_ctx[i].hadc == hadc) {
            return (drv_adc_inst_t)i;
        }
    }
    return DRV_ADC_INST_NUM;
}
