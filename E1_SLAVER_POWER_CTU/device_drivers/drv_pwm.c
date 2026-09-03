/**
 * @file    drv_pwm.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   PWM 设备驱动实现（内置通道路由表，多通道通用，E1_SLAVER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_pwm.h"

#include "log.h"
#include "tim.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_PWM_LOG_ENABLE 1

#if DRV_PWM_LOG_ENABLE
#define DRV_PWM_LOG_E(...) LOG_E("drv_pwm", __VA_ARGS__)
#define DRV_PWM_LOG_W(...) LOG_W("drv_pwm", __VA_ARGS__)
#define DRV_PWM_LOG_I(...) LOG_I("drv_pwm", __VA_ARGS__)
#define DRV_PWM_LOG_D(...) LOG_D("drv_pwm", __VA_ARGS__)
#else
#define DRV_PWM_LOG_E(...) ((void)0)
#define DRV_PWM_LOG_W(...) ((void)0)
#define DRV_PWM_LOG_I(...) ((void)0)
#define DRV_PWM_LOG_D(...) ((void)0)
#endif

/* Private types -------------------------------------------------------------*/

/** @brief 通道路由项：逻辑通道 → (定时器句柄, HAL 通道, 定时器时钟) */
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel; /**< HAL 的 TIM_CHANNEL_x */
    uint32_t timer_clk_hz; /**< 定时器输入时钟，用于频率换算 */
    const char* name;
} drv_pwm_route_t;

typedef struct {
    const drv_pwm_route_t* route;
    bool initialized;
    uint16_t duty_permille; /**< 已存千分比，用于频率变更时重算 compare */
} drv_pwm_ctx_t;

/* Private constants ---------------------------------------------------------*/

/** @brief 定时器重配置时目标频率上限保护系数 */
#define DRV_PWM_MAX_FREQ_DIV (2U)

/** @brief TIM4 默认 PWM 频率（组共用：状态灯调光 + 补光灯 DIM） */
#define DRV_PWM_TIM4_DEFAULT_FREQ_HZ (20000U)

/**
 * @brief 内置通道路由表（与 CubeMX tim.c 的 TIM 配置严格对应）
 *
 * TIM4: PSC=0 默认 ARR 由本驱动按 DRV_PWM_TIM4_DEFAULT_FREQ_HZ 重设
 *   - LED_PWM   : PB6 → TIM4_CH1
 *   - FILL_LED_PWM: PB8 → TIM4_CH3
 * TIM4 定时器时钟 = APB1(36MHz) × 2 = 72MHz
 */
static const drv_pwm_route_t s_routes[DRV_PWM_CH_MAX] = {
    [DRV_PWM_CH_LED]  = { &htim4, TIM_CHANNEL_1, 72000000U, "LED" },
    [DRV_PWM_CH_FILL] = { &htim4, TIM_CHANNEL_3, 72000000U, "FILL" },
};

/* Private variables ---------------------------------------------------------*/

static drv_pwm_ctx_t s_ctx[DRV_PWM_CH_MAX];
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static drv_pwm_error_t pwm_group_set_frequency(uint32_t freq_hz);

/* Exported functions --------------------------------------------------------*/

void drv_pwm_init(void)
{
    if (s_initialized) {
        return;
    }

    /* 先整组定频（PSC=0/ARR 重设），再逐个通道启动并置 0% 占空比 */
    if (pwm_group_set_frequency(DRV_PWM_TIM4_DEFAULT_FREQ_HZ) != DRV_PWM_OK) {
        DRV_PWM_LOG_E("PWM 组初始化失败: 频率设置异常");
        return;
    }

    for (uint32_t i = 0; i < DRV_PWM_CH_MAX; i++) {
        drv_pwm_ctx_t* ctx = &s_ctx[i];
        memset(ctx, 0, sizeof(*ctx));
        ctx->route = &s_routes[i];
        ctx->duty_permille = 0;

        /* 启动 PWM 输出（TIM 已在 CubeMX MX_TIMx_Init 中配置）并置 0%（关） */
        HAL_TIM_PWM_Start(ctx->route->htim, ctx->route->channel);
        __HAL_TIM_SET_COMPARE(ctx->route->htim, ctx->route->channel, 0);

        ctx->initialized = true;
    }

    s_initialized = true;
    DRV_PWM_LOG_I("PWM 初始化完成 (%u 路, 组频率 %uHz)", (unsigned)DRV_PWM_CH_MAX,
        (unsigned)DRV_PWM_TIM4_DEFAULT_FREQ_HZ);
}

void drv_pwm_deinit_all(void)
{
    for (uint32_t i = 0; i < DRV_PWM_CH_MAX; i++) {
        if (s_ctx[i].initialized && s_ctx[i].route) {
            HAL_TIM_PWM_Stop(s_ctx[i].route->htim, s_ctx[i].route->channel);
        }
        memset(&s_ctx[i], 0, sizeof(s_ctx[i]));
    }
    s_initialized = false;

    DRV_PWM_LOG_I("PWM 反初始化完成 (%u 路)", (unsigned)DRV_PWM_CH_MAX);
}

drv_pwm_error_t drv_pwm_set_duty(drv_pwm_channel_t ch, uint16_t duty_permille)
{
    if (ch >= DRV_PWM_CH_MAX) {
        return DRV_PWM_ERROR_INVALID_PARAM;
    }

    drv_pwm_ctx_t* ctx = &s_ctx[ch];
    if (!ctx->initialized) {
        return DRV_PWM_ERROR_UNINITIALIZED;
    }

    if (duty_permille > 1000) {
        DRV_PWM_LOG_W("PWM %s 占空比超限被截断: 输入=%u, 上限=1000",
            drv_pwm_channel_name(ch), (unsigned)duty_permille);
        duty_permille = 1000;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(ctx->route->htim);
    /* PWM1 模式: duty=0 → compare=0 恒低(关); duty=1000 → compare=arr+1 恒高(开) */
    uint32_t compare = (uint32_t)duty_permille * (arr + 1U) / 1000U;

    __HAL_TIM_SET_COMPARE(ctx->route->htim, ctx->route->channel, compare);
    ctx->duty_permille = duty_permille;

    return DRV_PWM_OK;
}

drv_pwm_error_t drv_pwm_set_frequency(drv_pwm_channel_t ch, uint32_t freq_hz)
{
    if (ch >= DRV_PWM_CH_MAX) {
        return DRV_PWM_ERROR_INVALID_PARAM;
    }

    drv_pwm_ctx_t* ctx = &s_ctx[ch];
    if (!ctx->initialized) {
        return DRV_PWM_ERROR_UNINITIALIZED;
    }

    if (freq_hz == 0U || freq_hz > ctx->route->timer_clk_hz / DRV_PWM_MAX_FREQ_DIV) {
        DRV_PWM_LOG_W("PWM %s 频率超限被忽略: 输入=%uHz, 合法区间 [1, %uHz]",
            drv_pwm_channel_name(ch), (unsigned)freq_hz,
            (unsigned)(ctx->route->timer_clk_hz / DRV_PWM_MAX_FREQ_DIV));
        return DRV_PWM_ERROR_INVALID_PARAM;
    }

    return pwm_group_set_frequency(freq_hz);
}

uint32_t drv_pwm_get_frequency(drv_pwm_channel_t ch)
{
    if (ch >= DRV_PWM_CH_MAX || !s_ctx[ch].initialized || !s_ctx[ch].route) {
        return 0;
    }

    TIM_HandleTypeDef* htim = s_ctx[ch].route->htim;
    uint32_t psc = htim->Instance->PSC;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

    return s_ctx[ch].route->timer_clk_hz / (psc + 1U) / (arr + 1U);
}

uint16_t drv_pwm_get_duty(drv_pwm_channel_t ch)
{
    if (ch >= DRV_PWM_CH_MAX || !s_ctx[ch].initialized) {
        return 0;
    }
    return s_ctx[ch].duty_permille;
}

const char* drv_pwm_channel_name(drv_pwm_channel_t ch)
{
    if (ch >= DRV_PWM_CH_MAX) {
        return "INVALID";
    }
    return s_routes[ch].name ? s_routes[ch].name : "UNNAMED";
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 整组设置 TIM4 PWM 频率（PSC/ARR 一次写入，各通道占空比保持）
 * @param freq_hz 目标频率
 */
static drv_pwm_error_t pwm_group_set_frequency(uint32_t freq_hz)
{
    if (freq_hz == 0U) {
        return DRV_PWM_ERROR_INVALID_PARAM;
    }

    TIM_HandleTypeDef* htim = &htim4;
    const uint32_t timer_clk = 72000000U;

    /* PSC 固定 0，频率只通过 counter period (ARR) 修改: freq = timer_clk/(ARR+1) */
    uint32_t arr = timer_clk / freq_hz - 1U;
    uint32_t psc = 0;
    if (arr > 0xFFFFU) {
        /* 仅低频时 ARR 超 16 位，才自动引入 PSC 分频保持 ARR ≤ 0xFFFF */
        uint32_t total_div = timer_clk / freq_hz;
        while ((total_div / (psc + 1U)) - 1U > 0xFFFFU) {
            psc++;
        }
        arr = total_div / (psc + 1U) - 1U;
    }

    htim->Init.Prescaler = psc;
    htim->Init.Period = arr;

    __HAL_TIM_SET_AUTORELOAD(htim, arr);    /* 写 ARR（PSC=0 场景即时生效） */
    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_COUNTER(htim, 0);         /* 计数清零，防止新旧周期拼接超长脉冲 */
    htim->Instance->EGR = TIM_EGR_UG;       /* 更新事件：重装 PSC/ARR 影子、清 UIF */

    /* 保持各通道已设定占空比 */
    for (uint32_t i = 0; i < DRV_PWM_CH_MAX; i++) {
        drv_pwm_ctx_t* ctx = &s_ctx[i];
        if (ctx->initialized && ctx->route) {
            uint32_t compare = (uint32_t)ctx->duty_permille * (arr + 1U) / 1000U;
            __HAL_TIM_SET_COMPARE(ctx->route->htim, ctx->route->channel, compare);
        }
    }

    DRV_PWM_LOG_D("PWM 组频率已更改: %uHz (PSC=%u, ARR=%u)",
        (unsigned)freq_hz, (unsigned)psc, (unsigned)arr);

    return DRV_PWM_OK;
}
