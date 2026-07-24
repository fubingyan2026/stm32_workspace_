/**
 * @file    drv_fan.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-8
 * @brief   风扇驱动实现（PWM + EXTI 脉冲计数，自包含引脚配置）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_fan.h"

#include "main.h"
#include "tim.h"

#include <string.h>

/* Private types -------------------------------------------------------------*/

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t           pwm_ch;
    uint16_t           fg_pin;
    uint8_t            pulse_per_rev;
} drv_fan_hw_t;

typedef struct {
    const drv_fan_hw_t*   hw;
    volatile uint32_t     pulse_count;
    volatile uint32_t     last_reported;
    bool                  initialized;
} drv_fan_ctx_t;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 风扇硬件配置表（基于 CubeMX tim.c + gpio.c）
 *
 * FAN0: PB8(TIM10_CH1) PWM, PE0(EXTI0) FG
 * FAN1: PB9(TIM11_CH1) PWM, PE1(EXTI1) FG
 */
static const drv_fan_hw_t s_fans[] = {
    { &htim10, TIM_CHANNEL_1, FAN0_FG_IO_Pin,  2 },
    { &htim11, TIM_CHANNEL_1, FAN0_FG_IOE1_Pin, 2 },
};
#define FAN_COUNT (sizeof(s_fans) / sizeof(s_fans[0]))

/* Private variables ---------------------------------------------------------*/

static drv_fan_ctx_t s_ctx[FAN_COUNT];

#define HTIM(p) ((TIM_HandleTypeDef*)(p))

/* Private function prototypes -----------------------------------------------*/

static int find_by_fg_pin(uint16_t gpio_pin);

/* Exported functions --------------------------------------------------------*/

void drv_fan_init(void)
{
    for (uint32_t i = 0; i < FAN_COUNT; i++) {
        drv_fan_ctx_t* ctx = &s_ctx[i];
        memset(ctx, 0, sizeof(*ctx));
        ctx->hw = &s_fans[i];

        /* 启动 PWM 输出（TIM 已在 CubeMX MX_TIMx_Init 中配置） */
        HAL_TIM_PWM_Start(HTIM(ctx->hw->htim), ctx->hw->pwm_ch);
        __HAL_TIM_SET_COMPARE(HTIM(ctx->hw->htim), ctx->hw->pwm_ch, 0);

        ctx->initialized = true;
    }
}

void drv_fan_deinit_all(void)
{
    for (uint32_t i = 0; i < FAN_COUNT; i++) {
        if (s_ctx[i].initialized && s_ctx[i].hw) {
            HAL_TIM_PWM_Stop(HTIM(s_ctx[i].hw->htim), s_ctx[i].hw->pwm_ch);
        }
        memset(&s_ctx[i], 0, sizeof(s_ctx[i]));
    }
}

uint32_t drv_fan_get_count(void)
{
    return FAN_COUNT;
}

void drv_fan_set_duty(uint32_t id, uint8_t duty)
{
    if (id >= FAN_COUNT || !s_ctx[id].initialized) {
        return;
    }

    if (duty > 100) {
        duty = 100;
    }

    TIM_HandleTypeDef* htim = HTIM(s_ctx[id].hw->htim);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t cmp = (uint32_t)duty * (arr + 1) / 100;

    if (cmp > arr) {
        cmp = arr;
    }

    __HAL_TIM_SET_COMPARE(htim, cmp, s_ctx[id].hw->pwm_ch);
}

uint32_t drv_fan_get_tach_delta(uint32_t id)
{
    if (id >= FAN_COUNT || !s_ctx[id].initialized) {
        return 0;
    }

    drv_fan_ctx_t* ctx = &s_ctx[id];
    uint32_t curr = ctx->pulse_count;
    uint32_t delta = curr - ctx->last_reported;
    ctx->last_reported = curr;

    return delta;
}

uint8_t drv_fan_get_pulse_per_rev(uint32_t id)
{
    if (id >= FAN_COUNT || !s_ctx[id].initialized) {
        return 0;
    }
    return s_ctx[id].hw->pulse_per_rev;
}

/* ===== HAL EXTI 回调 ===== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    int id = find_by_fg_pin(GPIO_Pin);
    if (id >= 0) {
        s_ctx[id].pulse_count++;
    }
}

/* Private functions ---------------------------------------------------------*/

static int find_by_fg_pin(uint16_t gpio_pin)
{
    for (uint32_t i = 0; i < FAN_COUNT; i++) {
        if (s_ctx[i].initialized && s_ctx[i].hw->fg_pin == gpio_pin) {
            return (int)i;
        }
    }
    return -1;
}
