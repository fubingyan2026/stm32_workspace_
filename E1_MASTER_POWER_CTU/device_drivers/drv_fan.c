/**
 * @file    drv_fan.c
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-09-03
 * @brief   风扇驱动实现（PWM 经 drv_pwm + EXTI 脉冲计数，E1_MASTER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_fan.h"

#include "log.h"
#include "main.h"
#include "drv_pwm.h"
#include "stm32f1xx_hal.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_FAN_LOG_ENABLE 0

#if DRV_FAN_LOG_ENABLE
#define DRV_FAN_LOG_E(...) LOG_E("drv_fan", __VA_ARGS__)
#define DRV_FAN_LOG_W(...) LOG_W("drv_fan", __VA_ARGS__)
#define DRV_FAN_LOG_I(...) LOG_I("drv_fan", __VA_ARGS__)
#define DRV_FAN_LOG_D(...) LOG_D("drv_fan", __VA_ARGS__)
#else
#define DRV_FAN_LOG_E(...) ((void)0)
#define DRV_FAN_LOG_W(...) ((void)0)
#define DRV_FAN_LOG_I(...) ((void)0)
#define DRV_FAN_LOG_D(...) ((void)0)
#endif

/* Private types -------------------------------------------------------------*/

typedef struct {
    drv_pwm_channel_t pwm_ch; /**< drv_pwm 逻辑通道 */
    uint16_t fg_pin;          /**< FG 引脚 (EXTI 编号 = 引脚号) */
    uint8_t  exti_irqn;       /**< EXTI IRQn */
    uint8_t  pulse_per_rev;
} drv_fan_hw_t;

typedef struct {
    const drv_fan_hw_t* hw;
    volatile uint32_t pulse_count;
    volatile uint32_t last_reported;
    bool initialized;
} drv_fan_ctx_t;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 风扇硬件配置表（基于 CubeMX gpio.c: FANx_FG_IO 为 IT_RISING + 上拉）
 *
 * FAN0: PWM=FAN0_PWM_IO(PB9→TIM4_CH4), FG=PA0(EXTI0)
 * FAN1: PWM=FAN1_PWM_IO(PB8→TIM4_CH3), FG=PA1(EXTI1)
 */
static const drv_fan_hw_t s_fans[DRV_FAN_MAX] = {
    { DRV_PWM_CH_FAN0, FAN0_FG_IO_Pin, EXTI0_IRQn, 2 },
    { DRV_PWM_CH_FAN1, FAN1_FG_IO_Pin, EXTI1_IRQn, 2 },
};

/* Private variables ---------------------------------------------------------*/

static drv_fan_ctx_t s_ctx[DRV_FAN_MAX];
static uint8_t s_last_duty[DRV_FAN_MAX]; /**< 上次占空比（变化才打印） */

/* Private function prototypes -----------------------------------------------*/

static int find_by_fg_pin(uint16_t gpio_pin);

/* Exported functions --------------------------------------------------------*/

void drv_fan_init(void)
{
    drv_pwm_init(); /* 幂等：确保 TIM4 组已启动 */

    for (uint32_t i = 0; i < DRV_FAN_MAX; i++) {
        drv_fan_ctx_t* ctx = &s_ctx[i];
        memset(ctx, 0, sizeof(*ctx));
        ctx->hw = &s_fans[i];

        drv_pwm_set_duty(ctx->hw->pwm_ch, 0);
        ctx->initialized = true;

        /* 使能 FG EXTI 中断（CubeMX gpio.c 已将引脚配置为 IT_RISING） */
        HAL_NVIC_SetPriority(ctx->hw->exti_irqn, 2, 0);
        HAL_NVIC_EnableIRQ(ctx->hw->exti_irqn);

        DRV_FAN_LOG_I("风扇%u 初始化完成 (PWM 启动, FG EXTI 使能)", (unsigned)i);
    }
}

void drv_fan_deinit_all(void)
{
    for (uint32_t i = 0; i < DRV_FAN_MAX; i++) {
        if (s_ctx[i].initialized && s_ctx[i].hw) {
            drv_pwm_set_duty(s_ctx[i].hw->pwm_ch, 0);
            HAL_NVIC_DisableIRQ(s_ctx[i].hw->exti_irqn);
        }
        memset(&s_ctx[i], 0, sizeof(s_ctx[i]));
    }

    DRV_FAN_LOG_I("风扇反初始化完成 (%u 路)", (unsigned)DRV_FAN_MAX);
}

uint32_t drv_fan_get_count(void)
{
    return DRV_FAN_MAX;
}

void drv_fan_set_duty(uint32_t id, uint8_t duty)
{
    if (id >= DRV_FAN_MAX || !s_ctx[id].initialized) {
        return;
    }

    if (duty > 100) {
        duty = 100;
    }

    /* 占空比变化才写 PWM（100ms 周期调用，10Hz×2 风扇防刷屏） */
    if (s_last_duty[id] != duty) {
        s_last_duty[id] = duty;
        drv_pwm_set_duty(s_ctx[id].hw->pwm_ch, (uint16_t)duty * 10U);
    }
}

uint32_t drv_fan_get_tach_delta(uint32_t id)
{
    if (id >= DRV_FAN_MAX || !s_ctx[id].initialized) {
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
    if (id >= DRV_FAN_MAX || !s_ctx[id].initialized) {
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
    for (uint32_t i = 0; i < DRV_FAN_MAX; i++) {
        if (s_ctx[i].initialized && s_ctx[i].hw->fg_pin == gpio_pin) {
            return (int)i;
        }
    }
    return -1;
}
