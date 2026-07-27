/**
 * @file    drv_pwm.c
 * @brief   TIM PWM 输出驱动实现
 *
 * 6 通道 PWM 输出，覆盖 TIM1/2/3/12 的已配置通道。
 * CubeMX 已配置好时基和引脚，本驱动仅做 HAL_TIM_PWM_Start + 占空比设置。
 */

#include "drv_pwm.h"

#include "tim.h"

#include <stdbool.h>

/* 模块日志开关 ----------------------------------------------------------------*/

#define PWM_LOG_ENABLE 0

#if PWM_LOG_ENABLE
#include "log.h"
#define PWM_LOG_E(...) LOG_E("pwm", __VA_ARGS__)
#define PWM_LOG_I(...) LOG_I("pwm", __VA_ARGS__)
#else
#define PWM_LOG_E(...) ((void)0)
#define PWM_LOG_I(...) ((void)0)
#endif

/* Private types -------------------------------------------------------------*/

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t           channel;
    uint32_t           period;
} drv_pwm_hw_t;

/* Private constants ---------------------------------------------------------*/

static const drv_pwm_hw_t s_hw[DRV_PWM_CH_NUM] = {
    [DRV_PWM_CH_TIM1_CH1]  = { .htim = &htim1,  .channel = TIM_CHANNEL_1, .period = 10000U },
    [DRV_PWM_CH_TIM1_CH3]  = { .htim = &htim1,  .channel = TIM_CHANNEL_3, .period = 10000U },
    [DRV_PWM_CH_TIM2_CH1]  = { .htim = &htim2,  .channel = TIM_CHANNEL_1, .period = 10000U },
    [DRV_PWM_CH_TIM2_CH3]  = { .htim = &htim2,  .channel = TIM_CHANNEL_3, .period = 10000U },
    [DRV_PWM_CH_TIM3_CH4]  = { .htim = &htim3,  .channel = TIM_CHANNEL_4, .period = 9999U },
    [DRV_PWM_CH_TIM12_CH2] = { .htim = &htim12, .channel = TIM_CHANNEL_2, .period = 1023U },
};

/* Private variables ---------------------------------------------------------*/

static bool s_init = false;

/* ====== API 实现 ===========================================================*/

drv_pwm_error_t drv_pwm_init(void)
{
    if (s_init) return DRV_PWM_OK;

    for (uint32_t i = 0; i < DRV_PWM_CH_NUM; i++) {
        if (HAL_TIM_PWM_Start(s_hw[i].htim, s_hw[i].channel) != HAL_OK) {
            PWM_LOG_E("PWM_Start ch=%lu failed", (unsigned long)i);
            return DRV_PWM_ERR_INIT;
        }
    }

    s_init = true;
    PWM_LOG_I("PWM init ok, %lu channels", (unsigned long)DRV_PWM_CH_NUM);
    return DRV_PWM_OK;
}

drv_pwm_error_t drv_pwm_set_duty(drv_pwm_channel_t ch, uint32_t pulse)
{
    if (!s_init) return DRV_PWM_ERR_INIT;
    if (ch >= DRV_PWM_CH_NUM) return DRV_PWM_ERR_PARAM;

    __HAL_TIM_SET_COMPARE(s_hw[ch].htim, s_hw[ch].channel, pulse);
    return DRV_PWM_OK;
}

drv_pwm_error_t drv_pwm_set_duty_percent(drv_pwm_channel_t ch, uint32_t percent)
{
    if (!s_init) return DRV_PWM_ERR_INIT;
    if (ch >= DRV_PWM_CH_NUM) return DRV_PWM_ERR_PARAM;
    if (percent > 100) percent = 100;

    uint32_t pulse = (uint32_t)((uint64_t)s_hw[ch].period * percent / 100);
    __HAL_TIM_SET_COMPARE(s_hw[ch].htim, s_hw[ch].channel, pulse);
    return DRV_PWM_OK;
}

uint32_t drv_pwm_get_period(drv_pwm_channel_t ch)
{
    if (ch >= DRV_PWM_CH_NUM) return 0;
    return s_hw[ch].period;
}
