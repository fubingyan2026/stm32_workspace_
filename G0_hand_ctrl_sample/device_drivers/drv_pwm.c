/**
 * @file    drv_pwm.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   PWM 设备驱动实现（TIM1/TIM4 输出比较通道）
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_pwm.h"

#include "log.h"
#include "tim.h"

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

/* Private constants ---------------------------------------------------------*/

/** @brief 占空比上限（对齐 CubeMX 周期 1024-1） */
#define DRV_PWM_DUTY_MAX (1023U)

/* Private variables ---------------------------------------------------------*/

/** @brief 通道 → (TIM 句柄, 通道) 映射 */
static TIM_HandleTypeDef* const s_ch_to_htim[DRV_PWM_CH_NUM] = {
    [DRV_PWM_TIM1_CH1] = &htim1,
    [DRV_PWM_TIM4_CH1] = &htim4,
    [DRV_PWM_TIM4_CH2] = &htim4,
};

static const uint32_t s_ch_to_hal[DRV_PWM_CH_NUM] = {
    [DRV_PWM_TIM1_CH1] = TIM_CHANNEL_1,
    [DRV_PWM_TIM4_CH1] = TIM_CHANNEL_1,
    [DRV_PWM_TIM4_CH2] = TIM_CHANNEL_2,
};

/** @brief 各通道初始化标志 */
static bool s_initialized[DRV_PWM_CH_NUM] = { false };

/* Exported functions --------------------------------------------------------*/

drv_pwm_error_t drv_pwm_init(void)
{
    for (uint32_t ch = 0; ch < DRV_PWM_CH_NUM; ch++) {
        if (HAL_TIM_PWM_Start(s_ch_to_htim[ch], s_ch_to_hal[ch]) != HAL_OK) {
            DRV_PWM_LOG_E("ch%u PWM start failed", (unsigned)ch);
            s_initialized[ch] = false;
            continue;
        }
        s_initialized[ch] = true;
    }

    DRV_PWM_LOG_I("PWM 初始化完成: %u/%u 通道启动", (unsigned)DRV_PWM_CH_NUM, (unsigned)DRV_PWM_CH_NUM);
    return DRV_PWM_OK;
}

void drv_pwm_deinit_all(void)
{
    for (uint32_t ch = 0; ch < DRV_PWM_CH_NUM; ch++) {
        if (!s_initialized[ch]) {
            continue;
        }
        HAL_TIM_PWM_Stop(s_ch_to_htim[ch], s_ch_to_hal[ch]);
        s_initialized[ch] = false;
    }
}

bool drv_pwm_is_initialized(drv_pwm_channel_t ch)
{
    if (ch >= DRV_PWM_CH_NUM) {
        return false;
    }
    return s_initialized[ch];
}

drv_pwm_error_t drv_pwm_set_duty(drv_pwm_channel_t ch, uint16_t duty)
{
    if (ch >= DRV_PWM_CH_NUM) {
        return DRV_PWM_ERROR_INVALID_PARAM;
    }
    if (!s_initialized[ch]) {
        return DRV_PWM_ERROR_UNINITIALIZED;
    }
    if (duty > DRV_PWM_DUTY_MAX) {
        duty = DRV_PWM_DUTY_MAX;
    }

    __HAL_TIM_SET_COMPARE(s_ch_to_htim[ch], s_ch_to_hal[ch], duty);
    return DRV_PWM_OK;
}
