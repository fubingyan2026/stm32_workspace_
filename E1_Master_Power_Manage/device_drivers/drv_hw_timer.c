/**
 * @file    drv_hw_timer.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-1
 * @brief   TIM6 硬件定时器驱动实现（用于 ADC 触发等周期性任务）
 * @attention
 *
 * Copyright (c) 2026 E1_PRO 项目组
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_hw_timer.h"

#include "log.h"
#include "tim.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_HW_TIMER_LOG_ENABLE 1

#if DRV_HW_TIMER_LOG_ENABLE
#define DRV_HW_TIMER_LOG_E(...) LOG_E("drv_hw_timer", __VA_ARGS__)
#define DRV_HW_TIMER_LOG_W(...) LOG_W("drv_hw_timer", __VA_ARGS__)
#define DRV_HW_TIMER_LOG_I(...) LOG_I("drv_hw_timer", __VA_ARGS__)
#define DRV_HW_TIMER_LOG_D(...) LOG_D("drv_hw_timer", __VA_ARGS__)
#else
#define DRV_HW_TIMER_LOG_E(...) ((void)0)
#define DRV_HW_TIMER_LOG_W(...) ((void)0)
#define DRV_HW_TIMER_LOG_I(...) ((void)0)
#define DRV_HW_TIMER_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define DRV_HW_TIMER_CH_NUM ((uint32_t)DRV_HW_TIMER_NUM)

/** @brief TIM 句柄 → 定时器实例号（TIM1_BASE + 0x400 递增） */
#define TIM_INSTANCE_NUM(htim) \
    (unsigned)(((uint32_t)(htim)->Instance - (uint32_t)TIM1_BASE) / 0x400U + 1U)

/* Private variables ---------------------------------------------------------*/

/** @brief 通道 → TIM 句柄映射 */
static TIM_HandleTypeDef* const s_ch_to_htim[DRV_HW_TIMER_CH_NUM] = {
    [DRV_HW_TIMER_ADC] = &htim6,
};

/** @brief 各通道注册的回调 */
static drv_hw_timer_callback_t s_callbacks[DRV_HW_TIMER_CH_NUM] = { NULL };

/** @brief 各通道初始化标志 */
static bool s_initialized[DRV_HW_TIMER_CH_NUM] = { false };

/** @brief 各通道运行标志 */
static volatile bool s_running[DRV_HW_TIMER_CH_NUM] = { false };

/* Private functions ---------------------------------------------------------*/

/**
 * @brief TIM 句柄 → 通道号转换
 */
static drv_hw_timer_ch_t drv_hw_timer_htim_to_ch(TIM_HandleTypeDef* htim)
{
    if (htim == &htim6)
        return DRV_HW_TIMER_ADC;
    return (drv_hw_timer_ch_t)-1;
}

/* Exported functions --------------------------------------------------------*/

void drv_hw_timer_init(void)
{
    for (uint32_t ch = 0; ch < DRV_HW_TIMER_CH_NUM; ch++) {
        s_initialized[ch] = true;
        s_running[ch] = false;
    }

    DRV_HW_TIMER_LOG_I("硬件定时器初始化完成 (%u 通道)", (unsigned)DRV_HW_TIMER_CH_NUM);
}

void drv_hw_timer_deinit_all(void)
{
    for (uint32_t ch = 0; ch < DRV_HW_TIMER_CH_NUM; ch++) {
        if (s_running[ch]) {
            drv_hw_timer_stop((drv_hw_timer_ch_t)ch);
        }
        s_initialized[ch] = false;
        s_callbacks[ch] = NULL;
    }
}

bool drv_hw_timer_is_initialized(drv_hw_timer_ch_t ch)
{
    if (ch >= DRV_HW_TIMER_CH_NUM)
        return false;
    return s_initialized[ch];
}

/* --- 启动 / 停止 --- */

drv_hw_timer_error_t drv_hw_timer_start(drv_hw_timer_ch_t ch)
{
    if (ch >= DRV_HW_TIMER_CH_NUM) {
        return DRV_HW_TIMER_ERROR_INVALID_PARAM;
    }
    if (!s_initialized[ch]) {
        return DRV_HW_TIMER_ERROR_UNINITIALIZED;
    }
    if (s_running[ch]) {
        return DRV_HW_TIMER_OK;
    }

    if (HAL_TIM_Base_Start_IT(s_ch_to_htim[ch]) != HAL_OK) {
        DRV_HW_TIMER_LOG_E("TIM%u 启动失败 (ch=%u)",
            TIM_INSTANCE_NUM(s_ch_to_htim[ch]), (unsigned)ch);
        return DRV_HW_TIMER_ERROR_UNINITIALIZED;
    }

    s_running[ch] = true;
    DRV_HW_TIMER_LOG_I("TIM%u 启动 (ch=%u)",
        TIM_INSTANCE_NUM(s_ch_to_htim[ch]), (unsigned)ch);
    return DRV_HW_TIMER_OK;
}

drv_hw_timer_error_t drv_hw_timer_stop(drv_hw_timer_ch_t ch)
{
    if (ch >= DRV_HW_TIMER_CH_NUM) {
        return DRV_HW_TIMER_ERROR_INVALID_PARAM;
    }
    if (!s_running[ch]) {
        return DRV_HW_TIMER_OK;
    }

    HAL_TIM_Base_Stop_IT(s_ch_to_htim[ch]);

    s_running[ch] = false;
    DRV_HW_TIMER_LOG_D("TIM%u 停止 (ch=%u)",
        TIM_INSTANCE_NUM(s_ch_to_htim[ch]), (unsigned)ch);
    return DRV_HW_TIMER_OK;
}

bool drv_hw_timer_is_running(drv_hw_timer_ch_t ch)
{
    if (ch >= DRV_HW_TIMER_CH_NUM)
        return false;
    return s_running[ch];
}

/* --- 回调注册 --- */

drv_hw_timer_error_t drv_hw_timer_register_callback(drv_hw_timer_ch_t ch,
    drv_hw_timer_callback_t callback)
{
    if (ch >= DRV_HW_TIMER_CH_NUM) {
        return DRV_HW_TIMER_ERROR_INVALID_PARAM;
    }
    if (!s_initialized[ch]) {
        return DRV_HW_TIMER_ERROR_UNINITIALIZED;
    }

    s_callbacks[ch] = callback;
    return DRV_HW_TIMER_OK;
}

/* ===== HAL 回调 ===== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    const drv_hw_timer_ch_t ch = drv_hw_timer_htim_to_ch(htim);
    if (ch == (drv_hw_timer_ch_t)-1) {
        return;
    }

    if (s_callbacks[ch]) {
        s_callbacks[ch](ch);
    }
}
