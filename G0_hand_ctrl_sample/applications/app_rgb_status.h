/**
 * @file    app_rgb_status.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   应用层 — RGB 三色状态指示灯管理（蓝/绿/红）
 * @attention
 *
 * 每个 LED 对应一个 static srv_signal_handle_t 实例，输出经 drv_pwm：
 *   Blue  — TIM1_CH1 (PA8)
 *   Green — TIM4_CH1 (PB6)
 *   Red   — TIM4_CH2 (PB7)
 */

#ifndef APP_RGB_STATUS_H
#define APP_RGB_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "srv_signal.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief RGB LED 通道枚举
 */
typedef enum {
    APP_RGB_CH_BLUE = 0, /**< 蓝色 — TIM1_CH1 (PA8) */
    APP_RGB_CH_GREEN,    /**< 绿色 — TIM4_CH1 (PB6) */
    APP_RGB_CH_RED,      /**< 红色 — TIM4_CH2 (PB7) */
    APP_RGB_CH_NUM,      /**< 通道总数 */
} app_rgb_channel_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/**
 * @brief 初始化 RGB 状态指示灯
 * @note  内部完成 drv_pwm_init + srv_signal_init + 注册三路 LED 静态实例
 */
void app_rgb_status_init(void);

/** @brief 状态刷新步进（由 task 层 sw_timer 周期调用，驱动全部 LED FSM） */
void app_rgb_status_step(void);

/* --- 实例访问 --- */

/**
 * @brief 获取指定 LED 的 srv_signal 句柄
 * @param ch LED 通道
 * @return 句柄指针；参数非法返回 NULL
 */
srv_signal_handle_t* app_rgb_status_get_led(app_rgb_channel_t ch);

/* --- 外部状态控制（供 CAN/按键/业务层驱动 LED 状态机） --- */

/**
 * @brief 设置指定 LED 的工作状态
 * @param ch    LED 通道
 * @param state 目标状态（OFF/ON/BLINK_CODE/BREATHING）
 * @return true 成功；false 参数非法或未初始化
 */
bool app_rgb_status_set_state(app_rgb_channel_t ch, srv_signal_state_t state);

/**
 * @brief 设置指定 LED 闪烁（编码闪烁）并切换状态
 * @param ch       LED 通道
 * @param cycle_ms 闪烁间隔(ms)，0 表示使用当前值
 * @param wait_ms  等待间隔(ms)，0 表示使用当前值
 * @param counts   闪烁次数（0=无限循环）
 * @return true 成功；false 参数非法或未初始化
 * @note  先下发闪烁参数再切换 BLINK_CODE 状态（FIFO 顺序保证）
 */
bool app_rgb_status_set_blink(app_rgb_channel_t ch, uint16_t cycle_ms,
    uint16_t wait_ms, uint16_t counts);

/**
 * @brief 设置呼吸并强制立即开始（先重置呼吸计时/相位，再切 BREATHING）
 * @param ch       LED 通道
 * @param cycle_ms 呼吸周期(ms)，0 表示使用当前值
 * @return true 成功；false 参数非法或未初始化
 */
bool app_rgb_status_set_breath(app_rgb_channel_t ch, uint16_t cycle_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_RGB_STATUS_H */
