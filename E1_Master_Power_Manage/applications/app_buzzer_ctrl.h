/**
 * @file    app_buzzer_ctrl.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   应用层 — 蜂鸣器行为切换策略
 * @attention
 *
 * 遵循 app 层规则：只调 service，不拥有 sw_timer，不触碰 HAL/驱动。
 *
 * ## 职责
 * - 通过 srv_signal（蜂鸣器 srv_signal 实例）切换蜂鸣器行为模式
 * - 模式映射：SILENT→OFF、CONTINUOUS→ON、BEEP_CODE→BLINK_CODE、BREATH→BREATHING
 *
 * ## 用法
 * @code
 *   app_buzzer_ctrl_init(buzzer_task_get_handle()); // buzzer_task_init 内注入
 *   app_buzzer_ctrl_set_mode(APP_BUZZER_MODE_BEEP_CODE);
 *   app_buzzer_ctrl_set_beep_code(3, 200, 500);     // 鸣 3 次，间隔 500ms
 * @endcode
 */

#ifndef __APP_BUZZER_CTRL_H
#define __APP_BUZZER_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "srv_signal.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 蜂鸣器行为模式
 */
typedef enum {
    APP_BUZZER_MODE_SILENT = 0, /**< 静音（对应 srv_signal OFF） */
    APP_BUZZER_MODE_CONTINUOUS, /**< 持续鸣响（对应 srv_signal ON） */
    APP_BUZZER_MODE_BEEP_CODE, /**< 编码鸣响：鸣 N 次后停（对应 srv_signal BLINK_CODE） */
    APP_BUZZER_MODE_BREATH, /**< 音量呼吸（对应 srv_signal BREATHING） */
    APP_BUZZER_MODE_COUNT, /**< 模式总数 */
} app_buzzer_mode_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化蜂鸣器控制策略
 * @param buzzer 蜂鸣器 srv_signal 句柄（由 buzzer_task 显式注入）
 * @note 默认静音；重复调用直接返回 OK（幂等）
 */
void app_buzzer_ctrl_init(srv_signal_handle_t* buzzer);

/**
 * @brief 切换蜂鸣器行为模式
 * @param mode 目标模式
 * @note 通过 srv_signal 异步命令下发；模式切换应低频调用（srv_signal 命令队列容量有限）
 */
void app_buzzer_ctrl_set_mode(app_buzzer_mode_t mode);

/**
 * @brief 配置编码鸣响参数（配合 APP_BUZZER_MODE_BEEP_CODE 使用）
 * @param beep_count 鸣响次数（0=无限循环）
 * @param cycle_ms   单次鸣响时间 (ms)，0=保持当前
 * @param wait_ms    鸣响间隔 (ms)，0=保持当前
 * @note 若当前正处于 BEEP_CODE 模式，会立即重发使新参数生效
 */
void app_buzzer_ctrl_set_beep_code(uint16_t beep_count, uint16_t cycle_ms,
    uint16_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BUZZER_CTRL_H */
