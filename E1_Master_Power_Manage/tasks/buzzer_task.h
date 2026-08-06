/**
 * @file    buzzer_task.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   蜂鸣器任务 — 将蜂鸣器注册为 srv_signal 实例
 * @attention
 *
 * 蜂鸣器复用 srv_signal 的状态机与异步命令队列（ON/OFF/BLINK_CODE/BREATHING），
 * write_output 回调把 srv_signal 逻辑值 0-1023 映射到 drv_buzzer_set 占空比 0-100。
 * 当前不被任何对象驱动（默认静音），架构就位，后续由 app 层通过句柄按需控制。
 */

#ifndef BUZZER_TASK_H
#define BUZZER_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "srv_signal.h"

/** @brief 初始化蜂鸣器：drv_buzzer 初始化 + 注册 srv_signal 实例 */
void buzzer_task_init(void);

/**
 * @brief 获取蜂鸣器 srv_signal 句柄
 * @return 句柄指针；未初始化返回 NULL
 * @note 供未来 app 层（如报警策略）显式注入使用，避免按名反查
 */
srv_signal_handle_t* buzzer_task_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_TASK_H */
