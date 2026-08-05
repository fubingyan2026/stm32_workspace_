/**
 * @file    ws2812_task.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-05
 * @brief   WS2812B 灯带任务声明
 */

#ifndef __WS2812_TASK_H
#define __WS2812_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化 WS2812B 灯带任务（初始化服务并启动 sw_timer 周期刷新） */
void ws2812_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_TASK_H */
