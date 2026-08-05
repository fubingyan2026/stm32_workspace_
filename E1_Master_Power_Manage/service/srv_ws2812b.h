/**
 * @file    srv_ws2812b.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-05
 * @brief   WS2812B 灯带效果服务 — 彗星流光演示
 *
 * service 层提供灯带效果逻辑（色相流转 + 彗星拖尾），直调 drv_ws2812b。
 * 不管理 sw_timer，由 task 层定期调用 srv_ws2812b_step()。
 */

#ifndef __SRV_WS2812B_H
#define __SRV_WS2812B_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 WS2812B 灯带服务（初始化全部通道，点亮前保持全灭）
 * @return 0=成功；非 0=底层驱动错误码
 */
int srv_ws2812b_init(void);

/**
 * @brief 周期步进：刷新彗星流光动画
 * @param elapsed_ms 距上次调用的毫秒数（动画按时间累计，与 tick 周期无关）
 */
void srv_ws2812b_step(uint16_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_WS2812B_H */
