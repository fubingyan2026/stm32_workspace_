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

#include <stdbool.h>
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

/**
 * @brief 设置灯带是否自动播放彗星动画
 * @param on true=自动动画（默认），false=手动模式（CAN RGB 控制后停止动画）
 */
void srv_ws2812b_set_auto(bool on);

/**
 * @brief 设置单个 LED 颜色（0x001 控制帧 LED RGB 字段）
 * @param index LED 索引：0-31=通道1(RGB1/SPI1), 32-63=通道2(RGB2/SPI3)
 * @param r,g,b RGB 亮度 (0-255)
 * @return 0=成功；非 0=索引越界（超过对应通道 LED 数）
 * @note  首次调用后进入手动模式（停止彗星动画），设置并刷新对应通道
 */
int srv_ws2812b_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_WS2812B_H */
