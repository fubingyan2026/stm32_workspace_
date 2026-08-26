/**
 * @file    srv_mz_sensor.h
 * @brief   Mz 扭矩传感器服务（经典 CAN 2.0A，1 Mbps，响应门控高频轮询）
 *
 * 协议按 docs/mz_sesor.md：
 *   - 查询：CAN ID 0x510，DLC 7，data = {04,00,00,00,00,00,00}（读功能码 04 + 数据地址 0x0000）
 *   - 应答：CAN ID 0x410，data = {04,00,00,xx,xx,xx,xx}，Byte3~6 = 大端 IEEE-754 float32，单位 N·m
 *   - 标零：CAN ID 0x510，DLC 7，data = {10,46,04,3F,80,00,00}（写功能码 10 + 地址 0x4604 + float32 1.0，通道1标零）
 *
 * 轮询策略（频率尽量高）：响应门控——上次查询收到应答（或 3ms 超时）后下一拍立即发新查询，
 * 1ms 定时器驱动下实测 ~1kHz。标零期间暂停轮询，完成后自动恢复。
 */

#ifndef __SRV_MZ_SENSOR_H
#define __SRV_MZ_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "srv_can_bus.h"

/* Exported types ------------------------------------------------------------*/

/** @brief Mz 传感器最新数据（ISR 写，主循环读） */
typedef struct {
    float mz_nm;         /**< 最新扭矩值，N·m */
    uint32_t seq;        /**< 有效应答序号（每次读到新 Mz 加 1） */
    uint32_t last_seen_ms; /**< 最后收到应答的时间 (millis) */
    bool online;         /**< 传感器在线（应答未超时） */
    uint32_t rate_qps;   /**< 实测查询率（每秒查询次数，1s 滚动窗口） */
    bool zero_ok;        /**< 最近一次标零已确认 */
} srv_mz_sensor_fb_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 Mz 传感器服务（绑定 CAN 总线 + 注册接收回调）
 * @param bus 已绑定的 CAN 总线句柄（can_task 接线处分配，通道任选）
 */
void srv_mz_sensor_init(const srv_can_bus_t* bus);

/**
 * @brief 周期步进（1ms 定时器调用）：响应门控发送查询/标零帧
 */
void srv_mz_sensor_step(void);

/**
 * @brief 接收处理回调（ISR 上下文，经 srv_can_bus 分发调用）
 * @param msg       CAN 报文指针
 * @param user_data 未使用
 */
void srv_mz_sensor_on_rx(const drv_can_msg_t* msg, void* user_data);

/**
 * @brief 请求 Mz 标零（通道1，写寄存器 0x4604 = 1.0）
 * @return true=请求已挂起；false=上一请求仍在处理中
 */
bool srv_mz_sensor_zero(void);

/**
 * @brief 获取传感器最新数据
 * @return 数据指针（模块生命周期内有效）
 */
const srv_mz_sensor_fb_t* srv_mz_sensor_get(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_MZ_SENSOR_H */
