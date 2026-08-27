/**
 * @file    srv_can.h
 * @brief   CAN 协议服务 — 协议骨架占位
 *
 * G0 手套 CAN 协议尚未定稿，本模块提供：
 *   - 占位帧 ID 常量（后续协议文档确认后修改）
 *   - on_rx / process / send_heartbeat 空壳，保证链路可验证
 */

#ifndef __SRV_CAN_H
#define __SRV_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "drv_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN ID 定义（占位，待协议文档确认） */
#define SRV_CAN_ID_HEARTBEAT 0x010 /**< 心跳/占位控制帧 (G0 → 对端) */
#define SRV_CAN_ID_ACK 0x011 /**< 应答/占位帧 (对端 → G0) */

/* API -----------------------------------------------------------------------*/

/** @brief 初始化 CAN 协议服务 */
void srv_can_init(void);

/**
 * @brief 处理收到的 CAN 帧（由 ISR 回调调用）
 * @param msg CAN 报文指针
 * @note   ISR 安全，仅缓存/计数，不直接做耗时操作
 */
void srv_can_on_rx(const drv_can_msg_t* msg);

/**
 * @brief 协议处理步进（主循环 sw_timer 调用）
 */
void srv_can_process(void);

/**
 * @brief 发送心跳帧（周期调用，用于链路验证）
 */
void srv_can_send_heartbeat(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_CAN_H */
