/**
 * @file    srv_pa430_temp_test.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-28
 * @brief   Motorevo (PA430) CAN FD 伺服电机 — MIT 模式**速度**耐久测试
 *
 * 用 MIT 模式实现速度控制（Kp=0，T_out = Kd×(V_ref−V)，T_ref=0）：
 * 广播控制帧 0x20（CAN FD DLC 64，槽 (ID-1)*8）周期下发，V_ref 按
 * 「正转 30s → 停 30s → 反转 30s → 停 30s」循环（一个循环 120s），
 * 主机侧对 V_ref 做线性斜坡软启停；电机持续在线累计满 24h 自动停止并失能。
 *
 * 通道：由 srv_motor_test_select.h 统一枚举选择（CAN1/CAN2 任选），
 * 本模块收发统一走 SRV_PA430_TEMP_CAN_CH；单例，不能两通道同选。
 *
 * 安全：Control Mode(参数 Index 11) 确认为 MIT(0x2) 之前**不下发控制帧、不使能**
 * （防上电时电机处于 Flash 非 MIT 模式把 0x20 误解执行导致猛动）；就绪后才使能+控制。
 */

#ifndef __SRV_PA430_TEMP_TEST_H
#define __SRV_PA430_TEMP_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "drv_can.h"
#include "srv_motor_test_select.h" /* 提供 SRV_PA430_TEMP_CAN_CH */

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 PA430 MIT 速度耐久测试服务
 * @note  SRV_PA430_TEMP_AUTO_START=1 时上电自动启动
 */
void srv_pa430_temp_test_init(void);

/**
 * @brief 启动测试：确认 MIT 模式 → 广播使能 → 按 正/停/反/停 循环下发速度
 */
void srv_pa430_temp_test_start(void);

/**
 * @brief 停止测试：速度斜坡回 0 → 广播失能
 */
void srv_pa430_temp_test_stop(void);

/**
 * @brief 周期步进（由 can_task 每 TASK_PERIOD_MS 调用）
 */
void srv_pa430_temp_test_step(void);

/**
 * @brief 处理 PA430 反馈帧/参数应答帧（由 can_task 按 SRV_PA430_TEMP_CAN_CH 分发调用）
 * @param  msg CAN 报文指针
 * @return true=已消费；false=非本协议帧
 * @note   ISR 上下文，只做数据记录与标志置位，不打日志
 */
bool srv_pa430_temp_test_on_rx(const drv_can_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PA430_TEMP_TEST_H */
