/**
 * @file    srv_ht_can2_torque_test.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-17
 * @brief   苇熠(HT) 伺服执行器 CAN 控制协议服务 — 速度模式多圈往复耐久测试（CAN2 版）
 *
 * 功能与 srv_ht_torque_test（CAN1/FDCAN1 版）完全一致，唯一区别是全部指令
 * 通过 FDCAN2（DRV_CAN_CH_2, PB12 RX / PB13 TX）发送，供 CAN1 已被占用的
 * 场景在独立总线上控制另一组苇熠电机。
 *
 * 电机控制指令按 docs/苇熠电机can协议文档.md 组帧发送（经典 CAN 2.0A, 1 Mbps,
 * CAN-ID 低 8 位 = 设备地址，data[0]=指令，data[1..]=参数，多字节大端）。
 *
 * RX 侧：测试协议帧（扫描应答/位置/报警/电压/在线心跳）由 can_task 按 CH_2
 * 分发到 srv_ht_can2_torque_test_on_rx() 消费，返回 true 表示已处理；本模块
 * 不依赖 srv_can / srv_motor / srv_motor_behavior。
 */

#ifndef __SRV_HT_CAN2_TORQUE_TEST_H
#define __SRV_HT_CAN2_TORQUE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "drv_can.h"

/* 接线选择 ------------------------------------------------------------------*/
/* CAN2（FDCAN2）测试模块选择统一在 service/srv_motor_test_select.h 的
 * SRV_MOTOR_TEST_SELECT_CAN2 枚举宏中完成：
 *   - SRV_MOTOR_TEST_HT_CAN2 = 0 本模块（苇熠速度模式往复 CAN2 版）；
 *   - SRV_MOTOR_TEST_PA430   = 1 Motorevo(PA430) MIT 测试。
 * can_task 按该选择接线 init/step/on_rx；两模块共用 FDCAN2 独立总线，
 * 同一时刻只激活一个。 */

/* Exported functions prototypes ---------------------------------------------*/

/* --- 生命周期 --- */

/**
 * @brief 初始化往复耐久测试服务
 * @note  SRV_HT_CAN2_TORQUE_TEST_AUTO_START=1 时上电自动进入测试模式（扫描 + 往复驱动）
 */
void srv_ht_can2_torque_test_init(void);

/* --- 测试模式 --- */

/**
 * @brief 启动往复耐久测试模式
 * @note  先扫描总线电机 ID（握手 0x00，探测 0x01~0x3F），扫描完成后
 *        对检测到的电机发送：使能 (0x2A 01) → 速度模式 (0x07 02)；之后按
 *        位置监控反向，速度由固件 RAMP_MS 斜坡平滑加减速，连续旋转不停顿
 */
void srv_ht_can2_torque_test_start(void);

/**
 * @brief 停止往复耐久测试模式（速度 0 停下 + 失能，发往所有检测到的电机）
 */
void srv_ht_can2_torque_test_stop(void);

/**
 * @brief 往复耐久测试周期步进（由 can_task 每 TASK_PERIOD_MS 调用）
 * @note  阶段 1 扫描总线电机 ID；阶段 2 在 0 与 +POS_LIMIT_DEG（50 圈）之间
 *        多圈往复（速度模式连续旋转），方向变化用 RAMP_MS 斜坡平滑加减速，
 *        每 POS_POLL_PERIOD_MS 用 0x06 读位置、到达端点（0 / +50R）即反向，
 *        命令仅发往检测到的电机；电机持续在线累计满 DURATION_MS（30 天）自动停止
 */
void srv_ht_can2_torque_test_step(void);

/* --- RX 帧处理 --- */

/**
 * @brief 处理测试协议接收帧（由 can_task 按 CH_2 分发调用）
 * @param  msg CAN 报文指针
 * @return true=测试协议帧，已消费（扫描应答/位置/报警/电压/已知电机在线刷新）；
 *         false=非测试帧（CAN2 专用总线，直接丢弃）
 * @note   ISR 上下文，只做数据记录与标志置位，不打日志
 */
bool srv_ht_can2_torque_test_on_rx(const drv_can_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_HT_CAN2_TORQUE_TEST_H */
