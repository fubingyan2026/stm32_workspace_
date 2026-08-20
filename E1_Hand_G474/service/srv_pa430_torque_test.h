/**
 * @file    srv_pa430_torque_test.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-13
 * @brief   Motorevo (PA430) CAN FD 伺服电机 — MIT 力位混合模式来回运动测试服务
 *
 * 电机控制指令按 docs/Motorevo电机CAN协议文档.md 组帧发送（CAN FD 广播帧，
 * 状态帧 0x10 / 控制帧 0x20，DLC 64，每电机槽 (ID-1)*8，多字节大端）。
 *
 * 与 srv_ht_torque_test（苇熠速度模式多圈往复）的区别：
 *   1. 使用 MIT 力位混合控制模式（Control Mode = 0x2），单机 8 字节封包
 *      携带 θ_ref / V_ref / Kp / Kd / T_ref（12bit/16bit 归一化映射）；
 *   2. 广播帧一帧同时驱动总线上 1~8 台电机（ID 1~8），各电机以自身 ID
 *      回复 DLC 8 反馈帧（θ / V / T / 温度 / 错误码）；
 *   3. 来回运动：θ_ref 在 +CAN_COM_MAX 与 CAN_COM_MIN 两端点间交替，
 *      读反馈帧 θ，|θ−目标| ≤ 阈值 判定到位后反向（位置反馈闭环）。
 *
 * RX 侧：反馈帧（CAN-ID = 电机 ID 1~8, DLC 8）由 can_task 按 CH_2 分发到
 * srv_pa430_torque_test_on_rx() 消费，返回 true 表示已处理。本模块不依赖
 * srv_motor / srv_motor_behavior。
 */

#ifndef __SRV_PA430_TORQUE_TEST_H
#define __SRV_PA430_TORQUE_TEST_H

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
 *   - SRV_MOTOR_TEST_HT_CAN2 = 0 苇熠(HT) 速度模式往复 CAN2 版；
 *   - SRV_MOTOR_TEST_PA430   = 1 本模块（Motorevo MIT 力位混合来回）。
 * can_task 按该选择接线 init/step/on_rx；两模块共用 FDCAN2 独立总线，
 * 同一时刻只激活一个，与 CAN1 上的测试并行运行、互不干扰。 */

/* Exported functions prototypes ---------------------------------------------*/

/* --- 生命周期 --- */

/**
 * @brief 初始化 PA430 来回运动测试服务
 * @note  SRV_PA430_AUTO_START=1 时上电自动进入测试模式（使能 + MIT 往复驱动）
 */
void srv_pa430_torque_test_init(void);

/* --- 测试模式 --- */

/**
 * @brief 启动来回运动测试模式
 * @note  对配置电机发广播使能（0x10, byte7=0xFC）→ 持续下发 MIT 控制帧（0x20），
 *        θ_ref 在 +CAN_COM_MAX 与 CAN_COM_MIN 两端点间交替，到位即反向
 */
void srv_pa430_torque_test_start(void);

/**
 * @brief 停止来回运动测试模式（θ_ref 回中 + 广播失能，发往所有配置电机）
 */
void srv_pa430_torque_test_stop(void);

/**
 * @brief 来回运动测试周期步进（由 can_task 每 TASK_PERIOD_MS 调用）
 * @note  周期重发广播 MIT 控制帧；消费反馈帧到位标志，所有配置电机到位后
 *        θ_ref 翻转到另一端；打印错误码变化、掉线告警与恢复在线重新使能；
 *        累计在线满 DURATION_MS（30 天，置 0 禁用）自动停止
 */
void srv_pa430_torque_test_step(void);

/* --- RX 帧处理 --- */

/**
 * @brief 处理 Motorevo 反馈帧（由 can_task 按 CH_2 分发调用）
 * @param  msg CAN 报文指针
 * @return true=反馈帧（CAN-ID 1~8, DLC≥8），已消费；
 *         false=非反馈帧（CAN2 专用总线，直接丢弃）
 * @note   ISR 上下文，只做数据记录与标志置位，不打日志
 */
bool srv_pa430_torque_test_on_rx(const drv_can_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PA430_TORQUE_TEST_H */
