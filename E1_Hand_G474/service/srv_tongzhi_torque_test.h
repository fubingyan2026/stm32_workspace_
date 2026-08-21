/**
 * @file    srv_tongzhi_torque_test.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-14
 * @brief   良志(ODrive) 伺服执行器 CAN 控制协议服务 — 位置模式梯形轨迹往复耐久测试
 *
 * 电机控制指令按 docs/良志电机can协议.md 组帧发送（经典 CAN 2.0A, 1 Mbps,
 * CAN-ID = (node_id<<5) | cmd_id，8 字节帧，小端，IEEE-754 单精度浮点）。
 *
 * 与 srv_pa430_torque_test（MIT 力位混合）的区别：
 *   1. 走 FDCAN1（DRV_CAN_CH_1），与苇熠(HT) 测试通过 srv_motor_test_select.h
 *      三选一（互斥）；PA430(Motorevo) 仍在 FDCAN2 独立并行；
 *   2. 使用位置模式 + 梯形轨迹（Set_Controller_Mode 3/5 + Set_Input_Pos），主机只交替
 *      下发目标位置（各自初始化零点 ±SRV_TONGZHI_POS_AMP_TURNS 转，零点 = 各电机初始化
 *      完成时读回的编码器位置），电机自带梯形规划平滑加减速，
 *      目标为锁存式，无需像 MIT 那样高频持续下发保持刚度；
 *   3. 多台电机按 node_id（0~63）寻址，靠电机周期推送的心跳(0x01)被动发现在线电机，
 *      编码器位置(0x09)由电机周期推送（默认 10ms），用于到位判定。
 *
 * RX 侧：心跳/编码器帧（CAN-ID=(node<<5)|0x01/0x09, DLC 8）由 can_task 按 CH_1 分发到
 * srv_tongzhi_torque_test_on_rx() 消费。本模块不依赖 srv_motor / srv_motor_behavior。
 */

#ifndef __SRV_TONGZHI_TORQUE_TEST_H
#define __SRV_TONGZHI_TORQUE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "drv_can.h"

/* 接线说明（选择由 service/srv_motor_test_select.h 的 SRV_MOTOR_TEST_SELECT 统一决定）：
 *   良志(ODrive) 测试模式走 FDCAN1（DRV_CAN_CH_1），与苇熠 HT 测试三选一
 *   （SRV_MOTOR_TEST_SELECT = SRV_MOTOR_TEST_TONGZHI 时激活本模块，can_task 把
 *   CH_1 全部帧路由给本模块，srv_can 停用）；
 *   选苇熠(HT_TORQUE/HT_TEMP) 时 CH_1 归苇熠测试，本模块不参与收发。
 *   PA430(Motorevo) 测试走 FDCAN2 独立总线，不受本开关影响。 */

/* Exported functions prototypes ---------------------------------------------*/

/* --- 生命周期 --- */

/**
 * @brief 初始化良志(ODrive)往复耐久测试服务
 * @note  SRV_TONGZHI_AUTO_START=1 时上电自动进入测试模式（心跳被动发现 + 往复驱动）
 */
void srv_tongzhi_torque_test_init(void);

/* --- 测试模式 --- */

/**
 * @brief 启动往复耐久测试模式
 * @note  等待电机周期心跳(0x01)被动发现 node_id，对发现的电机下发
 *        清错 → 闭环(Set_Axis_State 8) → 位置+梯形轨迹(Set_Controller_Mode 3/5) →
 *        梯形限速限加（Set_Traj_Vel_Limit / Set_Traj_Accel_Limits）；
 *        Set_Input_Pos 在 各自初始化零点±SRV_TONGZHI_POS_AMP_TURNS 两端点间交替，
 *        全部到位即反向（零点 = 各电机初始化完成后读回的当前位置，掉线恢复后重新锁存）
 */
void srv_tongzhi_torque_test_start(void);

/**
 * @brief 停止往复耐久测试模式（电机回 IDLE，发往所有已发现电机）
 */
void srv_tongzhi_torque_test_stop(void);

/**
 * @brief 往复耐久测试周期步进（由 can_task 每 TASK_PERIOD_MS 调用）
 * @note  消费心跳/编码器反馈：错误变化打印、闭环保持重试、到位翻转、
 *        掉线告警与恢复重发闭环、累计在线满 DURATION_MS（30 天）自动停止
 */
void srv_tongzhi_torque_test_step(void);

/* --- RX 帧处理 --- */

/**
 * @brief 处理良志(ODrive)心跳/编码器帧（由 can_task 按 CH_1 分发调用）
 * @param  msg CAN 报文指针
 * @return true=ODrive 帧（node_id 0~63, DLC≥8），已消费；
 *         false=非本协议帧（不应出现在专用总线）
 * @note   ISR 上下文，只做数据记录与标志置位，不打日志
 */
bool srv_tongzhi_torque_test_on_rx(const drv_can_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_TONGZHI_TORQUE_TEST_H */
