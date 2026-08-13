/**
 * @file    srv_can.h
 * @brief   CAN 电机控制协议服务 — 苇熠伺服执行器控制 + 旧 CAN FD 反馈帧打包
 *
 * 电机控制（测试模式）按 docs/苇熠电机can协议文档.md 组帧：
 *   经典 CAN 2.0A，1 Mbps，CAN-ID 低 8 位 = 设备地址，帧 ≤ 8B，
 *   data[0]=指令符，data[1..]=参数（多字节大端）。见 srv_can_test_*()。
 *
 * 旧 CAN FD 上位机协议（保留）：
 * 控制帧 (Host→Device, CAN ID 0x100):
 *   [0]      ctrl: 0x01=enable, 0x02=disable
 *   [1-18]   pos_ref[9]  (int16_t LE, Q7)
 *   [19-36]  spd_ref[9]  (int16_t LE, Q15)
 *   [37-54]  cur_ref[9]  (int16_t LE, Q15)
 *   [55-63]  reserved
 *
 * 反馈帧 (Device→Host, CAN ID 0x101, 100ms):
 *   [0]      global_state
 *   [1-9]    fsm_state[9]
 *   [10-27]  angle_fb[9] (int16_t LE, Q7)
 *   [28-45]  speed_fb[9] (int16_t LE, Q15)
 *   [46-63]  q_cur[9]    (int16_t LE, Q15)
 *
 * 状态帧 (Device→Host, CAN ID 0x102, 500ms):
 *   [0]      global_state
 *   [1-9]    fsm_state[9]
 *   [10-18]  err_code[9]
 *   [19-27]  temp[9]      (int8_t, °C = raw - 50)
 *   [28-45]  vbus[9]      (int16_t LE, V = raw / 128)
 *   [46-47]  reserved
 */

#ifndef __SRV_CAN_H
#define __SRV_CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "drv_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN ID 定义 */
#define SRV_CAN_ID_CTRL 0x100 /**< 控制帧 (Host→Device) */
#define SRV_CAN_ID_FEEDBACK 0x101 /**< 高频反馈 (角度/速度/Q电流) */
#define SRV_CAN_ID_STATUS 0x102 /**< 低频状态 (错误/温度/电压) */

/** @brief 帧长度（CAN FD DLC 编码: 0-8,12,16,20,24,32,48,64） */
#define SRV_CAN_CTRL_LEN 64U /**< 55B 数据 + 9B 保留 */
#define SRV_CAN_FB_LEN 64U /**< 64B 恰好填满 */
#define SRV_CAN_STATUS_LEN 48U /**< 46B 数据 + 2B 保留 */

/* API -----------------------------------------------------------------------*/

/** @brief 初始化 CAN 协议服务 */
void srv_can_init(void);

/**
 * @brief 处理收到的 CAN 帧（由 ISR 回调调用）
 * @param msg CAN 报文指针
 * @note  仅缓存数据，不直接调用电机 API（ISR 安全）
 */
void srv_can_on_rx(const drv_can_msg_t* msg);

/**
 * @brief 处理缓存的控制帧并下发到电机行为层
 * @note  由 can_task 的 sw_timer 在主循环调用
 */
void srv_can_process(void);

/**
 * @brief 构建并发送反馈帧
 * @note  由 can_task 的 sw_timer 周期调用
 */
void srv_can_send_feedback(void);

/**
 * @brief 构建并发送低频状态帧 (fsm_state/err_code/temp/vbus)
 * @note  由 can_task 低频周期调用
 */
void srv_can_send_status(void);

/* --- 测试模式 --- */

/**
 * @brief 启动电机测试模式
 * @note  苇熠协议：先扫描总线电机 ID（握手 0x00，探测 0x01~0x3F），
 *        扫描完成后对检测到的电机发送使能 + 速度模式 + 正转速度（经典 CAN 帧）；
 *        未检测到任何电机时回退到默认地址 SRV_CAN_TEST_DEFAULT_MOTOR_ADDR
 */
void srv_can_test_start(void);

/**
 * @brief 停止电机测试模式（发速度 0 + 失能，发往所有检测到的电机）
 */
void srv_can_test_stop(void);

/**
 * @brief 测试模式周期步进（can_task 每 10ms 调用）
 * @note  阶段 1 扫描总线电机 ID；阶段 2 驱动正转/停留/反转/停留 每阶段
 *        SRV_CAN_TEST_PHASE_MS 循环，每 1s 主动查询电机报警（0xFF，仅状态变化时打印），
 *        周期读取供电电压（0x87），24h 后自动停止
 */
void srv_can_test_step(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_CAN_H */
