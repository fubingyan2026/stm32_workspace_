/**
 * @file    srv_dm4310_ctrl.h
 * @author  maximillian
 * @version V3.0.0
 * @date    2026-08-28
 * @brief   DM4310 电机控制服务（单电机，MIT 模式）
 * @attention
 *
 * 精简版：仅保留一路 DM4310 电机的 MIT 模式控制与反馈获取。
 * MIT 帧：pos(16bit) vel(12bit) kp(12bit) kd(12bit) tor(12bit)，
 * 反馈：ID/state/pos/vel/tor/Tmos/Tcoil。
 */

#ifndef __SRV_DM4310_CTRL_H__
#define __SRV_DM4310_CTRL_H__

#include "dev_dm4310.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机枚举（当前仅使用 MOTOR_1，保留供后续扩展）
 * @note  数组索引 = 枚举值；新增电机时在末尾追加并同步 MOTOR_NUM
 */
typedef enum {
    MOTOR_1 = 0, /**< 1 号电机（当前启用） */
    MOTOR_2,     /**< 2 号电机（预留） */
    MOTOR_3,     /**< 3 号电机（预留） */
    MOTOR_NUM,   /**< 电机总数 */
} motor_num_t;

/**
 * @brief 初始化 DM4310 电机（MIT 模式，ID 可在 init 内修改）
 * @note  CAN 通道固定 DRV_CAN_CH_1（G0 仅一路 CAN）
 */
void srv_dm4310_ctrl_init(void);

/** @brief 使能电机（MIT 模式） */
void srv_dm4310_ctrl_enable(void);

/** @brief 禁用电机（MIT 模式），并清空 cmd/ctrl 参数 */
void srv_dm4310_ctrl_disable(void);

/**
 * @brief 设置 MIT 控制目标（写入 cmd → ctrl）
 * @param pos  位置给定 (-12.5, 12.5)
 * @param vel  速度给定 (-30, 30)
 * @param kp   位置刚度 (0, 500)
 * @param kd   阻尼 (0, 5)
 * @param tor  扭矩前馈 (-10, 10)
 */
void srv_dm4310_ctrl_set_target(float pos, float vel, float kp, float kd, float tor);

/** @brief 发送 MIT 控制帧（需电机已使能） */
void srv_dm4310_ctrl_send(void);

/**
 * @brief 轮询 CAN RX 队列，解析电机反馈（task 层周期调用）
 * @note  按帧 CAN ID == 电机 ID 匹配
 */
void srv_dm4310_ctrl_poll_rx(void);

/**
 * @brief 获取电机最新反馈
 * @return motor_t* 电机句柄（含 para 反馈字段），始终非空
 */
motor_t* srv_dm4310_ctrl_get_motor(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_DM4310_CTRL_H__ */
