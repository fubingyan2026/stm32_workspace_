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
#include "drv_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/** @brief 位置微调步长 (rad)，按键单击一次的变化量 */
#define SRV_DM4310_POS_STEP_RAD (0.1f)

/**
 * @brief 电机枚举（当前仅使用 MOTOR_1，保留供后续扩展）
 * @note  数组索引 = 枚举值；新增电机时在末尾追加并同步 MOTOR_NUM
 */
typedef enum {
    MOTOR_1 = 0, /**< 1 号电机（当前启用） */
    MOTOR_NUM,   /**< 电机总数 */
} motor_num_t;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机工作状态（fsm 状态机）
 */
typedef enum {
    SRV_DM4310_STATE_UNINIT = 0, /**< 未初始化 */
    SRV_DM4310_STATE_INIT,       /**< 初始化完成（参数已配置，未使能） */
    SRV_DM4310_STATE_ENABLED,    /**< 已使能（MIT 运行） */
    SRV_DM4310_STATE_DISABLED,   /**< 已禁用 */
    SRV_DM4310_STATE_MAX,
} srv_dm4310_state_t;

/**
 * @brief 初始化 DM4310 电机（MIT 模式，ID 可在 init 内修改）
 * @note  CAN 通道固定 DRV_CAN_CH_1（G0 仅一路 CAN）；
 *        默认使能电机并下发初始 MIT 参数（kp/kd/tor 归零）
 */
void srv_dm4310_ctrl_init(void);

/** @brief 获取当前电机状态 */
srv_dm4310_state_t srv_dm4310_ctrl_get_state(void);

/** @brief 状态机步进（task 层周期调用，驱动状态转换） */
void srv_dm4310_ctrl_step(void);

/**
 * @brief 位置微调：当前给定位置增加/减小一步并立即发送
 * @param delta_pos 位置增量（rad，正=增加，负=减小）
 * @note  MIT 模式下速度/扭矩给定保持 0
 */
void srv_dm4310_ctrl_pos_step(float delta_pos);

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

/** @brief 保存当前电机位置为零点（save_pos_zero） */
void srv_dm4310_ctrl_save_zero(void);

/**
 * @brief 轮询 CAN RX 队列，解析电机反馈（task 层周期调用）
 * @note  按帧 CAN ID == 电机 ID 匹配
 */
void srv_dm4310_ctrl_poll_rx(void);

/**
 * @brief 处理单帧 CAN 报文（由 can_task 的 RX 分发调用，避免队列双消费）
 * @param msg 已出队的 CAN 报文
 * @note  按帧 CAN ID == 电机 ID 匹配解析反馈；非电机帧直接忽略
 */
void srv_dm4310_ctrl_feed(const drv_can_msg_t* msg);

/**
 * @brief 获取电机最新反馈
 * @return motor_t* 电机句柄（含 para 反馈字段），始终非空
 */
motor_t* srv_dm4310_ctrl_get_motor(motor_num_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_DM4310_CTRL_H__ */
