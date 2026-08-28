/**
 * @file    dev_dm4310.h
 * @author  maximillian
 * @version V3.0.0
 * @date    2026-08-28
 * @brief   DM4310 电机设备驱动 — CAN 发送经回调注册解耦
 * @attention
 *
 * 驱动本身不依赖任何 CAN 驱动实现，发送动作通过 dm4310_can_send_register()
 * 注册的发送回调完成（由 service/应用层桥接到 drv_can）。
 * 电机 ID 与模式编号相加得到 CAN ID：MIT=0x000, POS=0x100, SPEED=0x200。
 */

#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__

#include "main.h"

/* 电机模式偏移（与 CAN ID 相加） */
#define MIT_MODE 0x000
#define POS_MODE 0x100
#define SPEED_MODE 0x200

/* 反馈量程 */
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

/* 电机回传信息结构体 */
typedef struct {
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

/* 电机参数设置结构体 */
typedef struct {
    int8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

/* 电机句柄 */
typedef struct {
    int8_t id;
    uint8_t start_flag;
    motor_fbpara_t para;
    motor_ctrl_t ctrl;
    motor_ctrl_t cmd;
} motor_t;

/**
 * @brief CAN 发送回调类型（由上层注册，桥接到具体 CAN 驱动）
 * @param id    CAN 帧 ID（已含模式偏移）
 * @param data  数据指针
 * @param len   数据长度（1-8）
 */
typedef void (*dm4310_can_send_cb_t)(uint16_t id, const uint8_t* data, uint8_t len);

/* CAN 发送回调注册 */
void dm4310_can_send_register(dm4310_can_send_cb_t cb);

/* 量程换算 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);

/* 电机控制接口 */
void dm4310_ctrl_send(motor_t* motor);
void dm4310_enable(motor_t* motor);
void dm4310_disable(motor_t* motor);
void dm4310_set(motor_t* motor);
void dm4310_clear_para(motor_t* motor);
void dm4310_clear_err(motor_t* motor);
void dm4310_fbdata(motor_t* motor, uint8_t* rx_data);

/* 底层模式命令 */
void enable_motor_mode(uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(uint16_t motor_id, uint16_t mode_id);
void mit_ctrl(uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void pos_speed_ctrl(uint16_t motor_id, float pos, float vel);
void speed_ctrl(uint16_t motor_id, float _vel);
void save_pos_zero(uint16_t motor_id, uint16_t mode_id);
void clear_err(uint16_t motor_id, uint16_t mode_id);

#endif /* __DM4310_DRV_H__ */
