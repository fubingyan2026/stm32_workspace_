/**
 * @file    dev_dm4310.c
 * @author  maximillian
 * @version V3.0.0
 * @date    2026-08-28
 * @brief   DM4310 电机设备驱动 — CAN 发送经回调注册解耦
 * @attention
 *
 * 驱动不直接依赖具体 CAN 驱动：发送统一走注册的 dm4310_can_send_cb_t 回调
 * （由 service/应用层桥接到 drv_can 队列发送）。
 */

#include "dev_dm4310.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/

static dm4310_can_send_cb_t s_send_cb;

/* Private function prototypes -----------------------------------------------*/

static void dm4310_can_send(uint16_t id, const uint8_t* data, uint8_t len);

/* ===== CAN 发送回调注册 ===== */

void dm4310_can_send_register(dm4310_can_send_cb_t cb)
{
    s_send_cb = cb;
}

/* ===== 电机控制接口 ===== */

/**
 * @brief  启用 DM4310 电机控制模式
 * @param[in] motor   电机句柄
 */
void dm4310_enable(motor_t* motor)
{
    switch (motor->ctrl.mode) {
    case 0:
        enable_motor_mode(motor->id, MIT_MODE);
        break;
    case 1:
        enable_motor_mode(motor->id, POS_MODE);
        break;
    case 2:
        enable_motor_mode(motor->id, SPEED_MODE);
        break;
    }
}

/**
 * @brief  禁用 DM4310 电机控制模式
 * @param[in] motor   电机句柄
 */
void dm4310_disable(motor_t* motor)
{
    switch (motor->ctrl.mode) {
    case 0:
        disable_motor_mode(motor->id, MIT_MODE);
        break;
    case 1:
        disable_motor_mode(motor->id, POS_MODE);
        break;
    case 2:
        disable_motor_mode(motor->id, SPEED_MODE);
        break;
    }
    dm4310_clear_para(motor);
}

/**
 * @brief  发送 DM4310 电机控制命令
 * @param[in] motor   电机句柄
 */
void dm4310_ctrl_send(motor_t* motor)
{
    switch (motor->ctrl.mode) {
    case 0:
        mit_ctrl(motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set,
            motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
        break;
    case 1:
        pos_speed_ctrl(motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set);
        break;
    case 2:
        speed_ctrl(motor->id, motor->ctrl.vel_set);
        break;
    }
}

/**
 * @brief  设置 DM4310 电机控制参数（cmd → ctrl）
 * @param[in] motor   电机句柄
 */
void dm4310_set(motor_t* motor)
{
    motor->ctrl.kd_set = motor->cmd.kd_set;
    motor->ctrl.kp_set = motor->cmd.kp_set;
    motor->ctrl.pos_set = motor->cmd.pos_set;
    motor->ctrl.vel_set = motor->cmd.vel_set;
    motor->ctrl.tor_set = motor->cmd.tor_set;
}

/**
 * @brief  清除 DM4310 电机控制参数（cmd + ctrl 全清零）
 * @param[in] motor   电机句柄
 */
void dm4310_clear_para(motor_t* motor)
{
    motor->cmd.kd_set = 0;
    motor->cmd.kp_set = 0;
    motor->cmd.pos_set = 0;
    motor->cmd.vel_set = 0;
    motor->cmd.tor_set = 0;

    motor->ctrl.kd_set = 0;
    motor->ctrl.kp_set = 0;
    motor->ctrl.pos_set = 0;
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0;
}

/**
 * @brief  清除 DM4310 电机错误
 * @param[in] motor   电机句柄
 */
void dm4310_clear_err(motor_t* motor)
{
    switch (motor->ctrl.mode) {
    case 0:
        clear_err(motor->id, MIT_MODE);
        break;
    case 1:
        clear_err(motor->id, POS_MODE);
        break;
    case 2:
        clear_err(motor->id, SPEED_MODE);
        break;
    }
}

/**
 * @brief  解析 DM4310 电机反馈数据
 * @param[in] motor    电机句柄（写入反馈）
 * @param[in] rx_data  8 字节反馈数据
 */
void dm4310_fbdata(motor_t* motor, uint8_t* rx_data)
{
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); /* (-12.5,12.5) */
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); /* (-30,30) */
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12); /* (-10,10) */
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
}

/* ===== 量程换算 ===== */

/**
 * @brief  浮点数 → 无符号整数（线性映射到指定位数）
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  无符号整数 → 浮点数（线性映射回原量程）
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ===== 底层模式命令 ===== */

/**
 * @brief  启用电机模式命令
 * @param[in] motor_id  电机 ID
 * @param[in] mode_id   模式偏移（MIT/POS/SPEED）
 */
void enable_motor_mode(uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    dm4310_can_send(id, data, 8);
}

/**
 * @brief  禁用电机模式命令
 */
void disable_motor_mode(uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;

    dm4310_can_send(id, data, 8);
}

/**
 * @brief  保存位置零点命令
 */
void save_pos_zero(uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFE;

    dm4310_can_send(id, data, 8);
}

/**
 * @brief  清除电机错误命令
 */
void clear_err(uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFB;

    dm4310_can_send(id, data, 8);
}

/**
 * @brief  MIT 模式控制帧
 */
void mit_ctrl(uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    dm4310_can_send(id, data, 8);
}

/**
 * @brief  位置-速度控制帧（float 直传）
 */
void pos_speed_ctrl(uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t* pbuf;
    uint8_t* vbuf;
    uint8_t data[8];

    id = motor_id + POS_MODE;
    pbuf = (uint8_t*)&pos;
    vbuf = (uint8_t*)&vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    dm4310_can_send(id, data, 8);
}

/**
 * @brief  速度控制帧（float 直传，4 字节）
 */
void speed_ctrl(uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t* vbuf;
    uint8_t data[4];

    id = motor_id + SPEED_MODE;
    vbuf = (uint8_t*)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);

    dm4310_can_send(id, data, 4);
}

/* ===== 私有发送封装：走注册的回调 ===== */

static void dm4310_can_send(uint16_t id, const uint8_t* data, uint8_t len)
{
    if (s_send_cb == NULL) {
        return;
    }
    s_send_cb(id, data, len);
}
