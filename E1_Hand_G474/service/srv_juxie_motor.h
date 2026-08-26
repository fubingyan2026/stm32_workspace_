/**
 * @file    srv_juxie_motor.h
 * @brief   橘虾(juxie) 伺服执行器 CAN FD MIT 模式电机服务（单机，Dev_ID 可配）
 *
 * 协议按 docs/juxie_canfd_cmd.md 组帧：
 *   - MIT 单轴控制帧：CAN ID `0x110 | Dev_ID`，CAN FD，数据段波特率 = 仲裁段 1M（不启用 BRS），DLC 9。
 *     Byte[0]=控制指令头（bit7 使能 / bit6 抱闸 / bit5 清错 / bit[4:1]=0x06 MIT），
 *     Byte[1~2]=目标位置 16bit 大端 [0..65535]↔(-Pos_Max~+Pos_Max)，
 *     Byte[3]+Byte[4][7:4]=目标速度 12bit，Byte[4][3:0]+Byte[5]=Kp 12bit [0..4095]↔[0..500]，
 *     Byte[6]+Byte[7][7:4]=Kd 12bit [0..4095]↔[0..5]，Byte[7][3:0]+Byte[8]=目标力矩 12bit。
 *   - 反馈帧：CAN ID `0x300 | Dev_ID`，DLC 16（带力矩传感器）/ DLC 12（不带），
 *     控制帧下发后由电机自动触发，无需单独查询。
 *
 * 上电默认零刚度零力矩（Kp=Kd=Tq=0）且失能/抱闸吸合，电机不会突动。
 * 主机经 UART 协议下发 MIT 目标（原始 12/16bit 打包值）与参数（使能/抱闸/清错/量程）。
 */

#ifndef __SRV_JUXIE_MOTOR_H
#define __SRV_JUXIE_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "srv_can_bus.h"

/* Exported types ------------------------------------------------------------*/

/** @brief 电机配置参数索引（对应 UART 0x1D1 命令的 param） */
enum {
    SRV_JUXIE_PARAM_ENABLE = 1,  /**< 使能(1)/失能(0) */
    SRV_JUXIE_PARAM_CLEAR_ERR,   /**< 清错误(1)，脉冲式下发后自动复位 */
    SRV_JUXIE_PARAM_BRAKE,       /**< 抱闸：1=释放，0=吸合 */
    SRV_JUXIE_PARAM_POS_MAX,     /**< 位置量程 Pos_Max，单位 0.1°（int16） */
    SRV_JUXIE_PARAM_VEL_MAX,     /**< 速度量程 Vel_Max，单位 rpm（int16） */
    SRV_JUXIE_PARAM_TQ_MAX,      /**< 力矩量程 T_Max，单位 0.01Nm（int16） */
};

/** @brief 电机反馈数据（ISR 写，主循环读） */
typedef struct {
    int32_t pos_deg_x100;   /**< 负载端实际位置，0.01°（±18000=±180°） */
    int16_t speed_rpm;      /**< 电机端实际速度，rpm */
    int16_t iq_ma;          /**< 实际电流(Iq)，mA */
    uint16_t err;           /**< 错误代码 */
    int16_t temp_x10;       /**< 电机线圈温度，0.1℃ */
    int16_t tq_001nm;       /**< 力矩反馈，0.01Nm（原 0.05Nm 换算，越界饱和） */
    uint8_t mode;           /**< 控制模式反馈（0x06=MIT） */
    uint8_t status;         /**< 状态位：bit7 使能 / bit6 抱闸 / bit5 报错 / bit4 到位 */
    uint32_t seq;           /**< 反馈帧序号 */
    uint32_t last_seen_ms;  /**< 最后收到反馈的时间 (millis) */
} srv_juxie_fb_t;

/** @brief 电机运行配置与 MIT 目标（主循环写，step 读） */
typedef struct {
    uint8_t enable;         /**< 使能控制位（1=使能） */
    uint8_t brake;          /**< 抱闸控制位（1=释放） */
    uint8_t clear_err;      /**< 清错误脉冲（下发一帧后由 step 复位） */
    uint16_t pos_max_x10deg;/**< 位置量程 Pos_Max，0.1° */
    uint16_t vel_max_rpm;   /**< 速度量程 Vel_Max，rpm */
    uint16_t tq_max_001nm;  /**< 力矩量程 T_Max，0.01Nm */

    uint16_t pos_raw;       /**< MIT 目标位置 16bit 原始值 [0..65535] */
    uint16_t vel_raw;       /**< MIT 目标速度 12bit 原始值 [0..4095] */
    uint16_t kp_raw;        /**< MIT Kp 12bit 原始值 [0..4095] */
    uint16_t kd_raw;        /**< MIT Kd 12bit 原始值 [0..4095] */
    uint16_t tq_raw;        /**< MIT 目标力矩 12bit 原始值 [0..4095] */
} srv_juxie_cfg_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化橘虾 MIT 电机服务（绑定 CAN 总线 + 注册接收回调）
 * @param bus 已绑定的 CAN 总线句柄（can_task 接线处分配，通道任选）
 */
void srv_juxie_motor_init(const srv_can_bus_t* bus);

/**
 * @brief 周期步进（1ms 定时器调用）：重发 MIT 单轴控制帧
 * @note  控制帧自动触发电机反馈；清错误位为脉冲，发完一帧即复位
 */
void srv_juxie_motor_step(void);

/**
 * @brief 接收处理回调（ISR 上下文，经 srv_can_bus 分发调用）
 * @param msg       CAN 报文指针
 * @param user_data 未使用
 */
void srv_juxie_motor_on_rx(const drv_can_msg_t* msg, void* user_data);

/**
 * @brief 设置 MIT 目标（原始打包值，与 juxie §4.1 载荷一致）
 * @param pos_raw 目标位置 16bit [0..65535]↔(-Pos_Max~+Pos_Max)
 * @param vel_raw 目标速度 12bit [0..4095]↔(-Vel_Max~+Vel_Max)
 * @param kp_raw  Kp 12bit [0..4095]↔[0..500]
 * @param kd_raw  Kd 12bit [0..4095]↔[0..5]
 * @param tq_raw  目标力矩 12bit [0..4095]↔(-T_Max~+T_Max)
 */
void srv_juxie_motor_set_mit_raw(uint16_t pos_raw, uint16_t vel_raw,
    uint16_t kp_raw, uint16_t kd_raw, uint16_t tq_raw);

/**
 * @brief 设置电机配置参数（对应 UART 0x1D1 命令）
 * @param param 参数索引（见 SRV_JUXIE_PARAM_*）
 * @param value int16 参数值（量程类参数只接受正数）
 * @return true=参数有效并已生效
 */
bool srv_juxie_motor_config(uint8_t param, int16_t value);

/**
 * @brief 触发电机标零（SDO 写 CAN ID 0x601，Index 0x2531 Sub 0 = 1）
 * @return true=已发送；false=总线忙/未初始化
 * @note   一次性下发，不等待电机应答；电机以当前角度为新零点
 */
bool srv_juxie_motor_zero(void);

/**
 * @brief 获取最新电机反馈
 * @return 反馈指针（模块生命周期内有效）
 */
const srv_juxie_fb_t* srv_juxie_motor_get_fb(void);

/**
 * @brief 获取电机运行配置
 * @return 配置指针（模块生命周期内有效）
 */
const srv_juxie_cfg_t* srv_juxie_motor_get_cfg(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_JUXIE_MOTOR_H */
