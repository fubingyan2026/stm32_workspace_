/**
 * @file    drv_can.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-8
 * @brief   CAN 设备驱动（经典 bxCAN，中断接收，句柄自包含）
 * @attention
 *
 * CubeMX 仅配置 CAN1 (PA11 RX / PA12 TX)。句柄表内置在 drv_can.c 中，
 * drv_can_init() 无需传参。经典 bxCAN：标准帧 11-bit ID，扩展帧 29-bit ID，DLC 0-8。
 */

#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN 通道枚举
 */
typedef enum {
    DRV_CAN_CH_1 = 0, /**< CAN1 — PA11(RX) / PA12(TX) */
    DRV_CAN_CH_NUM,   /**< 通道总数 */
} drv_can_channel_t;

/**
 * @brief 驱动错误码
 */
typedef enum {
    DRV_CAN_OK = 0,
    DRV_CAN_ERROR_NULL_PTR,
    DRV_CAN_ERROR_UNINITIALIZED,
    DRV_CAN_ERROR_TX_BUSY,
    DRV_CAN_ERROR_INVALID_PARAM,
} drv_can_error_t;

/**
 * @brief CAN 报文
 */
typedef struct {
    uint32_t id;          /**< CAN ID（标准 11-bit 或扩展 29-bit） */
    bool     is_extended; /**< true=扩展帧 */
    uint8_t  dlc;         /**< 数据长度 0-8 */
    uint8_t  data[8];     /**< 数据负载 */
} drv_can_msg_t;

/** @brief CAN 接收回调函数类型（中断上下文执行） */
typedef void (*drv_can_rx_callback_t)(drv_can_channel_t ch, const drv_can_msg_t* msg);

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/** @brief 初始化全部 CAN 通道（内部句柄表，无需传参） */
drv_can_error_t drv_can_init(void);

/** @brief 反初始化全部 CAN 通道 */
void drv_can_deinit_all(void);

bool drv_can_is_initialized(drv_can_channel_t ch);

/* --- 发送 --- */

/**
 * @brief 发送 CAN 报文（非阻塞）
 * @return DRV_CAN_ERROR_TX_BUSY 表示无可用邮箱
 */
drv_can_error_t drv_can_send(drv_can_channel_t ch, const drv_can_msg_t* msg);

/**
 * @brief 查询 TX 邮箱是否空闲
 * @param ch 通道号
 * @return true=有空闲邮箱
 */
bool drv_can_tx_ready(drv_can_channel_t ch);

/**
 * @brief 查询全部 TX 邮箱是否空闲（所有已提交帧均已发出）
 * @param ch 通道号
 * @return true=3 个邮箱全部空闲；未初始化视作空闲
 * @note  bxCAN TSR.TME 位由硬件在帧传输完成后置位，无需开启 TX 中断。
 *        用于「发出应答帧后再执行系统复位」等需确认帧已上总线的场景。
 */
bool drv_can_tx_all_done(drv_can_channel_t ch);

/* --- 接收回调 --- */

/**
 * @brief 注册接收回调（每通道独立注册）
 * @param ch       通道号
 * @param callback 回调函数（NULL=取消）
 * @note  回调在中断上下文中执行，应尽量简短
 */
drv_can_error_t drv_can_register_rx_callback(drv_can_channel_t ch,
    drv_can_rx_callback_t callback);

/* --- Bus-Off 检测 / 自恢复 --- */

/**
 * @brief 查询通道是否处于 Bus-Off 状态
 * @param ch 通道号
 * @return true=处于 Bus-Off（内核已置 ESR.BOFF 离线）
 */
bool drv_can_is_bus_off(drv_can_channel_t ch);

/**
 * @brief 从 Bus-Off 自动恢复（保留滤波器与接收回调）
 * @param ch 通道号
 * @return DRV_CAN_OK 表示已恢复或本就无需恢复
 * @note  内部经 HAL_CAN_Stop/Start 清 INIT，触发内核 128×11 隐性位恢复，
 *        无需芯片复位。建议在周期任务中检测到 bus-off 时调用。
 */
drv_can_error_t drv_can_recover(drv_can_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CAN_H */
