/**
 * @file    drv_can.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-12
 * @brief   CAN 设备驱动（经典 bxCAN，中断接收 + msg_fifo 收发缓冲队列）
 * @attention
 *
 * G0 手套 CubeMX 配置 CAN1 (PA11 RX / PA12 TX)。句柄表内置在 drv_can.c 中，
 * drv_can_init() 无需传参。经典 bxCAN：标准帧 11-bit ID，扩展帧 29-bit ID，DLC 0-8。
 *
 * 参考 E1_Hand_G474 的 drv_can 队列架构：
 *   - RX：中断回调 HAL_CAN_RxFifo0MsgPendingCallback 入队 msg_fifo，
 *         主循环 drv_can_rx_pop 出队消费（单生产者/单消费者，ISR 安全）
 *   - TX：主循环 drv_can_tx_enqueue 入队，drv_can_tx_flush 排空到 bxCAN TX 邮箱
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

/* --- 发送（经 TX 队列） --- */

/**
 * @brief 发送 CAN 报文（非阻塞，经 TX 缓冲队列）
 * @param ch  通道号
 * @param msg 报文指针
 * @return DRV_CAN_ERROR_TX_BUSY 表示 TX 队列满
 * @note  队列中的帧由 drv_can_tx_flush() 排空到 bxCAN TX 邮箱
 */
drv_can_error_t drv_can_send(drv_can_channel_t ch, const drv_can_msg_t* msg);

/**
 * @brief 查询 TX 队列中的待发报文数
 */
uint32_t drv_can_tx_pending(drv_can_channel_t ch);

/**
 * @brief 排空 TX 缓冲队列到 bxCAN TX 邮箱（主循环/任务周期调用）
 * @note  在 can_task 的 sw_timer 回调中调用
 */
void drv_can_tx_flush(drv_can_channel_t ch);

/**
 * @brief 查询 bxCAN TX 邮箱是否空闲（3 邮箱全空）
 */
bool drv_can_tx_all_done(drv_can_channel_t ch);

/* --- 接收（经 RX 队列） --- */

/**
 * @brief 出队一帧接收报文（主循环调用，单消费者）
 * @return true 成功；false 队列空
 * @note  接收帧由中断回调自动入队
 */
bool drv_can_rx_pop(drv_can_channel_t ch, drv_can_msg_t* msg);

/**
 * @brief 查询 RX 缓冲队列中的报文数
 */
uint32_t drv_can_rx_pending(drv_can_channel_t ch);

/**
 * @brief 清空 RX 接收缓冲队列
 */
void drv_can_rx_fifo_reset(drv_can_channel_t ch);

/* --- 接收回调（可选，与 RX 队列并存时同帧会被两处各处理一次） --- */

/**
 * @brief 注册接收回调（每通道独立注册）
 * @param ch       通道号
 * @param callback 回调函数（NULL=取消）
 * @note  回调在中断上下文中执行，应尽量简短。
 *        若使用 drv_can_rx_pop 队列消费，则无需注册本回调
 */
drv_can_error_t drv_can_register_rx_callback(drv_can_channel_t ch,
    drv_can_rx_callback_t callback);

/* --- Bus-Off 检测 / 自恢复 --- */

/**
 * @brief 查询通道是否处于 Bus-Off 状态
 */
bool drv_can_is_bus_off(drv_can_channel_t ch);

/**
 * @brief 从 Bus-Off 自动恢复（保留滤波器与接收回调）
 */
drv_can_error_t drv_can_recover(drv_can_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CAN_H */
