/**
 * @file    drv_can.h
 * @author  maximillian
 * @version V2.2.0
 * @date    2026-07-09
 * @brief   CAN 设备驱动（STM32G474 FDCAN，支持经典 CAN 和 CAN FD）
 * @attention
 *
 * CubeMX 配置 FDCAN1 (PA11 RX / PA12 TX) 与 FDCAN2 (PB12 RX / PB13 TX)。
 * 支持经典 CAN 和 CAN FD 帧格式。
 * drv_can_msg_t.dlc 始终为实际字节数（0-64），driver 内部自动与 FDCAN DLC 编码互转。
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
    DRV_CAN_CH_1 = 0, /**< FDCAN1 — PA11(RX) / PA12(TX) */
    DRV_CAN_CH_2, /**< FDCAN2 — PB12(RX) / PB13(TX) */
    DRV_CAN_CH_NUM, /**< 通道总数 */
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
 * @brief CAN 报文结构
 *
 * 同时支持经典 CAN 和 CAN FD。
 * dlc 始终为实际字节数（0~64），非 FDCAN DLC 编码值。
 */
typedef struct {
    uint32_t id; /**< CAN ID（标准 11-bit 或扩展 29-bit） */
    bool is_extended; /**< true=扩展帧 */
    bool is_fd; /**< true=CAN FD 帧 */
    uint8_t dlc; /**< 数据长度（字节数，0-64） */
    uint8_t data[64]; /**< 数据负载 */
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
 * @return DRV_CAN_ERROR_TX_BUSY 表示 Tx FIFO 满
 */
drv_can_error_t drv_can_send(drv_can_channel_t ch, const drv_can_msg_t* msg);

/**
 * @brief 查询 Tx FIFO 是否有空闲位置
 */
bool drv_can_tx_ready(drv_can_channel_t ch);

/* --- 状态监控 --- */

/**
 * @brief 轮询总线协议状态（主循环/任务周期调用，建议 ≥10ms 一次）
 * @param ch 通道号
 * @note  Bus-Off 时打印告警并自动触发恢复序列；
 *        Error-Passive 状态跳变时打印告警（无 ACK / 位错误累积的早期信号）
 */
void drv_can_poll_status(drv_can_channel_t ch);

/* --- 接收回调 --- */

/**
 * @brief 注册接收回调（每通道独立注册）
 * @param ch       通道号
 * @param callback 回调函数（NULL=取消）
 * @note  回调在中断上下文中执行，应尽量简短。
 *        若改用 RX 缓冲队列（drv_can_rx_pop）消费，则无需注册本回调，
 *        二者并存时同帧会被两处各处理一次
 */
drv_can_error_t drv_can_register_rx_callback(drv_can_channel_t ch,
    drv_can_rx_callback_t callback);

/* --- 消息缓冲队列（基于 msg_fifo，ISR 单生产者 / 主循环单消费者） --- */

/**
 * @brief 出队一帧接收报文（主循环调用，单消费者）
 * @param ch  通道号
 * @param msg 接收缓冲区（非空）
 * @return true 成功；false 队列空或参数非法
 * @note  接收帧由中断回调 HAL_FDCAN_RxFifo0Callback 自动入队，
 *        与旧接收回调并存（两者都消费同一帧时注意去重）
 */
bool drv_can_rx_pop(drv_can_channel_t ch, drv_can_msg_t* msg);

/**
 * @brief 查询接收缓冲队列中的报文数
 * @param ch 通道号
 * @return 队列内完整报文数；未初始化/参数非法返回 0
 */
uint32_t drv_can_rx_pending(drv_can_channel_t ch);

/**
 * @brief 清空接收缓冲队列（主循环调用，避免与中断入队并发时使用）
 * @param ch 通道号
 */
void drv_can_rx_fifo_reset(drv_can_channel_t ch);

/**
 * @brief 入队一帧待发送报文（主循环写入，单生产者）
 * @param ch  通道号
 * @param msg 待发送报文
 * @return DRV_CAN_OK 入队成功；
 *         DRV_CAN_ERROR_TX_BUSY 队列满（应降低发送频率或扩大队列）
 * @note  队列中的帧由周期调用的 drv_can_tx_flush() 排空到 FDCAN Tx FIFO，
 *        不受硬件 Tx FIFO 满限制，适合突发批量发送
 */
drv_can_error_t drv_can_tx_enqueue(drv_can_channel_t ch, const drv_can_msg_t* msg);

/**
 * @brief 查询待发送缓冲队列中的报文数
 * @param ch 通道号
 * @return 队列内完整报文数；未初始化/参数非法返回 0
 */
uint32_t drv_can_tx_pending(drv_can_channel_t ch);

/**
 * @brief 排空待发送缓冲队列到 FDCAN Tx FIFO（主循环/任务周期调用）
 * @param ch 通道号
 * @note  drv_can_poll_status() 内部已自动调用本函数；
 *        需要更及时发送时可在发送密集任务里额外调用
 */
void drv_can_tx_flush(drv_can_channel_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CAN_H */
