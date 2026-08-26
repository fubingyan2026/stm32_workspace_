/**
 * @file    srv_can_bus.h
 * @brief   CAN 总线抽象 — 收发接口回调注入，通道选择在接线处完成
 *
 * 服务层不直接出现 drv_can 通道号：服务持有 srv_can_bus_t 句柄，
 * 发送经 srv_can_bus_send()（内部封装 drv_can_send(bus->channel, ...)），
 * 接收由 srv_can_bus_bind() 注册的 rx_handler 回调消费（ISR 上下文）。
 * 换 CAN 通道只需改 can_task 接线处的 srv_can_bus_bind() 调用。
 *
 * srv_can_bus_init() 会占用每通道唯一的 drv_can 接收回调槽位（通道级分发），
 * 适用于「每个通道恰有一个服务拥有者」的拓扑。
 */

#ifndef __SRV_CAN_BUS_H
#define __SRV_CAN_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "drv_can.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 总线接收处理函数（ISR 上下文调用，须简短、不打日志）
 * @param msg        CAN 报文指针
 * @param user_data  srv_can_bus_bind 传入的用户上下文
 */
typedef void (*srv_can_bus_rx_handler_t)(const drv_can_msg_t* msg, void* user_data);

/**
 * @brief CAN 总线句柄
 * @note  由调用方（can_task 接线处）静态分配，srv_can_bus_bind 初始化，
 *        服务模块只读使用 channel 与计数，不修改。
 */
typedef struct {
    drv_can_channel_t channel;            /**< 绑定的 CAN 通道（接线时选择） */
    srv_can_bus_rx_handler_t rx_handler;  /**< 服务注册的接收处理回调 */
    void* rx_user_data;                   /**< 接收回调用户上下文 */
    uint32_t tx_cnt;                      /**< 发送成功计数 */
    uint32_t rx_cnt;                      /**< 接收帧计数 */
} srv_can_bus_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 CAN 总线分发（对全部通道注册唯一的通道级接收回调）
 * @note  须在 drv_can_init() 之后、绑定/发送之前调用一次；
 *        内部通过 drv_can_register_rx_callback 占用每通道回调槽位
 */
void srv_can_bus_init(void);

/**
 * @brief 绑定总线句柄到指定通道并注册接收处理回调
 * @param bus       总线句柄（调用方静态分配）
 * @param ch        目标 CAN 通道
 * @param rx        接收处理函数（NULL=仅发送不接收）
 * @param user_data 回调用户上下文
 * @return 操作结果错误码
 */
drv_can_error_t srv_can_bus_bind(srv_can_bus_t* bus,
    drv_can_channel_t ch,
    srv_can_bus_rx_handler_t rx,
    void* user_data);

/**
 * @brief 经绑定的通道发送 CAN 报文（非阻塞）
 * @param bus 总线句柄
 * @param msg CAN 报文指针
 * @return 操作结果错误码
 */
drv_can_error_t srv_can_bus_send(const srv_can_bus_t* bus, const drv_can_msg_t* msg);

/**
 * @brief 查询绑定通道的 Tx FIFO 是否有空闲位置
 * @param bus 总线句柄
 * @return true=可发送
 */
bool srv_can_bus_tx_ready(const srv_can_bus_t* bus);

/**
 * @brief 轮询绑定通道的总线协议状态（主循环/任务周期调用，建议 ≥1ms）
 * @param bus 总线句柄
 * @note  Bus-Off 恢复 + 错误状态告警（封装 drv_can_poll_status）
 */
void srv_can_bus_poll_status(const srv_can_bus_t* bus);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_CAN_BUS_H */
