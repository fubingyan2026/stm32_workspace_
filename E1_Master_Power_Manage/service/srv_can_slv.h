/**
 * @file    srv_can_slv.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   从电源板控制服务 — 0x002 协议 + ACK 重试
 * @attention
 *
 * 遵循 service 层回调注入模式，不直接调用硬件驱动。
 *
 * ## 协议（详见 docs/protocol_slaver.md）
 * - 主电源板 → 从电源板：0x002（HSD1_12V 控制 + 保留位）
 * - 从电源板 → 主电源板：0x002 ACK（8 字节任意内容，仅用于握手确认）
 * - 50ms 超时未收到 ACK 则重发
 *
 * ## 用法
 * @code
 *   srv_can_slv_config_t cfg = {
 *       .send_frame = my_send_frame,
 *       .get_ctrl   = my_get_ctrl,
 *   };
 *   srv_can_slv_init(&cfg);
 *   srv_can_slv_request();        // 请求发送控制帧
 *   srv_can_slv_task();            // 周期处理重试
 *   srv_can_slv_process_rx(id, data, len);  // 收到 CAN 帧时调用
 * @endcode
 */

#ifndef __SRV_CAN_SLV_H
#define __SRV_CAN_SLV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief 0x002 CAN ID — 从板控制 */
#define SRV_CAN_SLV_ID_CTRL (0x002U)

/** @brief 帧格式常量 */
#define SRV_CAN_SLV_FRAME_LEN (6U) /**< 控制帧长度（字节） */
#define SRV_CAN_SLV_ACK_LEN (8U) /**< ACK 帧长度（字节） */
#define SRV_CAN_SLV_RETRY_MS (50U) /**< 无 ACK 重试间隔（ms） */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 从板控制数据
 *
 * task 层通过 get_ctrl 回调填充此结构，service 打包为 0x002 帧发送。
 */
typedef struct {
    bool hsd1_12v_on; /**< HSD1_12V 输出：true=开, false=关 */
    bool reserved_channel; /**< 保留 */
} srv_can_slv_ctrl_t;

/**
 * @brief CAN 发送回调
 * @param can_id CAN ID
 * @param data   帧数据
 * @param len    数据长度
 * @return true=发送成功, false=忙
 */
typedef bool (*srv_can_slv_send_cb_t)(uint16_t can_id,
    const uint8_t* data, uint8_t len);

/**
 * @brief 读取当前控制状态回调
 * @param ctrl 待填充的控制数据体
 */
typedef void (*srv_can_slv_read_ctrl_cb_t)(srv_can_slv_ctrl_t* ctrl);

/** @brief 服务配置 */
typedef struct {
    srv_can_slv_send_cb_t send_frame; /**< CAN 发送回调（必填） */
    srv_can_slv_read_ctrl_cb_t get_ctrl; /**< 控制状态读取回调（必填） */
} srv_can_slv_config_t;

typedef enum {
    SRV_CAN_SLV_OK = 0,
    SRV_CAN_SLV_ERROR_NULL_PTR,
    SRV_CAN_SLV_ERROR_UNINITIALIZED,
} srv_can_slv_error_t;

/* Exported functions prototypes ---------------------------------------------*/

srv_can_slv_error_t srv_can_slv_init(const srv_can_slv_config_t* config);
void srv_can_slv_deinit(void);
bool srv_can_slv_is_initialized(void);

/** @brief 请求发送一次从板控制帧 */
void srv_can_slv_request(void);

/** @brief 周期任务：重试 pending 帧，发送新请求 */
void srv_can_slv_task(void);

/**
 * @brief 处理接收到的 CAN 帧（由 can_task 调用）
 * @param can_id 接收到的 CAN ID
 * @param data   帧数据
 * @param len    帧长度
 * @note  当收到 0x002 且长度为 8 且 pending 处于等待 ACK 状态时，视为 ACK
 */
void srv_can_slv_process_rx(uint32_t can_id, const uint8_t* data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_CAN_SLV_H */
