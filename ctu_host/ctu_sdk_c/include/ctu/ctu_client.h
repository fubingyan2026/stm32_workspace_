//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_client.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU 上位机客户端：覆盖查询 / 控制 / 清除锁存 / 升级的全部功能。
 */

#ifndef __CTU_CLIENT_H
#define __CTU_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ctu_boot.h"
#include "ctu_config.h"
#include "ctu_error.h"
#include "ctu_transport.h"
#include "ctu_types.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 客户端配置（含回调）
 */
typedef struct {
    uint32_t timeout_ms;              /**< 单次请求超时（ms），0 使用默认 200ms */
    ctu_transport_config_t transport; /**< 传输层配置 */
    ctu_boot_config_t boot;           /**< 升级配置 */
    ctu_log_cb_t log_cb;              /**< 日志回调（可为 NULL） */
    void* user;                       /**< 回调用户上下文 */
} ctu_client_config_t;

/**
 * @brief 客户端上下文
 */
typedef struct ctu_client ctu_client_t;

struct ctu_client {
    ctu_client_config_t config;     /**< 配置参数 */
    ctu_transport_t transport;      /**< 传输层上下文 */
    ctu_boot_context_t boot;        /**< 升级上下文 */
    volatile int cancel_requested;  /**< 取消标志（可跨线程 / 信号处理置位） */
    uint8_t last_device_err;        /**< 最近一次 0x7F 错误应答码 */
    bool initialized;               /**< 是否已初始化 */
};

/* Exported functions prototypes ---------------------------------------------*/

/* ---- 生命周期 ---- */

/**
 * @brief 初始化客户端（不打开串口）
 * @param client 客户端上下文
 * @param config 配置（可为 NULL，使用默认值）
 * @return 操作结果
 */
ctu_error_t ctu_client_init(ctu_client_t* client, const ctu_client_config_t* config);

/**
 * @brief 反初始化（同时关闭串口）
 */
void ctu_client_deinit(ctu_client_t* client);

/**
 * @brief 查询是否已初始化
 */
bool ctu_client_is_initialized(const ctu_client_t* client);

/**
 * @brief 打开串口
 * @param client 客户端上下文
 * @param port   设备名（如 /dev/ttyUSB0）
 * @param baud   波特率（如 115200）
 * @return 操作结果
 */
ctu_error_t ctu_client_open(ctu_client_t* client, const char* port, uint32_t baud);

/**
 * @brief 关闭串口
 */
void ctu_client_close(ctu_client_t* client);

/**
 * @brief 查询串口是否已打开
 */
bool ctu_client_is_open(const ctu_client_t* client);

/**
 * @brief 获取串口设备名
 */
const char* ctu_client_port_name(const ctu_client_t* client);

/**
 * @brief 请求取消正在进行的升级（可从其它线程或信号处理函数调用）
 */
void ctu_client_request_cancel(ctu_client_t* client);

/**
 * @brief 清除取消标志
 */
void ctu_client_clear_cancel(ctu_client_t* client);

/**
 * @brief 取最近一次设备错误码（0x7F 应答）
 */
uint8_t ctu_client_last_device_error(const ctu_client_t* client);

/**
 * @brief 取最近一次系统错误码（打开串口失败等）
 * @return errno 值（0 表示无错误）
 */
int ctu_client_last_errno(const ctu_client_t* client);

/**
 * @brief 取最近一次系统错误描述（如 "Permission denied"）
 */
const char* ctu_client_last_error_text(const ctu_client_t* client);

/* ---- 通用事务 ---- */

/**
 * @brief 发送一帧并等待指定命令的应答
 * @param client          客户端
 * @param frame           已组好的完整帧
 * @param frame_len       帧长度
 * @param device          期望应答的设备
 * @param expected_cmd    期望的命令码
 * @param content         应答内容段输出缓冲（可为 NULL）
 * @param content_capacity content 容量
 * @param content_len     输出内容段长度（可为 NULL）
 * @param elapsed_ms      输出往返耗时（可为 NULL）
 * @return CTU_OK 成功；CTU_ERROR_TIMEOUT 超时；CTU_ERROR_DEVICE 设备错误应答；
 *         CTU_ERROR_UNEXPECTED_REPLY 命令不符；其它为错误
 */
ctu_error_t ctu_client_request(ctu_client_t* client, const uint8_t* frame,
                               size_t frame_len, ctu_device_t device,
                               uint8_t expected_cmd, uint8_t* content,
                               size_t content_capacity, size_t* content_len,
                               uint32_t* elapsed_ms);

/* ---- 查询 ---- */

/** @brief 读 MASTER 系统状态（0x01） */
ctu_error_t ctu_client_read_master_status(ctu_client_t* client,
                                          ctu_master_status_t* out);

/** @brief 读 SLAVER 系统状态（0x01） */
ctu_error_t ctu_client_read_slaver_status(ctu_client_t* client,
                                          ctu_slaver_status_t* out);

/** @brief 读 MASTER 电压（0x02，mV） */
ctu_error_t ctu_client_read_master_voltage(ctu_client_t* client,
                                           ctu_master_voltage_t* out);

/** @brief 读 SLAVER 电压（0x02，mV） */
ctu_error_t ctu_client_read_slaver_voltage(ctu_client_t* client,
                                           ctu_slaver_voltage_t* out);

/** @brief 读 MASTER 温度（0x03，°C） */
ctu_error_t ctu_client_read_master_temp(ctu_client_t* client,
                                        ctu_master_temp_t* out);

/** @brief 读 SLAVER 温度 / VDDA（0x03） */
ctu_error_t ctu_client_read_slaver_temp(ctu_client_t* client,
                                        ctu_slaver_temp_t* out);

/** @brief 读固件 / Boot 信息（0x07） */
ctu_error_t ctu_client_read_info(ctu_client_t* client, ctu_device_t device,
                                 ctu_fw_info_t* out);

/* ---- 控制 ---- */

/**
 * @brief 设置 MASTER 蜂鸣器占空比（0x04，0-50%，超限截断）
 */
ctu_error_t ctu_client_set_buzzer_duty(ctu_client_t* client, uint8_t duty,
                                       ctu_ack_t* ack);

/**
 * @brief 设置 SLAVER 输出（0x04）
 * @param mask      输出位域（bit0=24V, bit1=12V_ISO, bit2=LSD1, bit3=LSD2）
 * @param fill_duty 补光亮度 0-1000（超限截断）
 */
ctu_error_t ctu_client_set_outputs(ctu_client_t* client, uint8_t mask,
                                   uint16_t fill_duty, ctu_ack_t* ack);

/**
 * @brief 清除故障锁存（0x05）
 */
ctu_error_t ctu_client_clear_fault_latch(ctu_client_t* client, ctu_device_t device,
                                         ctu_ack_t* ack);

/**
 * @brief 发送升级请求（0x06，板端 ACK 后复位进入 Bootloader）
 */
ctu_error_t ctu_client_request_upgrade(ctu_client_t* client, ctu_device_t device,
                                       ctu_ack_t* ack);

/* ---- 探测 ---- */

/**
 * @brief 探测设备是否在线（读一次状态）
 * @param online 输出是否在线（可为 NULL）
 * @return CTU_OK 表示探测动作完成（在线与否见 online）
 */
ctu_error_t ctu_client_probe(ctu_client_t* client, ctu_device_t device, bool* online);

/* ---- 升级 ---- */

/**
 * @brief 执行完整固件升级（0x06 邀请 + SELECT/START/DATA/END）
 * @param client   客户端（需已打开串口）
 * @param device   目标设备
 * @param filepath 固件 .bin 路径
 * @return 操作结果
 */
ctu_error_t ctu_client_upgrade(ctu_client_t* client, ctu_device_t device,
                               const char* filepath);

#ifdef __cplusplus
}
#endif

#endif /* __CTU_CLIENT_H */
