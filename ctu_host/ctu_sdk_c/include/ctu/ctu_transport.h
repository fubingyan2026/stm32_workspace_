//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_transport.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   串口传输层：Linux termios 收发 + z 帧流式解析（同步阻塞风格）。
 */

#ifndef __CTU_TRANSPORT_H
#define __CTU_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ctu_config.h"
#include "ctu_error.h"
#include "ctu_protocol.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 失败原因文本最大长度
 */
#define CTU_TRANSPORT_ERROR_TEXT_MAX 128U

/**
 * @brief 传输层配置
 */
typedef struct {
    uint32_t write_timeout_ms; /**< 单次写超时（ms），0 表示使用默认值 */
    uint32_t poll_slice_ms;    /**< 内部轮询切片（ms），0 表示使用默认值 */
} ctu_transport_config_t;

/**
 * @brief 串口传输层上下文
 */
typedef struct ctu_transport ctu_transport_t;

struct ctu_transport {
    ctu_transport_config_t config;     /**< 配置参数 */
    int fd;                            /**< 串口文件描述符，-1 表示未打开 */
    char port[CTU_PORT_NAME_MAX];      /**< 串口设备名（已解析为绝对路径） */
    uint32_t baud;                     /**< 波特率 */
    ctu_protocol_parser_t parser;      /**< 接收帧解析器 */
    uint32_t rx_frames;                /**< 统计：已解析帧数 */
    int last_errno;                    /**< 最近一次失败的系统错误码（0=无） */
    char last_error[CTU_TRANSPORT_ERROR_TEXT_MAX]; /**< 最近一次失败的可读描述 */
    bool opened;                       /**< 是否已打开 */
    bool initialized;                  /**< 是否已初始化 */
};

/**
 * @brief 串口枚举回调
 * @param name 设备名（如 /dev/ttyUSB0）
 * @param user 用户上下文
 */
typedef void (*ctu_port_cb_t)(const char* name, void* user);

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化传输层
 * @param transport 传输层上下文
 * @param config    配置（可为 NULL，使用默认值）
 * @return 操作结果
 */
ctu_error_t ctu_transport_init(ctu_transport_t* transport,
                               const ctu_transport_config_t* config);

/**
 * @brief 反初始化（同时关闭串口）
 * @param transport 传输层上下文
 */
void ctu_transport_deinit(ctu_transport_t* transport);

/**
 * @brief 查询是否已初始化
 */
bool ctu_transport_is_initialized(const ctu_transport_t* transport);

/**
 * @brief 打开并配置串口（8N1，原始模式，非阻塞）
 * @param transport 传输层上下文
 * @param port      设备名（如 /dev/ttyUSB0）
 * @param baud      波特率（如 115200）
 * @return 操作结果
 */
ctu_error_t ctu_transport_open(ctu_transport_t* transport, const char* port,
                               uint32_t baud);

/**
 * @brief 关闭串口
 */
void ctu_transport_close(ctu_transport_t* transport);

/**
 * @brief 查询串口是否已打开
 */
bool ctu_transport_is_open(const ctu_transport_t* transport);

/**
 * @brief 获取当前串口设备名
 * @return 静态字符串（未打开返回空串）
 */
const char* ctu_transport_port_name(const ctu_transport_t* transport);

/**
 * @brief 取最近一次操作失败的系统错误码
 * @return errno 值（0 表示无错误）
 */
int ctu_transport_last_errno(const ctu_transport_t* transport);

/**
 * @brief 取最近一次操作失败的可读描述
 * @return 例如 "Permission denied"、"No such file or directory"；无错误返回空串
 * @note  若因权限失败，描述后会附带加入 dialout 组的提示
 */
const char* ctu_transport_last_error_text(const ctu_transport_t* transport);

/**
 * @brief 发送一段数据（含完整帧）
 * @param transport  传输层上下文
 * @param data       数据指针
 * @param length     数据长度
 * @param timeout_ms 写超时（0 表示使用配置值）
 * @return 操作结果
 */
ctu_error_t ctu_transport_write(ctu_transport_t* transport, const uint8_t* data,
                                size_t length, uint32_t timeout_ms);

/**
 * @brief 清空接收方向滞留数据与解析缓存
 * @param transport 传输层上下文
 * @return 操作结果
 */
ctu_error_t ctu_transport_flush_input(ctu_transport_t* transport);

/**
 * @brief 在超时时间内等待一个已通过校验的帧
 * @param transport  传输层上下文
 * @param frame_out  输出帧缓冲
 * @param capacity   输出缓冲容量
 * @param frame_len  输出帧长度
 * @param timeout_ms 超时（ms）
 * @return CTU_OK 取到帧；CTU_ERROR_TIMEOUT 超时；其它为错误
 */
ctu_error_t ctu_transport_read_frame(ctu_transport_t* transport, uint8_t* frame_out,
                                     size_t capacity, size_t* frame_len,
                                     uint32_t timeout_ms);

/**
 * @brief 校验波特率是否受支持
 */
bool ctu_transport_baud_is_supported(uint32_t baud);

/**
 * @brief 枚举本机常见串口设备（/dev/ttyUSB*、/dev/ttyACM*、/dev/ttyS*）
 * @param callback 回调
 * @param user     用户上下文
 * @return 操作结果
 */
ctu_error_t ctu_transport_foreach_port(ctu_port_cb_t callback, void* user);

#ifdef __cplusplus
}
#endif

#endif /* __CTU_TRANSPORT_H */
