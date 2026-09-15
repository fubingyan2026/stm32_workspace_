//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_protocol.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU RS485 协议编解码：z 帧组包/解析、下行帧构建、数据段解码。
 * @note    纯计算模块，不依赖串口与系统调用，便于单元测试与移植。
 */

#ifndef __CTU_PROTOCOL_H
#define __CTU_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* public_layer 共享中间件：通用协议解析器（与固件 srv_com_* 同一实现） */
#include "protocol_parser.h"

#include "ctu_config.h"
#include "ctu_error.h"
#include "ctu_types.h"

/* Exported macro ------------------------------------------------------------*/

#define CTU_PROTOCOL_PARSER_INPUT_SIZE 512U                    /**< 输入 FIFO 容量 */
#define CTU_PROTOCOL_PARSER_OUTPUT_SIZE CTU_PROTOCOL_MAX_FRAME /**< 组帧输出缓冲 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 流式帧解析器上下文
 * @note  解析逻辑复用 public_layer 的 ``protocol_parser``（与固件端同一实现），
 *        本结构只提供静态存储与 SDK 风格的适配接口，避免重复实现。
 * @note  不使用 ``protocol_parser_tick()``，因此中间件的"空闲超时丢弃"不会触发，
 *        超时判定完全由 :mod:`ctu_transport` 的 deadline 负责。
 */
typedef struct {
    protocol_parser_context_t context;                      /**< 中间件解析器上下文 */
    uint8_t input_buffer[CTU_PROTOCOL_PARSER_INPUT_SIZE];   /**< 输入 FIFO 缓冲 */
    uint8_t output_buffer[CTU_PROTOCOL_PARSER_OUTPUT_SIZE]; /**< 完整帧输出缓冲 */
    uint32_t dropped;                                       /**< 丢弃的非法帧/字节计数 */
} ctu_protocol_parser_t;

/* Exported functions prototypes ---------------------------------------------*/

/* ---- 基础 ---- */

/**
 * @brief 计算 CRC8（多项式 0x31 反射形式 0x8C，初值 0xFF）
 * @param data   数据指针
 * @param length 数据长度
 * @return CRC8 值
 */
uint8_t ctu_protocol_crc8(const uint8_t* data, size_t length);

/**
 * @brief 计算 CRC16-XMODEM（多项式 0x1021，初值 0）
 * @param data   数据指针
 * @param length 数据长度
 * @return CRC16 值
 */
uint16_t ctu_protocol_crc16_xmodem(const uint8_t* data, size_t length);

/**
 * @brief 小端解包：无符号 16 位
 */
uint16_t ctu_protocol_u16_le(const uint8_t* data);

/**
 * @brief 小端解包：有符号 16 位
 */
int16_t ctu_protocol_i16_le(const uint8_t* data);

/**
 * @brief 小端解包：无符号 32 位
 */
uint32_t ctu_protocol_u32_le(const uint8_t* data);

/* ---- 组包 ---- */

/**
 * @brief 按协议打包一帧
 * @param cmd           命令码
 * @param payload       payload（首个字节约定为设备 ID），可为 NULL（长度需为 0）
 * @param payload_len   payload 长度（≤255）
 * @param out           输出缓冲区
 * @param out_capacity  输出缓冲区容量
 * @return 帧总长度；失败返回 0
 */
size_t ctu_protocol_build_frame(uint8_t cmd, const uint8_t* payload,
                                size_t payload_len, uint8_t* out,
                                size_t out_capacity);

/**
 * @brief 构建读系统状态帧（0x01）
 */
size_t ctu_protocol_build_read_status(uint8_t addr, uint8_t* out, size_t out_capacity);

/**
 * @brief 构建读电压帧（0x02）
 */
size_t ctu_protocol_build_read_volt(uint8_t addr, uint8_t* out, size_t out_capacity);

/**
 * @brief 构建读温度帧（0x03）
 */
size_t ctu_protocol_build_read_temp(uint8_t addr, uint8_t* out, size_t out_capacity);

/**
 * @brief 构建读固件信息帧（0x07）
 */
size_t ctu_protocol_build_read_info(uint8_t addr, uint8_t* out, size_t out_capacity);

/**
 * @brief 构建 MASTER 蜂鸣器控制帧（0x04）
 * @param duty 占空比 0-50（超出截断）
 */
size_t ctu_protocol_build_master_ctrl(uint8_t addr, uint8_t duty,
                                      uint8_t* out, size_t out_capacity);

/**
 * @brief 构建 SLAVER 输出控制帧（0x04）
 * @param mask      输出位域（bit0=24V, bit1=12V_ISO, bit2=LSD1, bit3=LSD2）
 * @param fill_duty 补光亮度 0-1000（超出截断）
 */
size_t ctu_protocol_build_slaver_ctrl(uint8_t addr, uint8_t mask,
                                      uint16_t fill_duty, uint8_t* out,
                                      size_t out_capacity);

/**
 * @brief 构建清除故障锁存帧（0x05）
 */
size_t ctu_protocol_build_reset_latch(uint8_t addr, uint8_t* out, size_t out_capacity);

/**
 * @brief 构建升级请求帧（0x06，App 请求升级 / Boot SELECT）
 */
size_t ctu_protocol_build_upgrade(uint8_t addr, uint8_t* out, size_t out_capacity);

/* ---- Boot 升级帧 ---- */

/**
 * @brief 构建 SELECT 帧（0x06）
 */
size_t ctu_protocol_build_boot_select(uint8_t addr, uint8_t* out, size_t out_capacity);

/**
 * @brief 构建 START 帧（0x08）
 */
size_t ctu_protocol_build_boot_start(uint8_t addr, uint32_t size,
                                     uint32_t checksum, uint8_t* out,
                                     size_t out_capacity);

/**
 * @brief 构建 DATA 帧（0x09）
 * @param block     块号
 * @param crc16     该块数据的 CRC16-XMODEM
 * @param chunk     块数据
 * @param chunk_len 块数据长度（≤ CTU_BOOT_DATA_MAX）
 */
size_t ctu_protocol_build_boot_data(uint8_t addr, uint16_t block, uint16_t crc16,
                                    const uint8_t* chunk, size_t chunk_len,
                                    uint8_t* out, size_t out_capacity);

/**
 * @brief 构建 END 帧（0x0A）
 */
size_t ctu_protocol_build_boot_end(uint8_t addr, uint8_t* out, size_t out_capacity);

/**
 * @brief 构建 ABORT 帧（0x0B）
 */
size_t ctu_protocol_build_boot_abort(uint8_t addr, uint8_t* out, size_t out_capacity);

/* ---- 解析 ---- */

/**
 * @brief 初始化帧解析器
 * @param parser 解析器上下文
 */
void ctu_protocol_parser_init(ctu_protocol_parser_t* parser);

/**
 * @brief 喂入接收数据（追加到内部缓存，超长时丢弃最旧字节）
 * @param parser 解析器上下文
 * @param data   数据指针
 * @param length 数据长度
 * @return 操作结果
 */
ctu_error_t ctu_protocol_parser_feed(ctu_protocol_parser_t* parser,
                                     const uint8_t* data, size_t length);

/**
 * @brief 取出一个已通过校验的完整帧
 * @param parser    解析器上下文
 * @param frame_out 输出帧缓冲
 * @param capacity  输出缓冲容量
 * @param frame_len 输出帧长度
 * @return true 表示取到帧，false 表示暂无完整帧
 */
bool ctu_protocol_parser_pop(ctu_protocol_parser_t* parser, uint8_t* frame_out,
                             size_t capacity, size_t* frame_len);

/* ---- 帧字段访问 ---- */

/**
 * @brief 取帧内设备 ID（payload 首字节）
 */
uint8_t ctu_protocol_frame_addr(const uint8_t* frame, size_t length);

/**
 * @brief 取帧命令码
 */
uint8_t ctu_protocol_frame_cmd(const uint8_t* frame, size_t length);

/**
 * @brief 取帧 payload 长度
 */
size_t ctu_protocol_frame_payload_len(const uint8_t* frame, size_t length);

/**
 * @brief 取帧 payload 指针（可直接当内容段使用，首字节为设备 ID）
 */
const uint8_t* ctu_protocol_frame_payload(const uint8_t* frame, size_t length);

/**
 * @brief 判断帧是否为错误应答（cmd == 0x7F）
 */
bool ctu_protocol_frame_is_error(const uint8_t* frame, size_t length);

/**
 * @brief 判断帧是否合法（长度、帧尾、CRC 校验通过）
 */
bool ctu_protocol_frame_is_valid(const uint8_t* frame, size_t length);

/**
 * @brief 把帧渲染为十六进制字符串
 * @param frame 帧
 * @param length 帧长
 * @param out   输出缓冲
 * @param capacity 输出容量
 * @return 写入的字符数（不含结尾 '\0'）
 */
size_t ctu_protocol_frame_to_hex(const uint8_t* frame, size_t length,
                                 char* out, size_t capacity);

/**
 * @brief 命令码可读名称
 * @param cmd 命令码（可含应答标志）
 * @return 静态字符串
 */
const char* ctu_protocol_cmd_name(uint8_t cmd);

/**
 * @brief 设备错误码可读名称
 */
const char* ctu_protocol_dev_err_str(uint8_t code);

/**
 * @brief Boot 错误码可读名称
 */
const char* ctu_protocol_boot_err_str(uint8_t code);

/* ---- 数据段解码（入参为去掉设备 ID 后的内容段）---- */

/**
 * @brief 解码 MASTER 状态（0x01 应答，内容段 2B）
 */
ctu_error_t ctu_protocol_decode_master_status(const uint8_t* data, size_t length,
                                              ctu_master_status_t* out);

/**
 * @brief 解码 SLAVER 状态（0x01 应答，内容段 2B）
 */
ctu_error_t ctu_protocol_decode_slaver_status(const uint8_t* data, size_t length,
                                              ctu_slaver_status_t* out);

/**
 * @brief 解码 MASTER 电压（0x02 应答，内容段 4B）
 */
ctu_error_t ctu_protocol_decode_master_voltage(const uint8_t* data, size_t length,
                                               ctu_master_voltage_t* out);

/**
 * @brief 解码 SLAVER 电压（0x02 应答，内容段 8B）
 */
ctu_error_t ctu_protocol_decode_slaver_voltage(const uint8_t* data, size_t length,
                                               ctu_slaver_voltage_t* out);

/**
 * @brief 解码 MASTER 温度（0x03 应答，内容段 6B）
 */
ctu_error_t ctu_protocol_decode_master_temp(const uint8_t* data, size_t length,
                                            ctu_master_temp_t* out);

/**
 * @brief 解码 SLAVER 温度 / VDDA（0x03 应答，内容段 4B）
 */
ctu_error_t ctu_protocol_decode_slaver_temp(const uint8_t* data, size_t length,
                                            ctu_slaver_temp_t* out);

/**
 * @brief 解码固件信息（0x07 应答，内容段 15B）
 */
ctu_error_t ctu_protocol_decode_fw_info(const uint8_t* data, size_t length,
                                        ctu_fw_info_t* out);

/* ---- 便捷判断 ---- */

/**
 * @brief MASTER 状态是否存在异常
 * @param status 状态
 * @return true 表示存在异常
 */
bool ctu_master_status_has_fault(const ctu_master_status_t* status);

/**
 * @brief SLAVER 状态是否存在异常
 * @param status 状态
 * @return true 表示存在异常
 */
bool ctu_slaver_status_has_fault(const ctu_slaver_status_t* status);

/**
 * @brief 由 SLAVER 状态组合输出位域
 * @param status 状态
 * @return 位域（bit0=24V, bit1=12V_ISO, bit2=LSD1, bit3=LSD2）
 */
uint8_t ctu_slaver_status_output_mask(const ctu_slaver_status_t* status);

#ifdef __cplusplus
}
#endif

#endif /* __CTU_PROTOCOL_H */
