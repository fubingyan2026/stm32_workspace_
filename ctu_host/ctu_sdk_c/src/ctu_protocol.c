//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_protocol.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU RS485 协议编解码实现。
 *
 * 帧格式：[0x7A][cmd][data_len][payload][CRC8][0x0A]
 *   - payload[0] = 设备 ID（下行=目标地址，上行=源地址）
 *   - CRC8：多项式 0x31（反射形式 0x8C），初值 0xFF，覆盖 帧头 ~ payload
 *   - 帧总长 = data_len + 5；多字节小端
 */

/* Includes ------------------------------------------------------------------*/
#include "ctu/ctu_protocol.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

#define CTU_BOOT_CRC16_POLY 0x1021U /**< CRC16-XMODEM 多项式 */

#define CTU_PROTOCOL_HEADER_LEN 1U                 /**< 帧头长度（0x7A） */
#define CTU_PROTOCOL_FOOTER_LEN 1U                 /**< 帧尾长度（0x0A） */
#define CTU_PROTOCOL_LEN_FIELD_MIN 3U              /**< 至少需要 header+cmd+len */
#define CTU_PROTOCOL_PARSER_MAX_ROUNDS 64U         /**< 单次 pop 的最大重试轮数 */

/* Private variables ---------------------------------------------------------*/

static const uint8_t s_ctu_protocol_header[CTU_PROTOCOL_HEADER_LEN] = {
    CTU_PROTOCOL_HEADER,
};
static const uint8_t s_ctu_protocol_footer[CTU_PROTOCOL_FOOTER_LEN] = {
    CTU_PROTOCOL_FOOT,
};

/* Private function prototypes -----------------------------------------------*/

static const char* ctu_protocol_boot_msg(uint8_t code);
static const char* ctu_protocol_dev_msg(uint8_t code);
static uint16_t ctu_protocol_parser_get_len(uint8_t* buffer, uint16_t len);
static protocol_parser_error_t ctu_protocol_parser_check(uint8_t* buffer, uint16_t len);

/* Exported functions --------------------------------------------------------*/

uint8_t ctu_protocol_crc8(const uint8_t* data, size_t length)
{
    uint8_t crc = 0xFFU;

    if (data == NULL) {
        return crc;
    }

    for (size_t i = 0U; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x01U) != 0U) {
                crc = (uint8_t)((crc >> 1U) ^ 0x8CU);
            } else {
                crc = (uint8_t)(crc >> 1U);
            }
        }
    }

    return crc;
}

uint16_t ctu_protocol_crc16_xmodem(const uint8_t* data, size_t length)
{
    uint16_t crc = 0x0000U;

    if (data == NULL) {
        return crc;
    }

    for (size_t i = 0U; i < length; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8U);
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((uint16_t)(crc << 1U) ^ CTU_BOOT_CRC16_POLY);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}

uint16_t ctu_protocol_u16_le(const uint8_t* data)
{
    if (data == NULL) {
        return 0U;
    }
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

int16_t ctu_protocol_i16_le(const uint8_t* data)
{
    if (data == NULL) {
        return 0;
    }
    return (int16_t)ctu_protocol_u16_le(data);
}

uint32_t ctu_protocol_u32_le(const uint8_t* data)
{
    if (data == NULL) {
        return 0U;
    }
    return ((uint32_t)data[0]) | ((uint32_t)data[1] << 8U)
        | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

size_t ctu_protocol_build_frame(uint8_t cmd, const uint8_t* payload,
                                size_t payload_len, uint8_t* out,
                                size_t out_capacity)
{
    if (out == NULL) {
        return 0U;
    }
    if (payload_len > CTU_PROTOCOL_MAX_PAYLOAD) {
        return 0U;
    }
    if ((payload == NULL) && (payload_len > 0U)) {
        return 0U;
    }
    if (out_capacity < (payload_len + CTU_PROTOCOL_OVERHEAD)) {
        return 0U;
    }

    out[0] = CTU_PROTOCOL_HEADER;
    out[1] = cmd;
    out[2] = (uint8_t)payload_len;
    if (payload_len > 0U) {
        memcpy(&out[3], payload, payload_len);
    }

    out[3U + payload_len] = ctu_protocol_crc8(out, 3U + payload_len);
    out[4U + payload_len] = CTU_PROTOCOL_FOOT;

    return payload_len + CTU_PROTOCOL_OVERHEAD;
}

size_t ctu_protocol_build_read_status(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    return ctu_protocol_build_frame((uint8_t)CTU_CMD_READ_STATUS, &addr, 1U, out, out_capacity);
}

size_t ctu_protocol_build_read_volt(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    return ctu_protocol_build_frame((uint8_t)CTU_CMD_READ_VOLT, &addr, 1U, out, out_capacity);
}

size_t ctu_protocol_build_read_temp(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    return ctu_protocol_build_frame((uint8_t)CTU_CMD_READ_TEMP, &addr, 1U, out, out_capacity);
}

size_t ctu_protocol_build_read_info(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    return ctu_protocol_build_frame((uint8_t)CTU_CMD_READ_INFO, &addr, 1U, out, out_capacity);
}

size_t ctu_protocol_build_master_ctrl(uint8_t addr, uint8_t duty,
                                      uint8_t* out, size_t out_capacity)
{
    uint8_t payload[2];

    if (duty > 50U) {
        duty = 50U; /* 超限截断 */
    }
    payload[0] = addr;
    payload[1] = duty;

    return ctu_protocol_build_frame((uint8_t)CTU_CMD_CTRL, payload, sizeof(payload),
                                    out, out_capacity);
}

size_t ctu_protocol_build_slaver_ctrl(uint8_t addr, uint8_t mask,
                                      uint16_t fill_duty, uint8_t* out,
                                      size_t out_capacity)
{
    uint8_t payload[5];

    if (fill_duty > 1000U) {
        fill_duty = 1000U; /* 超限截断 */
    }
    payload[0] = addr;
    payload[1] = (uint8_t)(mask & 0x0FU);
    payload[2] = 0x00U;
    payload[3] = (uint8_t)(fill_duty & 0xFFU);
    payload[4] = (uint8_t)((fill_duty >> 8U) & 0xFFU);

    return ctu_protocol_build_frame((uint8_t)CTU_CMD_CTRL, payload, sizeof(payload),
                                    out, out_capacity);
}

size_t ctu_protocol_build_reset_latch(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    uint8_t payload[2];

    payload[0] = addr;
    payload[1] = 0x01U;

    return ctu_protocol_build_frame((uint8_t)CTU_CMD_RESET_LATCH, payload,
                                    sizeof(payload), out, out_capacity);
}

size_t ctu_protocol_build_upgrade(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    uint8_t payload[2];

    payload[0] = addr;
    payload[1] = 0x01U;

    return ctu_protocol_build_frame((uint8_t)CTU_CMD_UPGRADE, payload,
                                    sizeof(payload), out, out_capacity);
}

size_t ctu_protocol_build_boot_select(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    return ctu_protocol_build_upgrade(addr, out, out_capacity);
}

size_t ctu_protocol_build_boot_start(uint8_t addr, uint32_t size,
                                     uint32_t checksum, uint8_t* out,
                                     size_t out_capacity)
{
    uint8_t payload[9];

    payload[0] = addr;
    payload[1] = (uint8_t)(size & 0xFFU);
    payload[2] = (uint8_t)((size >> 8U) & 0xFFU);
    payload[3] = (uint8_t)((size >> 16U) & 0xFFU);
    payload[4] = (uint8_t)((size >> 24U) & 0xFFU);
    payload[5] = (uint8_t)(checksum & 0xFFU);
    payload[6] = (uint8_t)((checksum >> 8U) & 0xFFU);
    payload[7] = (uint8_t)((checksum >> 16U) & 0xFFU);
    payload[8] = (uint8_t)((checksum >> 24U) & 0xFFU);

    return ctu_protocol_build_frame(0x08U, payload, sizeof(payload), out, out_capacity);
}

size_t ctu_protocol_build_boot_data(uint8_t addr, uint16_t block, uint16_t crc16,
                                    const uint8_t* chunk, size_t chunk_len,
                                    uint8_t* out, size_t out_capacity)
{
    /* addr(1) + 块号(2) + CRC16(2) + 数据(≤248) = 253 */
    uint8_t payload[CTU_BOOT_DATA_MAX + 5U];

    if (chunk == NULL) {
        return 0U;
    }
    if (chunk_len > CTU_BOOT_DATA_MAX) {
        return 0U;
    }

    payload[0] = addr;
    payload[1] = (uint8_t)(block & 0xFFU);
    payload[2] = (uint8_t)((block >> 8U) & 0xFFU);
    payload[3] = (uint8_t)(crc16 & 0xFFU);
    payload[4] = (uint8_t)((crc16 >> 8U) & 0xFFU);
    memcpy(&payload[5], chunk, chunk_len);

    return ctu_protocol_build_frame(0x09U, payload, chunk_len + 5U, out, out_capacity);
}

size_t ctu_protocol_build_boot_end(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    return ctu_protocol_build_frame(0x0AU, &addr, 1U, out, out_capacity);
}

size_t ctu_protocol_build_boot_abort(uint8_t addr, uint8_t* out, size_t out_capacity)
{
    return ctu_protocol_build_frame(0x0BU, &addr, 1U, out, out_capacity);
}

/* ---- 解析（适配 public_layer 的 protocol_parser）---- */

void ctu_protocol_parser_init(ctu_protocol_parser_t* parser)
{
    protocol_parser_config_t config;

    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));

    config.name = "ctu_rs485_z";
    config.header = s_ctu_protocol_header;
    config.footer = s_ctu_protocol_footer;
    config.output_buffer = parser->output_buffer;
    config.input_buffer = parser->input_buffer;
    config.get_len_cb = ctu_protocol_parser_get_len;
    config.check_cb = ctu_protocol_parser_check;
    config.header_len = CTU_PROTOCOL_HEADER_LEN;
    config.footer_len = CTU_PROTOCOL_FOOTER_LEN;
    config.input_buffer_len = (uint16_t)sizeof(parser->input_buffer);
    config.output_buffer_len = (uint16_t)sizeof(parser->output_buffer);

    (void)protocol_parser_init(&parser->context, &config);
}

ctu_error_t ctu_protocol_parser_feed(ctu_protocol_parser_t* parser,
                                     const uint8_t* data, size_t length)
{
    if ((parser == NULL) || (data == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (length == 0U) {
        return CTU_OK;
    }

    if (protocol_parser_feed(&parser->context, data, (uint32_t)length)
        != PROTOCOL_PARSER_OK) {
        /* FIFO 溢出：中间件已复位 FIFO，这里同步清状态并计数 */
        (void)protocol_parser_clear(&parser->context);
        parser->dropped += (uint32_t)length;
        return CTU_ERROR_BUFFER_TOO_SMALL;
    }

    return CTU_OK;
}

bool ctu_protocol_parser_pop(ctu_protocol_parser_t* parser, uint8_t* frame_out,
                             size_t capacity, size_t* frame_len)
{
    if ((parser == NULL) || (frame_out == NULL) || (frame_len == NULL)) {
        return false;
    }

    for (uint32_t round = 0U; round < CTU_PROTOCOL_PARSER_MAX_ROUNDS; round++) {
        uint16_t parsed_len = 0U;
        uint8_t* parsed = NULL;
        protocol_parser_error_t error = protocol_parser_parse(&parser->context,
                                                              &parsed_len, &parsed);

        if (error == PROTOCOL_PARSER_OK) {
            if ((size_t)parsed_len > capacity) {
                parser->dropped += parsed_len; /* 输出缓冲不足，丢弃该帧 */
                continue;
            }
            memcpy(frame_out, parsed, parsed_len);
            *frame_len = parsed_len;
            return true;
        }
        if (error == PROTOCOL_PARSER_ERROR_INCOMPLETE) {
            return false; /* 数据尚未收全 */
        }

        /* 校验/帧尾/空闲超时等：中间件已重新同步，继续尝试下一帧 */
        parser->dropped++;
    }

    return false;
}

/* ---- 帧字段访问 ---- */

uint8_t ctu_protocol_frame_addr(const uint8_t* frame, size_t length)
{
    const uint8_t* payload = ctu_protocol_frame_payload(frame, length);

    return (payload == NULL) ? 0U : payload[0];
}

uint8_t ctu_protocol_frame_cmd(const uint8_t* frame, size_t length)
{
    if ((frame == NULL) || (length < 2U)) {
        return 0U;
    }
    return frame[1];
}

size_t ctu_protocol_frame_payload_len(const uint8_t* frame, size_t length)
{
    if ((frame == NULL) || (length < 3U)) {
        return 0U;
    }
    return frame[2];
}

const uint8_t* ctu_protocol_frame_payload(const uint8_t* frame, size_t length)
{
    if ((frame == NULL) || (length < CTU_PROTOCOL_OVERHEAD)) {
        return NULL;
    }
    if ((size_t)frame[2] > (length - CTU_PROTOCOL_OVERHEAD)) {
        return NULL;
    }
    return &frame[3];
}

bool ctu_protocol_frame_is_error(const uint8_t* frame, size_t length)
{
    return ctu_protocol_frame_cmd(frame, length) == CTU_PROTOCOL_ERR_CMD;
}

bool ctu_protocol_frame_is_valid(const uint8_t* frame, size_t length)
{
    if (frame == NULL) {
        return false;
    }
    if (length < CTU_PROTOCOL_OVERHEAD) {
        return false;
    }
    if ((size_t)frame[2] != (length - CTU_PROTOCOL_OVERHEAD)) {
        return false;
    }
    if (frame[0] != CTU_PROTOCOL_HEADER) {
        return false;
    }
    if (frame[length - 1U] != CTU_PROTOCOL_FOOT) {
        return false;
    }

    return ctu_protocol_crc8(frame, length - 2U) == frame[length - 2U];
}

size_t ctu_protocol_frame_to_hex(const uint8_t* frame, size_t length,
                                 char* out, size_t capacity)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    size_t written = 0U;

    if ((frame == NULL) || (out == NULL) || (capacity == 0U)) {
        return 0U;
    }

    out[0] = '\0';
    for (size_t i = 0U; i < length; i++) {
        /* 每字节 3 字符（含空格），末尾需留 '\0' */
        if ((written + 4U) > capacity) {
            break;
        }
        if (i > 0U) {
            out[written] = ' ';
            written++;
        }
        out[written] = hex_digits[(frame[i] >> 4U) & 0x0FU];
        out[written + 1U] = hex_digits[frame[i] & 0x0FU];
        written += 2U;
    }
    out[written] = '\0';

    return written;
}

const char* ctu_protocol_cmd_name(uint8_t cmd)
{
    if (cmd == CTU_PROTOCOL_ERR_CMD) {
        return "错误应答";
    }

    switch (cmd & (uint8_t)(~CTU_PROTOCOL_REPLY_FLAG)) {
    case CTU_CMD_READ_STATUS:
        return "读系统状态";
    case CTU_CMD_READ_VOLT:
        return "读电压";
    case CTU_CMD_READ_TEMP:
        return "读温度";
    case CTU_CMD_CTRL:
        return "控制";
    case CTU_CMD_RESET_LATCH:
        return "清除故障锁存";
    case CTU_CMD_UPGRADE:
        return "升级请求";
    case CTU_CMD_READ_INFO:
        return "读固件信息";
    default:
        return "未知命令";
    }
}

const char* ctu_protocol_dev_err_str(uint8_t code)
{
    return ctu_protocol_dev_msg(code);
}

const char* ctu_protocol_boot_err_str(uint8_t code)
{
    return ctu_protocol_boot_msg(code);
}

/* ---- 数据段解码 ---- */

ctu_error_t ctu_protocol_decode_master_status(const uint8_t* data, size_t length,
                                              ctu_master_status_t* out)
{
    uint8_t byte0;
    uint8_t byte1;

    if ((data == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (length < 2U) {
        return CTU_ERROR_PROTOCOL;
    }

    memset(out, 0, sizeof(*out));
    byte0 = data[0];
    byte1 = data[1];

    out->estop = (byte0 & 0x01U) != 0U;
    out->rail_12v_fault = (byte0 & 0x02U) != 0U;
    out->rail_24v_fault = (byte0 & 0x04U) != 0U;
    out->vin_dcdc_fault = (byte0 & 0x08U) != 0U;
    out->aux_fault = (byte0 & 0x10U) != 0U;
    out->motor_fault = (byte0 & 0x20U) != 0U;
    out->fan0_fault = (byte1 & 0x01U) != 0U;
    out->fan1_fault = (byte1 & 0x02U) != 0U;
    out->ntc1_disconnected = (byte1 & 0x04U) != 0U;
    out->ntc2_disconnected = (byte1 & 0x08U) != 0U;

    return CTU_OK;
}

ctu_error_t ctu_protocol_decode_slaver_status(const uint8_t* data, size_t length,
                                              ctu_slaver_status_t* out)
{
    uint8_t byte0;
    uint8_t byte1;

    if ((data == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (length < 2U) {
        return CTU_ERROR_PROTOCOL;
    }

    memset(out, 0, sizeof(*out));
    byte0 = data[0];
    byte1 = data[1];

    out->fault_24v = (byte0 & 0x01U) != 0U;
    out->fault_12v = (byte0 & 0x02U) != 0U;
    out->fault_aux = (byte0 & 0x04U) != 0U;
    out->fault_motor = (byte0 & 0x08U) != 0U;
    out->fault_lsd1 = (byte0 & 0x10U) != 0U;
    out->fault_lsd2 = (byte0 & 0x20U) != 0U;
    out->out_24v = (byte1 & 0x01U) != 0U;
    out->out_12v = (byte1 & 0x02U) != 0U;
    out->out_lsd1 = (byte1 & 0x04U) != 0U;
    out->out_lsd2 = (byte1 & 0x08U) != 0U;
    out->latch_active = (byte1 & 0x10U) != 0U;

    return CTU_OK;
}

ctu_error_t ctu_protocol_decode_master_voltage(const uint8_t* data, size_t length,
                                               ctu_master_voltage_t* out)
{
    if ((data == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (length < 4U) {
        return CTU_ERROR_PROTOCOL;
    }

    memset(out, 0, sizeof(*out));
    out->vin_mv = ctu_protocol_u16_le(&data[0]);
    out->vin_dcdc_mv = ctu_protocol_u16_le(&data[2]);

    return CTU_OK;
}

ctu_error_t ctu_protocol_decode_slaver_voltage(const uint8_t* data, size_t length,
                                               ctu_slaver_voltage_t* out)
{
    if ((data == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (length < 8U) {
        return CTU_ERROR_PROTOCOL;
    }

    memset(out, 0, sizeof(*out));
    out->aux_mv = ctu_protocol_u16_le(&data[0]);
    out->motor_mv = ctu_protocol_u16_le(&data[2]);
    out->lsd1_mv = ctu_protocol_u16_le(&data[4]);
    out->lsd2_mv = ctu_protocol_u16_le(&data[6]);

    return CTU_OK;
}

ctu_error_t ctu_protocol_decode_master_temp(const uint8_t* data, size_t length,
                                            ctu_master_temp_t* out)
{
    if ((data == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (length < 6U) {
        return CTU_ERROR_PROTOCOL;
    }

    memset(out, 0, sizeof(*out));
    out->ntc1_c = (float)ctu_protocol_i16_le(&data[0]) / 100.0f;
    out->ntc2_c = (float)ctu_protocol_i16_le(&data[2]) / 100.0f;
    out->mcu_c = (float)ctu_protocol_i16_le(&data[4]) / 100.0f;

    return CTU_OK;
}

ctu_error_t ctu_protocol_decode_slaver_temp(const uint8_t* data, size_t length,
                                            ctu_slaver_temp_t* out)
{
    if ((data == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (length < 4U) {
        return CTU_ERROR_PROTOCOL;
    }

    memset(out, 0, sizeof(*out));
    out->mcu_c = (float)ctu_protocol_i16_le(&data[0]) / 100.0f;
    out->vdda_mv = ctu_protocol_u16_le(&data[2]);

    return CTU_OK;
}

ctu_error_t ctu_protocol_decode_fw_info(const uint8_t* data, size_t length,
                                        ctu_fw_info_t* out)
{
    if ((data == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (length < 15U) {
        return CTU_ERROR_PROTOCOL;
    }

    memset(out, 0, sizeof(*out));
    out->app_version = ctu_protocol_u16_le(&data[0]);
    out->meta_version = ctu_protocol_u16_le(&data[2]);
    out->fw_size = ctu_protocol_u32_le(&data[4]);
    out->fw_checksum = ctu_protocol_u32_le(&data[8]);
    out->reboot_counts = ctu_protocol_u16_le(&data[12]);
    out->flags = data[14];

    return CTU_OK;
}

/* ---- 便捷判断 ---- */

bool ctu_master_status_has_fault(const ctu_master_status_t* status)
{
    if (status == NULL) {
        return true;
    }

    return status->estop || status->rail_12v_fault || status->rail_24v_fault
        || status->vin_dcdc_fault || status->aux_fault || status->motor_fault
        || status->fan0_fault || status->fan1_fault
        || status->ntc1_disconnected || status->ntc2_disconnected;
}

bool ctu_slaver_status_has_fault(const ctu_slaver_status_t* status)
{
    if (status == NULL) {
        return true;
    }

    return status->fault_24v || status->fault_12v || status->fault_lsd1
        || status->fault_lsd2 || status->fault_aux || status->fault_motor
        || status->latch_active;
}

uint8_t ctu_slaver_status_output_mask(const ctu_slaver_status_t* status)
{
    uint8_t mask = 0U;

    if (status == NULL) {
        return 0U;
    }

    if (status->out_24v) {
        mask |= 0x01U;
    }
    if (status->out_12v) {
        mask |= 0x02U;
    }
    if (status->out_lsd1) {
        mask |= 0x04U;
    }
    if (status->out_lsd2) {
        mask |= 0x08U;
    }

    return mask;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief protocol_parser 帧长度回调
 * @note  buffer[0]='z'，buffer[1]=cmd，buffer[2]=data_len → 帧总长 = data_len + 5
 * @return 完整帧长度；0 表示长度字段尚未收全
 */
static uint16_t ctu_protocol_parser_get_len(uint8_t* buffer, uint16_t len)
{
    if (buffer == NULL) {
        return 0U;
    }
    if (len < CTU_PROTOCOL_LEN_FIELD_MIN) {
        return 0U; /* 尚不足 header + cmd + data_len */
    }

    return (uint16_t)buffer[2] + CTU_PROTOCOL_OVERHEAD;
}

/**
 * @brief protocol_parser 帧校验回调（CRC8）
 * @note  校验字节位于 len-2（帧尾 '\n' 位于 len-1）
 */
static protocol_parser_error_t ctu_protocol_parser_check(uint8_t* buffer, uint16_t len)
{
    if (buffer == NULL) {
        return PROTOCOL_PARSER_ERROR_NULL_PTR;
    }
    if (len < (CTU_PROTOCOL_LEN_FIELD_MIN + CTU_PROTOCOL_FOOTER_LEN)) {
        return PROTOCOL_PARSER_ERROR_INVALID_PARAM;
    }
    if (ctu_protocol_crc8(buffer, (size_t)len - 2U) == buffer[len - 2U]) {
        return PROTOCOL_PARSER_OK;
    }

    return PROTOCOL_PARSER_ERROR_CHECKSUM;
}

/**
 * @brief 设备错误码描述表
 */
static const char* ctu_protocol_dev_msg(uint8_t code)
{
    switch (code) {
    case CTU_DEV_ERR_NONE:
        return "无错误";
    case CTU_DEV_ERR_UNKNOWN_CMD:
        return "未知命令";
    case CTU_DEV_ERR_UNSUPPORTED:
        return "功能暂不支持";
    case CTU_DEV_ERR_BAD_LENGTH:
        return "帧长度与命令不匹配";
    default:
        return "未知错误码";
    }
}

/**
 * @brief Boot 错误码描述表
 */
static const char* ctu_protocol_boot_msg(uint8_t code)
{
    switch (code) {
    case CTU_BOOT_ERR_NONE:
        return "无错误";
    case CTU_BOOT_ERR_BAD_LENGTH:
        return "帧长非法";
    case CTU_BOOT_ERR_BAD_STATE:
        return "状态错/未选中";
    case CTU_BOOT_ERR_BAD_BLOCK:
        return "块号错";
    case CTU_BOOT_ERR_BAD_CRC16:
        return "数据 CRC16 错";
    case CTU_BOOT_ERR_FLASH:
        return "Flash 写失败";
    case CTU_BOOT_ERR_BAD_SIZE:
        return "长度/容量错";
    default:
        return "未知错误码";
    }
}
