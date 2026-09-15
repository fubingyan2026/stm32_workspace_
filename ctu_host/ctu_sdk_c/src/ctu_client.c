//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_client.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU 上位机客户端实现（同步请求/应答 + 升级编排）。
 */

/* Includes ------------------------------------------------------------------*/
#include "ctu/ctu_client.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

/* Private constants ---------------------------------------------------------*/

#define CTU_CLIENT_DEFAULT_TIMEOUT_MS 200U /**< 默认请求超时 */
#define CTU_CLIENT_LOG_PREFIX_MAX 8U       /**< 日志前缀最大长度（"TX  "/"RX  "） */
/* 260 字节帧 → 十六进制最多 260*3-1 = 779 字符 */
#define CTU_CLIENT_LOG_HEX_CAPACITY 800U
#define CTU_CLIENT_LOG_CAPACITY (CTU_CLIENT_LOG_HEX_CAPACITY + CTU_CLIENT_LOG_PREFIX_MAX + 2U)

/* Private function prototypes -----------------------------------------------*/

static uint64_t ctu_client_now_ms(void);
static void ctu_client_log(const ctu_client_t* client, const char* level,
                           const char* text);
static void ctu_client_log_frame(const ctu_client_t* client, const char* level,
                                 const char* prefix, const uint8_t* frame,
                                 size_t length);
static void ctu_client_fill_ack(ctu_ack_t* ack, ctu_device_t device, uint8_t command,
                                uint32_t elapsed_ms);
static ctu_error_t ctu_client_simple_ack(ctu_client_t* client, const uint8_t* frame,
                                         size_t frame_len, ctu_device_t device,
                                         uint8_t command, ctu_ack_t* ack);

/* Exported functions --------------------------------------------------------*/

ctu_error_t ctu_client_init(ctu_client_t* client, const ctu_client_config_t* config)
{
    if (client == NULL) {
        return CTU_ERROR_NULL_PTR;
    }

    memset(client, 0, sizeof(*client));
    if (config != NULL) {
        client->config = *config;
    }
    if (client->config.timeout_ms == 0U) {
        client->config.timeout_ms = CTU_CLIENT_DEFAULT_TIMEOUT_MS;
    }

    if (ctu_transport_init(&client->transport, &client->config.transport) != CTU_OK) {
        return CTU_ERROR_GENERIC;
    }

    /* 升级回调直接复用客户端回调 */
    client->config.boot.log_cb = client->config.log_cb;
    client->config.boot.user = client->config.user;
    if (ctu_boot_init(&client->boot, &client->config.boot) != CTU_OK) {
        return CTU_ERROR_GENERIC;
    }

    client->cancel_requested = 0;
    client->last_device_err = 0U;
    client->initialized = true;

    return CTU_OK;
}

void ctu_client_deinit(ctu_client_t* client)
{
    if (client == NULL) {
        return;
    }

    ctu_client_close(client);
    ctu_transport_deinit(&client->transport);
    client->initialized = false;
}

bool ctu_client_is_initialized(const ctu_client_t* client)
{
    return (client != NULL) && client->initialized;
}

ctu_error_t ctu_client_open(ctu_client_t* client, const char* port, uint32_t baud)
{
    if ((client == NULL) || (port == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!client->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }

    return ctu_transport_open(&client->transport, port, baud);
}

void ctu_client_close(ctu_client_t* client)
{
    if (client == NULL) {
        return;
    }

    ctu_transport_close(&client->transport);
}

bool ctu_client_is_open(const ctu_client_t* client)
{
    return (client != NULL) && ctu_transport_is_open(&client->transport);
}

const char* ctu_client_port_name(const ctu_client_t* client)
{
    if (client == NULL) {
        return "";
    }

    return ctu_transport_port_name(&client->transport);
}

void ctu_client_request_cancel(ctu_client_t* client)
{
    if (client != NULL) {
        client->cancel_requested = 1;
    }
}

void ctu_client_clear_cancel(ctu_client_t* client)
{
    if (client != NULL) {
        client->cancel_requested = 0;
    }
}

uint8_t ctu_client_last_device_error(const ctu_client_t* client)
{
    return (client == NULL) ? 0U : client->last_device_err;
}

int ctu_client_last_errno(const ctu_client_t* client)
{
    if (client == NULL) {
        return 0;
    }

    return ctu_transport_last_errno(&client->transport);
}

const char* ctu_client_last_error_text(const ctu_client_t* client)
{
    if (client == NULL) {
        return "";
    }

    return ctu_transport_last_error_text(&client->transport);
}

ctu_error_t ctu_client_request(ctu_client_t* client, const uint8_t* frame,
                               size_t frame_len, ctu_device_t device,
                               uint8_t expected_cmd, uint8_t* content,
                               size_t content_capacity, size_t* content_len,
                               uint32_t* elapsed_ms)
{
    uint64_t start_ms;
    uint64_t deadline_ms;
    uint8_t expected_base = (uint8_t)(expected_cmd & (uint8_t)(~CTU_PROTOCOL_REPLY_FLAG));

    if ((client == NULL) || (frame == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!client->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }
    if (!ctu_device_is_valid(device)) {
        return CTU_ERROR_INVALID_PARAM;
    }
    if ((content == NULL) && (content_capacity > 0U)) {
        return CTU_ERROR_INVALID_PARAM;
    }
    if (!ctu_transport_is_open(&client->transport)) {
        return CTU_ERROR_NOT_OPEN;
    }
    if (content_len != NULL) {
        *content_len = 0U;
    }
    if (elapsed_ms != NULL) {
        *elapsed_ms = 0U;
    }

    start_ms = ctu_client_now_ms();
    deadline_ms = start_ms + (uint64_t)client->config.timeout_ms;

    (void)ctu_transport_flush_input(&client->transport);
    if (ctu_transport_write(&client->transport, frame, frame_len, 0U) != CTU_OK) {
        ctu_client_log(client, "error", "发送失败");
        return CTU_ERROR_IO;
    }
    ctu_client_log_frame(client, "tx", "TX  ", frame, frame_len);

    while (ctu_client_now_ms() < deadline_ms) {
        uint8_t rx[CTU_PROTOCOL_MAX_FRAME];
        size_t rx_len = 0U;
        uint64_t remaining = deadline_ms - ctu_client_now_ms();
        ctu_error_t result = ctu_transport_read_frame(&client->transport, rx,
                                                      sizeof(rx), &rx_len,
                                                      (uint32_t)remaining);
        uint8_t addr;
        uint8_t cmd;
        const uint8_t* payload;
        size_t payload_len;

        if (result == CTU_ERROR_TIMEOUT) {
            break;
        }
        if (result != CTU_OK) {
            return result;
        }

        ctu_client_log_frame(client, "rx", "RX  ", rx, rx_len);

        addr = ctu_protocol_frame_addr(rx, rx_len);
        cmd = ctu_protocol_frame_cmd(rx, rx_len);
        payload = ctu_protocol_frame_payload(rx, rx_len);
        payload_len = ctu_protocol_frame_payload_len(rx, rx_len);

        if (cmd == CTU_PROTOCOL_ERR_CMD) {
            if (addr != (uint8_t)device) {
                continue; /* 其它设备的错误应答 */
            }
            client->last_device_err = (payload_len >= 2U) ? payload[1] : 0xFFU;
            return CTU_ERROR_DEVICE;
        }
        if (addr != (uint8_t)device) {
            continue; /* 总线上其它设备的帧 */
        }
        if ((cmd & (uint8_t)(~CTU_PROTOCOL_REPLY_FLAG)) != expected_base) {
            return CTU_ERROR_UNEXPECTED_REPLY;
        }

        if ((content != NULL) && (content_capacity > 0U)) {
            size_t copy_len = (payload_len > 0U) ? (payload_len - 1U) : 0U;
            if (copy_len > content_capacity) {
                copy_len = content_capacity;
            }
            memcpy(content, &payload[1], copy_len);
            if (content_len != NULL) {
                *content_len = copy_len;
            }
        }
        if (elapsed_ms != NULL) {
            *elapsed_ms = (uint32_t)(ctu_client_now_ms() - start_ms);
        }
        return CTU_OK;
    }

    return CTU_ERROR_TIMEOUT;
}

ctu_error_t ctu_client_read_master_status(ctu_client_t* client,
                                          ctu_master_status_t* out)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    uint8_t content[CTU_PROTOCOL_MAX_PAYLOAD];
    size_t content_len = 0U;
    size_t frame_len;
    ctu_error_t result;

    if ((client == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }

    frame_len = ctu_protocol_build_read_status(CTU_MASTER_ADDR, frame, sizeof(frame));
    result = ctu_client_request(client, frame, frame_len, CTU_DEVICE_MASTER,
                               CTU_CMD_READ_STATUS, content, sizeof(content),
                               &content_len, NULL);
    if (result != CTU_OK) {
        return result;
    }

    return ctu_protocol_decode_master_status(content, content_len, out);
}

ctu_error_t ctu_client_read_slaver_status(ctu_client_t* client,
                                          ctu_slaver_status_t* out)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    uint8_t content[CTU_PROTOCOL_MAX_PAYLOAD];
    size_t content_len = 0U;
    size_t frame_len;
    ctu_error_t result;

    if ((client == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }

    frame_len = ctu_protocol_build_read_status(CTU_SLAVER_ADDR, frame, sizeof(frame));
    result = ctu_client_request(client, frame, frame_len, CTU_DEVICE_SLAVER,
                               CTU_CMD_READ_STATUS, content, sizeof(content),
                               &content_len, NULL);
    if (result != CTU_OK) {
        return result;
    }

    return ctu_protocol_decode_slaver_status(content, content_len, out);
}

ctu_error_t ctu_client_read_master_voltage(ctu_client_t* client,
                                           ctu_master_voltage_t* out)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    uint8_t content[CTU_PROTOCOL_MAX_PAYLOAD];
    size_t content_len = 0U;
    size_t frame_len;
    ctu_error_t result;

    if ((client == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }

    frame_len = ctu_protocol_build_read_volt(CTU_MASTER_ADDR, frame, sizeof(frame));
    result = ctu_client_request(client, frame, frame_len, CTU_DEVICE_MASTER,
                               CTU_CMD_READ_VOLT, content, sizeof(content),
                               &content_len, NULL);
    if (result != CTU_OK) {
        return result;
    }

    return ctu_protocol_decode_master_voltage(content, content_len, out);
}

ctu_error_t ctu_client_read_slaver_voltage(ctu_client_t* client,
                                           ctu_slaver_voltage_t* out)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    uint8_t content[CTU_PROTOCOL_MAX_PAYLOAD];
    size_t content_len = 0U;
    size_t frame_len;
    ctu_error_t result;

    if ((client == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }

    frame_len = ctu_protocol_build_read_volt(CTU_SLAVER_ADDR, frame, sizeof(frame));
    result = ctu_client_request(client, frame, frame_len, CTU_DEVICE_SLAVER,
                               CTU_CMD_READ_VOLT, content, sizeof(content),
                               &content_len, NULL);
    if (result != CTU_OK) {
        return result;
    }

    return ctu_protocol_decode_slaver_voltage(content, content_len, out);
}

ctu_error_t ctu_client_read_master_temp(ctu_client_t* client, ctu_master_temp_t* out)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    uint8_t content[CTU_PROTOCOL_MAX_PAYLOAD];
    size_t content_len = 0U;
    size_t frame_len;
    ctu_error_t result;

    if ((client == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }

    frame_len = ctu_protocol_build_read_temp(CTU_MASTER_ADDR, frame, sizeof(frame));
    result = ctu_client_request(client, frame, frame_len, CTU_DEVICE_MASTER,
                               CTU_CMD_READ_TEMP, content, sizeof(content),
                               &content_len, NULL);
    if (result != CTU_OK) {
        return result;
    }

    return ctu_protocol_decode_master_temp(content, content_len, out);
}

ctu_error_t ctu_client_read_slaver_temp(ctu_client_t* client, ctu_slaver_temp_t* out)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    uint8_t content[CTU_PROTOCOL_MAX_PAYLOAD];
    size_t content_len = 0U;
    size_t frame_len;
    ctu_error_t result;

    if ((client == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }

    frame_len = ctu_protocol_build_read_temp(CTU_SLAVER_ADDR, frame, sizeof(frame));
    result = ctu_client_request(client, frame, frame_len, CTU_DEVICE_SLAVER,
                               CTU_CMD_READ_TEMP, content, sizeof(content),
                               &content_len, NULL);
    if (result != CTU_OK) {
        return result;
    }

    return ctu_protocol_decode_slaver_temp(content, content_len, out);
}

ctu_error_t ctu_client_read_info(ctu_client_t* client, ctu_device_t device,
                                 ctu_fw_info_t* out)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    uint8_t content[CTU_PROTOCOL_MAX_PAYLOAD];
    size_t content_len = 0U;
    size_t frame_len;
    ctu_error_t result;

    if ((client == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctu_device_is_valid(device)) {
        return CTU_ERROR_INVALID_PARAM;
    }

    frame_len = ctu_protocol_build_read_info((uint8_t)device, frame, sizeof(frame));
    result = ctu_client_request(client, frame, frame_len, device, CTU_CMD_READ_INFO,
                               content, sizeof(content), &content_len, NULL);
    if (result != CTU_OK) {
        return result;
    }

    return ctu_protocol_decode_fw_info(content, content_len, out);
}

ctu_error_t ctu_client_set_buzzer_duty(ctu_client_t* client, uint8_t duty,
                                       ctu_ack_t* ack)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    size_t frame_len;

    if (client == NULL) {
        return CTU_ERROR_NULL_PTR;
    }

    frame_len = ctu_protocol_build_master_ctrl(CTU_MASTER_ADDR, duty, frame,
                                               sizeof(frame));

    return ctu_client_simple_ack(client, frame, frame_len, CTU_DEVICE_MASTER,
                                 CTU_CMD_CTRL, ack);
}

ctu_error_t ctu_client_set_outputs(ctu_client_t* client, uint8_t mask,
                                   uint16_t fill_duty, ctu_ack_t* ack)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    size_t frame_len;

    if (client == NULL) {
        return CTU_ERROR_NULL_PTR;
    }

    frame_len = ctu_protocol_build_slaver_ctrl(CTU_SLAVER_ADDR, mask, fill_duty,
                                               frame, sizeof(frame));

    return ctu_client_simple_ack(client, frame, frame_len, CTU_DEVICE_SLAVER,
                                 CTU_CMD_CTRL, ack);
}

ctu_error_t ctu_client_clear_fault_latch(ctu_client_t* client, ctu_device_t device,
                                         ctu_ack_t* ack)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    size_t frame_len;

    if (client == NULL) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctu_device_is_valid(device)) {
        return CTU_ERROR_INVALID_PARAM;
    }

    frame_len = ctu_protocol_build_reset_latch((uint8_t)device, frame, sizeof(frame));

    return ctu_client_simple_ack(client, frame, frame_len, device,
                                 CTU_CMD_RESET_LATCH, ack);
}

ctu_error_t ctu_client_request_upgrade(ctu_client_t* client, ctu_device_t device,
                                       ctu_ack_t* ack)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    size_t frame_len;

    if (client == NULL) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctu_device_is_valid(device)) {
        return CTU_ERROR_INVALID_PARAM;
    }

    frame_len = ctu_protocol_build_upgrade((uint8_t)device, frame, sizeof(frame));

    return ctu_client_simple_ack(client, frame, frame_len, device,
                                 CTU_CMD_UPGRADE, ack);
}

ctu_error_t ctu_client_probe(ctu_client_t* client, ctu_device_t device, bool* online)
{
    ctu_error_t result;

    if (online != NULL) {
        *online = false;
    }
    if (client == NULL) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctu_device_is_valid(device)) {
        return CTU_ERROR_INVALID_PARAM;
    }

    if (device == CTU_DEVICE_MASTER) {
        ctu_master_status_t status;
        result = ctu_client_read_master_status(client, &status);
    } else {
        ctu_slaver_status_t status;
        result = ctu_client_read_slaver_status(client, &status);
    }

    if (result == CTU_OK) {
        if (online != NULL) {
            *online = true;
        }
        return CTU_OK;
    }

    return CTU_OK; /* 探测动作本身完成，在线与否由 online 给出 */
}

ctu_error_t ctu_client_upgrade(ctu_client_t* client, ctu_device_t device,
                               const char* filepath)
{
    if ((client == NULL) || (filepath == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!client->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }
    if (!ctu_device_is_valid(device)) {
        return CTU_ERROR_INVALID_PARAM;
    }
    if (!ctu_transport_is_open(&client->transport)) {
        return CTU_ERROR_NOT_OPEN;
    }

    ctu_client_clear_cancel(client);
    client->boot.device = device;
    client->boot.addr = (uint8_t)device;
    client->boot.cancel = &client->cancel_requested;

    return ctu_boot_transfer(&client->boot, &client->transport, filepath);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 读取单调时钟（毫秒）
 */
static uint64_t ctu_client_now_ms(void)
{
    struct timespec now;

    (void)clock_gettime(CLOCK_MONOTONIC, &now);

    return ((uint64_t)now.tv_sec * 1000U) + ((uint64_t)now.tv_nsec / 1000000U);
}

/**
 * @brief 文本日志回调包装
 */
static void ctu_client_log(const ctu_client_t* client, const char* level,
                           const char* text)
{
    if ((client == NULL) || (client->config.log_cb == NULL) || (text == NULL)) {
        return;
    }

    client->config.log_cb(level, text, client->config.user);
}

/**
 * @brief 收发帧日志
 */
static void ctu_client_log_frame(const ctu_client_t* client, const char* level,
                                 const char* prefix, const uint8_t* frame,
                                 size_t length)
{
    char hex[CTU_CLIENT_LOG_HEX_CAPACITY];
    char line[CTU_CLIENT_LOG_CAPACITY];
    int prefix_len;

    if ((client == NULL) || (client->config.log_cb == NULL)) {
        return;
    }
    if (prefix == NULL) {
        prefix = "";
    }
    /* 用精度限定前缀长度，避免 -Wformat-truncation（前缀为不定长指针） */
    prefix_len = (int)strnlen(prefix, CTU_CLIENT_LOG_PREFIX_MAX);

    (void)ctu_protocol_frame_to_hex(frame, length, hex, sizeof(hex));
    (void)snprintf(line, sizeof(line), "%.*s%s", prefix_len, prefix, hex);
    ctu_client_log(client, level, line);
}

/**
 * @brief 填充应答摘要
 */
static void ctu_client_fill_ack(ctu_ack_t* ack, ctu_device_t device, uint8_t command,
                                uint32_t elapsed_ms)
{
    if (ack == NULL) {
        return;
    }

    ack->device = device;
    ack->command = command;
    ack->elapsed_ms = elapsed_ms;
}

/**
 * @brief 无数据段命令：发送并等待应答
 */
static ctu_error_t ctu_client_simple_ack(ctu_client_t* client, const uint8_t* frame,
                                         size_t frame_len, ctu_device_t device,
                                         uint8_t command, ctu_ack_t* ack)
{
    uint32_t elapsed_ms = 0U;
    ctu_error_t result;

    result = ctu_client_request(client, frame, frame_len, device, command, NULL, 0U,
                                NULL, &elapsed_ms);
    if (result == CTU_OK) {
        ctu_client_fill_ack(ack, device, command, elapsed_ms);
    }

    return result;
}
