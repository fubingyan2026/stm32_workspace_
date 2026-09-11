/**
 * @file    srv_com_mst.c
 * @author  maximillian
 * @version V5.0.0
 * @date    2026-09-09
 * @brief   RS485 主机协议服务实现（E1_MASTER_POWER_CTU）
 *
 * 帧格式（两兄弟板共用帧头，设备用 payload 首字节 ID 区分/定向）：
 *   [ 'z' ][cmd][data_len][payload][CRC8][ '\n' ]，总长 = data_len + 5
 *   请求 payload[0]=目标 ID；应答 payload[0]=源 ID（本板）。
 * 命令码与 E1_SLAVER 统一（0x02 电压 / 0x03 温度 / 0x05 清锁存 / 0x06 升级预留）。
 * 解析 protocol_parser、打包 protocol_packer（公共 m_middlewares）。
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_com_mst.h"

#include "crc.h"
#include "drv_systick.h"
#include "log.h"
#include "protocol_packer.h"
#include "protocol_parser.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_COM_MST_LOG_ENABLE 1

#if SRV_COM_MST_LOG_ENABLE
#define SRV_COM_MST_LOG_E(...) LOG_E("srv_com_mst", __VA_ARGS__)
#define SRV_COM_MST_LOG_W(...) LOG_W("srv_com_mst", __VA_ARGS__)
#define SRV_COM_MST_LOG_I(...) LOG_I("srv_com_mst", __VA_ARGS__)
#define SRV_COM_MST_LOG_D(...) ((void)0)//LOG_D("srv_com_mst", __VA_ARGS__)
#else
#define SRV_COM_MST_LOG_E(...) ((void)0)
#define SRV_COM_MST_LOG_W(...) ((void)0)
#define SRV_COM_MST_LOG_I(...) ((void)0)
#define SRV_COM_MST_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 帧头/帧尾（两兄弟板共用，仅 'z'；设备区分在 payload 首字节 ID） */
static const uint8_t s_header[] = { 'z' };
static const uint8_t s_footer[] = { '\n' };

/** @brief parser 输入 kfifo 缓冲（必须为 2 的幂） */
#define SRV_COM_MST_INPUT_BUF_SIZE (256U)

/** @brief parser/packer 输出缓冲 */
#define SRV_COM_MST_OUTPUT_BUF_SIZE (SRV_COM_MST_MAX_FRAME_LEN)

/** @brief 单次 feed 最多解析的帧数（防止单次调用占用主循环过长） */
#define SRV_COM_MST_MAX_PARSE_PER_FEED (4U)

/** @brief 错误日志限频窗口 (ms) */
#define SRV_COM_MST_ERR_LOG_PERIOD_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static protocol_parser_context_t s_parser;
static protocol_packer_context_t s_packer;
static uint8_t s_input_buf[SRV_COM_MST_INPUT_BUF_SIZE];
static uint8_t s_parse_out[SRV_COM_MST_OUTPUT_BUF_SIZE];
static uint8_t s_pack_out[SRV_COM_MST_OUTPUT_BUF_SIZE];
static uint8_t s_cmd_buf[SRV_COM_MST_MAX_PAYLOAD_LEN + 4U]; /* cmd + dlen + id占位 + payload */

static srv_com_mst_read_cb_t s_read_data;
static srv_com_mst_ctrl_cb_t s_ctrl_cb;
static srv_com_mst_reset_cb_t s_reset_cb;
static srv_com_mst_upgrade_cb_t s_upgrade_cb;
static srv_com_mst_info_cb_t s_info_cb;
static srv_com_mst_send_cb_t s_send_frame;

static uint32_t s_last_err_log; /**< 错误日志限频时间戳 (ms) */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static uint16_t com_get_len_cb(uint8_t* buffer, uint16_t len);
static protocol_parser_error_t com_check_cb(uint8_t* buffer, uint16_t len);
static protocol_packer_error_t com_fill_len_cb(uint8_t* buffer, uint16_t payload_len);
static protocol_packer_error_t com_checksum_cb(const uint8_t* data, uint16_t len,
    uint8_t* checksum_out, uint16_t* checksum_len);

static void com_handle_frame(uint8_t cmd, const uint8_t* payload, uint8_t plen);
static void com_reply(uint8_t reply_cmd, const uint8_t* payload, uint8_t plen);
static void com_reply_err(srv_com_mst_err_code_t code);
static void com_log_error(const char* tag, int32_t err);

/* Exported functions --------------------------------------------------------*/

void srv_com_mst_init(const srv_com_mst_config_t* config)
{
    s_read_data = NULL;
    s_ctrl_cb = NULL;
    s_reset_cb = NULL;
    s_upgrade_cb = NULL;
    s_info_cb = NULL;
    s_send_frame = NULL;
    s_initialized = false;

    if (!config || !config->read_data || !config->ctrl || !config->send_frame) {
        SRV_COM_MST_LOG_E("初始化失败: config/read_data/ctrl/send_frame 缺失");
        return;
    }

    s_read_data = config->read_data;
    s_ctrl_cb = config->ctrl;
    s_reset_cb = config->reset_latch;
    s_upgrade_cb = config->upgrade;
    s_info_cb = config->info;
    s_send_frame = config->send_frame;

    /* protocol_parser：接收下行帧（帧头仅 'z'） */
    const protocol_parser_config_t parser_cfg = {
        .name = "srv_com_mst_rx",
        .header = s_header,
        .footer = s_footer,
        .output_buffer = s_parse_out,
        .input_buffer = s_input_buf,
        .get_len_cb = com_get_len_cb,
        .check_cb = com_check_cb,
        .header_len = 1,
        .footer_len = 1,
        .input_buffer_len = (uint16_t)sizeof(s_input_buf),
        .output_buffer_len = (uint16_t)sizeof(s_parse_out),
    };
    (void)protocol_parser_init(&s_parser, &parser_cfg);

    /* protocol_packer：打包应答帧 */
    const protocol_packer_config_t packer_cfg = {
        .name = "srv_com_mst_tx",
        .header = s_header,
        .footer = s_footer,
        .output_buffer = s_pack_out,
        .checksum_cb = com_checksum_cb,
        .fill_len_cb = com_fill_len_cb,
        .header_len = 1,
        .footer_len = 1,
        .checksum_len = 1,
        .output_buffer_len = (uint16_t)sizeof(s_pack_out),
    };
    (void)protocol_packer_init(&s_packer, &packer_cfg);

    s_last_err_log = 0;
    s_initialized = true;

    SRV_COM_MST_LOG_I("主机协议服务初始化完成 (z-frame + payload ID=0x%02X + CRC8)",
        (unsigned)SRV_COM_MST_DEV_ID);
}

void srv_com_mst_deinit(void)
{
    (void)protocol_parser_deinit(&s_parser);
    (void)protocol_packer_deinit(&s_packer);

    s_read_data = NULL;
    s_ctrl_cb = NULL;
    s_reset_cb = NULL;
    s_upgrade_cb = NULL;
    s_info_cb = NULL;
    s_send_frame = NULL;
    s_initialized = false;

    SRV_COM_MST_LOG_I("主机协议服务反初始化完成");
}

bool srv_com_mst_is_initialized(void)
{
    return s_initialized;
}

void srv_com_mst_rx_feed(const uint8_t* data, uint32_t len)
{
    if (!s_initialized || !data || len == 0U) {
        return;
    }

    (void)protocol_parser_feed(&s_parser, data, len);

    /* 解析完整帧并应答：单次最多 MAX_PARSE_PER_FEED 帧，其余下周期继续 */
    uint16_t frame_len = 0;
    uint8_t* frame = NULL;
    uint32_t parsed = 0;

    while (parsed < SRV_COM_MST_MAX_PARSE_PER_FEED) {
        const protocol_parser_error_t err = protocol_parser_parse(&s_parser, &frame_len, &frame);

        if (err == PROTOCOL_PARSER_OK) {
            /* 结构合法性（协议已校验 header/footer/crc，此处再校验长度字段一致性） */
            if (frame == NULL || frame_len < 5U
                || frame[0] != 'z' || frame[frame_len - 1U] != '\n') {
                com_log_error("非法帧结构", (int32_t)err);
                parsed++;
                continue;
            }

            const uint8_t data_len = frame[2];
            if (data_len != (uint8_t)(frame_len - 5U)
                || data_len > SRV_COM_MST_MAX_PAYLOAD_LEN) {
                com_log_error("data_len 不一致", (int32_t)err);
                parsed++;
                continue;
            }

            parsed++;
            com_handle_frame(frame[1], &frame[3], data_len);
            continue;
        }

        /* 不完整 / 空闲超时：等待更多数据，终止本轮解析 */
        if (err == PROTOCOL_PARSER_ERROR_INCOMPLETE
            || err == PROTOCOL_PARSER_ERROR_IDLE_TIMEOUT) {
            break;
        }

        /* 其他错误：parser 已自行跳过垃圾字节，限频告警后继续 */
        com_log_error("解析错误", (int32_t)err);
        parsed++;
    }
}

void srv_com_mst_rx_tick(void)
{
    if (!s_initialized) {
        return;
    }
    (void)protocol_parser_tick(&s_parser);
}

/* Private functions ---------------------------------------------------------*/

/* ---- protocol_parser 回调 ---- */

/**
 * @brief 帧长度计算回调
 * @note  buffer[0]='z', buffer[1]=cmd, buffer[2]=data_len
 */
static uint16_t com_get_len_cb(uint8_t* buffer, uint16_t len)
{
    if (buffer == NULL) {
        return 0;
    }
    if (len < 3U) {
        return 0; /* 尚不足 cmd + data_len 两个字段 */
    }
    return (uint16_t)buffer[2] + 5U;
}

/**
 * @brief 帧校验回调（CRC8）
 * @note  crc 字节位于 len-2（footer '\n' 在 len-1）
 */
static protocol_parser_error_t com_check_cb(uint8_t* buffer, uint16_t len)
{
    if (buffer == NULL || len < 3U) {
        return PROTOCOL_PARSER_ERROR_INVALID_PARAM;
    }

    const uint8_t expected = get_CRC8_check_sum(buffer, len - 2U, 0xFF);
    if (expected == buffer[len - 2U]) {
        return PROTOCOL_PARSER_OK;
    }
    return PROTOCOL_PARSER_ERROR_CHECKSUM;
}

/* ---- protocol_packer 回调 ---- */

/**
 * @brief 帧长度字段回填回调
 * @note  data 块 = [cmd][data_len占位][payload]，buffer[2] 为 data_len 字段
 */
static protocol_packer_error_t com_fill_len_cb(uint8_t* buffer, uint16_t payload_len)
{
    if (buffer == NULL) {
        return PROTOCOL_PACKER_ERROR_NULL_PTR;
    }
    if (payload_len < 2U) {
        return PROTOCOL_PACKER_ERROR_INVALID_PARAM;
    }

    buffer[2] = (uint8_t)(payload_len - 2U);
    return PROTOCOL_PACKER_OK;
}

/**
 * @brief CRC8 计算回调（覆盖 header + cmd + data_len + payload）
 */
static protocol_packer_error_t com_checksum_cb(const uint8_t* data, uint16_t len,
    uint8_t* checksum_out, uint16_t* checksum_len)
{
    if (data == NULL || checksum_out == NULL || checksum_len == NULL) {
        return PROTOCOL_PACKER_ERROR_NULL_PTR;
    }

    checksum_out[0] = get_CRC8_check_sum((uint8_t*)data, len, 0xFF);
    *checksum_len = 1;
    return PROTOCOL_PACKER_OK;
}

/* ---- 命令处理与应答 ---- */

/**
 * @brief 解析主机命令并立即回包（read_data/ctrl 回调均在主循环上下文执行）
 */
static void com_handle_frame(uint8_t cmd, const uint8_t* payload, uint8_t plen)
{
    /* 调试：打印帧中设备 ID（payload 首字节）及是否匹配本板 */
    const uint8_t rx_id = (plen >= 1U && payload != NULL) ? payload[0] : 0xFFU;
    SRV_COM_MST_LOG_D("RX cmd=0x%02X dev_id=0x%02X%s",
        (unsigned)cmd, (unsigned)rx_id,
        (rx_id == SRV_COM_MST_DEV_ID) ? " (本板)" : " (忽略)");
    (void)rx_id; /* 当 SRV_COM_MST_LOG_D 被编译裁剪时避免未用告警 */

    /* 定向：payload[0] 必须为本板设备 ID，否则忽略（其它板查询流量，不响应） */
    if (plen < 1U || payload == NULL || payload[0] != SRV_COM_MST_DEV_ID) {
        return;
    }

    const uint8_t* body = &payload[1];
    const uint8_t body_len = (uint8_t)(plen - 1U);

    SRV_COM_MST_LOG_D("收到命令: 0x%02X len=%u", (unsigned)cmd, (unsigned)body_len);

    uint8_t reply_payload[SRV_COM_MST_MAX_PAYLOAD_LEN];
    uint8_t reply_len = 0;
    uint8_t reply_cmd = (uint8_t)(cmd | SRV_COM_MST_CMD_REPLY_FLAG);

    switch (cmd) {
    case SRV_COM_MST_CMD_READ_STATUS: {
        srv_com_mst_report_t report;
        memset(&report, 0, sizeof(report));
        if (s_read_data) {
            s_read_data(&report);
        }
        memcpy(reply_payload, report.status.bytes, 2U);
        reply_len = 2U;
        break;
    }
    case SRV_COM_MST_CMD_READ_VOLT: {
        srv_com_mst_report_t report;
        memset(&report, 0, sizeof(report));
        if (s_read_data) {
            s_read_data(&report);
        }
        reply_payload[0] = (uint8_t)(report.vin_mv & 0xFFU);
        reply_payload[1] = (uint8_t)(report.vin_mv >> 8);
        reply_payload[2] = (uint8_t)(report.vin_dcdc_mv & 0xFFU);
        reply_payload[3] = (uint8_t)(report.vin_dcdc_mv >> 8);
        reply_len = 4U;
        break;
    }
    case SRV_COM_MST_CMD_READ_TEMP: {
        srv_com_mst_report_t report;
        memset(&report, 0, sizeof(report));
        if (s_read_data) {
            s_read_data(&report);
        }
        reply_payload[0] = (uint8_t)(report.ntc1_temp_x100 & 0xFFU);
        reply_payload[1] = (uint8_t)((uint16_t)report.ntc1_temp_x100 >> 8);
        reply_payload[2] = (uint8_t)(report.ntc2_temp_x100 & 0xFFU);
        reply_payload[3] = (uint8_t)((uint16_t)report.ntc2_temp_x100 >> 8);
        reply_payload[4] = (uint8_t)(report.mcu_temp_x100 & 0xFFU);
        reply_payload[5] = (uint8_t)((uint16_t)report.mcu_temp_x100 >> 8);
        reply_len = 6U;
        break;
    }
    case SRV_COM_MST_CMD_CTRL: {
        if (body_len != 1U) {
            com_reply_err(SRV_COM_MST_ERR_BAD_LEN);
            return;
        }
        if (s_ctrl_cb) {
            srv_com_mst_ctrl_t ctrl;
            ctrl.buzzer_duty = body[0];
            s_ctrl_cb(&ctrl);
        }
        reply_payload[0] = SRV_COM_MST_ERR_NONE;
        reply_len = 1U;
        break;
    }
    case SRV_COM_MST_CMD_RESET_LATCH: {
        if (body_len < 1U || body[0] != 0x01U) {
            com_reply_err(SRV_COM_MST_ERR_BAD_LEN);
            return;
        }
        if (s_reset_cb) {
            s_reset_cb();
            reply_payload[0] = SRV_COM_MST_ERR_NONE;
            reply_len = 1U;
        } else {
            com_reply_err(SRV_COM_MST_ERR_NOT_SUPPORTED);
        }
        break;
    }
    case SRV_COM_MST_CMD_UPGRADE: {
        if (body_len != 1U || body[0] != 0x01U) {
            com_reply_err(SRV_COM_MST_ERR_BAD_LEN);
            return;
        }
        if (s_upgrade_cb) {
            SRV_COM_MST_LOG_W("收到升级请求，将跳转 Bootloader");
            s_upgrade_cb();
            reply_payload[0] = SRV_COM_MST_ERR_NONE;
            reply_len = 1U;
        } else {
            com_reply_err(SRV_COM_MST_ERR_NOT_SUPPORTED);
            return;
        }
        break;
    }
    case SRV_COM_MST_CMD_READ_INFO: {
        if (!s_info_cb) {
            com_reply_err(SRV_COM_MST_ERR_NOT_SUPPORTED);
            return;
        }
        srv_com_mst_info_t info = { 0 };
        s_info_cb(&info);
        reply_payload[0] = (uint8_t)(info.app_version & 0xFFU);
        reply_payload[1] = (uint8_t)(info.app_version >> 8);
        reply_payload[2] = (uint8_t)(info.meta_version & 0xFFU);
        reply_payload[3] = (uint8_t)(info.meta_version >> 8);
        reply_payload[4] = (uint8_t)(info.fw_size & 0xFFU);
        reply_payload[5] = (uint8_t)((info.fw_size >> 8) & 0xFFU);
        reply_payload[6] = (uint8_t)((info.fw_size >> 16) & 0xFFU);
        reply_payload[7] = (uint8_t)((info.fw_size >> 24) & 0xFFU);
        reply_payload[8] = (uint8_t)(info.fw_checksum & 0xFFU);
        reply_payload[9] = (uint8_t)((info.fw_checksum >> 8) & 0xFFU);
        reply_payload[10] = (uint8_t)((info.fw_checksum >> 16) & 0xFFU);
        reply_payload[11] = (uint8_t)((info.fw_checksum >> 24) & 0xFFU);
        reply_payload[12] = (uint8_t)(info.reboot_counts & 0xFFU);
        reply_payload[13] = (uint8_t)(info.reboot_counts >> 8);
        reply_payload[14] = info.flags;
        reply_len = 15U;
        break;
    }
    default:
        SRV_COM_MST_LOG_W("未知命令: 0x%02X", (unsigned)cmd);
        com_reply_err(SRV_COM_MST_ERR_UNKNOWN_CMD);
        return;
    }

    com_reply(reply_cmd, reply_payload, reply_len);
}

/**
 * @brief 经 protocol_packer 打包并发送应答帧
 * @note  应答 payload 自动前置本板设备 ID（源 ID）
 */
static void com_reply(uint8_t reply_cmd, const uint8_t* payload, uint8_t plen)
{
    s_cmd_buf[0] = reply_cmd;
    s_cmd_buf[1] = 0; /* data_len 占位，由 fill_len_cb 回填 */
    s_cmd_buf[2] = SRV_COM_MST_DEV_ID; /* 源设备 ID */
    if (plen > 0U && payload != NULL) {
        memcpy(&s_cmd_buf[3], payload, plen);
    }

    uint8_t* frame = NULL;
    uint16_t frame_len = 0;
    const protocol_packer_error_t err = protocol_packer_pack(&s_packer,
        s_cmd_buf, (uint16_t)(3U + plen), &frame, &frame_len);

    if (err != PROTOCOL_PACKER_OK) {
        com_log_error("应答打包失败", (int32_t)err);
        return;
    }

    if (s_send_frame) {
        s_send_frame(frame, (uint32_t)frame_len);
    }
}

/**
 * @brief 发送错误应答帧（CMD=0x7F, payload[0]=错误码）
 */
static void com_reply_err(srv_com_mst_err_code_t code)
{
    const uint8_t code_b = (uint8_t)code;
    com_reply(SRV_COM_MST_CMD_ERR, &code_b, 1U);
}

/**
 * @brief 错误日志限频（1s 窗口）
 */
static void com_log_error(const char* tag, int32_t err)
{
    if (s_initialized
        && (uint32_t)(millis() - s_last_err_log) >= SRV_COM_MST_ERR_LOG_PERIOD_MS) {
        s_last_err_log = millis();
        SRV_COM_MST_LOG_W("%s: err=%d", tag, (int)err);
    }
}
