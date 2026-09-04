/**
 * @file    srv_uart_host.c
 * @brief   USART1 主机二进制定长帧协议服务实现
 *
 * 帧解析与打包复用共享中间件（public_layer/m_middlewares/protocol_tools）：
 *   - 解析：protocol_parser（帧头 55 AA 00 14 + get_len_cb 定长 20 + CRC 校验回调）
 *   - 打包：protocol_packer（帧头 + can_id/data 12 字节 + CRC16 4 字节校验码）
 * 应答经 drv_log_uart_send（TX 忙则丢弃本帧）。坏帧（CRC 错）整体丢弃重同步。
 * MIT 控制帧（0x1D0）与流式控制（0xE3）不逐帧应答，避免高扇出刷屏。
 */

#include "srv_uart_host.h"

#include "crc.h"
#include "drv_log_uart.h"
#include "drv_systick.h"
#include "protocol_packer.h"
#include "protocol_parser.h"
#include "srv_juxie_motor.h"
#include "srv_mz_sensor.h"

#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/
#define SRV_UART_HOST_LOG_ENABLE 1

#if SRV_UART_HOST_LOG_ENABLE
#define SRV_UART_HOST_LOG_I(...) LOG_I("srv_uart_host", __VA_ARGS__)
#define SRV_UART_HOST_LOG_W(...) LOG_W("srv_uart_host", __VA_ARGS__)
#define SRV_UART_HOST_LOG_E(...) LOG_E("srv_uart_host", __VA_ARGS__)
#else
#define SRV_UART_HOST_LOG_I(...) ((void)0)
#define SRV_UART_HOST_LOG_W(...) ((void)0)
#define SRV_UART_HOST_LOG_E(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define SRV_UART_HOST_FRAME_LEN 64U /**< 帧总长（字节） */
#define SRV_UART_HOST_HEAD_LEN 4U /**< 帧头长度（55 AA 00 14） */
#define SRV_UART_HOST_CRC_LEN 4U /**< 校验码长度（低 2 字节有效） */
#define SRV_UART_HOST_PAYLOAD_LEN (SRV_UART_HOST_FRAME_LEN - SRV_UART_HOST_HEAD_LEN - SRV_UART_HOST_CRC_LEN)

/** @brief 帧头（0x1400AA55 小端字节序） */
static const uint8_t s_frame_header[SRV_UART_HOST_HEAD_LEN] = { 0x55, 0xAA, 0x00, 0x14 };

/** @brief 解析器输入 kfifo 缓冲（2 的幂）。
 *        原 64B 只能装 3 帧，主机突发时 protocol_parser_feed 溢出会整包 reset 丢数据；
 *        加大到 256B（12 帧），消除突发丢帧 */
#define SRV_UART_HOST_PARSER_IN_LEN (1024 * 4U)

/** @brief 命令 can_id（主机→设备） */
#define SRV_UART_HOST_CID_MIT_CTRL 0x000001D0U
#define SRV_UART_HOST_CID_MIT_CFG 0x000001D1U
#define SRV_UART_HOST_CID_MOTOR_ZERO 0x000001D2U
#define SRV_UART_HOST_CID_GET_FB 0x000001E0U
#define SRV_UART_HOST_CID_GET_ST 0x000001E1U
#define SRV_UART_HOST_CID_GET_MZ 0x000000E2U
#define SRV_UART_HOST_CID_STREAM 0x000000E3U
#define SRV_UART_HOST_CID_ZERO_MZ 0x000000E4U

/** @brief 应答 can_id（设备→主机） */
#define SRV_UART_HOST_CID_RSP_FB 0x00E00100U
#define SRV_UART_HOST_CID_RSP_ST 0x00E10100U
#define SRV_UART_HOST_CID_RSP_MZ 0x00E20000U
#define SRV_UART_HOST_CID_RSP_STREAM 0x00E30000U
#define SRV_UART_HOST_CID_ACK 0x00FF0000U

/** @brief 流式默认间隔 (ms) */
#define SRV_UART_HOST_STREAM_DEFAULT_MS 100U
/** @brief 流式最大间隔 (ms) */
#define SRV_UART_HOST_STREAM_MAX_MS 1000U

/** @brief 应答 TX 队列长度：多个应答背靠背生成时排队发送，避免 DMA 忙时丢帧 */
#define SRV_UART_HOST_TX_QUEUE_LEN 32U

/* Private variables ---------------------------------------------------------*/

/** @brief 协议解析器实例与缓冲（静态分配） */
static protocol_parser_context_t s_parser;
static uint8_t s_parser_in[SRV_UART_HOST_PARSER_IN_LEN];
static uint8_t s_parser_out[SRV_UART_HOST_FRAME_LEN];

/** @brief 协议打包器实例与输出缓冲（DMA TX 异步读取，必须静态） */
static protocol_packer_context_t s_packer;
static uint8_t s_packer_out[SRV_UART_HOST_FRAME_LEN];

/** @brief 流式输出使能 */
static bool s_stream_on;

/** @brief 流式输出间隔 (ms) */
static uint32_t s_stream_interval_ms;

/** @brief 上次流式发送时间 (millis) */
static uint32_t s_stream_last_ms;

/** @brief 解析器空闲超时 tick 节拍 (millis)，主循环全速调用时须按 1ms 门控 */
static uint32_t s_parser_tick_last_ms;

/** @brief 应答 TX 队列（DMA 异步读取，队列缓冲静态存储） */
static uint8_t s_tx_q[SRV_UART_HOST_TX_QUEUE_LEN][SRV_UART_HOST_FRAME_LEN];
static uint8_t s_tx_q_head;
static uint8_t s_tx_q_tail;
static uint8_t s_tx_q_cnt;

/* Private function prototypes -----------------------------------------------*/

static uint16_t srv_uart_host_get_len(uint8_t* buffer, uint16_t len);
static protocol_parser_error_t srv_uart_host_check(uint8_t* buffer, uint16_t len);
static protocol_packer_error_t srv_uart_host_pack_crc(const uint8_t* data,
    uint16_t len, uint8_t* checksum_out, uint16_t* checksum_len);
static void srv_uart_host_handle_frame(const uint8_t* frame, uint16_t len);
static void srv_uart_host_send_frame(uint32_t cid, const uint8_t* data, uint8_t len);
static void srv_uart_host_tx_drain(void);
static void srv_uart_host_send_ack(uint8_t ok, uint8_t echo);
static void srv_uart_host_send_fb(void);
static void srv_uart_host_send_st(void);
static void srv_uart_host_send_mz(void);
static void srv_uart_host_send_stream(void);
static int16_t srv_uart_host_sat16(int32_t v);

/* Exported functions --------------------------------------------------------*/

void srv_uart_host_init(void)
{
    protocol_parser_config_t pcfg = {
        .name = "uart_host_rx",
        .header = s_frame_header,
        .footer = NULL,
        .output_buffer = s_parser_out,
        .input_buffer = s_parser_in,
        .get_len_cb = srv_uart_host_get_len,
        .check_cb = srv_uart_host_check,
        .header_len = SRV_UART_HOST_HEAD_LEN,
        .footer_len = 0,
        .input_buffer_len = sizeof(s_parser_in),
        .output_buffer_len = sizeof(s_parser_out),
    };
    protocol_packer_config_t pkg = {
        .name = "uart_host_tx",
        .header = s_frame_header,
        .footer = NULL,
        .output_buffer = s_packer_out,
        .checksum_cb = srv_uart_host_pack_crc,
        .fill_len_cb = NULL,
        .header_len = SRV_UART_HOST_HEAD_LEN,
        .footer_len = 0,
        .checksum_len = SRV_UART_HOST_CRC_LEN,
        .output_buffer_len = sizeof(s_packer_out),
    };
    protocol_parser_error_t perr = protocol_parser_init(&s_parser, &pcfg);
    protocol_packer_error_t kerr = protocol_packer_init(&s_packer, &pkg);

    s_stream_on = false;
    s_stream_interval_ms = SRV_UART_HOST_STREAM_DEFAULT_MS;
    s_stream_last_ms = 0;
    s_parser_tick_last_ms = 0;
    s_tx_q_head = 0;
    s_tx_q_tail = 0;
    s_tx_q_cnt = 0;

    if ((perr != PROTOCOL_PARSER_OK) || (kerr != PROTOCOL_PACKER_OK)) {
        SRV_UART_HOST_LOG_E("init failed: parser=%d packer=%d", (int)perr, (int)kerr);
        return;
    }
    SRV_UART_HOST_LOG_I("init: 20B fixed-length frame protocol ready (parser+packer)");
}

void srv_uart_host_step(void)
{
    /* 0. 排空应答 TX 队列（上一拍积压的帧） */
    srv_uart_host_tx_drain();

    /* 1. 读取 USART1 RX 并送入协议解析器（全速排空，防止 kfifo 积压溢出） */
    uint8_t tmp[32];
    uint32_t n = drv_log_uart_rx_read(tmp, sizeof(tmp));
    while (n > 0U) {
        (void)protocol_parser_feed(&s_parser, tmp, n);
        n = drv_log_uart_rx_read(tmp, sizeof(tmp));
    }

    /* 空闲超时驱动按 1ms 节拍：主循环全速调用时若逐拍 tick，空闲计时会飞速增长，
     * 导致跨拍到达的完整帧被误判为超时而清空解析器 */
    const uint32_t now_ms = millis();
    if ((now_ms - s_parser_tick_last_ms) >= 1U) {
        s_parser_tick_last_ms = now_ms;
        (void)protocol_parser_tick(&s_parser);
    }

    /* 2. 逐帧解析处理（循环排空解析器内所有完整帧） */
    uint16_t flen;
    uint8_t* frame;
    for (;;) {
        const protocol_parser_error_t err = protocol_parser_parse(&s_parser, &flen, &frame);
        if (err == PROTOCOL_PARSER_OK) {
            srv_uart_host_handle_frame(frame, flen);
            continue;
        }
        if (err != PROTOCOL_PARSER_ERROR_INCOMPLETE) {
            /* 坏帧（CRC/帧无效/空闲超时）：清空解析器整体重同步，避免解析器卡死 */
            (void)protocol_parser_clear(&s_parser);
        }
        break;
    }

    /* 3. 流式输出 */
    if (s_stream_on) {
        const uint32_t now = millis();
        if ((now - s_stream_last_ms) >= s_stream_interval_ms) {
            s_stream_last_ms = now;
            srv_uart_host_send_stream();
        }
    }

    /* 4. 排空本拍新入队的应答帧 */
    srv_uart_host_tx_drain();
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 帧长度计算回调（定长协议：满 20 字节即为一帧）
 * @param buffer 帧数据缓冲区（含帧头）
 * @param len    当前已缓存总长度
 * @return 完整帧长度（不足返回 0）
 */
static uint16_t srv_uart_host_get_len(uint8_t* buffer, uint16_t len)
{
    (void)buffer;
    return (len >= SRV_UART_HOST_FRAME_LEN) ? SRV_UART_HOST_FRAME_LEN : 0U;
}

/**
 * @brief 帧校验回调：CRC16_CCITT_FALSE 覆盖 can_id+data（buf[4..15]）
 * @param buffer 完整帧（含帧头）
 * @param len    帧长度
 * @return 校验结果错误码
 */
static protocol_parser_error_t srv_uart_host_check(uint8_t* buffer, uint16_t len)
{
    if (len != SRV_UART_HOST_FRAME_LEN) {
        return PROTOCOL_PARSER_ERROR_FRAME_INVALID;
    }
    const uint16_t crc = get_CRC16_CCITT_FALSE(&buffer[SRV_UART_HOST_HEAD_LEN], SRV_UART_HOST_PAYLOAD_LEN);
    const uint16_t recv = (uint16_t)((uint16_t)buffer[16] | ((uint16_t)buffer[17] << 8));
    if (crc != recv) {
        SRV_UART_HOST_LOG_W("CRC error: recv=0x%04X calc=0x%04X", (unsigned)recv, (unsigned)crc);
        return PROTOCOL_PARSER_ERROR_CHECKSUM;
    }
    return PROTOCOL_PARSER_OK;
}

/**
 * @brief 打包器校验码回调：CRC16_CCITT_FALSE 覆盖 buf[4..15]，写入 4 字节（高 2 字节为 0）
 * @param data          输出帧缓冲（含帧头）
 * @param len           帧头+负载长度（16）
 * @param checksum_out  校验码输出缓冲
 * @param checksum_len  校验码长度输出（固定 4）
 */
static protocol_packer_error_t srv_uart_host_pack_crc(const uint8_t* data,
    uint16_t len, uint8_t* checksum_out, uint16_t* checksum_len)
{
    (void)len;
    const uint16_t crc = get_CRC16_CCITT_FALSE((uint8_t*)&data[SRV_UART_HOST_HEAD_LEN], SRV_UART_HOST_PAYLOAD_LEN);
    checksum_out[0] = (uint8_t)(crc & 0xFFU);
    checksum_out[1] = (uint8_t)((crc >> 8) & 0xFFU);
    checksum_out[2] = 0;
    checksum_out[3] = 0;
    *checksum_len = SRV_UART_HOST_CRC_LEN;
    return PROTOCOL_PACKER_OK;
}

/**
 * @brief 解析并分发一条完整命令帧
 * @param frame 完整帧（20 字节，含帧头）
 * @param len   帧长度
 */
static void srv_uart_host_handle_frame(const uint8_t* frame, uint16_t len)
{
    if (!frame || (len < SRV_UART_HOST_FRAME_LEN)) {
        return;
    }

    const uint32_t cid = (uint32_t)frame[4]
        | ((uint32_t)frame[5] << 8)
        | ((uint32_t)frame[6] << 16)
        | ((uint32_t)frame[7] << 24);
    const uint8_t* d = &frame[8];

    switch (cid) {
    case SRV_UART_HOST_CID_MIT_CTRL: {
        /* data = juxie 载荷 Byte[1..8]（pos16 大端 + vel/kp/kd/tq 12bit 打包） */
        const uint16_t pos = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
        const uint16_t vel = (uint16_t)(((uint16_t)d[2] << 4) | ((uint16_t)d[3] >> 4));
        const uint16_t kp = (uint16_t)(((uint16_t)(d[3] & 0x0FU) << 8) | d[4]);
        const uint16_t kd = (uint16_t)(((uint16_t)d[5] << 4) | ((uint16_t)d[6] >> 4));
        const uint16_t tq = (uint16_t)(((uint16_t)(d[6] & 0x0FU) << 8) | d[7]);
        srv_juxie_motor_set_mit_raw(pos, vel, kp, kd, tq);
        /* 高频控制帧不逐帧应答 */
        break;
    }

    case SRV_UART_HOST_CID_MIT_CFG: {
        const int16_t value = (int16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
        const bool ok = srv_juxie_motor_config(d[0], value);
        srv_uart_host_send_ack(ok ? 1U : 0U, (uint8_t)(cid & 0xFFU));
        break;
    }

    case SRV_UART_HOST_CID_MOTOR_ZERO: {
        const bool ok = srv_juxie_motor_zero();
        srv_uart_host_send_ack(ok ? 1U : 0U, (uint8_t)(cid & 0xFFU));
        SRV_UART_HOST_LOG_I("motor zero request %s", ok ? "sent" : "busy");
        break;
    }

    case SRV_UART_HOST_CID_GET_FB:
        srv_uart_host_send_fb();
        break;

    case SRV_UART_HOST_CID_GET_ST:
        srv_uart_host_send_st();
        break;

    case SRV_UART_HOST_CID_GET_MZ:
        srv_uart_host_send_mz();
        break;

    case SRV_UART_HOST_CID_STREAM: {
        uint32_t interval = d[1];
        if (interval == 0U) {
            interval = SRV_UART_HOST_STREAM_DEFAULT_MS;
        }
        if (interval > SRV_UART_HOST_STREAM_MAX_MS) {
            interval = SRV_UART_HOST_STREAM_MAX_MS;
        }
        s_stream_on = (d[0] != 0U);
        s_stream_interval_ms = interval;
        s_stream_last_ms = millis();
        srv_uart_host_send_ack(1U, (uint8_t)(cid & 0xFFU));
        SRV_UART_HOST_LOG_I("stream %s interval=%lu ms",
            s_stream_on ? "on" : "off", (unsigned long)s_stream_interval_ms);
        break;
    }

    case SRV_UART_HOST_CID_ZERO_MZ: {
        const bool ok = srv_mz_sensor_zero();
        srv_uart_host_send_ack(ok ? 1U : 0U, (uint8_t)(cid & 0xFFU));
        SRV_UART_HOST_LOG_I("Mz zero request %s", ok ? "queued" : "busy");
        break;
    }

    default:
        SRV_UART_HOST_LOG_W("unknown command can_id=0x%08lX", (unsigned long)cid);
        srv_uart_host_send_ack(0U, (uint8_t)(cid & 0xFFU));
        break;
    }
}

/**
 * @brief 经协议打包器构建应答帧并入队（TX 忙时不丢帧，由 tx_drain 排队发送）
 * @param cid  应答 can_id
 * @param data data[8] 内容（不足补 0）
 * @param len  有效数据长度（≤8）
 */
static void srv_uart_host_send_frame(uint32_t cid, const uint8_t* data, uint8_t len)
{
    /* 队满：丢弃新帧（避免覆盖仍在 DMA 传输中的最旧帧） */
    if (s_tx_q_cnt >= SRV_UART_HOST_TX_QUEUE_LEN) {
        return;
    }

    /* 负载 = can_id(4B 小端) + data[8] */
    uint8_t payload[SRV_UART_HOST_PAYLOAD_LEN] = { 0 };
    payload[0] = (uint8_t)(cid & 0xFFU);
    payload[1] = (uint8_t)((cid >> 8) & 0xFFU);
    payload[2] = (uint8_t)((cid >> 16) & 0xFFU);
    payload[3] = (uint8_t)((cid >> 24) & 0xFFU);
    if (data && (len > 0U)) {
        memcpy(&payload[4], data, len > 8U ? 8U : len);
    }

    uint8_t* frame = NULL;
    uint16_t flen = 0;
    if (protocol_packer_pack(&s_packer, payload, SRV_UART_HOST_PAYLOAD_LEN,
            &frame, &flen)
        != PROTOCOL_PACKER_OK) {
        return;
    }
    if (flen != SRV_UART_HOST_FRAME_LEN) {
        return;
    }

    /* 拷贝入队（队列缓冲静态存储，DMA 异步读取安全） */
    memcpy(s_tx_q[s_tx_q_tail], frame, SRV_UART_HOST_FRAME_LEN);
    s_tx_q_tail = (uint8_t)((s_tx_q_tail + 1U) % SRV_UART_HOST_TX_QUEUE_LEN);
    s_tx_q_cnt++;
}

/**
 * @brief 排空应答 TX 队列：TX 空闲时逐帧发送（DMA 忙则等下一拍）
 * @note  1ms 步进调用；每帧 DMA 约 0.2ms（1M 波特率），一拍最多发一帧，
 *        对协议负载（读请求应答 + 100Hz 流式）绰绰有余
 */
static void srv_uart_host_tx_drain(void)
{
    while (s_tx_q_cnt > 0U) {
        if (drv_log_uart_is_tx_busy()) {
            return;
        }
        if (drv_log_uart_send(s_tx_q[s_tx_q_head], SRV_UART_HOST_FRAME_LEN) != DRV_LOG_UART_OK) {
            return;
        }
        s_tx_q_head = (uint8_t)((s_tx_q_head + 1U) % SRV_UART_HOST_TX_QUEUE_LEN);
        s_tx_q_cnt--;
    }
}

/**
 * @brief 发送 ACK/NAK
 * @param ok   1=成功，0=失败
 * @param echo 命令字节回显（命令 can_id 低 8 位）
 */
static void srv_uart_host_send_ack(uint8_t ok, uint8_t echo)
{
    uint8_t data[8] = { 0 };
    data[0] = ok;
    data[1] = echo;
    srv_uart_host_send_frame(SRV_UART_HOST_CID_ACK, data, 8);
}

/**
 * @brief 发送电机反馈帧（0x00E00100）
 * @note  pos/speed/iq/tq 均为小端 int16
 */
static void srv_uart_host_send_fb(void)
{
    const srv_juxie_fb_t* fb = srv_juxie_motor_get_fb();
    uint8_t data[8] = { 0 };

    const int16_t pos = srv_uart_host_sat16(fb->pos_deg_x100);
    data[0] = (uint8_t)(pos & 0xFFU);
    data[1] = (uint8_t)((pos >> 8) & 0xFFU);
    data[2] = (uint8_t)(fb->speed_rpm & 0xFFU);
    data[3] = (uint8_t)((fb->speed_rpm >> 8) & 0xFFU);
    data[4] = (uint8_t)(fb->iq_ma & 0xFFU);
    data[5] = (uint8_t)((fb->iq_ma >> 8) & 0xFFU);
    data[6] = (uint8_t)(fb->tq_001nm & 0xFFU);
    data[7] = (uint8_t)((fb->tq_001nm >> 8) & 0xFFU);

    srv_uart_host_send_frame(SRV_UART_HOST_CID_RSP_FB, data, 8);
}

/**
 * @brief 发送电机状态帧（0x00E10100）
 */
static void srv_uart_host_send_st(void)
{
    const srv_juxie_fb_t* fb = srv_juxie_motor_get_fb();
    uint8_t data[8] = { 0 };

    data[0] = (uint8_t)(fb->err & 0xFFU);
    data[1] = (uint8_t)((fb->err >> 8) & 0xFFU);
    data[2] = (uint8_t)(fb->temp_x10 & 0xFFU);
    data[3] = (uint8_t)((fb->temp_x10 >> 8) & 0xFFU);
    data[4] = fb->mode;
    data[5] = fb->status;

    srv_uart_host_send_frame(SRV_UART_HOST_CID_RSP_ST, data, 8);
}

/**
 * @brief 发送 Mz 数据帧（0x00E20000）
 */
static void srv_uart_host_send_mz(void)
{
    const srv_mz_sensor_fb_t* sz = srv_mz_sensor_get();
    uint8_t data[8] = { 0 };

    memcpy(&data[0], &sz->mz_nm, sizeof(sz->mz_nm)); /* float32 小端 */
    const uint16_t seq = (uint16_t)(sz->seq & 0xFFFFU); /* 低 16 位自然回绕 */
    data[4] = (uint8_t)(seq & 0xFFU);
    data[5] = (uint8_t)((seq >> 8) & 0xFFU);
    data[6] = (uint8_t)((sz->online ? 0x01U : 0x00U) | (sz->zero_ok ? 0x02U : 0x00U));

    srv_uart_host_send_frame(SRV_UART_HOST_CID_RSP_MZ, data, 8);
}

/**
 * @brief 发送流式帧（0x00E30000）：Mz + 电机位置 + 电机力矩
 */
static void srv_uart_host_send_stream(void)
{
    const srv_mz_sensor_fb_t* sz = srv_mz_sensor_get();
    const srv_juxie_fb_t* fb = srv_juxie_motor_get_fb();
    uint8_t data[8] = { 0 };

    memcpy(&data[0], &sz->mz_nm, sizeof(sz->mz_nm));
    const int16_t pos = srv_uart_host_sat16(fb->pos_deg_x100);
    data[4] = (uint8_t)(pos & 0xFFU);
    data[5] = (uint8_t)((pos >> 8) & 0xFFU);
    data[6] = (uint8_t)(fb->tq_001nm & 0xFFU);
    data[7] = (uint8_t)((fb->tq_001nm >> 8) & 0xFFU);

    srv_uart_host_send_frame(SRV_UART_HOST_CID_RSP_STREAM, data, 8);
}

/**
 * @brief int32 饱和到 int16
 */
static int16_t srv_uart_host_sat16(int32_t v)
{
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)v;
}
