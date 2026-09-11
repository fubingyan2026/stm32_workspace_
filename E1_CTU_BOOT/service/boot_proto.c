/**
 * @file    boot_proto.c
 * @brief   Bootloader RS485 寻址分块升级协议实现（方案 B）
 */

/* Includes ------------------------------------------------------------------*/
#include "boot_proto.h"

#include "crc.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

#define P_HDR    (0x7AU)
#define P_FOOT   (0x0AU)

/** @brief 解析状态 */
#define ST_HDR   (0U)
#define ST_CMD   (1U)
#define ST_LEN   (2U)
#define ST_PAY   (3U)
#define ST_CRC   (4U)
#define ST_FOOT  (5U)

/** @brief 最大负载（id + blk u16 + crc16 u16 + data） */
#define P_PAYLOAD_MAX (BOOT_PROTO_DATA_MAX + 5U)

/* CRC-16/xmodem：多项式 0x1021，初值 0 */
static uint16_t proto_crc16(const uint8_t* data, uint32_t len)
{
    uint16_t crc = 0U;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = ((crc & 0x8000U) != 0U)
                ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void proto_reset_rx(boot_proto_context_t* ctx)
{
    ctx->rx_cnt = 0U;
    ctx->rx_state = ST_HDR;
}

/** @brief 组装并发送应答帧 */
static void proto_reply(boot_proto_context_t* ctx, uint8_t cmd,
    const uint8_t* payload, uint8_t plen)
{
    uint8_t frame[16];
    frame[0] = P_HDR;
    frame[1] = cmd;
    frame[2] = plen;
    if (plen > 0U && payload != NULL) {
        memcpy(&frame[3], payload, plen);
    }
    const uint16_t n = (uint16_t)(3U + plen);
    frame[n] = get_CRC8_check_sum(frame, n, 0xFF);
    frame[n + 1U] = P_FOOT;
    if (ctx->cfg->tx) {
        (void)ctx->cfg->tx(frame, (uint32_t)(n + 2U));
    }
}

static bool proto_addr_ok(boot_proto_context_t* ctx, uint8_t id)
{
    if (ctx->my_id == 0U) {
        if (id != 0U) {
            ctx->my_id = id; /* 未定 ID：学习本次目标 ID */
            if (ctx->cfg->on_id) {
                ctx->cfg->on_id(ctx->cfg->user, id);
            }
        }
        return true;
    }
    return (id == ctx->my_id) || (id == BOOT_PROTO_ID_BROADCAST);
}

/** @brief 命令处理 */
static void proto_dispatch(boot_proto_context_t* ctx, uint8_t cmd,
    const uint8_t* pl, uint8_t len)
{
    uint8_t rep[8];
    const uint8_t id = (len >= 1U) ? pl[0] : 0xFFU;

    if (!proto_addr_ok(ctx, id)) {
        return; /* 非本机帧：静默丢弃 */
    }
    const uint8_t* body = (len >= 1U) ? &pl[1] : pl;
    const uint8_t blen = (len >= 1U) ? (uint8_t)(len - 1U) : 0U;
    const uint8_t src = ctx->my_id;

    switch (cmd) {
    case BOOT_PROTO_CMD_SELECT: {
        uint8_t err = BOOT_PROTO_ERR_NONE;
        if (blen != 1U || body[0] != 0x01U) {
            err = BOOT_PROTO_ERR_BAD_LEN;
        } else {
            ctx->selected = true;
            ctx->downloading = false;
        }
        rep[0] = src;
        rep[1] = err;
        proto_reply(ctx, (uint8_t)(cmd | 0x80U), rep, 2U);
        break;
    }
    case BOOT_PROTO_CMD_START: {
        uint8_t err;
        if (!ctx->selected || blen != 8U) {
            err = ctx->selected ? BOOT_PROTO_ERR_BAD_LEN : BOOT_PROTO_ERR_STATE;
        } else {
            ctx->fw_size = (uint32_t)body[0] | ((uint32_t)body[1] << 8)
                | ((uint32_t)body[2] << 16) | ((uint32_t)body[3] << 24);
            ctx->fw_sum = (uint32_t)body[4] | ((uint32_t)body[5] << 8)
                | ((uint32_t)body[6] << 16) | ((uint32_t)body[7] << 24);
            ctx->expected_blk = 0U;
            ctx->rx_bytes = 0U;
            err = ctx->cfg->on_start(ctx->cfg->user, ctx->fw_size, ctx->fw_sum);
            if (err == BOOT_PROTO_ERR_NONE) {
                ctx->downloading = true;
            }
        }
        rep[0] = src;
        rep[1] = err;
        proto_reply(ctx, (uint8_t)(cmd | 0x80U), rep, 2U);
        break;
    }
    case BOOT_PROTO_CMD_DATA: {
        uint8_t err = BOOT_PROTO_ERR_NONE;
        uint16_t blk = 0U;
        if (!ctx->downloading || blen < 4U) {
            err = BOOT_PROTO_ERR_STATE;
        } else {
            blk = (uint16_t)((uint16_t)body[0] | ((uint16_t)body[1] << 8));
            const uint16_t crc_rx = (uint16_t)((uint16_t)body[2]
                | ((uint16_t)body[3] << 8));
            const uint8_t* data = &body[4];
            const uint32_t dlen = (uint32_t)(blen - 4U);
            if (proto_crc16(data, dlen) != crc_rx) {
                err = BOOT_PROTO_ERR_CRC;
            } else if (blk == ctx->expected_blk) {
                err = ctx->cfg->on_data(ctx->cfg->user, blk, data, dlen);
                if (err == BOOT_PROTO_ERR_NONE) {
                    ctx->expected_blk++;
                    ctx->rx_bytes += dlen;
                }
            } else if (blk + 1U == ctx->expected_blk) {
                err = BOOT_PROTO_ERR_NONE; /* 重复块：重复 ACK，不重写 */
            } else {
                err = BOOT_PROTO_ERR_SEQ;
            }
        }
        rep[0] = src;
        rep[1] = err;
        rep[2] = (uint8_t)(blk & 0xFFU);
        rep[3] = (uint8_t)(blk >> 8);
        proto_reply(ctx, (uint8_t)(cmd | 0x80U), rep, 4U);
        break;
    }
    case BOOT_PROTO_CMD_END: {
        uint8_t err = BOOT_PROTO_ERR_NONE;
        if (!ctx->downloading) {
            err = BOOT_PROTO_ERR_STATE;
        } else if (ctx->rx_bytes != ctx->fw_size) {
            err = BOOT_PROTO_ERR_SIZE;
        }
        rep[0] = src;
        rep[1] = err;
        proto_reply(ctx, (uint8_t)(cmd | 0x80U), rep, 2U);
        if (err == BOOT_PROTO_ERR_NONE) {
            ctx->cfg->on_end(ctx->cfg->user, ctx->fw_size, ctx->fw_sum);
        }
        break;
    }
    case BOOT_PROTO_CMD_ABORT: {
        rep[0] = src;
        rep[1] = BOOT_PROTO_ERR_NONE;
        proto_reply(ctx, (uint8_t)(cmd | 0x80U), rep, 2U);
        ctx->cfg->on_abort(ctx->cfg->user);
        break;
    }
    default:
        break;
    }
}

/* Exported functions --------------------------------------------------------*/

void boot_proto_init(boot_proto_context_t* ctx, const boot_proto_config_t* cfg)
{
    if (!ctx || !cfg) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->cfg = cfg;
    ctx->my_id = cfg->my_id;
    proto_reset_rx(ctx);
}

void boot_proto_feed(boot_proto_context_t* ctx, uint8_t byte)
{
    if (!ctx || !ctx->cfg) {
        return;
    }

    switch (ctx->rx_state) {
    case ST_HDR:
        if (byte == P_HDR) {
            ctx->rx_buf[0] = byte;
            ctx->rx_cnt = 1U;
            ctx->rx_state = ST_CMD;
        }
        break;
    case ST_CMD:
        ctx->rx_buf[1] = byte;
        ctx->rx_cnt = 2U;
        ctx->rx_state = ST_LEN;
        break;
    case ST_LEN:
        if (byte > (uint8_t)P_PAYLOAD_MAX) {
            proto_reset_rx(ctx);
            break;
        }
        ctx->rx_buf[2] = byte;
        ctx->rx_cnt = 3U;
        ctx->rx_total = (uint16_t)byte + 5U;
        ctx->rx_state = (byte == 0U) ? ST_CRC : ST_PAY;
        break;
    case ST_PAY:
        ctx->rx_buf[ctx->rx_cnt] = byte;
        ctx->rx_cnt++;
        if (ctx->rx_cnt == (uint16_t)(ctx->rx_total - 2U)) {
            ctx->rx_state = ST_CRC;
        }
        break;
    case ST_CRC:
        ctx->rx_buf[ctx->rx_cnt] = byte;
        ctx->rx_cnt++;
        ctx->rx_state = ST_FOOT;
        break;
    case ST_FOOT:
        ctx->rx_buf[ctx->rx_cnt] = byte;
        ctx->rx_cnt++;
        if ((byte == P_FOOT) && (ctx->rx_cnt == ctx->rx_total)
            && (get_CRC8_check_sum(ctx->rx_buf,
                    (uint16_t)(ctx->rx_total - 2U), 0xFF)
                == ctx->rx_buf[ctx->rx_total - 2U])) {
            proto_dispatch(ctx, ctx->rx_buf[1], &ctx->rx_buf[3],
                ctx->rx_buf[2]);
        }
        proto_reset_rx(ctx);
        break;
    default:
        proto_reset_rx(ctx);
        break;
    }
}

bool boot_proto_selected(const boot_proto_context_t* ctx)
{
    return (ctx != NULL) && ctx->selected;
}
