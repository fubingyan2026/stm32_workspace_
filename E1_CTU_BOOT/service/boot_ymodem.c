/**
 * @file    boot_ymodem.c
 * @brief   YMODEM 接收器实现（Boot 侧，纯协议实现，无平台依赖）
 */

/* Includes ------------------------------------------------------------------*/
#include "boot_ymodem.h"

#include "log.h"

#include <stddef.h>

/* Private constants ---------------------------------------------------------*/

/** @brief SOH(128B) / STX(1KB) 起始符 */
#define YM_SOH (0x01U)
#define YM_STX (0x02U)
/** @brief 控制字符 */
#define YM_EOT (0x04U)
#define YM_ACK (0x06U)
#define YM_NAK (0x15U)
#define YM_CAN (0x18U)
#define YM_C   (0x43U)

/** @brief 头包块号（固定 0） */
#define YM_BLOCK_HEADER (0U)

/** @brief ACK 后向严格型发送端发起 'C' 提示的延迟（容忍自动续发型发送端先到数据） */
#define YM_PROMPT_DELAY_MS (50U)

/** @brief 半包装配停滞判定（ms）：超过该时长仍无字节 → 认为 DMA 丢包/半包，
 *         丢弃装配状态并 NAK 请发送端重发当前包（自愈，防永久失步） */
#define YM_PARTIAL_RESET_MS (120U)

/* CRC-16/xmodem：多项式 0x1021，初值 0 */
static uint16_t ym_crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((crc & 0x8000U) != 0U) {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void ym_send(boot_ymodem_context_t* ctx, uint8_t byte)
{
    if (ctx->cfg->tx_byte) {
        ctx->cfg->tx_byte(ctx->cfg->user, byte);
    }
}

/** @brief 复位包装配状态，回到等待起始符 */
static void ym_reset_pkt(boot_ymodem_context_t* ctx)
{
    ctx->pkt_cnt = 0U;
    ctx->pkt_total = 0U;
}

/** @brief 处理 EOT 字符 */
static void ym_handle_eot(boot_ymodem_context_t* ctx)
{
    if (!ctx->header_ok) {
        /* 未开始传输的 EOT（发送端无文件/取消）：维持等待 */
        return;
    }
    if (ctx->file_done) {
        return;
    }
    if (ctx->eot_phase == 0U) {
        ym_send(ctx, YM_NAK);   /* 第一次 EOT → NAK */
        ctx->eot_phase = 1U;
    } else {
        ym_send(ctx, YM_ACK);   /* 第二次 EOT → ACK，文件结束 */
        ctx->eot_phase = 0U;
        ctx->file_done = true;
    }
}

/** @brief 处理一帧完整数据包（含起始符与 CRC） */
static void ym_process_packet(boot_ymodem_context_t* ctx)
{
    const uint8_t seq = ctx->pkt_buf[1];
    const uint8_t seq_cmp = ctx->pkt_buf[2];
    const uint16_t data_len = (uint16_t)(ctx->pkt_total - 5U);
    const uint8_t* data = &ctx->pkt_buf[3];
    const uint8_t* crc_ptr = &ctx->pkt_buf[3U + data_len];

    /* 序号反码校验 */
    if ((uint8_t)(~seq) != seq_cmp) {
        LOG_W("ymodem", "seq hdr err blk=%u cmp=%u", (unsigned)seq, (unsigned)seq_cmp);
        ym_send(ctx, YM_NAK);
        return;
    }

    /* CRC-16 校验（发送端低字节在前） */
    uint16_t crc_rx = (uint16_t)((uint16_t)crc_ptr[0] | ((uint16_t)crc_ptr[1] << 8));
    uint16_t crc_calc = 0U;
    for (uint16_t i = 0U; i < data_len; i++) {
        crc_calc = ym_crc16_update(crc_calc, data[i]);
    }
    if (crc_calc != crc_rx) {
        LOG_W("ymodem", "crc fail blk=%u len=%u calc=%04X rx=%04X",
            (unsigned)seq, (unsigned)data_len, (unsigned)crc_calc, (unsigned)crc_rx);
        ym_send(ctx, YM_NAK);
        return;
    }

    if (seq == YM_BLOCK_HEADER) {
        /* ===== 头包（文件名/长度），块号 0 ===== */
        if (ctx->header_ok) {
            /* 传输中再次出现头包：单文件设计，拒绝 */
            ym_send(ctx, YM_CAN);
            ctx->aborted = true;
            return;
        }
        /* 解析 YMODEM 文件长度字段：data[124..126] 大端 3 字节 */
        ctx->file_len = 0U;
        if (data_len >= 127U) {
            ctx->file_len = ((uint32_t)data[124] << 16)
                          | ((uint32_t)data[125] << 8)
                          | (uint32_t)data[126];
        }

        ctx->header_ok = true;
        ctx->expected_seq = 1U;
        ctx->last_acked_seq = 0U;
        ctx->received_len = 0U;
        ctx->eot_phase = 0U;
        ym_send(ctx, YM_ACK);
        return;
    }

    /* ===== 数据块 ===== */
    if (!ctx->header_ok) {
        ym_send(ctx, YM_NAK);
        return;
    }

    if (seq == ctx->expected_seq) {
        /* 正常新块：回调落 Flash，成功才 ACK */
        if (ctx->cfg->data_cb
            && ctx->cfg->data_cb(ctx->cfg->user, data, data_len) != 0U) {
            ym_send(ctx, YM_CAN);
            ctx->aborted = true;
            return;
        }
        ctx->received_len += data_len;
        ctx->last_acked_seq = seq;
        ctx->expected_seq = (uint8_t)(seq + 1U);
        ym_send(ctx, YM_ACK);
    } else if (seq == ctx->last_acked_seq) {
        /* ACK 丢失后发送端重发的同一块：不再写 Flash，重复 ACK */
        ym_send(ctx, YM_ACK);
    } else {
        LOG_W("ymodem", "oob blk=%u exp=%u last=%u",
            (unsigned)seq, (unsigned)ctx->expected_seq, (unsigned)ctx->last_acked_seq);
        ym_send(ctx, YM_NAK);
    }
}

/* Exported functions --------------------------------------------------------*/

void boot_ymodem_init(boot_ymodem_context_t* ctx, const boot_ymodem_config_t* cfg)
{
    if (!ctx || !cfg) {
        return;
    }
    ctx->cfg = cfg;
    ctx->header_ok = false;
    ctx->file_done = false;
    ctx->aborted = false;
    ctx->expected_seq = 1U;
    ctx->last_acked_seq = 0U;
    ctx->eot_phase = 0U;
    ctx->received_len = 0U;
    ctx->file_len = 0U;
    ctx->last_rx_ms = 0U;
    ctx->last_prompt_ms = 0U;
    ym_reset_pkt(ctx);
}

void boot_ymodem_feed(boot_ymodem_context_t* ctx, uint8_t byte)
{
    if (!ctx || !ctx->cfg) {
        return;
    }
    if (ctx->aborted || ctx->file_done) {
        return;
    }
    /* 说明：字节活动时刻 last_rx_ms 由调用方在喂入一批字节后统一更新 */

    if (ctx->pkt_total == 0U) {
        /* 等待包起始符 */
        if (byte == YM_SOH) {
            ctx->pkt_total = BOOT_YM_PKT_SOH_LEN;
        } else if (byte == YM_STX) {
            ctx->pkt_total = BOOT_YM_PKT_STX_LEN;
        } else if (byte == YM_EOT) {
            ym_handle_eot(ctx);
            return;
        } else if (byte == YM_CAN) {
            ctx->aborted = true;
            return;
        } else {
            /* 其他字节视为噪声丢弃 */
            return;
        }
        ctx->pkt_buf[0] = byte;
        ctx->pkt_cnt = 1U;
        return;
    }

    /* 包体收集中 */
    if (ctx->pkt_cnt < ctx->pkt_total) {
        ctx->pkt_buf[ctx->pkt_cnt] = byte;
        ctx->pkt_cnt++;
    }
    if (ctx->pkt_cnt == ctx->pkt_total) {
        ym_process_packet(ctx);
        ym_reset_pkt(ctx);
    }
}

void boot_ymodem_poll(boot_ymodem_context_t* ctx, uint32_t now_ms)
{
    if (!ctx || !ctx->cfg) {
        return;
    }
    if (ctx->aborted || ctx->file_done) {
        return;
    }

    /* 包装配中：
       - 不打扰（自动续发型发送端数据在路上 / 首包进行中）
       - 但超过 YM_PARTIAL_RESET_MS 无字节 → 判为丢包/半包（如 UART 错误复位），
         丢弃装配状态并 NAK 请发送端重发当前包，防止装配永久失步 */
    if (ctx->pkt_total != 0U) {
        if (now_ms - ctx->last_rx_ms >= YM_PARTIAL_RESET_MS) {
            ym_reset_pkt(ctx);
            ym_send(ctx, YM_NAK);
            ctx->last_rx_ms = now_ms;
            ctx->last_prompt_ms = now_ms;
        }
        return;
    }

    if (!ctx->header_ok) {
        /* 等待首个头包：按周期发 'C' 邀请发送端 */
        if (now_ms - ctx->last_prompt_ms >= ctx->cfg->header_period_ms) {
            ym_send(ctx, YM_C);
            ctx->last_prompt_ms = now_ms;
            ctx->last_rx_ms = now_ms;
        }
        return;
    }

    /* 头包已确认且空闲等待：50ms 后发 'C' 提示严格型发送端；超时则同样以 'C' 请求重发 */
    const uint32_t delay = (ctx->cfg->packet_timeout_ms < YM_PROMPT_DELAY_MS)
        ? ctx->cfg->packet_timeout_ms : YM_PROMPT_DELAY_MS;
    if (now_ms - ctx->last_rx_ms >= delay) {
        ym_send(ctx, YM_C);
        ctx->last_rx_ms = now_ms;
        ctx->last_prompt_ms = now_ms;
    }
}

bool boot_ymodem_started(const boot_ymodem_context_t* ctx)
{
    return (ctx != NULL) && ctx->header_ok;
}

bool boot_ymodem_file_done(const boot_ymodem_context_t* ctx)
{
    return (ctx != NULL) && ctx->file_done;
}

bool boot_ymodem_aborted(const boot_ymodem_context_t* ctx)
{
    return (ctx != NULL) && ctx->aborted;
}

uint32_t boot_ymodem_received_len(const boot_ymodem_context_t* ctx)
{
    return (ctx != NULL) ? ctx->received_len : 0U;
}

uint32_t boot_ymodem_file_len(const boot_ymodem_context_t* ctx)
{
    return (ctx != NULL) ? ctx->file_len : 0U;
}
