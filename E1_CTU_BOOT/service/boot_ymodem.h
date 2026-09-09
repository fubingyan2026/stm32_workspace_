/**
 * @file    boot_ymodem.h
 * @brief   YMODEM 接收器（Boot 侧，纯协议实现，无平台依赖）
 * @attention
 *
 * 本模块实现标准 YMODEM *接收端*（CRC-16/xmodem 模式，支持 128B/1KB 数据包）：
 *   - 接收端先发 'C'，Host（发送端）回 block0（文件名/长度头包），随后数据包
 *   - 数据包按块序号 1..N 递增，CRC 校验通过才回调写 Flash，块级 ACK/NAK
 *   - EOT → NAK / EOT → ACK 结束单文件传输（file_done）
 *   - 1KB 数据包瞬时涌入：驱动层负责 RX 缓冲与读取，本模块按字节流喂入
 *
 * 字节收发（单字符）与"有效数据落 Flash"均通过 config 回调转交平台层，
 * 本模块不直接触碰 UART/Flash/时间源。
 */

#ifndef __BOOT_YMODEM_H
#define __BOOT_YMODEM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief SOH 包总长（含起始符/序号/CRC）：1+1+1+128+2 */
#define BOOT_YM_PKT_SOH_LEN (133U)
/** @brief STX 包总长（含起始符/序号/CRC）：1+1+1+1024+2 */
#define BOOT_YM_PKT_STX_LEN (1029U)
/** @brief 单包数据最大长度（STX） */
#define BOOT_YM_DATA_MAX    (1024U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 有效数据块回调：把 len 字节数据写入暂存分区
 * @param user 用户指针（config.user）
 * @param data 数据指针（当前数据块，len ≤ 1024）
 * @param len  数据长度
 * @return 0=成功；非 0=写入/校验失败（模块将发送 CAN 中止）
 */
typedef uint8_t (*boot_ym_data_cb_t)(void* user, const uint8_t* data, uint32_t len);

/** @brief 发送单字节（ACK/NAK/C/CAN），平台层保证半双工方向控制 */
typedef void (*boot_ym_tx_cb_t)(void* user, uint8_t byte);

/** @brief YMODEM 接收器配置 */
typedef struct {
    boot_ym_tx_cb_t tx_byte;              /**< 发送单字节回调 */
    boot_ym_data_cb_t data_cb;            /**< 有效数据块回调 */
    void* user;                           /**< 用户指针（透传回调） */
    uint32_t header_period_ms;            /**< 等待首个头包时 'C' 心跳周期 */
    uint32_t packet_timeout_ms;           /**< 已开始后数据包等待超时 → NAK 重发 */
} boot_ymodem_config_t;

/** @brief YMODEM 接收器上下文 */
typedef struct {
    const boot_ymodem_config_t* cfg;      /**< 配置指针 */

    /* 接收状态 */
    uint8_t  pkt_buf[BOOT_YM_PKT_STX_LEN]; /**< 原始包缓冲 */
    uint16_t pkt_cnt;                      /**< 当前包已收字节数 */
    uint16_t pkt_total;                    /**< 当前包总长（133/1029） */

    bool     header_ok;                    /**< 头包(block0)已 ACK */
    bool     file_done;                    /**< EOT/EOT 结束（等待上层提交） */
    bool     aborted;                      /**< 收到 CAN / 致命错误 */

    uint8_t  expected_seq;                 /**< 期望的下一个数据块序号 */
    uint8_t  last_acked_seq;               /**< 最近 ACK 的数据块序号 */

    uint8_t  eot_phase;                    /**< EOT 处理阶段（0=首个,1=次个） */

    uint32_t received_len;                 /**< 已接收数据字节数（含 1A 填充） */
    uint32_t file_len;                     /**< 头包声明的文件长度（0=未知） */
    uint32_t block0_ticks;                 /**< 保留 */

    uint32_t last_rx_ms;                   /**< 最近一次字节活动时刻 */
    uint32_t last_prompt_ms;               /**< 最近一次发送 'C'/NAK 时刻 */
} boot_ymodem_context_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 YMODEM 接收器
 * @param ctx 上下文指针
 * @param cfg 配置指针（须在 ctx 生命周期内有效）
 */
void boot_ymodem_init(boot_ymodem_context_t* ctx, const boot_ymodem_config_t* cfg);

/**
 * @brief 喂入一个接收字节（主循环按驱动字节流调用）
 * @param ctx  上下文指针
 * @param byte 字节
 */
void boot_ymodem_feed(boot_ymodem_context_t* ctx, uint8_t byte);

/**
 * @brief 周期调用：'C' 心跳与数据包超时重发调度
 * @param ctx    上下文指针
 * @param now_ms 当前时刻（ms）
 */
void boot_ymodem_poll(boot_ymodem_context_t* ctx, uint32_t now_ms);

/** @brief 是否已进入文件传输（头包已 ACK） */
bool boot_ymodem_started(const boot_ymodem_context_t* ctx);

/** @brief 文件是否已结束（EOT/EOT，等待上层提交窗口） */
bool boot_ymodem_file_done(const boot_ymodem_context_t* ctx);

/** @brief 是否已中止（CAN/错误） */
bool boot_ymodem_aborted(const boot_ymodem_context_t* ctx);

/**
 * @brief 查询已接收的数据字节数（含末块 1A 填充）
 * @param ctx 上下文指针
 * @return 字节数
 */
uint32_t boot_ymodem_received_len(const boot_ymodem_context_t* ctx);

/**
 * @brief 查询头包声明的文件长度
 * @param ctx 上下文指针
 * @return 文件长度；0=未知（未解析到头包/未提供）
 */
uint32_t boot_ymodem_file_len(const boot_ymodem_context_t* ctx);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_YMODEM_H */
