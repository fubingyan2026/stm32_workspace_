/**
 * @file    boot_session.h
 * @brief   升级会话（Boot/App 共用的下载状态机）：寻址分块接收 + 暂存槽写入
 * @attention
 *
 * 本模块把「下载」与「提交」分离，供 Bootloader 与 App 复用同一套流程：
 *   SELECT(0x06) → START(0x08，擦除暂存槽) → DATA(0x09×N，写槽 + 回读校验)
 *   → END(0x0A，长度/累加和校验) → on_commit(size, checksum)
 *
 * 本模块只负责把固件安全写入暂存槽（默认 App B 槽）并统计 32 位累加和，
 * 不切换活动分区、不擦写 App A 槽。提交策略由 on_commit 决定：
 *   - Boot：promote B→A 后复位；
 *   - App ：写 metadata{flag=2, size, sum} 后复位，由 Boot 上电续提交
 *           （App 运行于 A 槽，不能在运行时擦写 A）。
 *
 * 依赖 boot_proto（帧/块协议）+ boot_flash（分区擦写），平台无关，
 * 仅需调用者提供一个整帧发送回调（经 dev_rs485）。
 */

#ifndef __BOOT_SESSION_H
#define __BOOT_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "boot_flash.h"
#include "boot_proto.h"

/* Exported types ------------------------------------------------------------*/

/** @brief 整帧发送回调（经 dev_rs485，自带半双工方向控制；失败可忽略） */
typedef void (*boot_session_tx_cb_t)(const uint8_t* frame, uint32_t len);

/** @brief END 校验通过后的提交回调（size=已写入字节数，checksum=32 位累加和） */
typedef void (*boot_session_commit_cb_t)(void* user, uint32_t size,
    uint32_t checksum);

/** @brief 会话中止回调（ABORT，或 END 长度不符） */
typedef void (*boot_session_abort_cb_t)(void* user);

/** @brief 学习到本机 ID 回调（可选，用于持久化到 metadata.reserved） */
typedef void (*boot_session_id_cb_t)(void* user, uint8_t id);

/** @brief 升级会话配置 */
typedef struct {
    boot_flash_context_t* flash;        /**< 分区管理上下文（必填，调用者持有且已初始化） */
    boot_partition_t target;            /**< 暂存槽（必填，通常 BOOT_PARTITION_B） */
    boot_session_tx_cb_t tx;            /**< 整帧发送（必填） */
    boot_session_commit_cb_t on_commit; /**< END 后提交（必填） */
    boot_session_abort_cb_t on_abort;   /**< 中止（可空） */
    boot_session_id_cb_t on_id;         /**< 学习 ID（可空） */
    uint8_t my_id;                      /**< 本机 ID；0=未定（接受广播/任意并学习） */
    void* user;                         /**< 回调用户指针 */
} boot_session_config_t;

/** @brief 升级会话上下文（调用者静态分配） */
typedef struct {
    const boot_session_config_t* cfg;
    boot_proto_config_t proto_cfg; /**< 内部：boot_proto 配置（绑定下方静态回调） */
    boot_proto_context_t proto;    /**< 内部：协议解析状态 */
    uint32_t upg_offset;           /**< 已写入暂存槽的字节数 */
    uint32_t upg_checksum;         /**< 已写入字节的 32 位累加和 */
} boot_session_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化/重置升级会话（内部初始化 boot_proto；不擦写 Flash） */
void boot_session_init(boot_session_t* session, const boot_session_config_t* cfg);

/** @brief 喂入一个接收字节（主循环按字节流调用） */
void boot_session_feed(boot_session_t* session, uint8_t byte);

/** @brief 是否正在下载（供 LED 指示） */
bool boot_session_downloading(const boot_session_t* session);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_SESSION_H */
