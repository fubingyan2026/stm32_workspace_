/**
 * @file    srv_boot_ctrl.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-15
 * @brief   升级控制服务（E1_MASTER_POWER_CTU → E1_CTU_BOOT）
 * @attention
 *
 * 升级流程（App 内直接下载，全程不跳转、不断电）：
 *   主机 0x06 → 本服务进入「App 内升级会话」，485 字节改由 boot_session 解析；
 *   SELECT/START/DATA/END 全程在 App 内把新固件写入 App B 暂存槽并回读校验；
 *   END 校验通过 → 写共享 metadata{upgrade_flag=2, fw_size, fw_checksum}，
 *   **不复位、继续以当前固件运行**；下次重新上电时 Boot 见 flag==2 提升 B→A
 *   并复位运行新固件（即“重新上电才生效”）。
 *
 * 关键约束：
 *   - App 运行于 A 槽，绝不能擦写 A；只写 B 暂存槽（boot_session 负责）；
 *   - 会话期间主循环会被 Flash 擦写阻塞（F1 无 RWW），电源轨保持锁存；
 *   - ABORT 只退出会话，不复位（App 继续正常运行）；
 *   - 掉电中断下载不会变砖：A 不受影响，B 为脏数据，下次 START 重擦。
 *
 * 下载状态机与 Boot 共用 E1_CTU_BOOT/service/boot_session + boot_flash
 * （boot 相关代码统一由 Boot 工程持有）。布局/契约见
 * ../E1_CTU_BOOT/docs/boot_485_ymodem.md。
 */

#ifndef __SRV_BOOT_CTRL_H
#define __SRV_BOOT_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "boot_flash.h"

/* Exported types ------------------------------------------------------------*/

/** @brief 整帧发送回调（task 层 → dev_rs485_send，自带半双工方向控制） */
typedef void (*srv_boot_ctrl_tx_cb_t)(const uint8_t* frame, uint32_t len);

/** @brief 升级控制配置 */
typedef struct {
    srv_boot_ctrl_tx_cb_t tx; /**< 整帧发送（必填） */
    uint8_t dev_id;           /**< 本机 485 设备 ID（MASTER=0x01 / SLAVER=0x02） */
} srv_boot_ctrl_config_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化升级控制服务（仅保存配置，不写 Flash） */
void srv_boot_ctrl_init(const srv_boot_ctrl_config_t* cfg);

/**
 * @brief 收到 0x06 升级请求：进入 App 内升级会话
 * @return true=已进入会话（后续 485 字节应改喂 srv_boot_ctrl_feed）；false=失败
 * @note  仅只读加载 metadata；不写 Flash、不累加启动次数
 */
bool srv_boot_ctrl_enter_upgrade(void);

/** @brief 是否处于升级会话（决定 485 字节路由：会话内 → srv_boot_ctrl_feed） */
bool srv_boot_ctrl_is_upgrade_active(void);

/** @brief 升级会话字节流喂入（内部 boot_session → boot_proto 解析/应答） */
void srv_boot_ctrl_feed(const uint8_t* data, uint32_t len);

/** @brief 外部中止升级会话（如主机空闲超时），恢复普通 App 运行（不复位） */
void srv_boot_ctrl_abort_session(void);

/**
 * @brief 只读读取 Boot metadata（0x07 信息查询用）
 * @param metadata 输出：metadata
 * @return true=成功
 * @note  不写 Flash、不累加启动次数；与会话共用同一 ring_storage 实例，
 *        避免同区两个实例互相覆盖。
 */
bool srv_boot_ctrl_peek_metadata(boot_metadata_t* metadata);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_BOOT_CTRL_H */
