/**
 * @file    srv_boot_ctrl.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-11
 * @brief   升级跳转控制服务（E1_MASTER_POWER_CTU → E1_CTU_BOOT）
 * @attention
 *
 * 通过 boot_flash（E1_CTU_BOOT/service，boot 相关代码统一由 Boot 工程持有）
 * 向共享 Metadata 区写入 upgrade_flag=1 后复位，使 Bootloader 上电进入
 * RS485 YMODEM 升级模式。
 *
 * 分区/Metadata 契约见 ../E1_CTU_BOOT/docs/boot_485_ymodem.md：
 *   Meta 0x0803E000(8K, ring_storage, sector 2K)；
 *   upgrade_flag：1=下载中/请求升级。
 */

#ifndef __SRV_BOOT_CTRL_H
#define __SRV_BOOT_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 请求进入 Bootloader（写 upgrade_flag=1 到共享 Metadata 区）
 * @return true=成功（调用方随后复位即可）；false=失败（不应复位）
 * @note  Flash 擦写期间会短暂关中断（hal_flash 默认裸机锁）
 */
bool srv_boot_ctrl_request_upgrade(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_BOOT_CTRL_H */
