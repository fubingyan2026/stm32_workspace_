//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_config.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU SDK 编译期配置：版本、协议常量、容量上限。
 * @attention
 *
 * Copyright (c) 2025 E1.
 * All rights reserved.
 */

#ifndef __CTU_CONFIG_H
#define __CTU_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/

/* SDK 版本 */
#define CTU_SDK_VERSION_MAJOR 1U      /**< 主版本号 */
#define CTU_SDK_VERSION_MINOR 0U      /**< 次版本号 */
#define CTU_SDK_VERSION_PATCH 0U      /**< 修订号 */
#define CTU_SDK_VERSION_STRING "1.0.0" /**< 版本字符串 */

/* z 帧信封 */
#define CTU_PROTOCOL_HEADER 0x7AU     /**< 帧头 'z' */
#define CTU_PROTOCOL_FOOT 0x0AU       /**< 帧尾 '\n' */
#define CTU_PROTOCOL_REPLY_FLAG 0x80U /**< 应答命令码标志位 */
#define CTU_PROTOCOL_ERR_CMD 0x7FU    /**< 错误应答命令码 */

/* 帧容量：payload 最大 255B，帧总长 = payload + 5 */
#define CTU_PROTOCOL_MAX_PAYLOAD 255U                        /**< 最大 payload */
#define CTU_PROTOCOL_MAX_FRAME (CTU_PROTOCOL_MAX_PAYLOAD + 5U) /**< 最大帧长 */
#define CTU_PROTOCOL_OVERHEAD 5U                             /**< 帧头+cmd+len+crc+帧尾 */

/* 固件 / Boot 升级 */
#define CTU_FIRMWARE_MAX_SIZE 0x18000U /**< App 分区容量 96KB */
#define CTU_BOOT_DATA_MAX 248U         /**< 每块最大数据字节（与固件一致） */
#define CTU_BOOT_APP_VECT_ADDR 0x08008000U               /**< AppA 向量表基地址 */
#define CTU_BOOT_APP_END_ADDR (0x08008000U + 0x18000U)   /**< AppA 区间结束 */
#define CTU_BOOT_RAM_BASE 0x20000000U                    /**< RAM 起始 */
#define CTU_BOOT_RAM_END (0x20000000U + 48U * 1024U)     /**< RAM 结束（48KB） */

/* 设备地址 */
#define CTU_MASTER_ADDR 0x01U /**< E1_MASTER 主控板地址 */
#define CTU_SLAVER_ADDR 0x02U /**< E1_SLAVER 副电源模块地址 */

/* 默认通信参数 */
#define CTU_DEFAULT_BAUD 115200U       /**< 默认波特率 */
#define CTU_DEFAULT_TIMEOUT_MS 200U    /**< 默认单次请求超时（ms） */
#define CTU_BOOT_INVITE_DELAY_MS 500U  /**< 0x06 邀请后等待进入 Boot 的时间（ms） */
#define CTU_PORT_NAME_MAX 128U         /**< 串口设备名最大长度 */

#ifdef __cplusplus
}
#endif

#endif /* __CTU_CONFIG_H */
