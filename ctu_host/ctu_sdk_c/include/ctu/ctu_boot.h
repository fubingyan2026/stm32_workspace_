//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_boot.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   Boot 固件升级：0x06 邀请 + SELECT/START/DATA/END 分块传输。
 * @note    与固件 boot_proto 及上位机 boot_protocol.py 行为一致；
 *          升级期间必须独占串口，不得与轮询请求交错。
 */

#ifndef __CTU_BOOT_H
#define __CTU_BOOT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ctu_config.h"
#include "ctu_error.h"
#include "ctu_transport.h"
#include "ctu_types.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Boot 升级配置（含回调）
 */
typedef struct {
    uint32_t invite_delay_ms;   /**< 0x06 邀请后等待板端进入 Boot 的延时（ms） */
    uint32_t select_timeout_ms; /**< SELECT 应答超时（ms） */
    uint32_t block_timeout_ms;  /**< 每个 DATA 块应答超时（ms） */
    uint32_t end_timeout_ms;    /**< END 应答超时（ms，板端需校验与提交） */
    uint8_t max_retries;        /**< 单步骤最大重试次数 */
    ctu_progress_cb_t progress_cb; /**< 进度回调（可为 NULL） */
    ctu_phase_cb_t phase_cb;       /**< 阶段回调（可为 NULL） */
    ctu_log_cb_t log_cb;           /**< 日志回调（可为 NULL） */
    void* user;                    /**< 回调用户上下文 */
} ctu_boot_config_t;

/**
 * @brief Boot 升级上下文
 */
typedef struct {
    ctu_boot_config_t config;      /**< 配置与回调 */
    ctu_device_t device;           /**< 目标设备 */
    uint8_t addr;                  /**< 目标设备地址 */
    uint32_t fw_size;              /**< 固件字节数 */
    uint32_t fw_checksum;          /**< 固件累加和 */
    uint8_t last_error;            /**< 最近一次 Boot 错误码 */
    const volatile int* cancel;    /**< 取消标志指针（可为 NULL） */
    bool initialized;              /**< 是否已初始化 */
} ctu_boot_context_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 Boot 升级上下文
 * @param ctx    上下文
 * @param config 配置（可为 NULL，使用默认值）
 * @return 操作结果
 */
ctu_error_t ctu_boot_init(ctu_boot_context_t* ctx, const ctu_boot_config_t* config);

/**
 * @brief 校验固件镜像（向量表 sanity）
 * @param data       固件数据（至少 16B）
 * @param length     数据长度
 * @param reason     失败原因输出缓冲（可为 NULL）
 * @param capacity   reason 缓冲容量
 * @return CTU_OK 通过；CTU_ERROR_IMAGE 未通过
 */
ctu_error_t ctu_boot_check_image(const uint8_t* data, size_t length,
                                 char* reason, size_t capacity);

/**
 * @brief 校验固件文件并给出大小与累加和
 * @param filepath       文件路径
 * @param out_size       输出字节数（可为 NULL）
 * @param out_checksum   输出累加和（可为 NULL）
 * @param reason         失败原因输出缓冲（可为 NULL）
 * @param capacity       reason 缓冲容量
 * @return 操作结果
 */
ctu_error_t ctu_boot_check_file(const char* filepath, uint32_t* out_size,
                                uint32_t* out_checksum, char* reason,
                                size_t capacity);

/**
 * @brief 按文件名在常见构建输出目录中定位固件镜像（命令行辅助）
 * @param name     文件名（不含路径分隔符）或路径
 * @param out      输出缓冲，返回可直接使用的路径
 * @param capacity 输出容量
 * @return CTU_OK 找到；CTU_ERROR_FILE 未找到；其它为参数错误
 * @note  若 ``name`` 已含 ``/``，仅检查其是否存在；否则从当前目录逐级向上
 *        （最多 4 级）在各级目录的 build 产物子目录中按文件名查找，
 *        跳过 CMake 内部产物，并优先选择含 RelWithDebInfo 或 Release 的路径。
 */
ctu_error_t ctu_boot_locate_image(const char* name, char* out, size_t capacity);

/**
 * @brief 发送 0x06 邀请（App 复位进 Boot / 已在 Boot 则等同 SELECT）
 * @param ctx       上下文
 * @param transport 传输层
 * @return 操作结果
 */
ctu_error_t ctu_boot_invite(ctu_boot_context_t* ctx, ctu_transport_t* transport);

/**
 * @brief 执行完整固件升级
 * @param ctx       上下文（需已设置 device）
 * @param transport 传输层
 * @param filepath  固件文件路径
 * @return 操作结果
 */
ctu_error_t ctu_boot_transfer(ctu_boot_context_t* ctx, ctu_transport_t* transport,
                              const char* filepath);

/**
 * @brief 查询是否需要取消
 * @param ctx 上下文
 * @return true 表示已请求取消
 */
bool ctu_boot_is_canceled(const ctu_boot_context_t* ctx);

#ifdef __cplusplus
}
#endif

#endif /* __CTU_BOOT_H */
