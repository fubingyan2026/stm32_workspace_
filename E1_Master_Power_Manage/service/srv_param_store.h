/**
 * @file    srv_param_store.h
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-08-06
 * @brief   参数存储服务 — 基于 ring_storage 的单分区 Flash 参数持久化（APP 常用参数）
 * @attention
 *
 * 遵循 service 层 driver-wrapping 模式：自持一个 ring_storage 实例，直接包装 hal_flash。
 * 同层解耦：本服务只管理 APP 参数分区；BOOT 分区 metadata 由 srv_boot_ctrl 独立管理，
 * 两者不相互调用（各持一块独立 flash 实例）。
 *
 * ## 用法
 * @code
 *   srv_param_store_init();                    // hal_flash + ring_storage(APP 分区)
 *   srv_param_store_register("fan_max_duty", &g_fan_max, sizeof(g_fan_max));
 *   srv_param_store_load();                    // 恢复参数（首次返回 NO_VALID_FRAME）
 *   // ... 修改参数后
 *   srv_param_store_save();                    // 整帧持久化
 * @endcode
 */

#ifndef __SRV_PARAM_STORE_H
#define __SRV_PARAM_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 服务错误码
 */
typedef enum {
    SRV_PARAM_STORE_OK = 0, /**< 操作成功 */
    SRV_PARAM_STORE_ERROR_NULL_PTR, /**< 空指针参数 */
    SRV_PARAM_STORE_ERROR_INVALID_PARAM, /**< 非法参数 */
    SRV_PARAM_STORE_ERROR_UNINITIALIZED, /**< 服务未初始化 */
    SRV_PARAM_STORE_ERROR_HAL_FLASH, /**< hal_flash 底层错误 */
    SRV_PARAM_STORE_ERROR_RING_STORAGE, /**< ring_storage 底层错误 */
    SRV_PARAM_STORE_ERROR_NO_VALID_FRAME, /**< 分区无有效帧（首次使用） */
} srv_param_store_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化参数存储服务
 *
 * 初始化 hal_flash + 本分区 ring_storage。幂等：二次调用直接返回 OK。
 *
 * @return SRV_PARAM_STORE_OK 成功；否则为错误码
 */
srv_param_store_error_t srv_param_store_init(void);

/**
 * @brief 注册一个 KV 参数
 * @param key       KV 名称（长度 ≤ RING_STORAGE_KEY_MAX）
 * @param value     参数变量指针（须为静态/全局，save/load 直接解引用）
 * @param value_len 参数长度（字节）
 * @return SRV_PARAM_STORE_OK 成功；否则为错误码
 * @note 必须在 save/load 之前注册所有 KV；key 不可重复。
 */
srv_param_store_error_t srv_param_store_register(const char* key,
    void* value, uint16_t value_len);

/**
 * @brief 从 Flash 加载最新帧到已注册的 KV 变量
 * @return SRV_PARAM_STORE_OK 成功；
 *         SRV_PARAM_STORE_ERROR_NO_VALID_FRAME 首次使用（无有效帧，保持默认值）
 */
srv_param_store_error_t srv_param_store_load(void);

/**
 * @brief 将所有已注册 KV 整帧持久化到 Flash
 * @return SRV_PARAM_STORE_OK 成功；否则为错误码
 * @note 原子提交 + 断电安全（ring_storage 特性）；首次 save 创建第一帧。
 */
srv_param_store_error_t srv_param_store_save(void);

/**
 * @brief 错误码 → 中文名称（供日志打印）
 * @param err 服务错误码
 * @return 中文描述字符串；未知码返回 "未知错误"
 */
const char* srv_param_store_err_str(srv_param_store_error_t err);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PARAM_STORE_H */
