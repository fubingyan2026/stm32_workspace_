//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_error.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU SDK 统一错误码。
 */

#ifndef __CTU_ERROR_H
#define __CTU_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SDK 操作结果错误码
 */
typedef enum {
    CTU_OK = 0,                        /**< 操作成功 */
    CTU_ERROR_NULL_PTR,                /**< 空指针错误 */
    CTU_ERROR_INVALID_PARAM,           /**< 无效参数 */
    CTU_ERROR_UNINITIALIZED,           /**< 对象未初始化 */
    CTU_ERROR_BUFFER_TOO_SMALL,        /**< 缓冲区不足 */
    CTU_ERROR_NOT_OPEN,                /**< 串口未打开 */
    CTU_ERROR_IO,                      /**< 串口读写失败 */
    CTU_ERROR_PORT,                    /**< 串口打开/配置失败 */
    CTU_ERROR_TIMEOUT,                 /**< 等待应答超时 */
    CTU_ERROR_UNEXPECTED_REPLY,        /**< 应答命令码与请求不符 */
    CTU_ERROR_DEVICE,                  /**< 设备返回 0x7F 错误应答 */
    CTU_ERROR_PROTOCOL,                /**< 帧格式非法 */
    CTU_ERROR_STATE,                   /**< 状态错误（升级流程顺序不对） */
    CTU_ERROR_IMAGE,                   /**< 固件镜像预检失败 */
    CTU_ERROR_FILE,                    /**< 文件读写失败 */
    CTU_ERROR_UPGRADE,                 /**< 固件升级失败 */
    CTU_ERROR_CANCELED,                /**< 操作被取消 */
    CTU_ERROR_NOT_SUPPORTED,           /**< 当前平台不支持 */
    CTU_ERROR_GENERIC,                 /**< 未分类错误 */
} ctu_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 获取错误码的可读描述
 * @param error 错误码
 * @return 静态字符串（无需释放）
 */
const char* ctu_error_str(ctu_error_t error);

#ifdef __cplusplus
}
#endif

#endif /* __CTU_ERROR_H */
