//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_error.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU SDK 错误码描述。
 */

/* Includes ------------------------------------------------------------------*/
#include "ctu/ctu_error.h"

/* Exported functions --------------------------------------------------------*/

const char* ctu_error_str(ctu_error_t error)
{
    switch (error) {
    case CTU_OK:
        return "成功";
    case CTU_ERROR_NULL_PTR:
        return "空指针";
    case CTU_ERROR_INVALID_PARAM:
        return "无效参数";
    case CTU_ERROR_UNINITIALIZED:
        return "未初始化";
    case CTU_ERROR_BUFFER_TOO_SMALL:
        return "缓冲区不足";
    case CTU_ERROR_NOT_OPEN:
        return "串口未打开";
    case CTU_ERROR_IO:
        return "串口读写失败";
    case CTU_ERROR_PORT:
        return "串口打开/配置失败";
    case CTU_ERROR_TIMEOUT:
        return "等待应答超时";
    case CTU_ERROR_UNEXPECTED_REPLY:
        return "应答命令码不符";
    case CTU_ERROR_DEVICE:
        return "设备错误应答";
    case CTU_ERROR_PROTOCOL:
        return "帧格式非法";
    case CTU_ERROR_STATE:
        return "状态错误";
    case CTU_ERROR_IMAGE:
        return "固件镜像预检失败";
    case CTU_ERROR_FILE:
        return "文件读写失败";
    case CTU_ERROR_UPGRADE:
        return "固件升级失败";
    case CTU_ERROR_CANCELED:
        return "操作被取消";
    case CTU_ERROR_NOT_SUPPORTED:
        return "当前平台不支持";
    case CTU_ERROR_GENERIC:
    default:
        return "未分类错误";
    }
}
