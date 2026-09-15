//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU SDK 总入口头文件（一次性引入全部公共接口）。
 */

#ifndef __CTU_H
#define __CTU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ctu_boot.h"
#include "ctu_client.h"
#include "ctu_config.h"
#include "ctu_error.h"
#include "ctu_poller.h"
#include "ctu_protocol.h"
#include "ctu_transport.h"
#include "ctu_types.h"

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 获取 SDK 版本字符串
 * @return 静态字符串，如 "1.0.0"
 */
const char* ctu_version_string(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTU_H */
