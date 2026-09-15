//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_types.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU SDK 公共类型辅助函数。
 */

/* Includes ------------------------------------------------------------------*/
#include "ctu/ctu_types.h"

/* Exported functions --------------------------------------------------------*/

const char* ctu_device_name(ctu_device_t device)
{
    switch (device) {
    case CTU_DEVICE_MASTER:
        return "master";
    case CTU_DEVICE_SLAVER:
        return "slaver";
    default:
        return "unknown";
    }
}

bool ctu_device_is_valid(ctu_device_t device)
{
    return (device == CTU_DEVICE_MASTER) || (device == CTU_DEVICE_SLAVER);
}
