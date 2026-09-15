//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU SDK 版本信息。
 */

/* Includes ------------------------------------------------------------------*/
#include "ctu/ctu.h"

/* Exported functions --------------------------------------------------------*/

const char* ctu_version_string(void)
{
    return CTU_SDK_VERSION_STRING;
}
