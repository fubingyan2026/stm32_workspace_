/**
 * @file    drv_revision.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   硬件版本识别驱动实现
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_revision.h"

#include "log.h"
#include "main.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DRV_REVISION_LOG_ENABLE 1

#if DRV_REVISION_LOG_ENABLE
#define DRV_REVISION_LOG_E(...) LOG_E("drv_revision", __VA_ARGS__)
#define DRV_REVISION_LOG_W(...) LOG_W("drv_revision", __VA_ARGS__)
#define DRV_REVISION_LOG_I(...) LOG_I("drv_revision", __VA_ARGS__)
#define DRV_REVISION_LOG_D(...) LOG_D("drv_revision", __VA_ARGS__)
#else
#define DRV_REVISION_LOG_E(...) ((void)0)
#define DRV_REVISION_LOG_W(...) ((void)0)
#define DRV_REVISION_LOG_I(...) ((void)0)
#define DRV_REVISION_LOG_D(...) ((void)0)
#endif

/* Private variables ---------------------------------------------------------*/

/** @brief 版本打印标志：仅首次读取时输出，避免重复 */
static bool s_rev_logged;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 版本名称表
 *
 * 硬件确定后按实际版本映射修改，未定义的返回 "V{rev}"。
 */
static const char* s_rev_names[] = {
    [0] = "V0",
    [1] = "V1",
    [2] = "V2",
    [3] = "V3",
    [4] = "V4",
    [5] = "V5",
    [6] = "V6",
    [7] = "V7",
};

#define REV_MAX_INDEX (7U)

/* Exported functions --------------------------------------------------------*/

uint8_t drv_revision_read(void)
{
    uint8_t rev = 0;

    if (HAL_GPIO_ReadPin(REV_PD0_GPIO_Port, REV_PD0_Pin) == GPIO_PIN_SET) {
        rev |= (1U << 0);
    }
    if (HAL_GPIO_ReadPin(REV_PD1_GPIO_Port, REV_PD1_Pin) == GPIO_PIN_SET) {
        rev |= (1U << 1);
    }
    if (HAL_GPIO_ReadPin(REV_PD2_GPIO_Port, REV_PD2_Pin) == GPIO_PIN_SET) {
        rev |= (1U << 2);
    }

    /* 首次读取打印硬件版本（PD0/PD1/PD2 三引脚编码） */
    if (!s_rev_logged) {
        s_rev_logged = true;
        DRV_REVISION_LOG_I("硬件版本: rev=%u (PD0=%d PD1=%d PD2=%d)",
            (unsigned)rev,
            (int)((rev >> 0) & 1U),
            (int)((rev >> 1) & 1U),
            (int)((rev >> 2) & 1U));
    }

    return rev;
}

const char* drv_revision_name(void)
{
    uint8_t rev = drv_revision_read();

    if (rev <= REV_MAX_INDEX && s_rev_names[rev]) {
        return s_rev_names[rev];
    }

    DRV_REVISION_LOG_W("硬件版本未知: rev=%u 超出映射表 (上限%u)",
        (unsigned)rev, (unsigned)REV_MAX_INDEX);
    return "UNKNOWN";
}
