/**
 * @file    drv_revision.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   硬件版本识别驱动实现
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_revision.h"

#include "main.h"

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

    return rev;
}

const char* drv_revision_name(void)
{
    uint8_t rev = drv_revision_read();

    if (rev <= REV_MAX_INDEX && s_rev_names[rev]) {
        return s_rev_names[rev];
    }

    return "UNKNOWN";
}
