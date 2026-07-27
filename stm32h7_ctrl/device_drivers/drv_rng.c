/**
 * @file    drv_rng.c
 * @brief   STM32H7 硬件随机数生成器驱动实现
 *
 * HSI48 时钟源，CED 时钟错误检测使能。
 * 支持自动重试和错误恢复（seed error 时 reinit）。
 */

#include "drv_rng.h"

#include "rng.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

#define RNG_LOG_ENABLE 0

#if RNG_LOG_ENABLE
#include "log.h"
#define RNG_LOG_E(...) LOG_E("rng", __VA_ARGS__)
#define RNG_LOG_I(...) LOG_I("rng", __VA_ARGS__)
#else
#define RNG_LOG_E(...) ((void)0)
#define RNG_LOG_I(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define DRV_RNG_MAX_RETRY 100U

/* Private variables ---------------------------------------------------------*/

static bool s_init = false;

/* ====== API 实现 ===========================================================*/

drv_rng_error_t drv_rng_init(void)
{
    if (s_init) return DRV_RNG_OK;

    if (HAL_RNG_Init(&hrng) != HAL_OK) {
        RNG_LOG_E("RNG init failed");
        return DRV_RNG_ERR_INIT;
    }

    s_init = true;
    RNG_LOG_I("RNG init ok (HSI48, CED)");
    return DRV_RNG_OK;
}

drv_rng_error_t drv_rng_get_value(uint32_t* value)
{
    if (!value) return DRV_RNG_ERR_PARAM;
    if (!s_init) return DRV_RNG_ERR_INIT;

    for (uint32_t i = 0; i < DRV_RNG_MAX_RETRY; i++) {
        if (HAL_RNG_GenerateRandomNumber(&hrng, value) == HAL_OK) {
            return DRV_RNG_OK;
        }
        /* Seed error 时重新初始化 RNG */
        HAL_RNG_DeInit(&hrng);
        HAL_RNG_Init(&hrng);
    }

    RNG_LOG_E("RNG get_value timeout after %u retries", (unsigned)DRV_RNG_MAX_RETRY);
    return DRV_RNG_ERR_TIMEOUT;
}

drv_rng_error_t drv_rng_get_bytes(uint8_t* buf, uint32_t len)
{
    if (!buf) return DRV_RNG_ERR_PARAM;
    if (!s_init) return DRV_RNG_ERR_INIT;

    uint32_t val;
    uint32_t pos = 0;

    while (len >= 4) {
        drv_rng_error_t err = drv_rng_get_value(&val);
        if (err != DRV_RNG_OK) return err;
        buf[pos + 0] = (uint8_t)(val >> 0);
        buf[pos + 1] = (uint8_t)(val >> 8);
        buf[pos + 2] = (uint8_t)(val >> 16);
        buf[pos + 3] = (uint8_t)(val >> 24);
        pos += 4;
        len -= 4;
    }

    if (len > 0) {
        drv_rng_error_t err = drv_rng_get_value(&val);
        if (err != DRV_RNG_OK) return err;
        for (uint32_t i = 0; i < len; i++) {
            buf[pos++] = (uint8_t)(val >> (i * 8));
        }
    }

    return DRV_RNG_OK;
}

bool drv_rng_is_healthy(void)
{
    uint32_t val;
    return drv_rng_get_value(&val) == DRV_RNG_OK;
}
