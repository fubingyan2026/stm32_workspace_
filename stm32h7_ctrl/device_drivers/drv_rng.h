/**
 * @file    drv_rng.h
 * @brief   STM32H7 硬件随机数生成器驱动
 */

#ifndef __DRV_RNG_H
#define __DRV_RNG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ====== 错误码 =============================================================*/

typedef enum {
    DRV_RNG_OK = 0,
    DRV_RNG_ERR_PARAM  = -1,
    DRV_RNG_ERR_INIT   = -2,
    DRV_RNG_ERR_TIMEOUT = -3,
    DRV_RNG_ERR_CLOCK  = -4,
} drv_rng_error_t;

/* ====== API ================================================================*/

drv_rng_error_t drv_rng_init(void);
drv_rng_error_t drv_rng_get_value(uint32_t* value);
drv_rng_error_t drv_rng_get_bytes(uint8_t* buf, uint32_t len);
bool            drv_rng_is_healthy(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_RNG_H */
