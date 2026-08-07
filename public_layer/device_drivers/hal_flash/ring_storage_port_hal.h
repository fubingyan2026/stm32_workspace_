/**
 * @file    ring_storage_port_hal.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   ring_storage 的默认 hal_flash 平台 port（共享 helper）
 * @attention
 *
 * ring_storage_port.c（public_layer/hal_flash）提供了基于 hal_flash 的默认实现，
 * 但未在头文件声明（返回 ring_storage_error_t，与 ring_storage_port.h 的 int 契约
 * 存在类型不一致）。本 helper 集中声明 + 提供内联构造器，供各 service 复用，
 * 避免每个存储服务重复声明/构造同一套 port。
 */

#ifndef __RING_STORAGE_PORT_HAL_H
#define __RING_STORAGE_PORT_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ring_storage.h"

/* ring_storage_port.c 提供的默认 hal_flash 平台实现（public_layer/hal_flash，
 * 已由根 CMakeLists 编入本工程） */
extern ring_storage_error_t ring_storage_port_read(uint32_t addr,
    uint8_t* buf, size_t size);
extern ring_storage_error_t ring_storage_port_write(uint32_t addr,
    const uint8_t* buf, size_t size);
extern ring_storage_error_t ring_storage_port_erase(uint32_t addr, size_t size);
extern void ring_storage_port_lock(void);
extern void ring_storage_port_unlock(void);

/**
 * @brief 返回基于 hal_flash 的默认 ring_storage port
 * @return ring_storage_port_t 实例
 * @note read/write/erase 返回 ring_storage_error_t，与 _t 契约(int)不一致，
 *       显式转换（与 stm32_g474_boot/boot_flash.c 同款写法）。
 */
static inline ring_storage_port_t ring_storage_port_hal(void)
{
    const ring_storage_port_t port = {
        .read = (ring_storage_port_read_t)ring_storage_port_read,
        .write = (ring_storage_port_write_t)ring_storage_port_write,
        .erase = (ring_storage_port_erase_t)ring_storage_port_erase,
        .lock = ring_storage_port_lock,
        .unlock = ring_storage_port_unlock,
    };
    return port;
}

#ifdef __cplusplus
}
#endif

#endif /* __RING_STORAGE_PORT_HAL_H */
