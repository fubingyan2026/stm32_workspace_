/**
 * @file    ring_storage_port.h
 * @brief   环形缓冲区 Flash 存储平台抽象接口 — 回调函数版本
 * @author  maximilian
 * @version V2.0.0
 * @date    2026-07-24
 *
 * @note    用户实现回调函数并通过 ring_storage_config_t.port 注入。
 *          不再需要编译 ring_storage_port.c 到 middleware。
 *
 *          参考实现示例（STM32G4 + hal_flash）：
 * @code
 *          static int my_port_read(uint32_t addr, uint8_t *buf, size_t size) {
 *              return (int)hal_flash_read(addr - hal_flash_get_caps()->addr, buf, size);
 *          }
 *          static int my_port_write(uint32_t addr, const uint8_t *buf, size_t size) {
 *              return (int)hal_flash_write(addr - hal_flash_get_caps()->addr, buf, size);
 *          }
 *          static int my_port_erase(uint32_t addr, size_t size) {
 *              return (int)hal_flash_erase(addr - hal_flash_get_caps()->addr, size);
 *          }
 *          static void my_port_lock(void)   { hal_flash_lock(); }
 *          static void my_port_unlock(void) { hal_flash_unlock(); }
 *
 *          const ring_storage_port_t my_port = {
 *              .read   = my_port_read,
 *              .write  = my_port_write,
 *              .erase  = my_port_erase,
 *              .lock   = my_port_lock,
 *              .unlock = my_port_unlock,
 *          };
 *
 *          ring_storage_config_t cfg = {
 *              .start_addr  = 0x08078000,
 *              .port        = my_port,
 *              ...
 *          };
 * @endcode
 */

#ifndef __RING_STORAGE_PORT_H
#define __RING_STORAGE_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Flash 读取回调
 * @param[in]  addr  Flash 起始地址（绝对地址）
 * @param[out] buf   数据接收缓冲区
 * @param[in]  size  要读取的字节数
 * @return 0 成功，负值失败
 */
typedef int (*ring_storage_port_read_t)(uint32_t addr, uint8_t* buf, size_t size);

/**
 * @brief Flash 写入回调
 * @param[in] addr Flash 起始地址（绝对地址，须对齐到写入颗粒度）
 * @param[in] buf  要写入的数据缓冲区
 * @param[in] size 要写入的字节数（须为写入颗粒度的整数倍）
 * @return 0 成功，负值失败
 * @note 目标区域必须已擦除（全 0xFF）。
 *       建议实现时跳过全 0xFF 的写入单元以减少 Flash 编程次数。
 */
typedef int (*ring_storage_port_write_t)(uint32_t addr, const uint8_t* buf, size_t size);

/**
 * @brief Flash 扇区擦除回调
 * @param[in] addr 要擦除的 Flash 起始地址（须对齐到扇区边界）
 * @param[in] size 要擦除的字节数（须为扇区大小的整数倍）
 * @return 0 成功，负值失败
 */
typedef int (*ring_storage_port_erase_t)(uint32_t addr, size_t size);

/**
 * @brief 进入 Flash 操作临界区
 * @note 使用 BASEPRI 屏蔽中低优先级中断，允许高优先级中断继续执行。
 *       支持嵌套调用（引用计数）。
 */
typedef void (*ring_storage_port_lock_t)(void);

/**
 * @brief 退出 Flash 操作临界区
 */
typedef void (*ring_storage_port_unlock_t)(void);

/**
 * @brief Flash 操作回调集合
 *
 * 在 ring_storage_config_t 中赋值，ring_storage_init 时注入。
 * 所有回调不可为 NULL。
 */
typedef struct {
    ring_storage_port_read_t   read;   /**< Flash 读取 */
    ring_storage_port_write_t  write;  /**< Flash 写入 */
    ring_storage_port_erase_t  erase;  /**< Flash 扇区擦除 */
    ring_storage_port_lock_t   lock;   /**< 进入临界区 */
    ring_storage_port_unlock_t unlock; /**< 退出临界区 */
} ring_storage_port_t;

#ifdef __cplusplus
}
#endif

#endif /* __RING_STORAGE_PORT_H */
