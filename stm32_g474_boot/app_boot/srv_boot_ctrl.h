/**
 * @file    srv_boot_ctrl.h
 * @brief   Boot 控制服务 — App 侧管理 bootloader 共享 metadata，请求进入升级模式
 * @attention
 *
 * 本文件与 stm32_g474_boot 的 boot_flash.h 共享 boot_metadata_t 字节契约。
 * App 无需编译完整 boot 栈，仅通过本服务操作 metadata 区域（0x0801C000，16KB）。
 *
 * ## 用法
 * @code
 *   srv_boot_ctrl_init();              // App 启动时调用一次（幂等）
 *   srv_boot_ctrl_request_boot();      // 需要升级时调用 → 置标志 + 系统复位
 * @endcode
 *
 * ## Flash 布局（与 bootloader 一致）
 *   BOOT(64K) | APP(48K) | META(16K)
 *   0x08000000  0x08010000  0x0801C000
 */

#ifndef __SRV_BOOT_CTRL_H
#define __SRV_BOOT_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/** @brief 服务错误码 */
typedef enum {
    SRV_BOOT_CTRL_OK = 0,               /**< 操作成功 */
    SRV_BOOT_CTRL_ERROR_UNINITIALIZED,  /**< 服务未初始化 */
    SRV_BOOT_CTRL_ERROR_FLASH,          /**< Flash/ring_storage 底层错误 */
} srv_boot_ctrl_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 Boot 控制服务（幂等）
 *
 * 初始化 hal_flash，创建 BOOT 分区 ring_storage 实例（0x0801C000，16KB），
 * 注册 "meta" KV，首次使用初始化默认字段，上电计数 +1 并持久化。
 *
 * @return SRV_BOOT_CTRL_OK 成功；否则为错误码
 */
srv_boot_ctrl_error_t srv_boot_ctrl_init(void);

/**
 * @brief 请求进入 bootloader（置 upgrade_flag=1 → 系统复位）
 *
 * 复位后 bootloader 检测 upgrade_flag != 0 → 进入升级模式等待 CAN 指令。
 * 保存失败时不复位（避免标志未落盘却已复位导致下次正常启动跳过升级）。
 *
 * @return SRV_BOOT_CTRL_OK（复位前返回）；错误时返回错误码且不执行复位
 */
srv_boot_ctrl_error_t srv_boot_ctrl_request_boot(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_BOOT_CTRL_H */
