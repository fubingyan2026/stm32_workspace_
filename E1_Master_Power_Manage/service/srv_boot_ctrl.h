/**
 * @file    srv_boot_ctrl.h
 * @author  maximillian
 * @version V1.2.0
 * @date    2026-08-06
 * @brief   Boot 控制服务 — 管理 bootloader 共享 metadata 的生命周期与进入 boot 请求
 * @attention
 *
 * 遵循 service 层 driver-wrapping 模式：自持一个 ring_storage 实例（BOOT 分区），
 * 直接包装 hal_flash；复位经 drv_system_reset()（drv_systick）封装。
 * 同层解耦：不调用 srv_param_store（其管理 APP 参数分区），两块 flash 实例相互独立。
 *
 * boot_metadata_t 为本服务**私有类型**（定义在 srv_boot_ctrl.c），字段布局与
 * stm32_g474_boot/service/boot/boot_flash.h 的 boot_metadata_t 一致，构成与
 * bootloader 的共享字节契约。
 *
 * ## 职责
 *   - 注册/加载/递增 boot metadata（reboot_counts 上电计数）
 *   - 请求进入 bootloader：置 upgrade_flag=1 保存后系统复位，
 *     下次启动由 bootloader 判定 upgrade_flag != 0 → 进入 boot 模式
 *
 * ## 用法
 * @code
 *   srv_boot_ctrl_init();                    // 自持 BOOT 分区 ring_storage
 *   srv_boot_ctrl_request_boot();            // 置标志 + 复位
 * @endcode
 */

#ifndef __SRV_BOOT_CTRL_H
#define __SRV_BOOT_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 服务错误码
 */
typedef enum {
    SRV_BOOT_CTRL_OK = 0, /**< 操作成功 */
    SRV_BOOT_CTRL_ERROR_UNINITIALIZED, /**< 服务未初始化 */
    SRV_BOOT_CTRL_ERROR_FLASH, /**< Flash/ring_storage 底层错误 */
} srv_boot_ctrl_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 Boot 控制服务
 *
 * 自持 BOOT 分区 ring_storage，注册 "meta" KV 并加载；首次使用（无有效帧）
 * 初始化默认 metadata；随后 reboot_counts 上电计数 +1 并整帧持久化。幂等。
 *
 * @return SRV_BOOT_CTRL_OK 成功；否则为错误码
 */
srv_boot_ctrl_error_t srv_boot_ctrl_init(void);

/**
 * @brief 请求进入 bootloader
 *
 * 置 BOOT 分区 metadata 的 upgrade_flag=1 并保存，随后系统复位。
 * 复位后 bootloader 判定 upgrade_flag != 0 → 进入 boot 模式。
 * 保存失败时不复位（避免标志未落盘却已复位）。
 *
 * @return SRV_BOOT_CTRL_OK（复位前返回）；否则为错误码
 */
srv_boot_ctrl_error_t srv_boot_ctrl_request_boot(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_BOOT_CTRL_H */
