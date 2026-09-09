/**
 * @file    boot_task.h
 * @brief   Boot 主任务 — 启动决策 + RS485 YMODEM 升级接收（胶水层）
 * @attention
 *
 * 串联 YMODEM 接收器(boot_ymodem) → Flash 分区(boot_flash/hal_flash)，
 * 实现 485 升级状态机与"下载到 B 槽 → 校验 → 提升到 A 槽 → 复位"的提交流程。
 *
 * ## 使用
 * @code
 *   bool jumped = boot_task_try_boot_app();   // true = 已跳转 App，不会返回
 *   if (!jumped) {
 *       boot_task_init();
 *       for (;;) { boot_task_poll(); }
 *   }
 * @endcode
 */

#ifndef __BOOT_TASK_H
#define __BOOT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 上电启动决策：检查 Metadata，决定跳转 App 或进入 bootloader
 * @return true = 已跳转 App（不返回）；false = 进入 bootloader 升级模式
 */
bool boot_task_try_boot_app(void);

/**
 * @brief 初始化 bootloader 主任务（485 + Flash 暂存分区 + YMODEM + 指示灯）
 *
 * 调用后进入 YMODEM 升级接收状态，由 boot_task_poll() 驱动。
 */
void boot_task_init(void);

/**
 * @brief 周期轮询：搬运 485 字节、驱动 YMODEM、LED 指示、触发提交/复位
 */
void boot_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_TASK_H */
