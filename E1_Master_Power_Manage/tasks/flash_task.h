/**
 * @file    flash_task.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   Flash 存储任务 — 编排 flash 存储服务初始化并加载 APP 分区参数
 * @note    本任务不拥有 sw_timer；参数保存由上层按需调用 srv_param_store_save()。
 *          BOOT 分区 metadata 由 srv_boot_ctrl 管理，进 boot 请求调 srv_boot_ctrl_request_boot()。
 */

#ifndef FLASH_TASK_H
#define FLASH_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化 flash 存储：srv_param_store + srv_boot_ctrl + APP 分区计数 */
void flash_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_TASK_H */
