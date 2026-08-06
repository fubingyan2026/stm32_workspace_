/**
 * @file    flash_task.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   Flash 存储任务实现 — 编排 flash 存储服务初始化并加载 APP 分区参数
 *
 * 分层：
 *   - BOOT 分区 metadata 由 srv_boot_ctrl 管理（注册/加载/reboot_counts/request_boot）
 *   - 本任务只负责 APP 分区「restart_counts」上电计数，并编排初始化顺序：
 *     srv_param_store_init → srv_boot_ctrl_init → APP 参数
 * 首次使用（无有效帧）计数从 0 起算，之后启动直接加载恢复。
 */

/* Includes ------------------------------------------------------------------*/
#include "flash_task.h"

#include "log.h"
#include "srv_boot_ctrl.h"
#include "srv_param_store.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define FLASH_TASK_LOG_ENABLE 1

#if FLASH_TASK_LOG_ENABLE
#define FLASH_TASK_LOG_E(...) LOG_E("flash_task", __VA_ARGS__)
#define FLASH_TASK_LOG_W(...) LOG_W("flash_task", __VA_ARGS__)
#define FLASH_TASK_LOG_I(...) LOG_I("flash_task", __VA_ARGS__)
#define FLASH_TASK_LOG_D(...) LOG_D("flash_task", __VA_ARGS__)
#else
#define FLASH_TASK_LOG_E(...) ((void)0)
#define FLASH_TASK_LOG_W(...) ((void)0)
#define FLASH_TASK_LOG_I(...) ((void)0)
#define FLASH_TASK_LOG_D(...) ((void)0)
#endif

/* Private variables ---------------------------------------------------------*/

static uint32_t s_restart_counts; /**< 上电计数（APP 分区，每次上电 +1 持久化） */

/* Exported functions --------------------------------------------------------*/

void flash_task_init(void)
{
    /* 初始化参数存储服务（APP 常用参数分区） */
    if (srv_param_store_init() != SRV_PARAM_STORE_OK) {
        FLASH_TASK_LOG_E("参数存储服务初始化失败");
        return;
    }

    /* Boot metadata（BOOT 分区）由 boot 控制服务独立管理（自持 flash 实例） */
    if (srv_boot_ctrl_init() != SRV_BOOT_CTRL_OK) {
        FLASH_TASK_LOG_E("Boot 控制服务初始化失败");
        return;
    }

    /* 注册上电计数 KV */
    srv_param_store_register("restart_counts",
                             &s_restart_counts, sizeof(s_restart_counts));

    /* 加载参数；首次使用无有效帧时计数从 0 起算（本次上电计为第 1 次） */
    srv_param_store_error_t app_err = srv_param_store_load();
    if (app_err == SRV_PARAM_STORE_ERROR_NO_VALID_FRAME) {
        FLASH_TASK_LOG_I("APP 分区首次使用");
        s_restart_counts = 0;
        app_err = SRV_PARAM_STORE_OK;
    }
    if (app_err != SRV_PARAM_STORE_OK) {
        /* 加载失败时计数不可信，跳过本次计数避免覆盖有效值 */
        FLASH_TASK_LOG_W("APP 分区加载: %s", srv_param_store_err_str(app_err));
        return;
    }

    /* 上电计数：每次开机 +1 并整帧持久化 */
    s_restart_counts++;
    app_err = srv_param_store_save();
    if (app_err != SRV_PARAM_STORE_OK) {
        FLASH_TASK_LOG_W("APP 分区保存失败: %s", srv_param_store_err_str(app_err));
        return;
    }

    FLASH_TASK_LOG_I("Flash 存储任务初始化完成 (restart_counts=%u)",
        (unsigned)s_restart_counts);
}
