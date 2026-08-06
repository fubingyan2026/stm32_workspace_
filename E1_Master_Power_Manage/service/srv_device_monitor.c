/**
 * @file    srv_device_monitor.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   设备在线监控服务实现
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_device_monitor.h"

#include "daemon.h"
#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_DEVICE_MONITOR_LOG_ENABLE 1

#if SRV_DEVICE_MONITOR_LOG_ENABLE
#define SRV_DEVICE_MONITOR_LOG_E(...) LOG_E("srv_device_monitor", __VA_ARGS__)
#define SRV_DEVICE_MONITOR_LOG_W(...) LOG_W("srv_device_monitor", __VA_ARGS__)
#define SRV_DEVICE_MONITOR_LOG_I(...) LOG_I("srv_device_monitor", __VA_ARGS__)
#define SRV_DEVICE_MONITOR_LOG_D(...) LOG_D("srv_device_monitor", __VA_ARGS__)
#else
#define SRV_DEVICE_MONITOR_LOG_E(...) ((void)0)
#define SRV_DEVICE_MONITOR_LOG_W(...) ((void)0)
#define SRV_DEVICE_MONITOR_LOG_I(...) ((void)0)
#define SRV_DEVICE_MONITOR_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 默认喂狗超时：从板 0x002 重试 50ms ×5 */
#define SRV_DEVICE_MONITOR_DEF_SLAVER_TIMEOUT_MS (250U)
/** @brief 默认喂狗超时：双电池 0x200 每 100ms 一帧 ×5 */
#define SRV_DEVICE_MONITOR_DEF_DUAL_TIMEOUT_MS (500U)
/** @brief 默认初始化宽限期：上电期间不判离线 */
#define SRV_DEVICE_MONITOR_DEF_INIT_WAIT_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static daemon_context_t s_daemon_ctx[SRV_DEVICE_COUNT]; /**< 每设备一个守护实例 */
static bool s_initialized; /**< 服务初始化标志 */

/** @brief 设备守护进程名称（唯一标识，供 daemon_get_instance 查询） */
static const char* const s_device_name[SRV_DEVICE_COUNT] = {
    "device_slaver",
    "device_dual",
};

/* Exported functions --------------------------------------------------------*/

srv_device_monitor_error_t srv_device_monitor_init(
    const srv_device_monitor_config_t* config)
{
    /* 幂等：二次初始化直接返回 */
    if (s_initialized) {
        return SRV_DEVICE_MONITOR_OK;
    }

    /* daemon 系统单例初始化（时间源 = millis；本工程唯一使用者） */
    const daemon_error_t d_err = daemon_init(millis);
    if (d_err != DAEMON_OK && d_err != DAEMON_OK_EXISTED) {
        SRV_DEVICE_MONITOR_LOG_E("daemon 系统初始化失败 (err=%d)", (int)d_err);
        return SRV_DEVICE_MONITOR_ERROR_DAEMON;
    }

    const uint16_t timeout_ms[SRV_DEVICE_COUNT] = {
        (config && config->slaver_timeout_ms) ? config->slaver_timeout_ms
                                              : SRV_DEVICE_MONITOR_DEF_SLAVER_TIMEOUT_MS,
        (config && config->dual_timeout_ms) ? config->dual_timeout_ms
                                            : SRV_DEVICE_MONITOR_DEF_DUAL_TIMEOUT_MS,
    };
    const uint16_t init_wait_ms = (config && config->init_wait_ms)
        ? config->init_wait_ms
        : SRV_DEVICE_MONITOR_DEF_INIT_WAIT_MS;

    /* 静态注册所有被监控设备 */
    for (uint32_t i = 0; i < SRV_DEVICE_COUNT; i++) {
        const daemon_config_t dcfg = {
            .name = s_device_name[i],
            .owner_ptr = NULL,
            .offline_cb = NULL,
            .reload_timeout_ms = timeout_ms[i],
            .init_wait_time_ms = init_wait_ms,
        };
        const daemon_error_t reg_err = daemon_register_static(&dcfg, &s_daemon_ctx[i]);
        if (reg_err != DAEMON_OK) {
            SRV_DEVICE_MONITOR_LOG_E("设备 %s 注册守护失败 (err=%d)",
                s_device_name[i], (int)reg_err);
            return SRV_DEVICE_MONITOR_ERROR_DAEMON;
        }
    }

    s_initialized = true;
    SRV_DEVICE_MONITOR_LOG_I("设备在线监控初始化完成 (slaver=%ums dual=%ums init=%ums)",
        (unsigned)timeout_ms[SRV_DEVICE_SLAVER],
        (unsigned)timeout_ms[SRV_DEVICE_DUAL],
        (unsigned)init_wait_ms);

    return SRV_DEVICE_MONITOR_OK;
}

void srv_device_monitor_feed(srv_device_t dev)
{
    if (!s_initialized || dev >= SRV_DEVICE_COUNT) {
        return;
    }
    daemon_reload(&s_daemon_ctx[dev]);
}

bool srv_device_monitor_is_online(srv_device_t dev)
{
    if (!s_initialized || dev >= SRV_DEVICE_COUNT) {
        return false;
    }
    return daemon_is_online(&s_daemon_ctx[dev]);
}

void srv_device_monitor_step(void)
{
    daemon_task();
}
