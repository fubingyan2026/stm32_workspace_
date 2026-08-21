/**
 * @file    srv_device_monitor.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   设备在线监控服务 — 基于 daemon 框架的喂狗超时判定
 * @attention
 *
 * 遵循 service 层回调注入模式，不直接调用硬件驱动。
 *
 * ## 职责
 * - 基于 m_middlewares 的 daemon（周期喂狗 + 超时判离线）监控 CAN 子设备在线状态
 * - 当前监控两台设备：
 *   - SRV_DEVICE_SLAVER：副电源管理控制板（0x002 ACK 即心跳）
 *   - SRV_DEVICE_DUAL：双电池控制板（0x200/0x201/0x202 即心跳）
 * - 供上层（app_status_report / 指示灯策略）查询在线状态
 *
 * ## 用法
 * @code
 *   // 1. 初始化（config 传 NULL 使用默认超时参数；本模块拥有 daemon 系统单例）
 *   srv_device_monitor_init(NULL);
 *
 *   // 2. 收到心跳帧时喂狗（ISR 安全：仅时间戳更新）
 *   srv_device_monitor_feed(SRV_DEVICE_SLAVER);   // 0x002 ACK
 *   srv_device_monitor_feed(SRV_DEVICE_DUAL);     // 0x200/0x201/0x202
 *
 *   // 3. 周期调用 step（由 can_task 的 10ms sw_timer 驱动）
 *   srv_device_monitor_step();
 *
 *   // 4. 读取在线状态
 *   bool online = srv_device_monitor_is_online(SRV_DEVICE_DUAL);
 * @endcode
 */

#ifndef __SRV_DEVICE_MONITOR_H
#define __SRV_DEVICE_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 被监控设备枚举
 */
typedef enum {
    SRV_DEVICE_SLAVER = 0, /**< 副电源管理控制板（心跳：0x002 ACK） */
    SRV_DEVICE_DUAL,       /**< 双电池控制板（心跳：0x200/0x201/0x202） */
    SRV_DEVICE_COUNT,      /**< 设备数量 */
} srv_device_t;

/**
 * @brief 监控配置结构体
 *
 * 超时时间以 0 表示使用默认值。config 整体传 NULL 亦全部使用默认值。
 * daemon 的 init_wait 期间不判离线（上电宽限），避免启动阶段误报。
 */
typedef struct {
    uint16_t slaver_timeout_ms; /**< 从板喂狗超时判离线 (ms)，0=默认 250 */
    uint16_t dual_timeout_ms;   /**< 双电池板喂狗超时判离线 (ms)，0=默认 500 */
    uint16_t init_wait_ms;      /**< 初始化宽限期 (ms)，0=默认 1000 */
} srv_device_monitor_config_t;

/**
 * @brief 服务错误码
 */
typedef enum {
    SRV_DEVICE_MONITOR_OK = 0, /**< 操作成功 */
    SRV_DEVICE_MONITOR_ERROR_NULL_PTR, /**< 空指针参数 */
    SRV_DEVICE_MONITOR_ERROR_UNINITIALIZED, /**< 服务未初始化 */
    SRV_DEVICE_MONITOR_ERROR_DAEMON, /**< daemon 底层错误 */
    SRV_DEVICE_MONITOR_ERROR_INVALID_PARAM, /**< 非法设备枚举 */
} srv_device_monitor_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化设备在线监控服务
 *
 * 初始化 daemon 系统单例（时间源 = millis()）并注册所有被监控设备。
 * 可重复调用，二次调用直接返回 OK（幂等）。
 *
 * @param config 监控配置，传 NULL 使用默认超时参数
 * @return SRV_DEVICE_MONITOR_OK 成功；否则为错误码
 */
srv_device_monitor_error_t srv_device_monitor_init(
    const srv_device_monitor_config_t* config);

/**
 * @brief 喂狗：刷新设备心跳时间戳
 *
 * ISR 安全：仅做时间戳赋值，无日志、无锁、无动态内存。应在设备心跳帧
 * （0x002 ACK / 0x200/0x201/0x202）到达时调用。
 *
 * @param dev 设备枚举
 */
void srv_device_monitor_feed(srv_device_t dev);

/**
 * @brief 查询设备在线状态
 * @param dev 设备枚举
 * @return true=在线，false=离线或参数非法
 */
bool srv_device_monitor_is_online(srv_device_t dev);

/**
 * @brief 周期任务：驱动 daemon 超时检测与状态边沿回调
 *
 * 应由 task 层 sw_timer 周期调用（can_task 10ms）。超时阈值见配置，
 * 检测到状态变化时 daemon 内部触发离线回调。
 */
void srv_device_monitor_step(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_DEVICE_MONITOR_H */
