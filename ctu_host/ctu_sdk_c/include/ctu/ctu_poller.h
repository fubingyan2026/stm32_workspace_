//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_poller.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   周期轮询：对应上位机「自动轮询」，并统计丢包率与接收速率。
 * @note    C 版本不创建线程；由调用方按自己的节奏循环调用
 *          :func:`ctu_poller_poll_once`（例如使用定时器或 sleep）。
 */

#ifndef __CTU_POLLER_H
#define __CTU_POLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ctu_client.h"
#include "ctu_config.h"
#include "ctu_error.h"
#include "ctu_types.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 一轮查询的采样结果
 */
typedef struct {
    ctu_device_t device;                  /**< 采样设备 */
    bool status_valid;                    /**< 状态是否有效 */
    bool voltage_valid;                   /**< 电压是否有效 */
    bool temp_valid;                      /**< 温度是否有效 */
    ctu_master_status_t master_status;    /**< MASTER 状态 */
    ctu_slaver_status_t slaver_status;    /**< SLAVER 状态 */
    ctu_master_voltage_t master_voltage;  /**< MASTER 电压 */
    ctu_slaver_voltage_t slaver_voltage;  /**< SLAVER 电压 */
    ctu_master_temp_t master_temp;        /**< MASTER 温度 */
    ctu_slaver_temp_t slaver_temp;        /**< SLAVER 温度 */
} ctu_poll_sample_t;

/**
 * @brief 采样回调
 * @param sample 采样结果
 * @param user   用户上下文
 */
typedef void (*ctu_poll_sample_cb_t)(const ctu_poll_sample_t* sample, void* user);

/**
 * @brief 查询失败回调
 * @param device  设备
 * @param command 失败的命令码
 * @param error   错误码
 * @param user    用户上下文
 */
typedef void (*ctu_poll_error_cb_t)(ctu_device_t device, uint8_t command,
                                    ctu_error_t error, void* user);

/**
 * @brief 轮询配置（含回调）
 */
typedef struct {
    ctu_device_t devices[2];              /**< 参与轮询的设备（最多两块） */
    size_t device_count;                  /**< 设备数量 */
    ctu_poll_sample_cb_t sample_cb;       /**< 采样回调（可为 NULL） */
    ctu_poll_error_cb_t error_cb;         /**< 失败回调（可为 NULL） */
    void* user;                           /**< 回调用户上下文 */
} ctu_poller_config_t;

/**
 * @brief 轮询上下文
 */
typedef struct ctu_poller ctu_poller_t;

struct ctu_poller {
    ctu_poller_config_t config; /**< 配置参数 */
    ctu_client_t* client;       /**< 关联客户端 */
    ctu_poll_stats_t stats;     /**< 统计快照 */
    uint32_t rx_in_window;      /**< 当前 1s 窗口内成功应答数 */
    uint64_t window_start_ms;   /**< 窗口起始时间 */
    bool initialized;           /**< 是否已初始化 */
};

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化轮询器
 * @param poller 轮询上下文
 * @param client 已初始化的客户端
 * @param config 配置（可为 NULL，默认轮询两块板）
 * @return 操作结果
 */
ctu_error_t ctu_poller_init(ctu_poller_t* poller, ctu_client_t* client,
                            const ctu_poller_config_t* config);

/**
 * @brief 清零统计
 */
void ctu_poller_reset_stats(ctu_poller_t* poller);

/**
 * @brief 对配置中的所有设备执行一轮查询（状态 → 电压 → 温度）
 * @return 操作结果（单个设备失败不影响其它设备）
 */
ctu_error_t ctu_poller_poll_once(ctu_poller_t* poller);

/**
 * @brief 对单个设备执行一轮查询
 */
ctu_error_t ctu_poller_poll_device(ctu_poller_t* poller, ctu_device_t device);

/**
 * @brief 获取统计快照（同时刷新近 1s 接收速率）
 */
ctu_error_t ctu_poller_get_stats(ctu_poller_t* poller, ctu_poll_stats_t* out);

#ifdef __cplusplus
}
#endif

#endif /* __CTU_POLLER_H */
