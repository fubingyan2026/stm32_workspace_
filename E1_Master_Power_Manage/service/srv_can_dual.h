/**
 * @file    srv_can_dual.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   双电池 CAN 上行协议解析服务（0x200/0x201/0x202）
 * @attention
 *
 * 接收双电池管理模块的上行 CAN 帧，解析并缓存数据。
 * 通过 srv_can_dual_process_rx() 从 can_task RX 回调接入。
 *
 * 协议详见 docs/protocol_dual.md
 */

#ifndef __SRV_CAN_DUAL_H
#define __SRV_CAN_DUAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief 双电池 CAN ID */
#define SRV_CAN_DUAL_ID_CORE (0x200U) /**< 核心动态帧（100ms MUX） */
#define SRV_CAN_DUAL_ID_INFO (0x201U) /**< 低频信息帧（请求响应） */
#define SRV_CAN_DUAL_ID_FAULT (0x202U) /**< 详细故障帧 */

/** @brief 信息键值（0x201 帧 Byte0） */
#define SRV_CAN_DUAL_INFO_KEY_CAPACITY (0x01U) /**< 容量信息 */
#define SRV_CAN_DUAL_INFO_KEY_VERSION (0x03U) /**< 版本与循环信息 */

/** @brief 电池编号 */
#define SRV_CAN_DUAL_BAT_ID_1 (0x01U)
#define SRV_CAN_DUAL_BAT_ID_2 (0x02U)

/** @brief 故障严重等级 */
typedef enum {
    SRV_CAN_DUAL_FAULT_LV_NORMAL = 0,
    SRV_CAN_DUAL_FAULT_LV_MINOR,
    SRV_CAN_DUAL_FAULT_LV_SEVERE,
    SRV_CAN_DUAL_FAULT_LV_CRITICAL,
} srv_can_dual_fault_level_t;

/* Exported types ------------------------------------------------------------*/

/** @brief 0x200 — 电池核心动态数据 */
typedef struct {
    uint8_t bat_id; /**< 电池编号 */
    uint16_t voltage; /**< 总电压 (0.01V) */
    int16_t current; /**< 总电流 (0.01A, 正=放电) */
    uint8_t soc; /**< 荷电状态 (1%) */
    int8_t cell_temp; /**< 电芯温度 (°C) */
    bool is_charging; /**< 充电中 */
} srv_can_dual_core_t;

/** @brief 0x201 Key=0x01 — 容量信息 */
typedef struct {
    uint32_t design_cap; /**< 设计容量 (mAh) */
    uint16_t full_cap; /**< 满充容量 (mAh) */
} srv_can_dual_capacity_t;

/** @brief 0x201 Key=0x03 — 版本与循环 */
typedef struct {
    uint16_t hw_version; /**< 硬件版本 */
    uint16_t sw_version; /**< 软件版本 */
    uint16_t cycle_count; /**< 循环次数 */
} srv_can_dual_version_t;

/** @brief 0x202 电池1 (CAN 原生) 故障详情 */
typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t cell_ov : 1; /**< 电芯过压保护 */
            uint8_t total_ov : 1; /**< 总压过压保护 */
            uint8_t fully_charged : 1; /**< 充满保护 */
            uint8_t cell_uv : 1; /**< 电芯欠压保护 */
            uint8_t total_uv : 1; /**< 总压欠压保护 */
            uint8_t volt_rsv : 3; /**< 预留 */
        };
    } volt; /**< Byte1: 电压类故障 */

    union {
        uint8_t raw;
        struct {
            uint8_t short_circuit : 1; /**< 放电短路保护 */
            uint8_t dischg_oc : 1; /**< 放电过流保护 */
            uint8_t chg_oc : 1; /**< 充电过流保护 */
            uint8_t curr_rsv : 5; /**< 预留 */
        };
    } curr; /**< Byte2: 电流/短路故障 */

    union {
        uint8_t raw;
        struct {
            uint8_t chg_ov_temp : 1; /**< 充电高温保护 */
            uint8_t dischg_ov_temp : 1; /**< 放电高温保护 */
            uint8_t mos_ov_temp : 1; /**< MOS 过温保护 */
            uint8_t amb_ov_temp : 1; /**< 环境高温保护 */
            uint8_t amb_low_temp : 1; /**< 环境低温保护 */
            uint8_t temp_rsv : 3; /**< 预留 */
        };
    } temp; /**< Byte3: 温度故障 */

    union {
        uint8_t raw;
        struct {
            uint8_t temp_sensor_fail : 1; /**< 温度采集失效 */
            uint8_t volt_sensor_fail : 1; /**< 电压采集失效 */
            uint8_t dischg_mos_fail : 1; /**< 放电 MOS 失效 */
            uint8_t chg_mos_fail : 1; /**< 充电 MOS 失效 */
            uint8_t cell_imbalance : 1; /**< 电芯不均衡告警 */
            uint8_t hw_rsv : 3; /**< 预留 */
        };
    } hw; /**< Byte4: 硬件/采集故障 */

    uint16_t warnings; /**< Byte5-6: 其他预警位掩码 */
    srv_can_dual_fault_level_t level; /**< Byte7: 严重等级 */
} srv_can_dual_fault_bat1_t;

/** @brief 0x202 电池2 (RYDER) 故障详情 */
typedef struct {
    union {
        uint8_t raw;
        struct {
            uint8_t chg_temp_prot : 2; /**< 充电高/低温保护(2位) */
            uint8_t dischg_temp_prot : 2; /**< 放电高/低温保护(2位) */
            uint8_t chg_temp_warn : 2; /**< 充电高/低温告警(2位) */
            uint8_t dischg_temp_warn : 2; /**< 放电高/低温告警(2位) */
        };
    } temp; /**< Byte1: 温度保护/告警 */

    union {
        uint8_t raw;
        struct {
            uint8_t chg_ov_prot : 1; /**< 充电过压保护 */
            uint8_t chg_ov_hw : 1; /**< 充电过压保护(硬件) */
            uint8_t chg_ov_second : 1; /**< 充电过压二次保护 */
            uint8_t dischg_uv_prot : 1; /**< 放电欠压保护 */
            uint8_t dischg_uv_hw : 1; /**< 放电欠压保护(硬件) */
            uint8_t volt_rsv : 3; /**< 预留 */
        };
    } volt; /**< Byte2: 电压保护 */

    union {
        uint8_t raw;
        struct {
            uint8_t chg_oc_prot : 1; /**< 充电过流保护 */
            uint8_t short_circuit : 1; /**< 短路保护 */
            uint8_t dischg_oc_prot : 1; /**< 放电过流保护 */
            uint8_t dischg_oc_second : 1; /**< 放电过流二次保护 */
            uint8_t dischg_oc_hw : 1; /**< 放电过流硬件保护 */
            uint8_t hw_defected : 1; /**< 电池硬件损坏 */
            uint8_t curr_rsv : 2; /**< 预留 */
        };
    } curr; /**< Byte3: 电流/硬件故障 */

    union {
        uint8_t raw;
        struct {
            uint8_t chg_ov_warn : 1; /**< 充电过压告警 */
            uint8_t dischg_uv_warn : 1; /**< 放电欠压告警 */
            uint8_t chg_oc_warn : 1; /**< 充电过流告警 */
            uint8_t dischg_oc_warn : 1; /**< 放电过流告警 */
            uint8_t chg_fet_fail : 1; /**< 充电 FET 失效 */
            uint8_t dischg_fet_fail : 1; /**< 放电 FET 失效 */
            uint8_t fuse_blown : 1; /**< 三端保险丝熔断 */
            uint8_t fuse_rsv : 1; /**< 预留 */
        };
    } fet; /**< Byte4: 告警+FET/保险 */

    uint16_t extra_warnings; /**< Byte5-6: 其他告警位掩码 */
    srv_can_dual_fault_level_t level; /**< Byte7: 严重等级 */
} srv_can_dual_fault_bat2_t;

/**
 * @brief 双电池完整数据体
 *
 * 由 RX 解析自动填充，通过 srv_can_dual_get_snapshot() 读取。
 */
typedef struct {
    /* 电池1 核心数据 */
    srv_can_dual_core_t bat1_core;
    srv_can_dual_capacity_t bat1_capacity;
    srv_can_dual_version_t bat1_version;
    srv_can_dual_fault_bat1_t bat1_fault;
    bool bat1_online;

    /* 电池2 核心数据 */
    srv_can_dual_core_t bat2_core;
    srv_can_dual_capacity_t bat2_capacity;
    srv_can_dual_version_t bat2_version;
    srv_can_dual_fault_bat2_t bat2_fault;
    bool bat2_online;

    uint32_t response_count; /**< 总响应计数 */
} srv_can_dual_data_t;

/**
 * @brief CAN 发送回调（用于向双电池模块请求数据）
 * @return true=发送成功
 */
typedef bool (*srv_can_dual_send_cb_t)(uint16_t can_id, const uint8_t* data, uint8_t len);

/** @brief 服务配置 */
typedef struct {
    srv_can_dual_send_cb_t send_frame; /**< CAN 发送回调 */
} srv_can_dual_config_t;

typedef enum {
    SRV_CAN_DUAL_OK = 0,
    SRV_CAN_DUAL_ERROR_NULL_PTR,
    SRV_CAN_DUAL_ERROR_UNINITIALIZED,
} srv_can_dual_error_t;

/* Exported functions prototypes ---------------------------------------------*/

srv_can_dual_error_t srv_can_dual_init(const srv_can_dual_config_t* config);
void srv_can_dual_deinit(void);
bool srv_can_dual_is_initialized(void);

/**
 * @brief 处理接收到的双电池 CAN 帧
 * @param can_id CAN ID (0x200/0x201/0x202)
 * @param data   帧数据 (8 字节)
 * @param len    帧长度
 */
void srv_can_dual_process_rx(uint32_t can_id, const uint8_t* data, uint8_t len);

/** @brief 获取数据快照 */
const srv_can_dual_data_t* srv_can_dual_get_snapshot(void);

/** @brief 请求 0x201 信息帧（异步，结果由下一个 RX 帧更新） */
void srv_can_dual_request_info(uint8_t bat_id, uint8_t info_key);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_CAN_DUAL_H */
