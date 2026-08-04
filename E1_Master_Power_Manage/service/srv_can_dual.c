/**
 * @file    srv_can_dual.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   双电池 CAN 上行协议解析服务实现
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_can_dual.h"

#include <string.h>

#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_CAN_DUAL_LOG_ENABLE 1

#if SRV_CAN_DUAL_LOG_ENABLE
#define SRV_CAN_DUAL_LOG_E(...) LOG_E("srv_can_dual", __VA_ARGS__)
#define SRV_CAN_DUAL_LOG_W(...) LOG_W("srv_can_dual", __VA_ARGS__)
#define SRV_CAN_DUAL_LOG_I(...) LOG_I("srv_can_dual", __VA_ARGS__)
#define SRV_CAN_DUAL_LOG_D(...) LOG_D("srv_can_dual", __VA_ARGS__)
#else
#define SRV_CAN_DUAL_LOG_E(...) ((void)0)
#define SRV_CAN_DUAL_LOG_W(...) ((void)0)
#define SRV_CAN_DUAL_LOG_I(...) ((void)0)
#define SRV_CAN_DUAL_LOG_D(...) ((void)0)
#endif

/** @brief 核心帧遥测日志限频窗口 (ms)：0x200 每 100ms 一帧，需限频防刷屏 */
#define SRV_CAN_DUAL_CORE_LOG_PERIOD_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static srv_can_dual_config_t s_config;
static srv_can_dual_data_t s_data;
static bool s_initialized;
static uint32_t s_core_log_ts; /**< 上次核心帧遥测日志时间戳 (ms) */

/* Private function prototypes -----------------------------------------------*/

static void parse_core(uint8_t bat_id, const uint8_t* data);
static void parse_info(const uint8_t* data);
static void parse_fault(uint8_t bat_id, const uint8_t* data);

/* Exported functions --------------------------------------------------------*/

srv_can_dual_error_t srv_can_dual_init(const srv_can_dual_config_t* config)
{
    if (!config || !config->send_frame) {
        return SRV_CAN_DUAL_ERROR_NULL_PTR;
    }

    if (s_initialized) {
        srv_can_dual_deinit();
    }

    s_config = *config;
    memset(&s_data, 0, sizeof(s_data));
    s_initialized = true;

    SRV_CAN_DUAL_LOG_I("双电池 CAN 解析服务初始化完成");

    return SRV_CAN_DUAL_OK;
}

void srv_can_dual_deinit(void)
{
    memset(&s_config, 0, sizeof(s_config));
    memset(&s_data, 0, sizeof(s_data));
    s_initialized = false;
}

bool srv_can_dual_is_initialized(void)
{
    return s_initialized;
}

void srv_can_dual_process_rx(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    if (!s_initialized || !data || len < 8) {
        return;
    }

    s_data.response_count++;

    switch (can_id) {
    case SRV_CAN_DUAL_ID_CORE: /* 0x200 */
        if (data[0] == SRV_CAN_DUAL_BAT_ID_1 || data[0] == SRV_CAN_DUAL_BAT_ID_2) {
            parse_core(data[0], data);
        }
        break;

    case SRV_CAN_DUAL_ID_INFO: /* 0x201 */
        parse_info(data);
        break;

    case SRV_CAN_DUAL_ID_FAULT: /* 0x202 */
        if (data[0] == SRV_CAN_DUAL_BAT_ID_1 || data[0] == SRV_CAN_DUAL_BAT_ID_2) {
            parse_fault(data[0], data);
        }
        break;

    default:
        break;
    }
}

const srv_can_dual_data_t* srv_can_dual_get_snapshot(void)
{
    return &s_data;
}

void srv_can_dual_request_info(uint8_t bat_id, uint8_t info_key)
{
    if (!s_initialized || !s_config.send_frame)
        return;

    uint8_t frame[8];
    memset(frame, 0, 8);
    frame[0] = bat_id;
    frame[1] = info_key;

    s_config.send_frame(SRV_CAN_DUAL_ID_INFO, frame, 8);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 解析 0x200 核心动态帧
 *
 * 数据布局（小端）：
 *   Byte 0: bat_id
 *   Byte 1-2: voltage (uint16, 0.01V)
 *   Byte 3-4: current (int16, 0.01A, 正=放电)
 *   Byte 5: soc (%)
 *   Byte 6: cell_temp (int8, °C, 有符号)
 *   Byte 7: is_charging (0/1)
 */
static void parse_core(uint8_t bat_id, const uint8_t* data)
{
    srv_can_dual_core_t* c = (bat_id == SRV_CAN_DUAL_BAT_ID_1) ? &s_data.bat1_core : &s_data.bat2_core;
    bool* online = (bat_id == SRV_CAN_DUAL_BAT_ID_1) ? &s_data.bat1_online : &s_data.bat2_online;

    c->bat_id = data[0];
    c->voltage = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    c->current = (int16_t)((uint16_t)data[3] | ((uint16_t)data[4] << 8));
    c->soc = data[5];
    c->cell_temp = (int8_t)data[6];
    c->is_charging = (data[7] != 0);
    *online = true;

    /* 核心帧遥测（限频 1s；电压 0.01V / 电流 0.01A 手拆小数，禁止 %f） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_core_log_ts) >= SRV_CAN_DUAL_CORE_LOG_PERIOD_MS) {
        s_core_log_ts = now_ms;
        SRV_CAN_DUAL_LOG_D("电池%u 核心帧: 电压=%u.%02uV 电流=%d.%02uA SOC=%u%% 温度=%d°C %s",
            (unsigned)bat_id,
            (unsigned)(c->voltage / 100), (unsigned)(c->voltage % 100),
            (int)(c->current / 100),
            (unsigned)((c->current < 0) ? (unsigned)(-(c->current)) % 100 : (unsigned)(c->current % 100)),
            (unsigned)c->soc,
            (int)c->cell_temp,
            c->is_charging ? "充电" : "放电");
    }
}

/**
 * @brief 解析 0x201 低频信息帧
 *
 * Byte 0: info_key → 决定后续字节解析方式
 * Key 0x01: design_cap(uint32) + full_cap(uint16) + reserved
 * Key 0x03: hw_ver(uint16) + sw_ver(uint16) + cycle_count(uint16) + reserved
 *
 * @note  0x201 帧不含 bat_id，假定与上一个 0x200 帧来自同一电池。
 *        这里同时更新两个电池（通常只有一个在线）。
 */
static void parse_info(const uint8_t* data)
{
    uint8_t key = data[0];

    SRV_CAN_DUAL_LOG_D("电池信息帧: key=0x%02X", (unsigned)key);

    if (key == SRV_CAN_DUAL_INFO_KEY_CAPACITY) {
        srv_can_dual_capacity_t* cap1 = &s_data.bat1_capacity;
        srv_can_dual_capacity_t* cap2 = &s_data.bat2_capacity;

        uint32_t design = (uint32_t)data[1] | ((uint32_t)data[2] << 8)
            | ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
        uint16_t full = (uint16_t)data[5] | ((uint16_t)data[6] << 8);

        cap1->design_cap = design;
        cap1->full_cap = full;
        cap2->design_cap = design;
        cap2->full_cap = full;
        return;
    }

    if (key == SRV_CAN_DUAL_INFO_KEY_VERSION) {
        srv_can_dual_version_t* ver1 = &s_data.bat1_version;
        srv_can_dual_version_t* ver2 = &s_data.bat2_version;

        ver1->hw_version = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
        ver1->sw_version = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
        ver1->cycle_count = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
        ver2->hw_version = ver1->hw_version;
        ver2->sw_version = ver1->sw_version;
        ver2->cycle_count = ver1->cycle_count;
        return;
    }
}

/**
 * @brief 解析 0x202 详细故障帧
 *
 * Byte 0: bat_id
 * Byte 1-7: 故障结构体（电池1 = CAN 原生, 电池2 = RYDER）
 */
static void parse_fault(uint8_t bat_id, const uint8_t* data)
{
    if (bat_id == SRV_CAN_DUAL_BAT_ID_1) {
        srv_can_dual_fault_bat1_t* f = &s_data.bat1_fault;
        f->volt.raw = data[1];
        f->curr.raw = data[2];
        f->temp.raw = data[3];
        f->hw.raw = data[4];
        f->warnings = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
        f->level = (data[7] <= SRV_CAN_DUAL_FAULT_LV_CRITICAL)
            ? (srv_can_dual_fault_level_t)data[7]
            : SRV_CAN_DUAL_FAULT_LV_NORMAL;

        /* 故障日志（按严重等级：SEVERE 以上 E，MINOR W） */
        if (f->level >= SRV_CAN_DUAL_FAULT_LV_SEVERE) {
            SRV_CAN_DUAL_LOG_E("电池1 严重故障(level=%u): 电压=0x%02X 电流=0x%02X 温度=0x%02X 硬件=0x%02X 预警=0x%04X",
                (unsigned)f->level, (unsigned)f->volt.raw, (unsigned)f->curr.raw,
                (unsigned)f->temp.raw, (unsigned)f->hw.raw, (unsigned)f->warnings);
        } else if (f->level == SRV_CAN_DUAL_FAULT_LV_MINOR) {
            SRV_CAN_DUAL_LOG_W("电池1 轻微故障(level=%u): 电压=0x%02X 电流=0x%02X 温度=0x%02X 硬件=0x%02X 预警=0x%04X",
                (unsigned)f->level, (unsigned)f->volt.raw, (unsigned)f->curr.raw,
                (unsigned)f->temp.raw, (unsigned)f->hw.raw, (unsigned)f->warnings);
        }
        return;
    }

    if (bat_id == SRV_CAN_DUAL_BAT_ID_2) {
        srv_can_dual_fault_bat2_t* f = &s_data.bat2_fault;
        f->temp.raw = data[1];
        f->volt.raw = data[2];
        f->curr.raw = data[3];
        f->fet.raw = data[4];
        f->extra_warnings = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
        f->level = (data[7] <= SRV_CAN_DUAL_FAULT_LV_CRITICAL)
            ? (srv_can_dual_fault_level_t)data[7]
            : SRV_CAN_DUAL_FAULT_LV_NORMAL;

        /* 故障日志（电池2 为 RYDER 协议，字段布局不同） */
        if (f->level >= SRV_CAN_DUAL_FAULT_LV_SEVERE) {
            SRV_CAN_DUAL_LOG_E("电池2 严重故障(level=%u): 温度=0x%02X 电压=0x%02X 电流=0x%02X FET/告警=0x%02X 预警=0x%04X",
                (unsigned)f->level, (unsigned)f->temp.raw, (unsigned)f->volt.raw,
                (unsigned)f->curr.raw, (unsigned)f->fet.raw, (unsigned)f->extra_warnings);
        } else if (f->level == SRV_CAN_DUAL_FAULT_LV_MINOR) {
            SRV_CAN_DUAL_LOG_W("电池2 轻微故障(level=%u): 温度=0x%02X 电压=0x%02X 电流=0x%02X FET/告警=0x%02X 预警=0x%04X",
                (unsigned)f->level, (unsigned)f->temp.raw, (unsigned)f->volt.raw,
                (unsigned)f->curr.raw, (unsigned)f->fet.raw, (unsigned)f->extra_warnings);
        }
        return;
    }
}
