/**
 * @file    srv_adc.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   ADC 采样服务实现 — 原始值到物理量换算
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_adc.h"

#include "drv_adc.h"
#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_ADC_LOG_ENABLE 1

#if SRV_ADC_LOG_ENABLE
#define SRV_ADC_LOG_E(...) LOG_E("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_W(...) LOG_W("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_I(...) LOG_I("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_D(...) LOG_D("srv_adc", __VA_ARGS__)
#else
#define SRV_ADC_LOG_E(...) ((void)0)
#define SRV_ADC_LOG_W(...) ((void)0)
#define SRV_ADC_LOG_I(...) ((void)0)
#define SRV_ADC_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 参考电压 (mV) 与 ADC 满量程（12-bit = 4095） */
#define SRV_ADC_VREF_MV (3300U)
#define SRV_ADC_FULL_SCALE (4095U)

/** @brief 采样遥测日志限频窗口 (ms) */
#define SRV_ADC_TELEMETRY_LOG_PERIOD_MS (1000U)

/* Private variables ---------------------------------------------------------*/

/** @brief 初始化标志 */
static bool s_initialized;

/** @brief 最新换算快照 */
static srv_adc_data_t s_latest;

/** @brief 上次遥测日志时间戳 (ms) */
static uint32_t s_telemetry_log_ts;

/* Exported functions --------------------------------------------------------*/

void srv_adc_init(void)
{
    s_latest.timestamp_ms = 0;
    s_latest.vin_mv = 0;
    s_latest.vin_raw = 0;
    s_latest.v_imon_raw = 0;
    s_latest.valid = false;
    s_telemetry_log_ts = 0;
    s_initialized = true;

    SRV_ADC_LOG_I("ADC 采样服务初始化完成");
}

void srv_adc_trigger(void)
{
    if (!s_initialized) {
        return;
    }
    (void)drv_adc_trigger();
}

void srv_adc_step(void)
{
    if (!s_initialized) {
        return;
    }

    /* 使用最近一次已完成采样的原始值（drv_adc 的 DMA 缓冲内） */
    s_latest.vin_raw = (uint16_t)drv_adc_read_raw(DRV_ADC_CH_VIN);
    s_latest.v_imon_raw = (uint16_t)drv_adc_read_raw(DRV_ADC_CH_V_IMON);
    s_latest.vin_mv = (uint32_t)s_latest.vin_raw * SRV_ADC_VREF_MV / SRV_ADC_FULL_SCALE;
    s_latest.timestamp_ms = millis();
    s_latest.valid = true;

    /* 周期遥测日志 */
    const uint32_t now_ms = s_latest.timestamp_ms;
    if ((uint32_t)(now_ms - s_telemetry_log_ts) >= SRV_ADC_TELEMETRY_LOG_PERIOD_MS) {
        s_telemetry_log_ts = now_ms;
        SRV_ADC_LOG_I("VIN=%lumV (%u) V_IMON_raw=%u",
            (unsigned long)s_latest.vin_mv,
            (unsigned)s_latest.vin_raw,
            (unsigned)s_latest.v_imon_raw);
    }
}

bool srv_adc_get_latest(srv_adc_data_t* sample)
{
    if (!s_initialized || !sample) {
        return false;
    }
    *sample = s_latest;
    return s_latest.valid;
}
