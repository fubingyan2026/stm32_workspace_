/**
 * @file    app_status_report.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-05
 * @brief   应用层 — 系统状态上报聚合实现
 */

/* Includes ------------------------------------------------------------------*/
#include "app_status_report.h"

#include "log.h"
#include "srv_adc.h"
#include "srv_can_dual.h"
#include "srv_device_monitor.h"
#include "srv_fan_ctrl.h"
#include "srv_pwr_det.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define APP_STATUS_REPORT_LOG_ENABLE 1

#if APP_STATUS_REPORT_LOG_ENABLE
#define APP_STATUS_REPORT_LOG_E(...) LOG_E("app_status_report", __VA_ARGS__)
#define APP_STATUS_REPORT_LOG_W(...) LOG_W("app_status_report", __VA_ARGS__)
#define APP_STATUS_REPORT_LOG_I(...) LOG_I("app_status_report", __VA_ARGS__)
#define APP_STATUS_REPORT_LOG_D(...) LOG_D("app_status_report", __VA_ARGS__)
#else
#define APP_STATUS_REPORT_LOG_E(...) ((void)0)
#define APP_STATUS_REPORT_LOG_W(...) ((void)0)
#define APP_STATUS_REPORT_LOG_I(...) ((void)0)
#define APP_STATUS_REPORT_LOG_D(...) ((void)0)
#endif

/* Exported functions --------------------------------------------------------*/

void app_status_report_fill(srv_can_mst_data_t* d)
{
    if (!d) {
        return;
    }
    memset(d, 0, sizeof(*d));

    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);

    d->status.bits.err_12v_ext = !st.ext_12v_ok;
    d->status.bits.err_24v_ext = !st.ext_24v_ok;
    d->status.bits.err_24v_comp = !st.comp_24v_ok;
    d->status.bits.err_power = !st.aux_power_ok;
    d->status.bits.err_hsd1_12v = st.hsd_fault;
    d->status.bits.err_dbr = st.dbr_ocp;
    d->status.bits.err_motor = !st.motor_power_ok;
    d->status.bits.err_chg_out = st.motor_chg_ocp;

    /* 模拟输入 A_IN1_IO/2_IO/3_IO（统一由 srv_adc 经 CD4051B 采样阈值判定） */
    const uint8_t ain_mask = srv_adc_read_ain();
    d->status.bits.a_in1_io = (ain_mask >> 0) & 1U;
    d->status.bits.a_in2_io = (ain_mask >> 1) & 1U;
    d->status.bits.a_in3_io = (ain_mask >> 2) & 1U;

    /* 急停状态 */
    d->status.bits.stop_key_state = st.estop_on;

    /* 子设备在线状态（srv_device_monitor 喂狗超时判定） */
    d->status.bits.device_online_slaver = srv_device_monitor_is_online(SRV_DEVICE_SLAVER);
    d->status.bits.device_online_dual = srv_device_monitor_is_online(SRV_DEVICE_DUAL);

    /* 风扇故障（逐路检测） */
    d->status.bits.err_fan0 = srv_fan_ctrl_is_fault(0);
    d->status.bits.err_fan1 = srv_fan_ctrl_is_fault(1);

    /* 双电池数据 */
    const srv_can_dual_data_t* dual = srv_can_dual_get_snapshot();
    if (dual) {
        d->status.bits.device_online_bat1 = dual->bat1_online;
        d->status.bits.device_online_bat2 = dual->bat2_online;

        d->status.bits.battery_key_state = dual->bat1_online || dual->bat2_online;
        d->status.bits.battery_charging = dual->bat1_core.is_charging || dual->bat2_core.is_charging;

        d->status.bits.bat1_soc = 66; // dual->bat1_core.soc;
        d->status.bits.bat2_soc = 85; // dual->bat2_core.soc;
        d->bat1.voltage_dv = dual->bat1_core.voltage / 10; /* 0.01V → 0.1V */
        d->bat2.voltage_dv = dual->bat2_core.voltage / 10;
        d->bat1.current_da = dual->bat1_core.current / 10; /* 0.01A → 0.1A */
        d->bat2.current_da = dual->bat2_core.current / 10;
        d->bat1.temp_c = dual->bat1_core.cell_temp;
        d->bat2.temp_c = dual->bat2_core.cell_temp;
        d->bat1.charging = dual->bat1_core.is_charging;
        d->bat2.charging = dual->bat2_core.is_charging;
        d->bat1.capacity_mah = dual->bat1_capacity.design_cap;
        d->bat2.capacity_mah = dual->bat2_capacity.design_cap;
        d->bat1.cycle_count = dual->bat1_version.cycle_count;
        d->bat2.cycle_count = dual->bat2_version.cycle_count;
        d->bat1.hw_version = dual->bat1_version.hw_version;
        d->bat2.hw_version = dual->bat2_version.hw_version;
        d->bat1.sw_version = dual->bat1_version.sw_version;
        d->bat2.sw_version = dual->bat2_version.sw_version;

        /* 故障码 */
        d->bat1_fault = dual->bat1_fault;
        d->bat2_fault = dual->bat2_fault;
    }
}
