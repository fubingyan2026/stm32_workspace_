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
#include "srv_device_monitor.h"
#include "srv_fan_ctrl.h"
#include "srv_pwr_ctrl.h"
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
    d->status.bits.err_24v_computer = !st.comp_24v_ok;
    d->status.bits.err_aux_power = !st.aux_power_ok;
    d->status.bits.err_hsd_fault = st.hsd_fault;
    d->status.bits.err_dbr = st.dbr_ocp;
    d->status.bits.err_motor_power = !st.motor_power_ok;
    d->status.bits.err_chg_out = st.motor_chg_ocp;

    /* 模拟输入 A_IN1_IO/2_IO/3_IO（统一由 srv_adc 经 CD4051B 采样阈值判定） */
    const uint8_t ain_mask = srv_adc_read_ain();
    d->status.bits.a_in1_io = (ain_mask >> 0) & 1U;
    d->status.bits.a_in2_io = (ain_mask >> 1) & 1U;
    d->status.bits.a_in3_io = (ain_mask >> 2) & 1U;

    /* 急停状态 */
    d->status.bits.stop_key_state = st.estop_on;

    /* 子设备在线状态（srv_device_monitor 喂狗超时判定） */
  
    /* 风扇故障（逐路检测） */
    d->status.bits.err_fan0 = srv_fan_ctrl_is_fault(0);
    d->status.bits.err_fan1 = srv_fan_ctrl_is_fault(1);

    /* NTC 连接状态（0x001 byte1 bit6/7）+ 温度（0x016）+ 电源电压/预充故障（0x017） */
    srv_adc_data_t adc;
    if (srv_adc_get_latest(&adc)) {
        d->status.bits.err_ntc1 = (adc.ntc1_status == SRV_ADC_CALC_ERR_OPEN);
        d->status.bits.err_ntc2 = (adc.ntc2_status == SRV_ADC_CALC_ERR_OPEN);
        d->volt_temp.data.ntc1_temp_x100 = adc.ntc1_temp_x100;
        d->volt_temp.data.ntc2_temp_x100 = adc.ntc2_temp_x100;
        d->volt_temp.data.mcu_temp_x100 = adc.mcu_temp_x100;
        d->power_fault.data.vin_mv = (uint16_t)(adc.vin_mv & 0xFFFFU);
        d->power_fault.data.motor_power_mv = (uint16_t)(adc.motor_power_mv & 0xFFFFU);
        d->power_fault.data.aux_power_mv = (uint16_t)(adc.aux_power_mv & 0xFFFFU);
    }

    /* 电机预充故障码（0x017 Byte6） */
    d->power_fault.data.precharge_fault = srv_pwr_ctrl_get_precharge_fault();

}
