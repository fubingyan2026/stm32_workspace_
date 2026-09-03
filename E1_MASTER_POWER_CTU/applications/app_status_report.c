/**
 * @file    app_status_report.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   应用层 — 系统状态上报聚合实现 (E1_MASTER_POWER_CTU)
 */

/* Includes ------------------------------------------------------------------*/
#include "app_status_report.h"

#include "log.h"
#include "srv_adc.h"
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

void app_status_report_fill(srv_com_mst_report_t* report)
{
    if (!report) {
        return;
    }
    memset(report, 0, sizeof(*report));

    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);

    report->status.bits.stop_key_state = st.estop_on;
    report->status.bits.err_12v = !st.p12v_ok;
    report->status.bits.err_24v = !st.dc24v_ok;
    report->status.bits.err_vin_dcdc = !st.lm5060_ok;
    report->status.bits.err_aux_power = !st.aux_power_ok;
    report->status.bits.err_motor_power = !st.motor_power_ok;

    /* 风扇故障（逐路检测） */
    report->status.bits.err_fan0 = srv_fan_ctrl_is_fault(0);
    report->status.bits.err_fan1 = srv_fan_ctrl_is_fault(1);

    /* NTC 连接状态 + 温度 + 电压 */
    srv_adc_data_t adc;
    if (srv_adc_get_latest(&adc)) {
        report->status.bits.err_ntc1 = (adc.ntc1_status == SRV_ADC_CALC_ERR_OPEN);
        report->status.bits.err_ntc2 = (adc.ntc2_status == SRV_ADC_CALC_ERR_OPEN);
        report->ntc1_temp_x100 = adc.ntc1_temp_x100;
        report->ntc2_temp_x100 = adc.ntc2_temp_x100;
        report->mcu_temp_x100 = adc.mcu_temp_x100;
        report->vin_mv = (uint16_t)(adc.vin_mv & 0xFFFFU);
        report->vin_dcdc_mv = (uint16_t)(adc.vin_dcdc_mv & 0xFFFFU);
    }
}
