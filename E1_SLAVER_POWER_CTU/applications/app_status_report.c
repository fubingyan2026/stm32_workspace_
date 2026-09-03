/**
 * @file    app_status_report.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   应用层 — 系统状态上报聚合实现 (E1_SLAVER_POWER_CTU)
 */

/* Includes ------------------------------------------------------------------*/
#include "app_status_report.h"

#include "log.h"
#include "srv_adc.h"
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

void app_status_report_fill(srv_com_slv_report_t* report)
{
    if (!report) {
        return;
    }
    memset(report, 0, sizeof(*report));

    /* 故障锁存（err_* = 对应输出锁存位） */
    const uint8_t latch = srv_pwr_ctrl_get_latch_mask();
    report->status.bits.err_24v = (latch & SRV_PWR_OUT_MASK_DC24V) != 0;
    report->status.bits.err_12v = (latch & SRV_PWR_OUT_MASK_ISO12V) != 0;
    report->status.bits.err_lsd1 = (latch & SRV_PWR_OUT_MASK_LSD1) != 0;
    report->status.bits.err_lsd2 = (latch & SRV_PWR_OUT_MASK_LSD2) != 0;

    /* 实际输出 + 锁存标志 */
    const uint8_t on = srv_pwr_ctrl_get_on_outputs();
    report->status.bits.out_24v = (on & SRV_PWR_OUT_MASK_DC24V) != 0;
    report->status.bits.out_12v = (on & SRV_PWR_OUT_MASK_ISO12V) != 0;
    report->status.bits.out_lsd1 = (on & SRV_PWR_OUT_MASK_LSD1) != 0;
    report->status.bits.out_lsd2 = (on & SRV_PWR_OUT_MASK_LSD2) != 0;
    report->status.bits.latch_active = (latch != 0);

    /* 电压/温度/输入存在（含 AUX/MOTOR 缺失判定） */
    srv_adc_data_t adc;
    if (srv_adc_get_latest(&adc)) {
        report->aux_mv = adc.aux_mv;
        report->motor_mv = adc.motor_mv;
        report->lsd1_mv = adc.lsd1_mv;
        report->lsd2_mv = adc.lsd2_mv;
        report->mcu_temp_x100 = adc.mcu_temp_x100;
        report->vdda_mv = adc.vdda_mv;

        report->status.bits.err_aux = (adc.aux_mv < SRV_PWR_AUX_PRESENT_MV);
        report->status.bits.err_motor = (adc.motor_mv < SRV_PWR_MOTOR_PRESENT_MV);
    }
}
