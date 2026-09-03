/**
 * @file    app_status_report.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   应用层 — 系统状态上报聚合 (E1_SLAVER_POWER_CTU)
 *
 * 用例：聚合 srv_pwr_det / srv_adc / srv_pwr_ctrl 各服务数据，
 * 填充 srv_com_slv_report_t，供 RS485 主机查询应答。
 * 只调用 service，不拥有 sw_timer，不触碰 HAL/驱动。
 */

#ifndef __APP_STATUS_REPORT_H
#define __APP_STATUS_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "srv_com_slv.h"

/**
 * @brief 填充系统状态上报数据体
 * @param report 待填充的数据体（可为 NULL，此时静默返回）
 * @note  签名与 srv_com_slv_read_cb_t 一致，可直接注册为 read_data 回调
 */
void app_status_report_fill(srv_com_slv_report_t* report);

#ifdef __cplusplus
}
#endif

#endif /* __APP_STATUS_REPORT_H */
