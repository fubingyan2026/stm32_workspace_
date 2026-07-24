/**
 * @file    power_detect.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   电源状态监控服务实现
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_det.h"

#include "drv_status.h"

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void srv_pwr_det_init(void)
{
    s_initialized = true;
}

void srv_pwr_det_read(srv_pwr_det_status_t* status)
{
    if (!status)
        return;

    uint32_t sta = s_initialized ? drv_status_read_all() : 0;

    status->ext_12v_ok = (sta >> DRV_STATUS_12V_PGOOD) & 1;
    status->ext_24v_ok = (sta >> DRV_STATUS_24V_PGOOD) & 1;
    status->comp_24v_ok = (sta >> DRV_STATUS_24V_COMP_PGD) & 1;
    status->aux_power_ok = (sta >> DRV_STATUS_AUX_PGD) & 1;
    status->motor_power_ok = (sta >> DRV_STATUS_MOTOR_PGD) & 1;
    status->hsd_fault = (sta >> DRV_STATUS_HSD_FAULT) & 1;
    status->dbr_ocp = (sta >> DRV_STATUS_DBR_OCP_FLAG) & 1;
    status->motor_chg_ocp = (sta >> DRV_STATUS_MOTOR_CHG_OCP) & 1;
    status->estop_on = (sta >> DRV_STATUS_E_STOP_ON) & 1;

    /* 数字输入 */
    status->din1 = (sta >> DRV_STATUS_DIN1) & 1;
    status->din2 = (sta >> DRV_STATUS_DIN2) & 1;
    status->din3 = (sta >> DRV_STATUS_DIN3) & 1;
}

bool srv_pwr_det_all_power_ok(void)
{
    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);
    return st.ext_12v_ok && st.ext_24v_ok && st.comp_24v_ok
        && st.aux_power_ok && st.motor_power_ok;
}

bool srv_pwr_det_has_hsd_fault(void)
{
    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);
    return st.hsd_fault;
}

bool srv_pwr_det_has_dbr_ocp(void)
{
    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);
    return st.dbr_ocp;
}

bool srv_pwr_det_is_estop(void)
{
    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);
    return st.estop_on;
}

bool srv_pwr_det_has_motor_chg_ocp(void)
{
    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);
    return st.motor_chg_ocp;
}

uint8_t srv_pwr_det_read_din(void)
{
    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);
    uint8_t mask = 0;
    if (st.din1)
        mask |= (1U << 0);
    if (st.din2)
        mask |= (1U << 1);
    if (st.din3)
        mask |= (1U << 2);
    return mask;
}
