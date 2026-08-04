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
#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_PWR_DET_LOG_ENABLE 1

#if SRV_PWR_DET_LOG_ENABLE
#define SRV_PWR_DET_LOG_E(...) LOG_E("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_W(...) LOG_W("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_I(...) LOG_I("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_D(...) LOG_D("srv_pwr_det", __VA_ARGS__)
#else
#define SRV_PWR_DET_LOG_E(...) ((void)0)
#define SRV_PWR_DET_LOG_W(...) ((void)0)
#define SRV_PWR_DET_LOG_I(...) ((void)0)
#define SRV_PWR_DET_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 状态遥测日志限频窗口 (ms)：10ms 轮询需限频防刷屏 */
#define SRV_PWR_DET_LOG_PERIOD_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/** @brief 状态遥测日志时间戳 (ms) */
static uint32_t s_pwr_det_log_ts;

/** @brief 需要做边沿检测的故障类信号（0→1 断言 / 1→0 清除） */
static const drv_status_signal_t s_fault_signals[] = {
    DRV_STATUS_HSD_FAULT,      /**< HSD 高边驱动器故障汇总 */
    DRV_STATUS_DBR_OCP_FLAG,   /**< 制动电阻过流 */
    DRV_STATUS_MOTOR_CHG_OCP,  /**< 电机充电过流 */
    DRV_STATUS_E_STOP_ON,      /**< 急停 */
};
#define SRV_PWR_DET_FAULT_SIG_NUM \
    (uint32_t)(sizeof(s_fault_signals) / sizeof(s_fault_signals[0]))

/** @brief 上次轮询掩码（用于边沿检测） */
static uint32_t s_pwr_det_prev_mask;
/** @brief 首次读取标志：上电初始状态不判边沿，避免误报 */
static bool s_pwr_det_prev_valid;

/* Exported functions --------------------------------------------------------*/

void srv_pwr_det_init(void)
{
    /* 本服务封装 drv_status 读取 PGOOD/E-STOP/故障标志，必须先初始化其状态位 */
    drv_status_init();
    s_initialized = true;
    s_pwr_det_prev_valid = false;
    SRV_PWR_DET_LOG_I("电源状态监控服务初始化完成");
}

void srv_pwr_det_read(srv_pwr_det_status_t* status)
{
    if (!status)
        return;

    uint32_t sta = s_initialized ? drv_status_read_all() : 0;

    /* 故障信号边沿检测：仅状态变化时打印，避免 10ms 轮询刷屏 */
    if (s_pwr_det_prev_valid) {
        for (uint32_t i = 0; i < SRV_PWR_DET_FAULT_SIG_NUM; i++) {
            const drv_status_signal_t sig = s_fault_signals[i];
            const uint32_t bit = 1UL << (uint32_t)sig;
            const bool now_set = (sta & bit) != 0;
            const bool was_set = (s_pwr_det_prev_mask & bit) != 0;
            if (now_set && !was_set) {
                SRV_PWR_DET_LOG_E("故障信号断言: %s", drv_status_name(sig));
            } else if (!now_set && was_set) {
                SRV_PWR_DET_LOG_I("故障信号清除: %s", drv_status_name(sig));
            }
        }
    } else {
        s_pwr_det_prev_valid = true;
    }
    s_pwr_det_prev_mask = sta;

    status->ext_12v_ok = (sta >> DRV_STATUS_12V_PGOOD) & 1;
    status->ext_24v_ok = (sta >> DRV_STATUS_24V_PGOOD) & 1;
    status->comp_24v_ok = (sta >> DRV_STATUS_24V_COMP_PGD) & 1;
    status->aux_power_ok = (sta >> DRV_STATUS_AUX_PGD) & 1;
    status->motor_power_ok = (sta >> DRV_STATUS_MOTOR_PGD) & 1;
    status->hsd_fault = (sta >> DRV_STATUS_HSD_FAULT) & 1;
    status->dbr_ocp = (sta >> DRV_STATUS_DBR_OCP_FLAG) & 1;
    status->motor_chg_ocp = (sta >> DRV_STATUS_MOTOR_CHG_OCP) & 1;
    status->estop_on = (sta >> DRV_STATUS_E_STOP_ON) & 1;

    /* 状态遥测日志（限频 1s：10ms 轮询防刷屏） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_pwr_det_log_ts) >= SRV_PWR_DET_LOG_PERIOD_MS) {
        s_pwr_det_log_ts = now_ms;
        SRV_PWR_DET_LOG_D("电源状态: 12V=%u 24V=%u 工控24V=%u 辅助=%u 电机=%u | HSD故障=%u DBR过流=%u 电充过流=%u 急停=%u",
            (unsigned)status->ext_12v_ok, (unsigned)status->ext_24v_ok,
            (unsigned)status->comp_24v_ok, (unsigned)status->aux_power_ok,
            (unsigned)status->motor_power_ok,
            (unsigned)status->hsd_fault, (unsigned)status->dbr_ocp,
            (unsigned)status->motor_chg_ocp, (unsigned)status->estop_on);
    }
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
