/**
 * @file    app_fault_policy.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-05
 * @brief   应用层 — 故障保护策略实现
 */

/* Includes ------------------------------------------------------------------*/
#include "app_fault_policy.h"

#include "log.h"
#include "srv_fan_ctrl.h"
#include "srv_pwr_ctrl.h"
#include "srv_pwr_det.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define APP_FAULT_POLICY_LOG_ENABLE 1

#if APP_FAULT_POLICY_LOG_ENABLE
#define APP_FAULT_POLICY_LOG_E(...) LOG_E("app_fault_policy", __VA_ARGS__)
#define APP_FAULT_POLICY_LOG_W(...) LOG_W("app_fault_policy", __VA_ARGS__)
#define APP_FAULT_POLICY_LOG_I(...) LOG_I("app_fault_policy", __VA_ARGS__)
#define APP_FAULT_POLICY_LOG_D(...) LOG_D("app_fault_policy", __VA_ARGS__)
#else
#define APP_FAULT_POLICY_LOG_E(...) ((void)0)
#define APP_FAULT_POLICY_LOG_W(...) ((void)0)
#define APP_FAULT_POLICY_LOG_I(...) ((void)0)
#define APP_FAULT_POLICY_LOG_D(...) ((void)0)
#endif

/* Private variables ---------------------------------------------------------*/

static bool s_tripped; /**< 保护锁存标志 */

/* Private function prototypes -----------------------------------------------*/

/** @brief 判定关键电源故障（可调策略：哪些条件必须立即断电） */
static bool fault_policy_critical(const srv_pwr_det_status_t* st);

/* Exported functions --------------------------------------------------------*/

void app_fault_policy_init(void)
{
    s_tripped = false;
}

void app_fault_policy_step(uint16_t elapsed_ms)
{
    (void)elapsed_ms;

    /* 已锁存：保持断电状态，等待操作员显式复位 */
    if (s_tripped) {
        return;
    }

    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);

    /*
     * 触发条件：
     * - E-STOP 按下：无条件触发（安全按钮，与上电状态无关）
     * - 已上电完成且出现关键电源故障：立即关断
     * 上电时序过程中的 PGOOD 超时由 srv_pwr_ctrl FSM 内部内联锁处理，
     * 不在此处重复判定。
     */
    const bool powered = srv_pwr_ctrl_is_powered_on();
    if (st.estop_on || (powered && fault_policy_critical(&st))) {
        s_tripped = true;

        /* 1. 紧急断电：关闭全部输出轨并复位 FSM */
        srv_pwr_ctrl_emergency_off();

        /* 2. 风扇满速散热（关闭温控自动，强制最高转速） */
        srv_fan_ctrl_set_auto(false);
        srv_fan_ctrl_set_duty(0, 100U);
        srv_fan_ctrl_set_duty(1, 100U);

        /* 3. 触发日志（锁存后仅打印一次） */
        APP_FAULT_POLICY_LOG_E("保护触发并锁存: estop=%d motor_pwr=%d aux_pwr=%d "
            "dbr_ocp=%d chg_ocp=%d hsd_fault=%d",
            (int)st.estop_on, (int)st.motor_power_ok, (int)st.aux_power_ok,
            (int)st.dbr_ocp, (int)st.motor_chg_ocp, (int)st.hsd_fault);
    }
}

bool app_fault_policy_is_tripped(void)
{
    return s_tripped;
}

void app_fault_policy_reset(void)
{
    if (s_tripped) {
        s_tripped = false;
        APP_FAULT_POLICY_LOG_I("保护锁存已解除（操作员确认）");
    }
}

/* Private functions ---------------------------------------------------------*/

static bool fault_policy_critical(const srv_pwr_det_status_t* st)
{
    /* 关键电源轨丢失或过流/硬件故障 → 必须立即断电。
       外部输入轨（ext_12v/24v/comp）仅上报不触发，避免输入瞬断导致误关断。 */
    return !st->motor_power_ok
        || !st->aux_power_ok
        || st->dbr_ocp
        || st->motor_chg_ocp
        || st->hsd_fault;
}
