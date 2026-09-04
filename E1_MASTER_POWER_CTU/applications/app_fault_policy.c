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
#include "utils_math.h"

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

/** @brief 急停释放沿检测状态（复用共享 utils_edge_detect） */
static utils_edge_det_t s_estop_edge;

/* Private function prototypes -----------------------------------------------*/

/** @brief 判定关键电源故障（可调策略：哪些条件必须立即断电） */
static bool fault_policy_critical(const srv_pwr_det_status_t* st);

/** @brief 打印进入关键故障判定的具体原因（仅触发时调用） */
static void fault_policy_log_reasons(const srv_pwr_det_status_t* st);

/* Exported functions --------------------------------------------------------*/

void app_fault_policy_init(void)
{
    s_tripped = false;
    utils_edge_det_init(&s_estop_edge);
    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);
    if (!st.estop_on) {
        app_fault_policy_reset();
        srv_pwr_ctrl_request_on();
    }
}

void app_fault_policy_step(uint16_t elapsed_ms)
{
    (void)elapsed_ms;

    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);

    /* 已锁存：保持断电状态，等待操作员显式复位 */
    if (s_tripped && st.estop_on) {
        return;
    }

    /*
     * 触发条件：
     * - E-STOP 按下：无条件触发（安全按钮，与上电状态无关）
     * - MOTOR 已使能且其 PGD 丢失：立即关断 MOTOR
     * 三路常开轨（VIN_DC-DC/24V/AUX）的 PGD 仅监测不在此触发。
     */
    const bool powered = srv_pwr_ctrl_is_powered_on();
    if (st.estop_on || (powered && fault_policy_critical(&st))) {
        s_tripped = true;

        /* 1. 紧急断电：强制关闭 MOTOR（VIN/24V/AUX 常开轨保持） */
        srv_pwr_ctrl_emergency_off();

        /* 2. 风扇满速散热（关闭温控自动，强制最高转速） */
        srv_fan_ctrl_set_auto(false);
        srv_fan_ctrl_set_duty(0, 100U);
        srv_fan_ctrl_set_duty(1, 100U);

        /* 3. 打印触发原因明细 */
        fault_policy_log_reasons(&st);
    }

    /* 急停释放沿（有效急停 1→0）：解除锁存并重新使能 MOTOR */
    if (utils_edge_detect(&s_estop_edge, st.estop_on) == UTILS_EDGE_FALLING) {
        app_fault_policy_reset();
        srv_pwr_ctrl_request_on();
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
    /* MOTOR 使能后其 PGD 丢失 → 立即断电（独立受控轨，见 srv_pwr_ctrl 语义）。
       VIN_DC-DC/24V/AUX 三路常开轨 PGD 仅作状态监测/上报，异常时不触发整机断电，
       以保证急停对 MOTOR 的控制不受三路健康影响。 */
    return srv_pwr_ctrl_is_motor_enabled() && !st->motor_power_ok;
}

static void fault_policy_log_reasons(const srv_pwr_det_status_t* st)
{
    if (st->estop_on) {
        APP_FAULT_POLICY_LOG_W("  [原因] E-STOP 急停触发");
    }
    if (srv_pwr_ctrl_is_motor_enabled() && !st->motor_power_ok) {
        APP_FAULT_POLICY_LOG_E("  [原因] MOTOR_POWER PGD 丢失 (motor_pgd=0)");
    }
    /* 三路常开轨 (VIN_DC-DC/24V/AUX) PGD=0 的持续错误日志由 srv_pwr_det 统一输出，此处不重复 */
}
