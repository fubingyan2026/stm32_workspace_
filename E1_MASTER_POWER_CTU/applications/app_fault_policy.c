/**
 * @file    app_fault_policy.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-05
 * @brief   应用层 — 故障保护策略实现（急停只控制 MOTOR）
 *
 * 电源语义（见 srv_pwr_ctrl）：
 * - VIN_DC-DC/24V/AUX 为默认上电流程供电轨，急停/故障不关断（由 srv_pwr_det 监测上报）
 * - MOTOR_EN 为受控负载轨：急停按下/有效急停或有 MOTOR PGD 故障 → 立即关断 MOTOR；
 *   急停释放且上电成功后自动重新使能 MOTOR
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

static bool s_tripped; /**< 保护锁存标志（MOTOR 关断保持到复位/急停释放） */

/** @brief 急停释放沿检测状态（复用共享 utils_edge_detect） */
static utils_edge_det_t s_estop_edge;

/* Private function prototypes -----------------------------------------------*/

/** @brief 判定 MOTOR 关断条件（可调策略） */
static bool fault_policy_critical(const srv_pwr_det_status_t* st,
    const srv_pwr_ctrl_state_t* ps);

/** @brief 打印触发原因明细（仅触发时调用） */
static void fault_policy_log_reasons(const srv_pwr_det_status_t* st,
    const srv_pwr_ctrl_state_t* ps);

/* Exported functions --------------------------------------------------------*/

void app_fault_policy_init(void)
{
    s_tripped = false;
    utils_edge_det_init(&s_estop_edge);
    APP_FAULT_POLICY_LOG_I("故障保护策略初始化完成 (急停只控制 MOTOR)");
}

void app_fault_policy_step(uint16_t elapsed_ms)
{
    (void)elapsed_ms;

    srv_pwr_det_status_t st;
    srv_pwr_det_read(&st);

    /* 电源状态快照（一次取全部） */
    const srv_pwr_ctrl_state_t ps = srv_pwr_ctrl_get_state();

    /* 已锁存且急停仍按下：保持 MOTOR 关断 */
    if (s_tripped && st.estop_on) {
        if (ps.motor_en) {
            srv_pwr_ctrl_motor_set(false);
        }
        return;
    }

    /*
     * 触发条件（仅影响 MOTOR，VIN/24V/AUX 常供轨不受影响）：
     * - E-STOP 按下：无条件关断 MOTOR
     * - 上电成功且 MOTOR 已使能但其 PGD 丢失：关断 MOTOR 并锁存
     */
    if (st.estop_on || (ps.powered_on && fault_policy_critical(&st, &ps))) {
        s_tripped = true;

        /* 1. 关断 MOTOR（急停只控制 MOTOR） */
        srv_pwr_ctrl_motor_set(false);

        /* 2. 风扇满速散热（关闭温控自动，强制最高转速） */
        srv_fan_ctrl_set_auto(false);
        srv_fan_ctrl_set_duty(0, 100U);
        srv_fan_ctrl_set_duty(1, 100U);

        /* 3. 打印触发原因明细 */
        fault_policy_log_reasons(&st, &ps);
    }

    /* 急停释放沿：解除锁存 */
    if (utils_edge_detect(&s_estop_edge, st.estop_on) == UTILS_EDGE_FALLING) {
        app_fault_policy_reset();
    }

    /* MOTOR 使能门控：未锁存、上电成功且急停未按下 → 使能；否则确保关断 */
    const bool motor_want = !s_tripped && ps.powered_on && !st.estop_on;
    if (motor_want && !ps.motor_en) {
        srv_pwr_ctrl_motor_set(true);
    } else if (!motor_want && ps.motor_en) {
        srv_pwr_ctrl_motor_set(false);
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

static bool fault_policy_critical(const srv_pwr_det_status_t* st,
    const srv_pwr_ctrl_state_t* ps)
{
    /* MOTOR 使能后其 PGD 丢失 → 关断 MOTOR。其余电源轨异常由 srv_pwr_det 监测上报。 */
    return ps->motor_en && !st->motor_power_ok;
}

static void fault_policy_log_reasons(const srv_pwr_det_status_t* st,
    const srv_pwr_ctrl_state_t* ps)
{
    if (st->estop_on) {
        APP_FAULT_POLICY_LOG_W("  [原因] E-STOP 急停触发 (MOTOR 已关断, VIN/24V/AUX 保持)");
    }
    if (ps->motor_en && !st->motor_power_ok) {
        APP_FAULT_POLICY_LOG_E("  [原因] MOTOR_POWER PGD 丢失 (motor_pgd=0)");
    }
}
