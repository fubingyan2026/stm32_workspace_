/**
 * @file    srv_pwr_ctrl.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-03
 * @brief   电源控制服务实现（fsm 库顺序上电 FSM, E1_MASTER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_ctrl.h"

#include "drv_power.h"
#include "drv_status.h"
#include "fsm.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_PWR_CTRL_LOG_ENABLE 1

#if SRV_PWR_CTRL_LOG_ENABLE
#define SRV_PWR_CTRL_LOG_E(...) LOG_E("srv_pwr_ctrl", __VA_ARGS__)
#define SRV_PWR_CTRL_LOG_W(...) LOG_W("srv_pwr_ctrl", __VA_ARGS__)
#define SRV_PWR_CTRL_LOG_I(...) LOG_I("srv_pwr_ctrl", __VA_ARGS__)
#define SRV_PWR_CTRL_LOG_D(...) LOG_D("srv_pwr_ctrl", __VA_ARGS__)
#else
#define SRV_PWR_CTRL_LOG_E(...) ((void)0)
#define SRV_PWR_CTRL_LOG_W(...) ((void)0)
#define SRV_PWR_CTRL_LOG_I(...) ((void)0)
#define SRV_PWR_CTRL_LOG_D(...) ((void)0)
#endif

/* Private types -------------------------------------------------------------*/

/**
 * @brief 电源上电时序状态（fsm 状态，序号也作状态名表下标）
 */
typedef enum {
    PWR_STATE_IDLE = 0,   /**< 待机：全部轨关闭 */
    PWR_STATE_WAIT_VIN,   /**< 等待 VIN_DC-DC(LM5060) PGD */
    PWR_STATE_WAIT_24V,   /**< 等待 DC-DC 24V PGD */
    PWR_STATE_WAIT_AUX,   /**< 等待 AUX(LM5069) PGD */
    PWR_STATE_WAIT_MOTOR, /**< 等待 MOTOR(LM5069) PGD */
    PWR_STATE_POWERED,    /**< 上电完成 */

    PWR_STATE_COUNT,
} pwr_seq_state_t;

/** @brief 单实例上下文（config-in-context：句柄内嵌 fsm + 运行时状态） */
typedef struct {
    fsm_t fsm; /**< fsm 状态机实例 */
    uint16_t step_elapsed_ms; /**< 本次 step 周期 (ms)，handler 内使用 */
    uint32_t stage_elapsed_ms; /**< 当前阶段计时 (ms) */
    uint8_t pgd_ok_steps; /**< PGD 连续有效步数（去抖） */

    /** @brief 各轨当前使能标志（供故障策略门控 PGD 判定） */
    bool vin_on;
    bool dc24v_on;
    bool aux_on;
    bool motor_on;
} pwr_ctrl_t;

/* Private constants ---------------------------------------------------------*/

/** @brief 各阶段 PGOOD 等待超时 (ms) */
#define PWR_SEQ_VIN_PGD_TIMEOUT_MS (300U)
#define PWR_SEQ_24V_PGD_TIMEOUT_MS (800U)
#define PWR_SEQ_AUX_PGD_TIMEOUT_MS (500U)
#define PWR_SEQ_MOTOR_PGD_TIMEOUT_MS (800U)

/** @brief PGOOD 采样去抖：PGD 稳定判定所需连续有效步数（1ms/步） */
#define PWR_SEQ_PGD_DEBOUNCE_STEPS (10U)

/* Private variables ---------------------------------------------------------*/

static pwr_ctrl_t s_pc;

/** @brief fsm 静态配置表（handler 表 + 全连通转换矩阵） */
static fsm_handler_t s_pwr_handlers[PWR_STATE_COUNT];
static fsm_guard_t s_pwr_transitions[PWR_STATE_COUNT * PWR_STATE_COUNT];

/** @brief 状态名称表（调试日志用） */
static const char* s_pwr_state_names[PWR_STATE_COUNT] = {
    "IDLE", "WAIT_VIN", "WAIT_24V", "WAIT_AUX", "WAIT_MOTOR", "POWERED",
};

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t pwr_state_idle(fsm_t* ctx);
static fsm_state_t pwr_state_wait_vin(fsm_t* ctx);
static fsm_state_t pwr_state_wait_24v(fsm_t* ctx);
static fsm_state_t pwr_state_wait_aux(fsm_t* ctx);
static fsm_state_t pwr_state_wait_motor(fsm_t* ctx);
static fsm_state_t pwr_state_powered(fsm_t* ctx);
static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state);

static void pwr_seq_shutdown(pwr_ctrl_t* pc);
static bool pwr_pgd_ok(pwr_ctrl_t* pc, drv_status_signal_t sig);

/* Exported functions --------------------------------------------------------*/

void srv_pwr_ctrl_init(void)
{
    drv_power_init();

    memset(&s_pc, 0, sizeof(s_pc));

    /* 填充 fsm 共享配置表（handler 表 + 全连通转换矩阵） */
    s_pwr_handlers[PWR_STATE_IDLE] = pwr_state_idle;
    s_pwr_handlers[PWR_STATE_WAIT_VIN] = pwr_state_wait_vin;
    s_pwr_handlers[PWR_STATE_WAIT_24V] = pwr_state_wait_24v;
    s_pwr_handlers[PWR_STATE_WAIT_AUX] = pwr_state_wait_aux;
    s_pwr_handlers[PWR_STATE_WAIT_MOTOR] = pwr_state_wait_motor;
    s_pwr_handlers[PWR_STATE_POWERED] = pwr_state_powered;

    fsm_config_t fsm_cfg = {
        .handlers = s_pwr_handlers,
        .transitions = s_pwr_transitions,
        .state_count = PWR_STATE_COUNT,
        .entry_cb = pwr_entry_cb,
        .state_names = s_pwr_state_names,
        .user_data = &s_pc,
    };
    fsm_fill(&fsm_cfg, fsm_always_true);

    fsm_init(&s_pc.fsm, PWR_STATE_IDLE, &fsm_cfg);

    SRV_PWR_CTRL_LOG_I("电源控制服务初始化完成 (fsm 顺序上电)");
}

void srv_pwr_ctrl_step(uint16_t elapsed_ms)
{
    /* fsm 未初始化的兜底（fsm_init 失败场景下不致崩溃） */
    if (!s_pc.fsm.initialized) {
        return;
    }

    s_pc.step_elapsed_ms = elapsed_ms;
    fsm_step(&s_pc.fsm);
}

void srv_pwr_ctrl_request_on(void)
{
    if (!s_pc.fsm.initialized) {
        return;
    }

    if (fsm_current_state(&s_pc.fsm) == PWR_STATE_POWERED) {
        return;
    }

    /* 确保从干净状态重启时序（关轨 + 复位阶段计时），再由 fsm 切入等待态 */
    pwr_seq_shutdown(&s_pc);
    fsm_goto(&s_pc.fsm, PWR_STATE_WAIT_VIN);
    SRV_PWR_CTRL_LOG_I("收到上电请求，开始顺序上电");
}

void srv_pwr_ctrl_emergency_off(void)
{
    if (!s_pc.fsm.initialized) {
        return;
    }

    if (fsm_current_state(&s_pc.fsm) == PWR_STATE_IDLE) {
        return;
    }

    SRV_PWR_CTRL_LOG_E("紧急断电: 立即关闭全部电源轨");
    pwr_seq_shutdown(&s_pc);
    fsm_goto(&s_pc.fsm, PWR_STATE_IDLE);
}

bool srv_pwr_ctrl_is_powered_on(void)
{
    return s_pc.fsm.initialized
        && fsm_current_state(&s_pc.fsm) == PWR_STATE_POWERED;
}

bool srv_pwr_ctrl_is_vin_enabled(void)
{
    return s_pc.vin_on;
}

bool srv_pwr_ctrl_is_dc24v_enabled(void)
{
    return s_pc.dc24v_on;
}

bool srv_pwr_ctrl_is_aux_enabled(void)
{
    return s_pc.aux_on;
}

bool srv_pwr_ctrl_is_motor_enabled(void)
{
    return s_pc.motor_on;
}

/* Private functions ---------------------------------------------------------*/

/* ---- fsm 状态 handler：各等待态累计超时并判 PGD ---- */

static fsm_state_t pwr_state_idle(fsm_t* ctx)
{
    (void)ctx;
    return PWR_STATE_IDLE;
}

static fsm_state_t pwr_state_wait_vin(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);
    pc->stage_elapsed_ms += pc->step_elapsed_ms;

    if (pwr_pgd_ok(pc, DRV_STATUS_LM5060_PGD)) {
        pc->stage_elapsed_ms = 0;
        return PWR_STATE_WAIT_24V;
    }
    if (pc->stage_elapsed_ms >= PWR_SEQ_VIN_PGD_TIMEOUT_MS) {
        SRV_PWR_CTRL_LOG_E("上电失败: VIN_DC-DC(LM5060) PGD 超时");
        pwr_seq_shutdown(pc);
        return PWR_STATE_IDLE;
    }
    return PWR_STATE_WAIT_VIN;
}

static fsm_state_t pwr_state_wait_24v(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);
    pc->stage_elapsed_ms += pc->step_elapsed_ms;

    if (pwr_pgd_ok(pc, DRV_STATUS_DC24V_PGD)) {
        pc->stage_elapsed_ms = 0;
        return PWR_STATE_WAIT_AUX;
    }
    if (pc->stage_elapsed_ms >= PWR_SEQ_24V_PGD_TIMEOUT_MS) {
        SRV_PWR_CTRL_LOG_E("上电失败: DC-DC 24V PGD 超时");
        pwr_seq_shutdown(pc);
        return PWR_STATE_IDLE;
    }
    return PWR_STATE_WAIT_24V;
}

static fsm_state_t pwr_state_wait_aux(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);
    pc->stage_elapsed_ms += pc->step_elapsed_ms;

    if (pwr_pgd_ok(pc, DRV_STATUS_AUX_PGD)) {
        pc->stage_elapsed_ms = 0;
        return PWR_STATE_WAIT_MOTOR;
    }
    if (pc->stage_elapsed_ms >= PWR_SEQ_AUX_PGD_TIMEOUT_MS) {
        SRV_PWR_CTRL_LOG_E("上电失败: AUX PGD 超时");
        pwr_seq_shutdown(pc);
        return PWR_STATE_IDLE;
    }
    return PWR_STATE_WAIT_AUX;
}

static fsm_state_t pwr_state_wait_motor(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);
    pc->stage_elapsed_ms += pc->step_elapsed_ms;

    if (pwr_pgd_ok(pc, DRV_STATUS_MOTOR_PGD)) {
        pc->stage_elapsed_ms = 0;
        return PWR_STATE_POWERED;
    }
    if (pc->stage_elapsed_ms >= PWR_SEQ_MOTOR_PGD_TIMEOUT_MS) {
        SRV_PWR_CTRL_LOG_E("上电失败: MOTOR PGD 超时");
        pwr_seq_shutdown(pc);
        return PWR_STATE_IDLE;
    }
    return PWR_STATE_WAIT_MOTOR;
}

static fsm_state_t pwr_state_powered(fsm_t* ctx)
{
    (void)ctx;
    return PWR_STATE_POWERED;
}

/* ---- fsm 进入回调：进入等待态即使能对应电源轨，进入 POWERED 打日志 ---- */

static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);

    /* 进入新阶段复位去抖计数与阶段计时起点 */
    pc->pgd_ok_steps = 0;
    pc->stage_elapsed_ms = 0;

    switch (state) {
    case PWR_STATE_WAIT_VIN:
        pc->vin_on = true;
        drv_power_set(DRV_POWER_RAIL_VIN_DCDC, true);
        SRV_PWR_CTRL_LOG_I("上电: VIN_DC-DC_EN 已使能 (LM5060)");
        break;
    case PWR_STATE_WAIT_24V:
        pc->dc24v_on = true;
        drv_power_set(DRV_POWER_RAIL_DC24V, true);
        SRV_PWR_CTRL_LOG_I("上电: DC_DC_24V_EN 已使能");
        break;
    case PWR_STATE_WAIT_AUX:
        pc->aux_on = true;
        drv_power_set(DRV_POWER_RAIL_AUX, true);
        SRV_PWR_CTRL_LOG_I("上电: AUX_POWER_EN 已使能");
        break;
    case PWR_STATE_WAIT_MOTOR:
        pc->motor_on = true;
        drv_power_set(DRV_POWER_RAIL_MOTOR, true);
        SRV_PWR_CTRL_LOG_I("上电: MOTOR_POWER_EN 已使能");
        break;
    case PWR_STATE_POWERED:
        SRV_PWR_CTRL_LOG_I("上电完成: 全部电源轨就绪");
        break;
    default:
        break;
    }
}

/* ---- 内部工具 ---- */

/** @brief 立即关闭全部电源轨并复位上下文到待机态（不操作 fsm 状态） */
static void pwr_seq_shutdown(pwr_ctrl_t* pc)
{
    drv_power_set(DRV_POWER_RAIL_MOTOR, false);
    drv_power_set(DRV_POWER_RAIL_AUX, false);
    drv_power_set(DRV_POWER_RAIL_DC24V, false);
    drv_power_set(DRV_POWER_RAIL_VIN_DCDC, false);

    pc->vin_on = false;
    pc->dc24v_on = false;
    pc->aux_on = false;
    pc->motor_on = false;
    pc->stage_elapsed_ms = 0;
    pc->pgd_ok_steps = 0;
}

/** @brief PGD 采样去抖判定：连续 PWR_SEQ_PGD_DEBOUNCE_STEPS 步有效即认为就绪 */
static bool pwr_pgd_ok(pwr_ctrl_t* pc, drv_status_signal_t sig)
{
    if (drv_status_read(sig)) {
        if (pc->pgd_ok_steps < PWR_SEQ_PGD_DEBOUNCE_STEPS) {
            pc->pgd_ok_steps++;
        }
        return pc->pgd_ok_steps >= PWR_SEQ_PGD_DEBOUNCE_STEPS;
    }

    pc->pgd_ok_steps = 0;
    return false;
}
