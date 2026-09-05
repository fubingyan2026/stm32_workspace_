/**
 * @file    srv_pwr_ctrl.c
 * @author  maximillian
 * @version V5.0.0
 * @date    2026-09-05
 * @brief   电源控制服务实现（默认 5 步上电流程 FSM, E1_MASTER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_ctrl.h"

#include "drv_power.h"
#include "drv_status.h"
#include "drv_systick.h"
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
 * @brief 上电流程状态（fsm 状态，序号也作状态名表下标）
 */
typedef enum {
    PWR_STATE_IDLE = 0, /**< 待机：全轨关闭（仅初始化短暂存在/异常预留） */
    PWR_STATE_CHECK_VIN, /**< 步骤1：VIN ∈ [36,58]V */
    PWR_STATE_CHECK_VIN_DCDC, /**< 步骤2：VIN_DC-DC ≥ 90%·VIN */
    PWR_STATE_EN_VIN, /**< 步骤3：使能 VIN_DC-DC_EN + 100ms 延时 */
    PWR_STATE_EN_24V, /**< 步骤4/5：使能 24V_EN，等待双 PGD 高 */
    PWR_STATE_POWERED, /**< 上电成功（使能 AUX；MOTOR 由上层策略控制） */

    PWR_STATE_COUNT,
} pwr_seq_state_t;

/** @brief 单实例上下文（config-in-context：句柄内嵌 fsm + 运行时状态） */
typedef struct {
    fsm_t fsm; /**< fsm 状态机实例 */
    srv_pwr_ctrl_volt_cb_t volt_cb; /**< 电压读取回调 */

    uint16_t step_elapsed_ms; /**< 本次 step 周期 (ms) */
    uint32_t stage_ms; /**< 当前阶段计时 (ms) */
    uint8_t pgd_ok_steps; /**< 双 PGD 连续有效步数（去抖） */
    uint32_t wait_log_ts; /**< 等待条件日志限频 (ms) */

    bool motor_desired; /**< MOTOR 使能请求（延后设定：上电成功(POWERED)后自动生效） */

    /** @brief 各轨当前使能标志 */
    bool vin_on;
    bool dc24v_on;
    bool aux_on;
    bool motor_on;
} pwr_ctrl_t;

/* Private constants ---------------------------------------------------------*/

/** @brief 步骤1：VIN 输入允许范围 (mV) */
#define PWR_VIN_OK_MIN_MV (36000U)
#define PWR_VIN_OK_MAX_MV (58000U)

/** @brief 步骤2：VIN_DC-DC 需达到 VIN 的百分比（‰，900=90.0%） */
#define PWR_VIN_DCDC_MIN_PERMILLE (900U)

/** @brief 步骤3：VIN_DC-DC_EN 使能后稳定延时 (ms) */
#define PWR_VIN_EN_SETTLE_MS (100U)

/** @brief 步骤5：双 PGD 连续有效去抖步数（1ms/步） */
#define PWR_DUAL_PGD_DEBOUNCE_STEPS (10U)

/** @brief 等待条件日志限频 (ms) */
#define PWR_WAIT_LOG_PERIOD_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static pwr_ctrl_t s_pc;

static fsm_handler_t s_pwr_handlers[PWR_STATE_COUNT];
static fsm_guard_t s_pwr_transitions[PWR_STATE_COUNT * PWR_STATE_COUNT];

static const char* s_pwr_state_names[PWR_STATE_COUNT] = {
    "IDLE",
    "CHECK_VIN",
    "CHECK_VIN_DCDC",
    "EN_VIN",
    "EN_24V",
    "POWERED",
};

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t pwr_state_idle(fsm_t* ctx);
static fsm_state_t pwr_state_check_vin(fsm_t* ctx);
static fsm_state_t pwr_state_check_vin_dcdc(fsm_t* ctx);
static fsm_state_t pwr_state_en_vin(fsm_t* ctx);
static fsm_state_t pwr_state_en_24v(fsm_t* ctx);
static fsm_state_t pwr_state_powered(fsm_t* ctx);
static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state);
static bool pwr_dual_pgd_ok(pwr_ctrl_t* pc);
static void pwr_read_voltage(pwr_ctrl_t* pc, uint32_t* vin_mv, uint32_t* vin_dcdc_mv);
static bool pwr_wait_log(pwr_ctrl_t* pc);

/* Exported functions --------------------------------------------------------*/

void srv_pwr_ctrl_init(const srv_pwr_ctrl_config_t* config)
{
    drv_power_init();

    memset(&s_pc, 0, sizeof(s_pc));
    if (config) {
        s_pc.volt_cb = config->read_voltage;
    }

    s_pwr_handlers[PWR_STATE_IDLE] = pwr_state_idle;
    s_pwr_handlers[PWR_STATE_CHECK_VIN] = pwr_state_check_vin;
    s_pwr_handlers[PWR_STATE_CHECK_VIN_DCDC] = pwr_state_check_vin_dcdc;
    s_pwr_handlers[PWR_STATE_EN_VIN] = pwr_state_en_vin;
    s_pwr_handlers[PWR_STATE_EN_24V] = pwr_state_en_24v;
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

    /* AUX 直通电源：初始化即使能，不等待其他轨上电成功 */
    drv_power_set(DRV_POWER_RAIL_AUX, true);
    s_pc.aux_on = true;
    SRV_PWR_CTRL_LOG_I("AUX_POWER_EN 已直接使能 (不参与上电流程等待)");

    /* 默认上电流程：程序启动即自动进入步骤1 */
    fsm_goto(&s_pc.fsm, PWR_STATE_CHECK_VIN);
    SRV_PWR_CTRL_LOG_I("电源控制服务初始化完成，默认 5 步上电流程已启动");
}

void srv_pwr_ctrl_step(uint16_t elapsed_ms)
{
    if (!s_pc.fsm.initialized) {
        return;
    }

    s_pc.step_elapsed_ms = elapsed_ms;
    fsm_step(&s_pc.fsm);
}

void srv_pwr_ctrl_motor_set(bool on)
{
    s_pc.motor_desired = on;

    if (on) {
        /* 延后设定：上电流程尚未成功 → 仅记录请求，POWERED 后自动生效 */
        if (srv_pwr_ctrl_get_state().powered_on) {
            s_pc.motor_on = true;
            drv_power_set(DRV_POWER_RAIL_MOTOR, true);
            SRV_PWR_CTRL_LOG_I("MOTOR_POWER_EN 已使能");
        } else {
            SRV_PWR_CTRL_LOG_W("上电流程未完成，MOTOR 使能请求已延后 (POWERED 后自动开启)");
        }
        return;
    }

    /* 关断任何时候都立即生效 */
    s_pc.motor_on = false;
    drv_power_set(DRV_POWER_RAIL_MOTOR, false);
    SRV_PWR_CTRL_LOG_E("MOTOR_POWER_EN 已关闭 (急停/故障)");
}

srv_pwr_ctrl_state_t srv_pwr_ctrl_get_state(void)
{
    srv_pwr_ctrl_state_t state;

    state.powered_on = s_pc.fsm.initialized
        && fsm_current_state(&s_pc.fsm) == PWR_STATE_POWERED;
    state.vin_en = s_pc.vin_on;
    state.dc24v_en = s_pc.dc24v_on;
    state.aux_en = s_pc.aux_on;
    state.motor_en = s_pc.motor_on;

    return state;
}

/* Private functions ---------------------------------------------------------*/

/* ---- fsm 状态 handler ---- */

static fsm_state_t pwr_state_idle(fsm_t* ctx)
{
    (void)ctx;
    return PWR_STATE_IDLE;
}

/** 步骤1：VIN 输入范围判定 */
static fsm_state_t pwr_state_check_vin(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);

    uint32_t vin_mv = 0;
    uint32_t vin_dcdc_mv = 0;
    pwr_read_voltage(pc, &vin_mv, &vin_dcdc_mv);

    if (vin_mv >= PWR_VIN_OK_MIN_MV && vin_mv <= PWR_VIN_OK_MAX_MV) {
        pc->stage_ms = 0;
        return PWR_STATE_CHECK_VIN_DCDC;
    }

    if (pwr_wait_log(pc)) {
        SRV_PWR_CTRL_LOG_W("等待输入电压合格: VIN=%umV (需 %u~%u mV)",
            (unsigned)vin_mv, (unsigned)PWR_VIN_OK_MIN_MV, (unsigned)PWR_VIN_OK_MAX_MV);
    }
    return PWR_STATE_CHECK_VIN;
}

/** 步骤2：VIN_DC-DC ≥ 90% VIN */
static fsm_state_t pwr_state_check_vin_dcdc(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);

    uint32_t vin_mv = 0;
    uint32_t vin_dcdc_mv = 0;
    pwr_read_voltage(pc, &vin_mv, &vin_dcdc_mv);

    /* 比较 VIN_DC-DC(千分比) >= 90%·VIN */
    if (vin_mv > 0U
        && vin_dcdc_mv * 1000U / vin_mv >= PWR_VIN_DCDC_MIN_PERMILLE) {
        pc->stage_ms = 0;
        return PWR_STATE_EN_VIN;
    }

    if (pwr_wait_log(pc)) {
        SRV_PWR_CTRL_LOG_W("等待 VIN_DC-DC 接近 VIN: vin=%umV vin_dcdc=%umV (需≥90%%VIN)",
            (unsigned)vin_mv, (unsigned)vin_dcdc_mv);
    }
    return PWR_STATE_CHECK_VIN_DCDC;
}

/** 步骤3：VIN_DC-DC_EN 已使能，等待 100ms 稳定 */
static fsm_state_t pwr_state_en_vin(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);

    pc->stage_ms += pc->step_elapsed_ms;
    if (pc->stage_ms >= PWR_VIN_EN_SETTLE_MS) {
        pc->stage_ms = 0;
        return PWR_STATE_EN_24V;
    }
    return PWR_STATE_EN_VIN;
}

/** 步骤4/5：DC_DC_24V_EN 已使能，双 PGD 均高即上电成功 */
static fsm_state_t pwr_state_en_24v(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);

    if (pwr_dual_pgd_ok(pc)) {
        pc->stage_ms = 0;
        return PWR_STATE_POWERED;
    }

    if (pwr_wait_log(pc)) {
        const bool p24 = drv_status_read(DRV_STATUS_DC24V_PGD);
        const bool plm = drv_status_read(DRV_STATUS_LM5060_PGD);
        SRV_PWR_CTRL_LOG_W("等待双 PGD: DC24V_PGD=%u LM5060_PGD=%u",
            (unsigned)p24, (unsigned)plm);
    }
    return PWR_STATE_EN_24V;
}

static fsm_state_t pwr_state_powered(fsm_t* ctx)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);

    /* 延后使能：上电成功后若有 MOTOR 使能请求则自动开启 */
    if (pc->motor_desired && !pc->motor_on) {
        pc->motor_on = true;
        drv_power_set(DRV_POWER_RAIL_MOTOR, true);
        SRV_PWR_CTRL_LOG_I("MOTOR_POWER_EN 已使能 (上电成功后延后生效)");
    }

    return PWR_STATE_POWERED;
}

/* ---- fsm 进入回调 ---- */

static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);
    pc->stage_ms = 0;
    pc->pgd_ok_steps = 0;
    pc->wait_log_ts = 0;

    switch (state) {
    case PWR_STATE_CHECK_VIN:
        SRV_PWR_CTRL_LOG_I("步骤1: 检测 VIN 输入电压范围");
        break;
    case PWR_STATE_CHECK_VIN_DCDC:
        SRV_PWR_CTRL_LOG_I("步骤2: 检测 VIN_DC-DC ≥ 90%% VIN");
        break;
    case PWR_STATE_EN_VIN:
        pc->vin_on = true;
        drv_power_set(DRV_POWER_RAIL_VIN_DCDC, true);
        SRV_PWR_CTRL_LOG_I("步骤3: VIN_DC-DC_EN 已使能 (100ms 稳定延时)");
        break;
    case PWR_STATE_EN_24V:
        pc->dc24v_on = true;
        drv_power_set(DRV_POWER_RAIL_DC24V, true);
        SRV_PWR_CTRL_LOG_I("步骤4: DC_DC_24V_EN 已使能");
        break;
    case PWR_STATE_POWERED:
        SRV_PWR_CTRL_LOG_I("步骤5完成: 双 PGD 就绪，上电成功 (AUX 于初始化时已使能)");
        break;
    case PWR_STATE_IDLE:
    default:
        /* 全轨电源复位（见 pwr_rails_power_off），MOTOR 由上层策略控制 */
        break;
    }
}

/* ---- 内部工具 ---- */

/** @brief 双 PGD（24V 与 LM5060）连续有效去抖 */
static bool pwr_dual_pgd_ok(pwr_ctrl_t* pc)
{
    const bool p24 = drv_status_read(DRV_STATUS_DC24V_PGD);
    const bool plm = drv_status_read(DRV_STATUS_LM5060_PGD);

    if (p24 && plm) {
        if (pc->pgd_ok_steps < PWR_DUAL_PGD_DEBOUNCE_STEPS) {
            pc->pgd_ok_steps++;
        }
        return pc->pgd_ok_steps >= PWR_DUAL_PGD_DEBOUNCE_STEPS;
    }

    pc->pgd_ok_steps = 0;
    return false;
}

static void pwr_read_voltage(pwr_ctrl_t* pc, uint32_t* vin_mv, uint32_t* vin_dcdc_mv)
{
    if (pc->volt_cb) {
        pc->volt_cb(vin_mv, vin_dcdc_mv);
    }
}

static bool pwr_wait_log(pwr_ctrl_t* pc)
{
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - pc->wait_log_ts) >= PWR_WAIT_LOG_PERIOD_MS) {
        pc->wait_log_ts = now_ms;
        return true;
    }
    return false;
}
