/**
 * @file    srv_pwr_ctrl.c
 * @author  maximillian
 * @version V3.0.0
 * @date    2026-09-04
 * @brief   电源控制服务实现（fsm 两态机：三路默认常开 + MOTOR 独立受控, E1_MASTER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_ctrl.h"

#include "drv_power.h"
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
 * @brief MOTOR 使能状态（fsm 两态，序号也作状态名表下标）
 */
typedef enum {
    PWR_STATE_IDLE = 0, /**< MOTOR 关闭（三路常开轨不受影响） */
    PWR_STATE_POWERED,  /**< MOTOR 使能（三路常开轨不受影响） */

    PWR_STATE_COUNT,
} pwr_seq_state_t;

/** @brief 单实例上下文（config-in-context：句柄内嵌 fsm + 运行时状态） */
typedef struct {
    fsm_t fsm; /**< fsm 状态机实例 */

    /** @brief 各轨当前使能标志（三路常开轨初始化后恒 true；MOTOR 视 FSM） */
    bool vin_on;
    bool dc24v_on;
    bool aux_on;
    bool motor_on;
} pwr_ctrl_t;

/* Private variables ---------------------------------------------------------*/

static pwr_ctrl_t s_pc;

/** @brief fsm 静态配置表（handler 表 + 全连通转换矩阵） */
static fsm_handler_t s_pwr_handlers[PWR_STATE_COUNT];
static fsm_guard_t s_pwr_transitions[PWR_STATE_COUNT * PWR_STATE_COUNT];

/** @brief 状态名称表（调试日志用） */
static const char* s_pwr_state_names[PWR_STATE_COUNT] = {
    "IDLE", "POWERED",
};

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t pwr_state_idle(fsm_t* ctx);
static fsm_state_t pwr_state_powered(fsm_t* ctx);
static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state);

/* Exported functions --------------------------------------------------------*/

void srv_pwr_ctrl_init(void)
{
    drv_power_init();

    memset(&s_pc, 0, sizeof(s_pc));

    /* 填充 fsm 共享配置表（handler 表 + 全连通转换矩阵） */
    s_pwr_handlers[PWR_STATE_IDLE] = pwr_state_idle;
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

    /* 三路默认常开：立即使能，不随 FSM/急停开关，PGD 仅作监测 */
    drv_power_set(DRV_POWER_RAIL_VIN_DCDC, true);
    drv_power_set(DRV_POWER_RAIL_DC24V, true);
    drv_power_set(DRV_POWER_RAIL_AUX, true);
    s_pc.vin_on = true;
    s_pc.dc24v_on = true;
    s_pc.aux_on = true;
    s_pc.motor_on = false; /* MOTOR 独立受控，默认关闭 */
    SRV_PWR_CTRL_LOG_I("电源控制服务初始化完成 (VIN/24V/AUX 常开, MOTOR 独立受控)");
}

void srv_pwr_ctrl_step(uint16_t elapsed_ms)
{
    /* fsm 未初始化的兜底（fsm_init 失败场景下不致崩溃） */
    if (!s_pc.fsm.initialized) {
        return;
    }

    (void)elapsed_ms;
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

    fsm_goto(&s_pc.fsm, PWR_STATE_POWERED); /* 下一拍 entry 拉高 MOTOR_EN */
    SRV_PWR_CTRL_LOG_I("收到 MOTOR 使能请求");
}

void srv_pwr_ctrl_emergency_off(void)
{
    if (!s_pc.fsm.initialized) {
        return;
    }

    /* 无条件直接强制关闭 MOTOR（不受 FSM 状态 / 三路健康影响），并同步标志 */
    drv_power_set(DRV_POWER_RAIL_MOTOR, false);
    s_pc.motor_on = false;

    /* 回到 IDLE 保持 FSM 一致（entry IDLE 会再次确保 MOTOR_EN=0，幂等） */
    if (fsm_current_state(&s_pc.fsm) != PWR_STATE_IDLE) {
        fsm_goto(&s_pc.fsm, PWR_STATE_IDLE);
    }

    SRV_PWR_CTRL_LOG_E("紧急断电: MOTOR_EN 已强制关闭 (VIN/24V/AUX 保持常开)");
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

/* ---- fsm 状态 handler ---- */

static fsm_state_t pwr_state_idle(fsm_t* ctx)
{
    (void)ctx;
    return PWR_STATE_IDLE;
}

static fsm_state_t pwr_state_powered(fsm_t* ctx)
{
    (void)ctx;
    return PWR_STATE_POWERED;
}

/* ---- fsm 进入回调：按状态驱动 MOTOR_EN ---- */

static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state)
{
    pwr_ctrl_t* pc = (pwr_ctrl_t*)fsm_user_data(ctx);

    switch (state) {
    case PWR_STATE_POWERED:
        pc->motor_on = true;
        drv_power_set(DRV_POWER_RAIL_MOTOR, true);
        SRV_PWR_CTRL_LOG_I("MOTOR_POWER_EN 已使能");
        break;
    case PWR_STATE_IDLE:
    default:
        pc->motor_on = false;
        drv_power_set(DRV_POWER_RAIL_MOTOR, false);
        break;
    }
}
