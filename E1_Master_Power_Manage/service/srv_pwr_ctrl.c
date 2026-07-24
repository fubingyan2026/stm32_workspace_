/**
 * @file    srv_pwr_ctrl.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-07-2
 * @brief   电源控制服务实现
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_ctrl.h"

#include <string.h>

#include "drv_power.h"
#include "fsm.h"

/* Private constants ---------------------------------------------------------*/

#define STEADY_TIME_MS (50U)

/* FSM 状态 -----------------------------------------------------------------*/

enum {
    PWR_STATE_IDLE = 0,
    PWR_STATE_AUX,
    PWR_STATE_PRECHARGE,
    PWR_STATE_MOTOR,
    PWR_STATE_DONE,
    PWR_STATE_COUNT,
};

/* Private types -------------------------------------------------------------*/

typedef struct {
    bool power_on_requested;
    uint16_t steady_ms;
} power_ctrl_ctx_t;

/* Private variables ---------------------------------------------------------*/

static fsm_t s_fsm;
static power_ctrl_ctx_t s_ctx;

static fsm_handler_t s_handlers[PWR_STATE_COUNT];
static fsm_guard_t s_transitions[PWR_STATE_COUNT * PWR_STATE_COUNT];
static const char* s_state_names[PWR_STATE_COUNT] = {
    "IDLE",
    "AUX",
    "PRECHARGE",
    "MOTOR",
    "DONE",
};

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t pwr_state_idle(fsm_t* ctx);
static fsm_state_t pwr_state_aux(fsm_t* ctx);
static fsm_state_t pwr_state_precharge(fsm_t* ctx);
static fsm_state_t pwr_state_motor(fsm_t* ctx);
static fsm_state_t pwr_state_done(fsm_t* ctx);
static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state);

/* Exported functions --------------------------------------------------------*/

void srv_pwr_ctrl_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    drv_power_init();

    s_handlers[PWR_STATE_IDLE] = pwr_state_idle;
    s_handlers[PWR_STATE_AUX] = pwr_state_aux;
    s_handlers[PWR_STATE_PRECHARGE] = pwr_state_precharge;
    s_handlers[PWR_STATE_MOTOR] = pwr_state_motor;
    s_handlers[PWR_STATE_DONE] = pwr_state_done;

    fsm_config_t config = {
        .handlers = s_handlers,
        .transitions = s_transitions,
        .state_count = PWR_STATE_COUNT,
        .entry_cb = pwr_entry_cb,
        .exit_cb = NULL,
        .state_names = s_state_names,
        .user_data = &s_ctx,
    };

    fsm_fill(&config, fsm_always_true);
    fsm_init(&s_fsm, PWR_STATE_IDLE, &config);
}

void srv_pwr_ctrl_step(uint16_t elapsed_ms)
{
    s_ctx.steady_ms += elapsed_ms;
    fsm_step(&s_fsm);
}

void srv_pwr_ctrl_request_on(void)
{
    s_ctx.power_on_requested = true;
}

void srv_pwr_ctrl_emergency_off(void)
{
    s_ctx.power_on_requested = false;
    s_ctx.steady_ms = 0;

    fsm_goto(&s_fsm, PWR_STATE_IDLE);

    for (uint32_t i = 0; i <= DRV_POWER_RAIL_DBR_LSD_EN; i++) {
        drv_power_set((drv_power_rail_t)i, false);
    }
}

bool srv_pwr_ctrl_is_powered_on(void)
{
    return fsm_current_state(&s_fsm) == PWR_STATE_DONE;
}

/* Private functions ---------------------------------------------------------*/

static fsm_state_t pwr_state_idle(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);
    return p->power_on_requested ? PWR_STATE_AUX : PWR_STATE_IDLE;
}

static fsm_state_t pwr_state_aux(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);
    return (p->steady_ms >= STEADY_TIME_MS) ? PWR_STATE_PRECHARGE : PWR_STATE_AUX;
}

static fsm_state_t pwr_state_precharge(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);
    return (p->steady_ms >= STEADY_TIME_MS) ? PWR_STATE_MOTOR : PWR_STATE_PRECHARGE;
}

static fsm_state_t pwr_state_motor(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);
    return (p->steady_ms >= STEADY_TIME_MS) ? PWR_STATE_DONE : PWR_STATE_MOTOR;
}

static fsm_state_t pwr_state_done(fsm_t* ctx)
{
    (void)ctx;
    return PWR_STATE_DONE;
}

/* --- FSM Entry Callbacks --- */

static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);
    p->steady_ms = 0;

    switch (state) {
    case PWR_STATE_AUX:
        drv_power_set(DRV_POWER_RAIL_DC_DC_EN, true);
        drv_power_set(DRV_POWER_RAIL_AUX_EN, true);
        break;
    case PWR_STATE_PRECHARGE:
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, true);
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_IN, true);
        drv_power_set(DRV_POWER_RAIL_HSD1_12V_DIAG, true);
        drv_power_set(DRV_POWER_RAIL_HSD1_24V_DIAG, true);
        drv_power_set(DRV_POWER_RAIL_HSD2_24V_DIAG, true);
        break;
    case PWR_STATE_MOTOR:
        drv_power_set(DRV_POWER_RAIL_MOTOR_EN, true);
        drv_power_set(DRV_POWER_RAIL_HSD1_12V, true);
        drv_power_set(DRV_POWER_RAIL_HSD1_24V, true);
        drv_power_set(DRV_POWER_RAIL_HSD2_24V, true);
        break;
    default:
        break;
    }
}
