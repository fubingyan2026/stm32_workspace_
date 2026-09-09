/**
 * @file    app_status_indicator.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   应用层 — 状态指示灯策略实现（单颗状态灯, E1_SLAVER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "app_status_indicator.h"

#include "log.h"
#include "srv_pwr_ctrl.h"
#include "srv_signal.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define APP_STATUS_INDICATOR_LOG_ENABLE 1

#if APP_STATUS_INDICATOR_LOG_ENABLE
#define APP_STATUS_INDICATOR_LOG_E(...) LOG_E("app_status_indicator", __VA_ARGS__)
#define APP_STATUS_INDICATOR_LOG_W(...) LOG_W("app_status_indicator", __VA_ARGS__)
#define APP_STATUS_INDICATOR_LOG_I(...) LOG_I("app_status_indicator", __VA_ARGS__)
#define APP_STATUS_INDICATOR_LOG_D(...) LOG_D("app_status_indicator", __VA_ARGS__)
#else
#define APP_STATUS_INDICATOR_LOG_E(...) ((void)0)
#define APP_STATUS_INDICATOR_LOG_W(...) ((void)0)
#define APP_STATUS_INDICATOR_LOG_I(...) ((void)0)
#define APP_STATUS_INDICATOR_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 状态评估节流周期 (ms) */
#define APP_IND_EVAL_PERIOD_MS (100U)

/* 状态灯灯效参数 */
#define APP_IND_FAULT_BLINK_CYCLE_MS (100U)/**< 关键电源轨故障：慢闪 */ 
#define APP_IND_BUZY_BLINK_CYCLE_MS (500U) /**< 急停：快闪 */

/* Private types -------------------------------------------------------------*/

/**
 * @brief 指示等级（数值仅用于排序比较，P0 最高优先）
 */
typedef enum {
    APP_IND_LEVEL_IDLE = 0, /**< 空闲：无期望输出 */
    APP_IND_LEVEL_NORMAL, /**< 输出正常 */
    APP_IND_LEVEL_BUSY, /**< 使能进行中（期望与实态不一致） */
    APP_IND_LEVEL_FAULT, /**< 故障锁存 */
    APP_IND_LEVEL_COUNT,
} app_ind_level_t;

/* Private variables ---------------------------------------------------------*/

static srv_signal_handle_t* s_status_led; /**< 状态灯实例（led_task 注入） */
static app_ind_level_t s_cur_level; /**< 当前等级（哨兵值强制首轮下发） */
static uint16_t s_eval_ms; /**< 节流累计 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static app_ind_level_t ind_evaluate(void);
static void ind_apply(app_ind_level_t level);
static void ind_set_state(srv_signal_handle_t* h, srv_signal_state_t state);
static void ind_set_blink(srv_signal_handle_t* h, uint16_t cycle_ms, uint16_t wait_ms);

/* Exported functions --------------------------------------------------------*/

void app_status_indicator_init(srv_signal_handle_t* status_led)
{
    if (!status_led) {
        APP_STATUS_INDICATOR_LOG_E("状态灯实例为空, 指示灯策略失效");
        return;
    }

    s_status_led = status_led;
    s_cur_level = APP_IND_LEVEL_COUNT; /* 哨兵：首轮必下发 */
    s_eval_ms = 0;
    s_initialized = true;

    APP_STATUS_INDICATOR_LOG_I("状态指示灯策略初始化完成 (eval=%ums)",
        (unsigned)APP_IND_EVAL_PERIOD_MS);
}

void app_status_indicator_step(uint16_t elapsed_ms)
{
    if (!s_initialized) {
        return;
    }

    /* 节流：周期评估，避免高频读服务 */
    s_eval_ms += elapsed_ms;
    if (s_eval_ms < APP_IND_EVAL_PERIOD_MS) {
        return;
    }
    s_eval_ms = 0;

    const app_ind_level_t level = ind_evaluate();

    /* 仅等级变化时下发命令，避免刷爆 srv_signal 异步队列 */
    if (level != s_cur_level) {
        s_cur_level = level;
        ind_apply(level);
        APP_STATUS_INDICATOR_LOG_I("状态等级切换: %u", (unsigned)level);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 按优先级评估当前状态等级（先判 P0 故障）
 */
static app_ind_level_t ind_evaluate(void)
{
    /* P0 故障锁存（最高优先级） */
    if (srv_pwr_ctrl_is_any_latched()) {
        return APP_IND_LEVEL_FAULT;
    }

    const uint8_t desired = srv_pwr_ctrl_get_desired();

    /* 期望全部就绪 → 正常；否则视为使能进行中 */
    const uint8_t on = srv_pwr_ctrl_get_on_outputs();
    if (on == desired) {
        return APP_IND_LEVEL_NORMAL;
    }

    return APP_IND_LEVEL_BUSY;
}

/**
 * @brief 将状态等级映射为单颗状态灯灯效命令
 */
static void ind_apply(app_ind_level_t level)
{
    switch (level) {
    case APP_IND_LEVEL_FAULT:
        ind_set_blink(s_status_led, APP_IND_FAULT_BLINK_CYCLE_MS, 0);
        ind_set_state(s_status_led, SRV_SIGNAL_STATE_BLINK_CODE);
        break;

    case APP_IND_LEVEL_BUSY:
        ind_set_blink(s_status_led, APP_IND_BUZY_BLINK_CYCLE_MS, 0);
        ind_set_state(s_status_led, SRV_SIGNAL_STATE_BLINK_CODE);
        break;

    case APP_IND_LEVEL_NORMAL:
        ind_set_state(s_status_led, SRV_SIGNAL_STATE_BREATHING);
        break;

    case APP_IND_LEVEL_IDLE:
    default:
        ind_set_state(s_status_led, SRV_SIGNAL_STATE_ON);
        break;
    }
}

static void ind_set_state(srv_signal_handle_t* h, srv_signal_state_t state)
{
    if (h) {
        srv_signal_set_state(h, state);
    }
}

static void ind_set_blink(srv_signal_handle_t* h, uint16_t cycle_ms, uint16_t wait_ms)
{
    if (!h) {
        return;
    }

    const srv_signal_cmd_t cmd = {
        .set_state = SRV_SIGNAL_STATE_BLINK_CODE,
        .blink_cycle_ms = cycle_ms,
        .blink_wait_ms = wait_ms,
        .blink_code_counts = 0, /* 无限循环，直到等级切换 */
    };
    srv_signal_set_blink_interval(h, &cmd);
}
