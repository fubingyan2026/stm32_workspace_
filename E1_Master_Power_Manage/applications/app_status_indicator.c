/**
 * @file    app_status_indicator.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   应用层 — 状态指示灯策略实现
 */

/* Includes ------------------------------------------------------------------*/
#include "app_status_indicator.h"

#include "app_fault_policy.h"
#include "app_status_report.h"
#include "log.h"
#include "srv_can_mst.h"
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

/** @brief 状态评估节流周期 (ms)：LED 平滑由 srv_signal_task_refresh 的 10ms 保证 */
#define APP_IND_EVAL_PERIOD_MS (100U)

/* 红灯灯效参数 */
#define APP_IND_ESTOP_BLINK_CYCLE_MS (100U) /**< 急停：快闪 */
#define APP_IND_ESTOP_BLINK_WAIT_MS (100U)
#define APP_IND_CRITICAL_BLINK_CYCLE_MS (300U) /**< 关键电源轨故障：慢闪 */
#define APP_IND_CRITICAL_BLINK_WAIT_MS (300U)
#define APP_IND_OFFLINE_BLINK_CYCLE_MS (500U) /**< 子设备离线：中速闪烁 */
#define APP_IND_OFFLINE_BLINK_WAIT_MS (500U)

/* Private types -------------------------------------------------------------*/

/**
 * @brief 指示等级（数值仅用于排序比较，P0 最高优先）
 */
typedef enum {
    APP_IND_LEVEL_NORMAL = 0, /**< 正常运行 */
    APP_IND_LEVEL_WARNING, /**< 警告：次级轨/HSD-LSD/风扇/NTC/电池温度 */
    APP_IND_LEVEL_DEVICE_OFFLINE, /**< 子设备离线 */
    APP_IND_LEVEL_CRITICAL, /**< 关键电源轨故障 */
    APP_IND_LEVEL_ESTOP, /**< 急停 / 故障锁存 */
    APP_IND_LEVEL_COUNT,
} app_ind_level_t;

/* Private variables ---------------------------------------------------------*/

static srv_signal_handle_t* s_blue; /**< 蓝色 LED 实例（led_task 注入） */
static srv_signal_handle_t* s_red; /**< 红色 LED 实例 */
static app_ind_level_t s_cur_level; /**< 当前等级（哨兵值强制首轮下发） */
static uint16_t s_eval_ms; /**< 节流累计 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static app_ind_level_t ind_evaluate(const srv_can_mst_status_frame_t* st);
static void ind_apply(app_ind_level_t level);
static void ind_set_state(srv_signal_handle_t* h, srv_signal_state_t state);
static void ind_set_blink(srv_signal_handle_t* h, uint16_t cycle_ms, uint16_t wait_ms);

/* Exported functions --------------------------------------------------------*/

void app_status_indicator_init(srv_signal_handle_t* blue, srv_signal_handle_t* red)
{
    if (!blue || !red) {
        APP_STATUS_INDICATOR_LOG_E("LED 实例为空 (blue=%p red=%p), 指示灯策略失效",
            (void*)blue, (void*)red);
        return;
    }

    s_blue = blue;
    s_red = red;
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

    /* 聚合系统状态：复用上报应用的故障位域，保证灯效与总线上报一致 */
    srv_can_mst_data_t st;
    app_status_report_fill(&st);

    const app_ind_level_t level = ind_evaluate(&st.status);

    /* 仅等级变化时下发命令，避免刷爆 srv_signal 异步队列 */
    if (level != s_cur_level) {
        s_cur_level = level;
        // ind_apply(level);
        APP_STATUS_INDICATOR_LOG_I("状态等级切换: %u", (unsigned)level);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 按优先级评估当前状态等级（高者优先，先判 P0）
 */
static app_ind_level_t ind_evaluate(const srv_can_mst_status_frame_t* st)
{
    /* P0 急停 / 故障锁存（最高优先级） */
    if (app_fault_policy_is_tripped() || st->bits.stop_key_state) {
        return APP_IND_LEVEL_ESTOP;
    }

    /* P1 关键电源轨故障 */
    if (st->bits.err_vin || st->bits.err_vin_dcdc || st->bits.err_power
        || st->bits.err_motor || st->bits.err_chg_out || st->bits.err_dbr
        || st->bits.seq_vin_fault || st->bits.seq_chg_fault
        || st->bits.seq_motor_fault) {
        return APP_IND_LEVEL_CRITICAL;
    }

    /* P2 子设备离线（双电池板 / 副电源板） */
    if (!st->bits.device_online_slaver || !st->bits.device_online_dual) {
        return APP_IND_LEVEL_DEVICE_OFFLINE;
    }

    /* P3 警告：次级电源轨 / 输出轨 / HSD-LSD / 风扇 / NTC / 电池温度 */
    if (st->bits.err_12v_int || st->bits.err_5v_int || st->bits.err_12v_ext
        || st->bits.err_24v_ext || st->bits.err_12v_user || st->bits.err_24v_user
        || st->bits.err_24v_comp || st->bits.err_hsd1_12v
        || st->bits.err_hsd2_12v || st->bits.err_hsd3_12v
        || st->bits.err_hsd1_24v || st->bits.err_hsd2_24v
        || st->bits.err_hsd3_24v || st->bits.err_lsd1_24v
        || st->bits.err_lsd2_24v || st->bits.err_fan0 || st->bits.err_fan1
        || st->bits.err_ntc0 || st->bits.err_ntc1 || st->bits.err_ntc2
        || st->bits.err_ntc3 || st->bits.err_ntc4 || st->bits.err_ntc5
        || st->bits.err_ntc6 || st->bits.err_ntc7
        || st->bits.battery_temp_error) {
        return APP_IND_LEVEL_WARNING;
    }

    return APP_IND_LEVEL_NORMAL;
}

/**
 * @brief 将状态等级映射为蓝/红双 LED 灯效命令
 *
 * @note 红灯闪烁必须先在 srv_signal_set_state(BLINK_CODE) 前下发闪烁参数
 *       （srv_signal 异步队列 FIFO 保证参数先生效、状态后切换）。
 */
static void ind_apply(app_ind_level_t level)
{
    switch (level) {
    case APP_IND_LEVEL_ESTOP:
        ind_set_blink(s_red, APP_IND_ESTOP_BLINK_CYCLE_MS, APP_IND_ESTOP_BLINK_WAIT_MS);
        ind_set_state(s_red, SRV_SIGNAL_STATE_BLINK_CODE);
        ind_set_state(s_blue, SRV_SIGNAL_STATE_OFF);
        break;

    case APP_IND_LEVEL_CRITICAL:
        ind_set_blink(s_red, APP_IND_CRITICAL_BLINK_CYCLE_MS, APP_IND_CRITICAL_BLINK_WAIT_MS);
        ind_set_state(s_red, SRV_SIGNAL_STATE_BLINK_CODE);
        ind_set_state(s_blue, SRV_SIGNAL_STATE_OFF);
        break;

    case APP_IND_LEVEL_DEVICE_OFFLINE:
        ind_set_blink(s_red, APP_IND_OFFLINE_BLINK_CYCLE_MS, APP_IND_OFFLINE_BLINK_WAIT_MS);
        ind_set_state(s_red, SRV_SIGNAL_STATE_BLINK_CODE);
        ind_set_state(s_blue, SRV_SIGNAL_STATE_BREATHING);
        break;

    case APP_IND_LEVEL_WARNING:
        ind_set_state(s_red, SRV_SIGNAL_STATE_BREATHING);
        ind_set_state(s_blue, SRV_SIGNAL_STATE_BREATHING);
        break;

    case APP_IND_LEVEL_NORMAL:
    default:
        ind_set_state(s_red, SRV_SIGNAL_STATE_OFF);
        ind_set_state(s_blue, SRV_SIGNAL_STATE_BREATHING);
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
        .blink_cycle_ms = cycle_ms,
        .blink_wait_ms = wait_ms,
        .blink_code_counts = 0, /* 无限循环，直到等级切换 */
    };
    srv_signal_set_blink_interval(h, &cmd);
}
