/**
 * @file    app_fault_policy.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   应用层 — 故障保护策略实现（E1_SLAVER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "app_fault_policy.h"

#include "log.h"
#include "srv_adc.h"
#include "srv_pwr_ctrl.h"

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

static uint8_t s_prev_latch_mask; /**< 上次锁存掩码（用于上升沿日志） */
static uint16_t s_aux_lost_ms; /**< DC24V 运行中 AUX 缺失累计 (ms) */

/* Private functions prototypes ----------------------------------------------*/

static void log_latch_transitions(void);
static void monitor_bus_aux(uint16_t elapsed_ms);

/* Exported functions --------------------------------------------------------*/

void app_fault_policy_init(void)
{
    s_prev_latch_mask = srv_pwr_ctrl_get_latch_mask();
    s_aux_lost_ms = 0;
    APP_FAULT_POLICY_LOG_I("故障保护策略初始化完成");
}

void app_fault_policy_step(uint16_t elapsed_ms)
{
    log_latch_transitions();

    /* 母线级监控（无锁存时评估，避免重复触发刷屏） */
    if (!srv_pwr_ctrl_is_any_latched()) {
        monitor_bus_aux(elapsed_ms);
    } else {
        s_aux_lost_ms = 0;
    }
}

bool app_fault_policy_is_tripped(void)
{
    return srv_pwr_ctrl_is_any_latched();
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 锁存掩码上升沿日志（哪位输出新触发锁存）
 */
static void log_latch_transitions(void)
{
    const uint8_t latch = srv_pwr_ctrl_get_latch_mask();
    const uint8_t rising = (uint8_t)(latch & (uint8_t)~s_prev_latch_mask);

    if ((rising & SRV_PWR_OUT_MASK_DC24V) != 0) {
        APP_FAULT_POLICY_LOG_E("故障保护触发: DC24V(24V 降压) 故障锁存");
    }
    if ((rising & SRV_PWR_OUT_MASK_ISO12V) != 0) {
        APP_FAULT_POLICY_LOG_E("故障保护触发: ISO12V(12V_ISO) 故障锁存");
    }
    if ((rising & SRV_PWR_OUT_MASK_LSD1) != 0) {
        APP_FAULT_POLICY_LOG_E("故障保护触发: LSD1 故障锁存");
    }
    if ((rising & SRV_PWR_OUT_MASK_LSD2) != 0) {
        APP_FAULT_POLICY_LOG_E("故障保护触发: LSD2 故障锁存");
    }

    s_prev_latch_mask = latch;
}

/**
 * @brief 母线级监控：DC24V 实际运行中 AUX 输入跌落后持续缺失 → 锁存 DC24V
 * @note  仅在 DC24V 处于 ON（此前已成功上电）时评估；AUX 从未满足时使能被门控等待，
 *        不在此锁存（主机经 0x01 状态帧可见 err_aux）。
 */
static void monitor_bus_aux(uint16_t elapsed_ms)
{
    const uint8_t on = srv_pwr_ctrl_get_on_outputs();
    if ((on & SRV_PWR_OUT_MASK_DC24V) == 0) {
        s_aux_lost_ms = 0;
        return;
    }

    srv_adc_data_t adc;
    bool aux_ok = false;
    if (srv_adc_get_latest(&adc)) {
        aux_ok = (adc.aux_mv >= SRV_PWR_AUX_PRESENT_MV);
    }

    if (!aux_ok) {
        s_aux_lost_ms += elapsed_ms;
        if (s_aux_lost_ms >= SRV_PWR_LOSS_DEBOUNCE_MS) {
            APP_FAULT_POLICY_LOG_E("AUX 母线缺失超时 (%ums) → 锁存 DC24V",
                (unsigned)SRV_PWR_LOSS_DEBOUNCE_MS);
            srv_pwr_ctrl_latch_outputs(SRV_PWR_OUT_MASK_DC24V);
            s_aux_lost_ms = 0;
        }
    } else {
        s_aux_lost_ms = 0;
    }
}
