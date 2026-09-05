/**
 * @file    srv_pwr_det.c
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-09-03
 * @brief   电源状态监控服务实现 (E1_MASTER_POWER_CTU)
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_det.h"

#include "drv_status.h"
#include "drv_systick.h"
#include "log.h"
#include "utils_math.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_PWR_DET_LOG_ENABLE 1

#if SRV_PWR_DET_LOG_ENABLE
#define SRV_PWR_DET_LOG_E(...) LOG_E("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_W(...) LOG_W("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_I(...) LOG_I("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_D(...) LOG_D("srv_pwr_det", __VA_ARGS__)
#else
#define SRV_PWR_DET_LOG_E(...) ((void)0)
#define SRV_PWR_DET_LOG_W(...) ((void)0)
#define SRV_PWR_DET_LOG_I(...) ((void)0)
#define SRV_PWR_DET_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 状态遥测日志限频窗口 (ms)：轮询需限频防刷屏 */
#define SRV_PWR_DET_LOG_PERIOD_MS (1000U)

/** @brief 常开轨异常持续重报周期 (ms)：异常持续期间每秒打一条错误日志 */
#define SRV_PWR_DET_RAIL_ERR_REPORT_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/** @brief E-STOP 冗余数据读取回调（接线层注入，见 srv_pwr_det_init） */
static srv_pwr_det_estop_redun_cb_t s_estop_redun_cb;

/** @brief 常开轨使能掩码回调（接线层注入；NULL=视为全部使能） */
static srv_pwr_det_rail_en_cb_t s_rail_en_cb;

/** @brief 状态遥测日志时间戳 (ms) */
static uint32_t s_pwr_det_log_ts;

/** @brief 需要做边沿检测的故障类信号（0→1 断言 / 1→0 清除） */
static const drv_status_signal_t s_fault_signals[] = {
    DRV_STATUS_E_STOP_ON, /**< 急停 */
};
#define SRV_PWR_DET_FAULT_SIG_NUM \
    (uint32_t)(sizeof(s_fault_signals) / sizeof(s_fault_signals[0]))

/** @brief 故障信号边沿检测状态（每信号一个，复用共享 utils_edge_detect） */
static utils_edge_det_t s_fault_edge[SRV_PWR_DET_FAULT_SIG_NUM];

/**
 * @brief 三路常开轨（异常即打错误日志，边沿 + 持续重报）
 * @note  与急停不同：这些轨默认常开，PGD=0 判异常；异常期间持续每秒重报
 */
static const drv_status_signal_t s_rail_mon_signals[] = {
    DRV_STATUS_LM5060_PGD, /**< VIN_DC-DC(LM5060) */
    DRV_STATUS_DC24V_PGD,  /**< DC-DC 24V(MP9931N) */
    DRV_STATUS_AUX_PGD,    /**< AUX(LM5069) */
};
#define SRV_PWR_DET_RAIL_MON_NUM \
    (uint32_t)(sizeof(s_rail_mon_signals) / sizeof(s_rail_mon_signals[0]))

/** @brief 常开轨异常边沿检测状态（每轨一个） */
static utils_edge_det_t s_rail_edge[SRV_PWR_DET_RAIL_MON_NUM];
/** @brief 常开轨异常持续重报时间戳 (ms) */
static uint32_t s_rail_mon_log_ts;

/* Private function prototypes -----------------------------------------------*/

static void srv_pwr_det_rail_monitor(uint32_t sta);

/* Exported functions --------------------------------------------------------*/

void srv_pwr_det_init(srv_pwr_det_estop_redun_cb_t estop_redun_cb,
    srv_pwr_det_rail_en_cb_t rail_en_cb)
{
    /* 本服务封装 drv_status 读取 PGOOD/E-STOP，必须先初始化其状态位 */
    drv_status_init();

    s_estop_redun_cb = estop_redun_cb;
    s_rail_en_cb = rail_en_cb;
    s_initialized = true;

    /* 边沿检测状态建基线由 utils_edge_detect 首次调用完成，这里显式复位以清晰 */
    for (uint32_t i = 0; i < SRV_PWR_DET_FAULT_SIG_NUM; i++) {
        utils_edge_det_init(&s_fault_edge[i]);
    }
    for (uint32_t i = 0; i < SRV_PWR_DET_RAIL_MON_NUM; i++) {
        utils_edge_det_init(&s_rail_edge[i]);
    }
    s_rail_mon_log_ts = 0;

    SRV_PWR_DET_LOG_I("电源状态监控服务初始化完成 (冗余回调已注入=%d)",
        (int)(estop_redun_cb != NULL));
}

void srv_pwr_det_read(srv_pwr_det_status_t* status)
{
    if (!status) {
        return;
    }

    uint32_t sta = s_initialized ? drv_status_read_all() : 0;

    /* 三路常开轨异常监控：PGD 丢失即打错误日志（边沿 + 持续重报） */
    srv_pwr_det_rail_monitor(sta);

    /* 故障信号边沿检测（共享 utils_edge_detect）：仅状态变化时打印，避免轮询刷屏 */
    for (uint32_t i = 0; i < SRV_PWR_DET_FAULT_SIG_NUM; i++) {
        const drv_status_signal_t sig = s_fault_signals[i];
        const uint32_t bit = 1UL << (uint32_t)sig;
        const bool now_set = (sta & bit) != 0;

        switch (utils_edge_detect(&s_fault_edge[i], now_set)) {
        case UTILS_EDGE_RISING:
            SRV_PWR_DET_LOG_E("数字侧急停输入断言: %s (是否形成有效急停还取决于冗余侧)",
                drv_status_name(sig));
            break;
        case UTILS_EDGE_FALLING:
            SRV_PWR_DET_LOG_I("数字侧急停输入清除: %s", drv_status_name(sig));
            break;
        default:
            break;
        }
    }

    status->lm5060_ok = ((sta >> DRV_STATUS_LM5060_PGD) & 1U) != 0;
    status->dc24v_ok = ((sta >> DRV_STATUS_DC24V_PGD) & 1U) != 0;
    status->p12v_ok = ((sta >> DRV_STATUS_12V_PGD) & 1U) != 0;
    status->aux_power_ok = ((sta >> DRV_STATUS_AUX_PGD) & 1U) != 0;
    status->motor_power_ok = ((sta >> DRV_STATUS_MOTOR_PGD) & 1U) != 0;

    /* 有效急停 = 数字 E_STOP_ON 按下 且 E-STOP 冗余闭合掩码不全闭合（双判据 AND）：
     * 数字按下但冗余仍全部闭合 → 判为线缆/按钮信号异常（estop_inconsistent），不触发断电；
     * 冗余回调缺失或首轮采样未完成时冗余侧视为无效，不置有效急停。 */
    const bool estop_digital = ((sta >> DRV_STATUS_E_STOP_ON) & 1U) != 0;

    bool estop_adc_valid = false;
    uint8_t estop_closed = 0U;
    if (s_estop_redun_cb != NULL) {
        srv_pwr_det_estop_redun_t redun;
        s_estop_redun_cb(&redun);
        estop_adc_valid = redun.valid;
        estop_closed = redun.closed_mask;
    }

    const bool estop_redund_open = estop_adc_valid && (estop_closed != SRV_PWR_DET_ESTOP_ALL_CLOSED_MASK);
    status->estop_on = estop_digital && estop_redund_open;
    status->estop_inconsistent = estop_adc_valid && (estop_digital != estop_redund_open);

    /* 状态遥测日志（限频 1s） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_pwr_det_log_ts) >= SRV_PWR_DET_LOG_PERIOD_MS) {
        s_pwr_det_log_ts = now_ms;

        if (status->estop_inconsistent) {
            SRV_PWR_DET_LOG_E("急停判据不一致: 数字按下=%u 冗余闭合掩码=0x%02X (异常，不触发断电，检查线缆/按钮)",
                (unsigned)estop_digital, (unsigned)estop_closed);
        }

        SRV_PWR_DET_LOG_D("电源状态: LM5060=%u 24V=%u 12V=%u 辅助=%u 电机=%u 急停=%u",
            (unsigned)status->lm5060_ok, (unsigned)status->dc24v_ok,
            (unsigned)status->p12v_ok, (unsigned)status->aux_power_ok,
            (unsigned)status->motor_power_ok, (unsigned)status->estop_on);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 三路常开轨 PGD 异常监控
 * @param sta drv_status_read_all() 原始位掩码
 * @note  由 srv_pwr_det_read() 每次轮询调用：
 *   - 异常出现边沿 → 立即打一条错误日志
 *   - 异常持续期间 → 每 SRV_PWR_DET_RAIL_ERR_REPORT_MS(1s) 聚合重报一条错误日志
 *   - 恢复边沿 → 打一条信息日志
 */
static void srv_pwr_det_rail_monitor(uint32_t sta)
{
    if (!s_initialized) {
        return;
    }

    /* 使能门控：仅监测“已使能”的轨（上电流程逐步使能，未使能轨 PGD 恒低属正常） */
    const uint8_t en_mask = s_rail_en_cb ? s_rail_en_cb() : 0xFFU;

    bool any_abn = false;

    for (uint32_t i = 0; i < SRV_PWR_DET_RAIL_MON_NUM; i++) {
        if (((en_mask >> i) & 1U) == 0U) {
            continue; /* 轨未使能：不建基线、不判异常 */
        }

        const drv_status_signal_t sig = s_rail_mon_signals[i];
        const bool abn = ((sta >> sig) & 1U) == 0U; /* PGOOD 低电平=异常 */

        /* 边沿检测复用共享 utils_edge_detect（首采建基线，不误报） */
        switch (utils_edge_detect(&s_rail_edge[i], abn)) {
        case UTILS_EDGE_RISING:
            SRV_PWR_DET_LOG_E("常开轨异常: %s PGD 丢失 (pgd=0)", drv_status_name(sig));
            break;
        case UTILS_EDGE_FALLING:
            SRV_PWR_DET_LOG_I("常开轨恢复: %s PGD 就绪", drv_status_name(sig));
            break;
        default:
            break;
        }

        any_abn |= abn;
    }

    /* 异常持续期间聚合重报（1s 一次，逐路列出当前已使能且异常轨） */
    if (any_abn) {
        const uint32_t now_ms = millis();
        if ((uint32_t)(now_ms - s_rail_mon_log_ts) >= SRV_PWR_DET_RAIL_ERR_REPORT_MS) {
            s_rail_mon_log_ts = now_ms;

            char rail_list[64] = { 0 };
            for (uint32_t i = 0; i < SRV_PWR_DET_RAIL_MON_NUM; i++) {
                if (((en_mask >> i) & 1U) == 0U) {
                    continue;
                }
                const drv_status_signal_t sig = s_rail_mon_signals[i];
                if (((sta >> sig) & 1U) == 0U) {
                    if (rail_list[0] != '\0') {
                        (void)strncat(rail_list, ", ", sizeof(rail_list) - 1U);
                    }
                    (void)strncat(rail_list, drv_status_name(sig),
                        sizeof(rail_list) - strlen(rail_list) - 1U);
                }
            }
            SRV_PWR_DET_LOG_E("常开电源轨异常持续中: %s (PGD=0)", rail_list);
        }
    } else {
        s_rail_mon_log_ts = 0;
    }
}
