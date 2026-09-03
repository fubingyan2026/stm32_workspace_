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

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/** @brief E-STOP 冗余数据读取回调（接线层注入，见 srv_pwr_det_init） */
static srv_pwr_det_estop_redun_cb_t s_estop_redun_cb;

/** @brief 状态遥测日志时间戳 (ms) */
static uint32_t s_pwr_det_log_ts;

/** @brief 需要做边沿检测的故障类信号（0→1 断言 / 1→0 清除） */
static const drv_status_signal_t s_fault_signals[] = {
    DRV_STATUS_E_STOP_ON, /**< 急停 */
};
#define SRV_PWR_DET_FAULT_SIG_NUM \
    (uint32_t)(sizeof(s_fault_signals) / sizeof(s_fault_signals[0]))

/** @brief 上次轮询掩码（用于边沿检测） */
static uint32_t s_pwr_det_prev_mask;
/** @brief 首次读取标志：上电初始状态不判边沿，避免误报 */
static bool s_pwr_det_prev_valid;

/* Exported functions --------------------------------------------------------*/

void srv_pwr_det_init(srv_pwr_det_estop_redun_cb_t estop_redun_cb)
{
    /* 本服务封装 drv_status 读取 PGOOD/E-STOP，必须先初始化其状态位 */
    drv_status_init();

    s_estop_redun_cb = estop_redun_cb;
    s_initialized = true;
    s_pwr_det_prev_valid = false;

    SRV_PWR_DET_LOG_I("电源状态监控服务初始化完成 (冗余回调已注入=%d)",
        (int)(estop_redun_cb != NULL));
}

void srv_pwr_det_read(srv_pwr_det_status_t* status)
{
    if (!status) {
        return;
    }

    uint32_t sta = s_initialized ? drv_status_read_all() : 0;

    /* 故障信号边沿检测：仅状态变化时打印，避免轮询刷屏 */
    if (s_pwr_det_prev_valid) {
        for (uint32_t i = 0; i < SRV_PWR_DET_FAULT_SIG_NUM; i++) {
            const drv_status_signal_t sig = s_fault_signals[i];
            const uint32_t bit = 1UL << (uint32_t)sig;
            const bool now_set = (sta & bit) != 0;
            const bool was_set = (s_pwr_det_prev_mask & bit) != 0;
            if (now_set && !was_set) {
                SRV_PWR_DET_LOG_E("数字侧急停输入断言: %s (是否形成有效急停还取决于冗余侧)",
                    drv_status_name(sig));
            } else if (!now_set && was_set) {
                SRV_PWR_DET_LOG_I("数字侧急停输入清除: %s", drv_status_name(sig));
            }
        }
    } else {
        s_pwr_det_prev_valid = true;
    }
    s_pwr_det_prev_mask = sta;

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
