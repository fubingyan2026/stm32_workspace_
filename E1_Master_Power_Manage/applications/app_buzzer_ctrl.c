/**
 * @file    app_buzzer_ctrl.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   应用层 — 蜂鸣器行为切换策略实现
 *
 * 将行为模式映射为对蜂鸣器 srv_signal 伪实例的异步命令：
 *   SILENT     → OFF
 *   CONTINUOUS → ON
 *   BEEP_CODE  → set_blink_interval + BLINK_CODE（鸣 N 次）
 *   BREATH     → BREATHING（音量呼吸）
 */

/* Includes ------------------------------------------------------------------*/
#include "app_buzzer_ctrl.h"

#include "log.h"
#include "srv_signal.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define APP_BUZZER_CTRL_LOG_ENABLE 1

#if APP_BUZZER_CTRL_LOG_ENABLE
#define APP_BUZZER_CTRL_LOG_E(...) LOG_E("app_buzzer_ctrl", __VA_ARGS__)
#define APP_BUZZER_CTRL_LOG_W(...) LOG_W("app_buzzer_ctrl", __VA_ARGS__)
#define APP_BUZZER_CTRL_LOG_I(...) LOG_I("app_buzzer_ctrl", __VA_ARGS__)
#define APP_BUZZER_CTRL_LOG_D(...) LOG_D("app_buzzer_ctrl", __VA_ARGS__)
#else
#define APP_BUZZER_CTRL_LOG_E(...) ((void)0)
#define APP_BUZZER_CTRL_LOG_W(...) ((void)0)
#define APP_BUZZER_CTRL_LOG_I(...) ((void)0)
#define APP_BUZZER_CTRL_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 默认编码鸣响参数 */
#define APP_BUZZER_DEF_BEEP_CYCLE_MS (200U)
#define APP_BUZZER_DEF_BEEP_WAIT_MS (200U)
#define APP_BUZZER_DEF_BEEP_COUNT (3U)

/* Private variables ---------------------------------------------------------*/

static srv_signal_handle_t* s_buzzer; /**< 蜂鸣器 srv_signal 实例（buzzer_task 注入） */
static app_buzzer_mode_t s_mode; /**< 当前行为模式 */
static srv_signal_cmd_t s_beep_cmd; /**< 编码鸣响参数 */
static bool s_initialized;

/* Exported functions --------------------------------------------------------*/

void app_buzzer_ctrl_init(srv_signal_handle_t* buzzer)
{
    if (!buzzer) {
        APP_BUZZER_CTRL_LOG_E("蜂鸣器句柄为空, 控制策略失效");
        return;
    }

    s_buzzer = buzzer;
    s_mode = APP_BUZZER_MODE_SILENT;
    s_beep_cmd = (srv_signal_cmd_t){
        .set_state = SRV_SIGNAL_STATE_BLINK_CODE,
        .blink_cycle_ms = APP_BUZZER_DEF_BEEP_CYCLE_MS,
        .blink_wait_ms = APP_BUZZER_DEF_BEEP_WAIT_MS,
        .blink_code_counts = APP_BUZZER_DEF_BEEP_COUNT,
    };
    s_initialized = true;

    APP_BUZZER_CTRL_LOG_I("蜂鸣器控制策略初始化完成 (默认静音)");
}

void app_buzzer_ctrl_set_mode(app_buzzer_mode_t mode)
{
    if (!s_initialized || mode >= APP_BUZZER_MODE_COUNT) {
        return;
    }

    s_mode = mode;

    switch (mode) {
    case APP_BUZZER_MODE_SILENT:
        srv_signal_set_state(s_buzzer, SRV_SIGNAL_STATE_OFF);
        break;
    case APP_BUZZER_MODE_CONTINUOUS:
        srv_signal_set_state(s_buzzer, SRV_SIGNAL_STATE_ON);
        break;
    case APP_BUZZER_MODE_BEEP_CODE:
        /* 先落参数再切状态：srv_signal 异步队列 FIFO 保证参数先生效 */
        srv_signal_set_blink_interval(s_buzzer, &s_beep_cmd);
        srv_signal_set_state(s_buzzer, SRV_SIGNAL_STATE_BLINK_CODE);
        break;
    case APP_BUZZER_MODE_BREATH:
        srv_signal_set_state(s_buzzer, SRV_SIGNAL_STATE_BREATHING);
        break;
    default:
        break;
    }

    APP_BUZZER_CTRL_LOG_I("蜂鸣器模式切换: %u", (unsigned)mode);
}

void app_buzzer_ctrl_set_beep_code(uint16_t beep_count, uint16_t cycle_ms,
    uint16_t wait_ms)
{
    if (!s_initialized) {
        return;
    }

    /* beep_count=0 表示无限循环（srv_signal 约定）；cycle/wait 为 0 表示保持当前 */
    s_beep_cmd.blink_code_counts = beep_count;
    if (cycle_ms > 0)
        s_beep_cmd.blink_cycle_ms = cycle_ms;
    if (wait_ms > 0)
        s_beep_cmd.blink_wait_ms = wait_ms;

    APP_BUZZER_CTRL_LOG_I("蜂鸣器编码鸣响参数: %u 次 cycle=%ums wait=%ums",
        (unsigned)s_beep_cmd.blink_code_counts,
        (unsigned)s_beep_cmd.blink_cycle_ms,
        (unsigned)s_beep_cmd.blink_wait_ms);

    /* 若正在编码鸣响，立即重发使新参数生效（srv_signal 在熄灭沿应用，无毛刺） */
    if (s_mode == APP_BUZZER_MODE_BEEP_CODE) {
        srv_signal_set_blink_interval(s_buzzer, &s_beep_cmd);
        srv_signal_set_state(s_buzzer, SRV_SIGNAL_STATE_BLINK_CODE);
    }
}
