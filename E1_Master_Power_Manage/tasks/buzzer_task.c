/**
 * @file    buzzer_task.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-06
 * @brief   蜂鸣器任务实现 — 复用 srv_signal 状态机驱动蜂鸣器
 * @attention
 *
 * 蜂鸣器作为 srv_signal 的一个srv_signal 实例注册：
 *   - write_output 回调把 srv_signal 逻辑值 0-1023 映射到 drv_buzzer_set 占空比 0-100
 *   - 免费获得 ON/OFF/BLINK_CODE/BREATHING 状态机 + 异步命令队列
 *   - 刷新由 led_task 的 srv_signal_task_refresh() 遍历所有实例覆盖，本任务不拥有定时器
 *
 * 当前不被任何对象驱动（默认静音），架构就位，后续 app 层通过句柄按需控制。
 */

/* Includes ------------------------------------------------------------------*/
#include "buzzer_task.h"

#include "app_buzzer_ctrl.h"
#include "drv_buzzer.h"
#include "log.h"
#include "srv_signal.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define BUZZER_TASK_LOG_ENABLE 1

#if BUZZER_TASK_LOG_ENABLE
#define BUZZER_TASK_LOG_E(...) LOG_E("buzzer_task", __VA_ARGS__)
#define BUZZER_TASK_LOG_W(...) LOG_W("buzzer_task", __VA_ARGS__)
#define BUZZER_TASK_LOG_I(...) LOG_I("buzzer_task", __VA_ARGS__)
#define BUZZER_TASK_LOG_D(...) LOG_D("buzzer_task", __VA_ARGS__)
#else
#define BUZZER_TASK_LOG_E(...) ((void)0)
#define BUZZER_TASK_LOG_W(...) ((void)0)
#define BUZZER_TASK_LOG_I(...) ((void)0)
#define BUZZER_TASK_LOG_D(...) ((void)0)
#endif

/* Private variables ---------------------------------------------------------*/

static srv_signal_handle_t s_buzzer; /**< 蜂鸣器 srv_signal 实例（静态分配） */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void buzzer_write_output(uint16_t value);

/* Exported functions --------------------------------------------------------*/

void buzzer_task_init(void)
{
    drv_buzzer_init();

    /* write_output: srv_signal 逻辑值 0-1023 → 蜂鸣器占空比 0-100 */
    const srv_signal_config_t cfg = {
        .name = "buzzer",
        .init_state = SRV_SIGNAL_STATE_OFF, /* 暂不被驱动，默认静音 */
        .write_output = buzzer_write_output,
    };
    if (srv_signal_register_static(&cfg, &s_buzzer) != SRV_SIGNAL_OK) {
        BUZZER_TASK_LOG_E("蜂鸣器 srv_signal 实例注册失败");
        return;
    }

    /* 蜂鸣器行为切换策略（注入句柄，需在注册之后） */
    app_buzzer_ctrl_init(&s_buzzer);

    s_initialized = true;
    BUZZER_TASK_LOG_I("蜂鸣器任务初始化完成: 已注册为 srv_signal 伪实例 (暂未驱动)");
}

srv_signal_handle_t* buzzer_task_get_handle(void)
{
    return s_initialized ? &s_buzzer : NULL;
}

/* Private functions ---------------------------------------------------------*/

static void buzzer_write_output(uint16_t value)
{
    drv_buzzer_set((uint8_t)((uint32_t)value * 100U / 1023U));
}
