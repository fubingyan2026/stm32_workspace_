/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.c
 * @brief   运行 LED 状态指示任务（srv_signal 驱动，参考 E1_Master_Power_Manage boot 变体）
 *
 * 按 Boot 升级状态切换运行 LED（PA2）灯效：
 *   IDLE            → 慢闪（等待升级指令）
 *   START/DATA      → 快闪（数据传输中）
 *   VERIFY_PENDING  → 中速闪（校验中）
 *   REBOOT_PENDING  → 常亮（即将重启）
 */

/* Includes ------------------------------------------------------------------*/
#include "led_task.h"

#include "boot_fsm.h"   /* BOOT_STATE_* */
#include "boot_task.h"  /* boot_task_get_state */
#include "drv_led.h"
#include "drv_systick.h"
#include "log.h"
#include "srv_signal.h"
#include "sw_timer.h"

/* Private constants ---------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define LED_TASK_LOG_ENABLE 1

#if LED_TASK_LOG_ENABLE
#define LED_TASK_LOG_E(...) LOG_E("led_task", __VA_ARGS__)
#define LED_TASK_LOG_W(...) LOG_W("led_task", __VA_ARGS__)
#define LED_TASK_LOG_I(...) LOG_I("led_task", __VA_ARGS__)
#define LED_TASK_LOG_D(...) LOG_D("led_task", __VA_ARGS__)
#else
#define LED_TASK_LOG_E(...) ((void)0)
#define LED_TASK_LOG_W(...) ((void)0)
#define LED_TASK_LOG_I(...) ((void)0)
#define LED_TASK_LOG_D(...) ((void)0)
#endif

/** @brief LED 刷新周期 (ms) */
#define LED_TASK_REFRESH_PERIOD_MS (10U)

/** @brief 各升级状态闪烁半周期 (ms) */
#define LED_TASK_BLINK_WAIT_CYCLE_MS   (400U)  /**< IDLE：等待升级，慢闪 */
#define LED_TASK_BLINK_TX_CYCLE_MS     (50U)   /**< START/DATA：传输中，快闪 */
#define LED_TASK_BLINK_VERIFY_CYCLE_MS (200U)  /**< VERIFY：校验中，中速闪 */

/** @brief srv_signal 0-1023 → GPIO 数字电平阈值（ON=1023 ≥ 阈值 → 点亮） */
#define LED_TASK_WRITE_THRESHOLD (512U)

/* Private variables ---------------------------------------------------------*/

static srv_signal_handle_t s_led;
static sw_timer_t s_led_timer;

/** @brief 上次 Boot 状态（仅状态变化时下发灯效命令，避免刷命令队列） */
static uint8_t s_last_boot_state = 0xFFU;

/* Private function prototypes -----------------------------------------------*/

static void led_write_output(uint16_t value);
static void led_boot_update(void);
static void led_timer_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

void led_task_init(void)
{
    drv_led_init();
    srv_signal_init(millis);

    /* 运行 LED：初始灭，首个 tick 由 led_boot_update 按状态切换灯效 */
    srv_signal_config_t cfg = {
        .name = "status",
        .init_state = SRV_SIGNAL_STATE_OFF,
        .write_output = led_write_output,
    };
    srv_signal_register_static(&cfg, &s_led);

    /* 启动 sw_timer 驱动 srv_signal FSM */
    const sw_timer_config_t timer_cfg = {
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = led_timer_cb,
        .user_data = NULL,
    };
    sw_timer_init(&s_led_timer, &timer_cfg);
    sw_timer_start(&s_led_timer, LED_TASK_REFRESH_PERIOD_MS, 0);

    LED_TASK_LOG_I("运行 LED 任务初始化完成: srv_signal 按升级状态指示 (period=%ums)",
        (unsigned)LED_TASK_REFRESH_PERIOD_MS);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief srv_signal 输出回调：0-1023 PWM 逻辑值 → GPIO 数字电平
 */
static void led_write_output(uint16_t value)
{
    if (value >= LED_TASK_WRITE_THRESHOLD) {
        drv_led_on();
    } else {
        drv_led_off();
    }
}

/**
 * @brief 按 Boot 升级状态切换运行 LED 灯效（仅状态变化时下发命令）
 *
 * 依赖 boot_task_get_state()：FSM 未初始化时返回 BOOT_STATE_IDLE，
 * 故 boot_task_init() 之前视为等待升级（慢闪）。
 */
static void led_boot_update(void)
{
    uint8_t state = boot_task_get_state();
    if (state == s_last_boot_state) {
        return;
    }
    s_last_boot_state = state;

    switch (state) {
    case BOOT_STATE_IDLE:
        /* 等待升级：慢闪 */
        srv_signal_set_state(&s_led, SRV_SIGNAL_STATE_BLINK_CODE);
        srv_signal_set_blink_interval(&s_led, &(srv_signal_cmd_t) {
            .set_state = SRV_SIGNAL_STATE_BLINK_CODE,
            .blink_cycle_ms = LED_TASK_BLINK_WAIT_CYCLE_MS,
            .blink_wait_ms = LED_TASK_BLINK_WAIT_CYCLE_MS,
            .blink_code_counts = 0U });
        break;
    case BOOT_STATE_START:
    case BOOT_STATE_DATA_TRANSFER:
        /* 传输中：快闪 */
        srv_signal_set_state(&s_led, SRV_SIGNAL_STATE_BLINK_CODE);
        srv_signal_set_blink_interval(&s_led, &(srv_signal_cmd_t) {
            .set_state = SRV_SIGNAL_STATE_BLINK_CODE,
            .blink_cycle_ms = LED_TASK_BLINK_TX_CYCLE_MS,
            .blink_wait_ms = LED_TASK_BLINK_TX_CYCLE_MS,
            .blink_code_counts = 0U });
        break;
    case BOOT_STATE_VERIFY_PENDING:
        /* 校验中：中速闪 */
        srv_signal_set_state(&s_led, SRV_SIGNAL_STATE_BLINK_CODE);
        srv_signal_set_blink_interval(&s_led, &(srv_signal_cmd_t) {
            .set_state = SRV_SIGNAL_STATE_BLINK_CODE,
            .blink_cycle_ms = LED_TASK_BLINK_VERIFY_CYCLE_MS,
            .blink_wait_ms = LED_TASK_BLINK_VERIFY_CYCLE_MS,
            .blink_code_counts = 0U });
        break;
    case BOOT_STATE_REBOOT_PENDING:
        /* 即将重启：常亮 */
        srv_signal_set_state(&s_led, SRV_SIGNAL_STATE_ON);
        break;
    default:
        break;
    }
}

/**
 * @brief sw_timer 回调：更新灯效并驱动 srv_signal FSM 步进
 */
static void led_timer_cb(void* user_data)
{
    (void)user_data;

    led_boot_update();
    srv_signal_task_refresh();
}
