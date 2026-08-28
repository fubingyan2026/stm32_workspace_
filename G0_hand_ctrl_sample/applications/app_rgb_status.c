/**
 * @file    app_rgb_status.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   应用层 — RGB 三色状态指示灯管理实现（蓝/绿/红）
 */

/* Includes ------------------------------------------------------------------*/
#include "app_rgb_status.h"

#include "drv_pwm.h"
#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define APP_RGB_STATUS_LOG_ENABLE 1

#if APP_RGB_STATUS_LOG_ENABLE
#define APP_RGB_STATUS_LOG_E(...) LOG_E("app_rgb_status", __VA_ARGS__)
#define APP_RGB_STATUS_LOG_W(...) LOG_W("app_rgb_status", __VA_ARGS__)
#define APP_RGB_STATUS_LOG_I(...) LOG_I("app_rgb_status", __VA_ARGS__)
#define APP_RGB_STATUS_LOG_D(...) LOG_D("app_rgb_status", __VA_ARGS__)
#else
#define APP_RGB_STATUS_LOG_E(...) ((void)0)
#define APP_RGB_STATUS_LOG_W(...) ((void)0)
#define APP_RGB_STATUS_LOG_I(...) ((void)0)
#define APP_RGB_STATUS_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 呼吸参数 */
#define APP_RGB_BREATH_CYCLE_MS (1500U)
#define APP_RGB_BREATH_MIN_DUTY (20U)
#define APP_RGB_BREATH_MAX_DUTY (1024U)

/* Private variables ---------------------------------------------------------*/

/** @brief 每路 LED 独立的 static srv_signal 实例 */
static srv_signal_handle_t s_led_blue;
static srv_signal_handle_t s_led_green;
static srv_signal_handle_t s_led_red;

/** @brief 通道 → 实例映射 */
static srv_signal_handle_t* const s_led[APP_RGB_CH_NUM] = {
    [APP_RGB_CH_BLUE] = &s_led_blue,
    [APP_RGB_CH_GREEN] = &s_led_green,
    [APP_RGB_CH_RED] = &s_led_red,
};

/** @brief 初始化标志 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void led_blue_write(uint16_t value);

static void led_green_write(uint16_t value);

static void led_red_write(uint16_t value);

/* Exported functions --------------------------------------------------------*/

void app_rgb_status_init(void)
{
    if (s_initialized) {
        return;
    }

    /* PWM 输出（3 通道） */
    (void)drv_pwm_init();

    /* srv_signal 系统初始化（时间戳回调） */
    srv_signal_init(millis);

    /* 每路 LED 独立实例：蓝灯默认呼吸，绿灯/红灯默认关闭 */
    const srv_signal_config_t cfg[APP_RGB_CH_NUM] = {
        [APP_RGB_CH_BLUE] = {
            .name = "led_blue",
            .init_state = SRV_SIGNAL_STATE_BREATHING,
            .write_output = led_blue_write,
            .breath_cycle_ms = APP_RGB_BREATH_CYCLE_MS,
            .breath_min_duty = APP_RGB_BREATH_MIN_DUTY,
            .breath_max_duty = APP_RGB_BREATH_MAX_DUTY,
        },
        [APP_RGB_CH_GREEN] = {
            .name = "led_green",
            .init_state = SRV_SIGNAL_STATE_BREATHING,
            .write_output = led_green_write,
            .breath_cycle_ms = APP_RGB_BREATH_CYCLE_MS,
            .breath_min_duty = APP_RGB_BREATH_MIN_DUTY,
            .breath_max_duty = APP_RGB_BREATH_MAX_DUTY,
        },
        [APP_RGB_CH_RED] = {
            .name = "led_red",
            .init_state = SRV_SIGNAL_STATE_BREATHING,
            .write_output = led_red_write,
            .breath_cycle_ms = APP_RGB_BREATH_CYCLE_MS,
            .breath_min_duty = APP_RGB_BREATH_MIN_DUTY,
            .breath_max_duty = APP_RGB_BREATH_MAX_DUTY,
        },
    };

    for (uint32_t i = 0; i < APP_RGB_CH_NUM; i++) {
        const srv_signal_error_t err = srv_signal_register_static(&cfg[i], s_led[i]);
        if (err < 0) {
            APP_RGB_STATUS_LOG_E("LED %s 注册失败: %d", cfg[i].name, (int)err);
        }
    }

    s_initialized = true;

    APP_RGB_STATUS_LOG_I("RGB 状态指示灯初始化完成 (%u 路)", (unsigned)APP_RGB_CH_NUM);
}

void app_rgb_status_step(void)
{
    if (!s_initialized) {
        return;
    }
    srv_signal_task_refresh();
}

srv_signal_handle_t* app_rgb_status_get_led(app_rgb_channel_t ch)
{
    if (ch >= APP_RGB_CH_NUM) {
        return NULL;
    }
    return s_led[ch];
}

bool app_rgb_status_set_state(app_rgb_channel_t ch, srv_signal_state_t state)
{
    if (ch >= APP_RGB_CH_NUM || !s_initialized || !s_led[ch]) {
        return false;
    }
    if (state >= SRV_SIGNAL_STATE_MAX) {
        return false;
    }

    srv_signal_set_state(s_led[ch], state);
    return true;
}

bool app_rgb_status_set_blink(app_rgb_channel_t ch, uint16_t cycle_ms,
    uint16_t wait_ms, uint16_t counts)
{
    if (ch >= APP_RGB_CH_NUM || !s_initialized || !s_led[ch]) {
        return false;
    }

    const srv_signal_cmd_t cmd = {
        .blink_cycle_ms = cycle_ms,
        .blink_wait_ms = wait_ms,
        .blink_code_counts = counts,
    };

    /* 先下发闪烁参数，再切换 BLINK_CODE（FIFO 顺序保证参数先生效） */
    if (srv_signal_set_blink_interval(s_led[ch], &cmd) < 0) {
        return false;
    }
    srv_signal_set_state(s_led[ch], SRV_SIGNAL_STATE_BLINK_CODE);
    return true;
}

/* Private functions ---------------------------------------------------------*/

static void led_blue_write(uint16_t value)
{
    (void)drv_pwm_set_duty(DRV_PWM_TIM1_CH1, value);
}

static void led_green_write(uint16_t value)
{
    (void)drv_pwm_set_duty(DRV_PWM_TIM4_CH1, value);
}

static void led_red_write(uint16_t value)
{
    (void)drv_pwm_set_duty(DRV_PWM_TIM4_CH2, value);
}
