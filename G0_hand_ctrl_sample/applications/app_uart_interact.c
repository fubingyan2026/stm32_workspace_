/**
 * @file    app_uart_interact.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-31
 * @brief   应用层 — UART 交互实现（收发集中管理：心跳/按键/电机控制与反馈）
 */

/* Includes ------------------------------------------------------------------*/
#include "app_uart_interact.h"

#include "app_rgb_status.h"
#include "drv_key.h"
#include "drv_systick.h"
#include "key_base.h"
#include "log.h"
#include "srv_dm4310_ctrl.h"
#include "srv_key.h"
#include "srv_uart_rx_cmd.h"
#include "srv_uart_tx_cmd.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define APP_UART_INTERACT_LOG_ENABLE 1

#if APP_UART_INTERACT_LOG_ENABLE
#define APP_UART_INTERACT_LOG_E(...) LOG_E("app_uart_interact", __VA_ARGS__)
#define APP_UART_INTERACT_LOG_W(...) LOG_W("app_uart_interact", __VA_ARGS__)
#define APP_UART_INTERACT_LOG_I(...) LOG_I("app_uart_interact", __VA_ARGS__)
#define APP_UART_INTERACT_LOG_D(...) LOG_D("app_uart_interact", __VA_ARGS__)
#else
#define APP_UART_INTERACT_LOG_E(...) ((void)0)
#define APP_UART_INTERACT_LOG_W(...) ((void)0)
#define APP_UART_INTERACT_LOG_I(...) ((void)0)
#define APP_UART_INTERACT_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define APP_UART_KEY_EVENT_PAYLOAD_LEN (2U)
#define APP_UART_HEARTBEAT_PAYLOAD_LEN (2U)
#define APP_UART_MOTOR_TARGET_PAYLOAD_LEN (20U) /**< pos/vel/kp/kd/tor × 4B */
#define APP_UART_MOTOR_FEEDBACK_PAYLOAD_LEN (21U) /**< state(1) + pos/vel/tor/Tmos/Tcoil × 4B */

#define APP_UART_HEARTBEAT_INTERVAL_MS (1000U)
#define APP_UART_MOTOR_PRINT_PERIOD_MS (1000U)

/** @brief 双键组合长按时间 (ms)：KEY1+KEY2 同时按住触发电机保存零点 */
#define APP_UART_COMBO_SAVE_ZERO_MS (3000U)

/** @brief 电机反馈请求最小回复间隔 (ms)：丢弃窗口内重复请求，防止 TX 打满 */
#define APP_UART_FEEDBACK_REPLY_MIN_MS (1U)

/* Private variables ---------------------------------------------------------*/

static uint32_t s_last_heartbeat_ms;
static uint32_t s_last_motor_print_ms;
static uint32_t s_last_feedback_reply_ms;
static uint16_t s_heartbeat_tick;
static uint32_t s_combo_press_ts; /**< 双键同时按住起始时间戳 (0=未同时按下) */
static bool s_combo_triggered; /**< 组合长按已触发标志（防重复触发） */

/* Private function prototypes -----------------------------------------------*/

static void app_uart_key_event_cb(const char* name, key_base_event_t event);

static uint8_t app_uart_key_index(const char* name);

static void app_uart_rx_cmd_cb(uint8_t cmd, const uint8_t* data, uint8_t data_len);

static void app_uart_send_heartbeat(void);

static void app_uart_motor_send_feedback(void);

static void app_uart_motor_print_status(void);

static void app_uart_led_handle_cmd(const uint8_t* data, uint8_t data_len);

static void app_uart_combo_save_zero_check(void);

/* Exported functions --------------------------------------------------------*/

void app_uart_interact_init(void)
{
    srv_uart_tx_cmd_init();
    srv_uart_rx_cmd_init(app_uart_rx_cmd_cb);
    srv_key_register_event_cb(app_uart_key_event_cb);

    s_last_heartbeat_ms = millis();
    s_last_feedback_reply_ms = 0;
    s_heartbeat_tick = 0;

    APP_UART_INTERACT_LOG_I("UART 交互应用初始化完成");
}

void app_uart_interact_step(void)
{
    const uint32_t now_ms = millis();

    if ((uint32_t)(now_ms - s_last_heartbeat_ms) >= APP_UART_HEARTBEAT_INTERVAL_MS) {
        s_last_heartbeat_ms = now_ms;
        app_uart_send_heartbeat();
    }

    if ((uint32_t)(now_ms - s_last_motor_print_ms) >= APP_UART_MOTOR_PRINT_PERIOD_MS) {
        s_last_motor_print_ms = now_ms;
        app_uart_motor_print_status();
    }

    /* 双键组合长按检测（KEY1+KEY2 同时按住 3s → 保存电机零点） */
    app_uart_combo_save_zero_check();
}

/* Private functions ---------------------------------------------------------*/
static void app_uart_key_event_cb(const char* name, key_base_event_t event)
{
    /* 仅上报按下/松开边沿事件，其余事件不处理 */
    if (event != KEY_BASE_EVENT_PRESS && event != KEY_BASE_EVENT_RELEASE) {
        return;
    }

    /* 按下事件：KEY1 位置增加，KEY2 位置减小 */
    // if (event == KEY_BASE_EVENT_PRESS) {
    //     const uint8_t idx = app_uart_key_index(name);
    //     if (idx == 0U) {
    //         srv_dm4310_ctrl_pos_step(SRV_DM4310_POS_STEP_RAD);
    //     } else if (idx == 1U) {
    //         srv_dm4310_ctrl_pos_step(-SRV_DM4310_POS_STEP_RAD);
    //     }
    // }

    uint8_t payload[APP_UART_KEY_EVENT_PAYLOAD_LEN];
    payload[0] = app_uart_key_index(name);
    payload[1] = (uint8_t)event;

    const srv_uart_tx_cmd_error_t err = srv_uart_tx_cmd_send(APP_UART_CMD_KEY_EVENT, payload, APP_UART_KEY_EVENT_PAYLOAD_LEN);
    if (err != SRV_UART_TX_CMD_OK) {
        APP_UART_INTERACT_LOG_W("按键事件上报失败: %d (%s) key=%s event=%u",
            (int)err, srv_uart_tx_cmd_err_str(err),
            name ? name : "?", (unsigned)event);
    }
}

static uint8_t app_uart_key_index(const char* name)
{
    return (name != NULL && strcmp(name, "key2") == 0) ? 1U : 0U;
}

static void app_uart_rx_cmd_cb(uint8_t cmd, const uint8_t* data, uint8_t data_len)
{
    switch (cmd) {
    case APP_UART_CMD_MOTOR_SET_TARGET: {
        if (data == NULL || data_len < APP_UART_MOTOR_TARGET_PAYLOAD_LEN) {
            APP_UART_INTERACT_LOG_W("电机设置目标命令负载不足 (%u)", (unsigned)data_len);
            break;
        }

        float pos, vel, kp, kd, tor;
        memcpy(&pos, &data[0], 4);
        memcpy(&vel, &data[4], 4);
        memcpy(&kp, &data[8], 4);
        memcpy(&kd, &data[12], 4);
        memcpy(&tor, &data[16], 4);

        srv_dm4310_ctrl_set_target(pos, vel, kp, kd, tor);
        break;
    }

    case APP_UART_CMD_MOTOR_REQ_FEEDBACK:
        app_uart_motor_send_feedback();
        break;

    case APP_UART_CMD_LED_CTRL:
        app_uart_led_handle_cmd(data, data_len);
        break;

    default:
        APP_UART_INTERACT_LOG_D("收到未处理命令 cmd=0x%02X data_len=%u",
            (unsigned)cmd, (unsigned)data_len);
        break;
    }
}

static void app_uart_send_heartbeat(void)
{
    uint8_t payload[APP_UART_HEARTBEAT_PAYLOAD_LEN];
    payload[0] = (uint8_t)(s_heartbeat_tick & 0xFFU);
    payload[1] = (uint8_t)((s_heartbeat_tick >> 8) & 0xFFU);

    if (srv_uart_tx_cmd_send(APP_UART_CMD_HEARTBEAT, payload,
            APP_UART_HEARTBEAT_PAYLOAD_LEN)
        == SRV_UART_TX_CMD_OK) {
        s_heartbeat_tick++;
    }
}

/* ===== DM4310 电机命令 ===== */

static void app_uart_motor_send_feedback(void)
{
    /* 回复节流：请求反馈最小间隔 5ms（上限 200 帧/s），丢弃窗口内重复请求，
       避免启动/高频请求瞬间把 TX 队列打满；上位机 5ms+ 周期请求不受影响 */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_last_feedback_reply_ms) < APP_UART_FEEDBACK_REPLY_MIN_MS) {
        return;
    }

    motor_t* motor = srv_dm4310_ctrl_get_motor(MOTOR_1);
    if (motor == NULL) {
        return;
    }

    uint8_t payload[APP_UART_MOTOR_FEEDBACK_PAYLOAD_LEN];
    payload[0] = (uint8_t)(motor->para.state & 0xFFU);
    memcpy(&payload[1], &motor->para.pos, 4);
    memcpy(&payload[5], &motor->para.vel, 4);
    memcpy(&payload[9], &motor->para.tor, 4);
    memcpy(&payload[13], &motor->para.Tmos, 4);
    memcpy(&payload[17], &motor->para.Tcoil, 4);

    const srv_uart_tx_cmd_error_t err = srv_uart_tx_cmd_send(APP_UART_CMD_MOTOR_FEEDBACK_REPORT, payload,
        APP_UART_MOTOR_FEEDBACK_PAYLOAD_LEN);
    if (err == SRV_UART_TX_CMD_OK) {
        s_last_feedback_reply_ms = now_ms;
    } else {
        APP_UART_INTERACT_LOG_W("电机反馈上报失败:(%s)", srv_uart_tx_cmd_err_str(err));
    }
}

static void app_uart_motor_print_status(void)
{
    motor_t* motor = srv_dm4310_ctrl_get_motor(MOTOR_1);
    if (motor == NULL) {
        return;
    }

    // const motor_fbpara_t* fb = &motor->para;

    // /* state 单独一帧，打印数值 + 含义 */
    // static const char* const state_desc[] = {
    //     "失能", "使能", "电机侧未识别", "输出轴未识别",
    //     "未知", "读取编码器错误", "未知", "读取编码器错误",
    //     "超压", "欠压", "过电流", "MOS过温",
    //     "电机线圈过温", "通讯丢失", "未知", "过载",
    // };
    // const uint8_t st = (uint8_t)(fb->state & 0x0F);
    // const char* desc = (st < 16U) ? state_desc[st] : "未知";

    // APP_UART_INTERACT_LOG_I("电机状态 state=%d (%s)", (int)fb->state, desc);

    // /* 物理量单独一帧 */
    // APP_UART_INTERACT_LOG_I("电机反馈 pos=%.3f vel=%.3f tor=%.3f Tmos=%.3f Tcoil=%.3f",
    //     fb->pos, fb->vel, fb->tor, fb->Tmos, fb->Tcoil);
}

/* ===== LED 控制命令 ===== */

/**
 * @brief 处理 LED 控制命令
 * @param data     负载：ch(1) + action(1) [+ action 参数]
 * @param data_len 负载长度
 * @note  action: OFF/ON/BREATH 无参数；BLINK 后随 2B 间隔 ms（小端）
 */
static void app_uart_led_handle_cmd(const uint8_t* data, uint8_t data_len)
{
    if (data == NULL || data_len < 2U) {
        APP_UART_INTERACT_LOG_W("LED 控制命令负载不足 (%u)", (unsigned)data_len);
        return;
    }

    const uint8_t ch = data[0];
    const uint8_t action = data[1];
    if (ch >= APP_RGB_CH_NUM) {
        APP_UART_INTERACT_LOG_W("LED 控制命令非法通道: %u", (unsigned)ch);
        return;
    }

    switch (action) {
    case APP_UART_LED_ACTION_OFF:
        (void)app_rgb_status_set_state((app_rgb_channel_t)ch, SRV_SIGNAL_STATE_OFF);
        break;

    case APP_UART_LED_ACTION_ON:
        (void)app_rgb_status_set_state((app_rgb_channel_t)ch, SRV_SIGNAL_STATE_ON);
        break;

    case APP_UART_LED_ACTION_BLINK:
        if (data_len < 4U) {
            APP_UART_INTERACT_LOG_W("LED 闪烁命令负载不足 (%u)", (unsigned)data_len);
            break;
        }
        {
            const uint16_t interval_ms = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
            (void)app_rgb_status_set_blink((app_rgb_channel_t)ch,
                interval_ms, 0U, 0U);
        }
        break;

    case APP_UART_LED_ACTION_BREATH:
        /* 显式重置呼吸计时/相位后切换，保证从 ON/OFF/BLINK 切回立即呼吸 */
        (void)app_rgb_status_set_breath((app_rgb_channel_t)ch, 0U);
        break;

    default:
        APP_UART_INTERACT_LOG_W("LED 控制命令非法 action: %u", (unsigned)action);
        break;
    }
}

/**
 * @brief 双键组合长按检测：KEY1+KEY2 同时按住 3s → 保存电机当前位置为零点
 */
static void app_uart_combo_save_zero_check(void)
{
    const bool k1 = drv_key_is_pressed(DRV_KEY_CH_1);
    const bool k2 = drv_key_is_pressed(DRV_KEY_CH_2);

    if (k1 && k2) {
        if (s_combo_press_ts == 0U) {
            s_combo_press_ts = millis();
            s_combo_triggered = false;
        } else if (!s_combo_triggered
            && (uint32_t)(millis() - s_combo_press_ts) >= APP_UART_COMBO_SAVE_ZERO_MS) {
            s_combo_triggered = true;
            srv_dm4310_ctrl_save_zero();
        }
    } else {
        /* 任一键松开：复位组合计时 */
        s_combo_press_ts = 0U;
        s_combo_triggered = false;
    }
}
