/**
 * @file    srv_dm4310_ctrl.c
 * @author  maximillian
 * @version V4.0.0
 * @date    2026-08-31
 * @brief   DM4310 电机控制服务（单电机，MIT 模式，fsm 状态机）
 * @attention
 *
 * 使用 public_layer fsm 库管理电机状态：
 *   UNINIT → INIT（参数配置完成）→ ENABLED（使能运行）→ DISABLED
 * 状态副作用集中在 entry 回调：进 INIT 配置默认参数、进 ENABLED 使能+下发、
 * 进 DISABLED 禁用+清参。发送走 dev_dm4310（经回调桥接 drv_can 队列），
 * 接收走 drv_can_rx_pop 轮询。
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_dm4310_ctrl.h"

#include "drv_can.h"
#include "drv_systick.h"
#include "fsm.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_DM4310_CTRL_LOG_ENABLE 1

#if SRV_DM4310_CTRL_LOG_ENABLE
#define SRV_DM4310_CTRL_LOG_E(...) LOG_E("srv_dm4310_ctrl", __VA_ARGS__)
#define SRV_DM4310_CTRL_LOG_W(...) LOG_W("srv_dm4310_ctrl", __VA_ARGS__)
#define SRV_DM4310_CTRL_LOG_I(...) LOG_I("srv_dm4310_ctrl", __VA_ARGS__)
#define SRV_DM4310_CTRL_LOG_D(...) LOG_D("srv_dm4310_ctrl", __VA_ARGS__)
#else
#define SRV_DM4310_CTRL_LOG_E(...) ((void)0)
#define SRV_DM4310_CTRL_LOG_W(...) ((void)0)
#define SRV_DM4310_CTRL_LOG_I(...) ((void)0)
#define SRV_DM4310_CTRL_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief MIT 默认刚度/阻尼（合适初值，可按需调整） */
#define SRV_DM4310_DEFAULT_KP (10.0f)
#define SRV_DM4310_DEFAULT_KD (0.5f)

/** @brief 扭矩给定限幅 (±N·m)，保护电机/负载 */
#define SRV_DM4310_TORQUE_LIMIT_NM (0.1f)

/** @brief RX 数据日志限频窗口 (ms)：CAN 反馈 1kHz 下防刷屏 */
#define SRV_DM4310_RX_LOG_PERIOD_MS (500U)

/** @brief TX 数据日志限频窗口 (ms)：实时 500Hz 发送防刷屏 */
#define SRV_DM4310_TX_LOG_PERIOD_MS (500U)

/** @brief TX 失败日志限频窗口 (ms) */
#define SRV_DM4310_TX_ERR_LOG_PERIOD_MS (1000U)

/** @brief 实时 MIT 控制帧发送周期 (ms)：ENABLED 状态下限频发送 */
#define SRV_DM4310_CTRL_SEND_PERIOD_MS (100U)

/** @brief 初始化到使能命令的等待延时 (ms)：等待电机/总线稳定 */
#define SRV_DM4310_INIT_ENABLE_DELAY_MS (1500U)

/** @brief 使能帧发出后到开始控制帧的延时 (ms) */
#define SRV_DM4310_ENABLE_CTRL_DELAY_MS (500U)

/* Private variables ---------------------------------------------------------*/

static fsm_t s_fsm;
static fsm_handler_t s_handlers[SRV_DM4310_STATE_MAX];
static fsm_guard_t s_transitions[SRV_DM4310_STATE_MAX * SRV_DM4310_STATE_MAX];
static const char* s_state_names[SRV_DM4310_STATE_MAX] = {
    "UNINIT", "INIT", "ENABLED", "DISABLED",
};

static motor_t s_motor[MOTOR_NUM];
static bool s_init_request;
static bool s_enable_request;
static bool s_disable_request;
static bool s_enable_frame_sent; /**< 使能帧已发送完成标志（发送控制帧前置条件） */
static uint32_t s_enable_frame_ts; /**< 使能帧发出时间戳 (ms) */
static uint32_t s_rx_log_ts; /**< 上次 RX 数据日志时间戳 (ms) */
static uint32_t s_tx_log_ts; /**< 上次 TX 数据日志时间戳 (ms) */
static uint32_t s_tx_err_log_ts; /**< 上次 TX 失败日志时间戳 (ms) */
static uint32_t s_ctrl_send_ts; /**< 上次 MIT 控制帧发送时间戳 (ms) */
static uint32_t s_init_entry_ts; /**< 进入 INIT 状态时间戳 (ms) */

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t dm4310_uninit_handler(fsm_t* ctx);

static fsm_state_t dm4310_init_handler(fsm_t* ctx);

static fsm_state_t dm4310_enabled_handler(fsm_t* ctx);

static fsm_state_t dm4310_disabled_handler(fsm_t* ctx);

static void dm4310_on_entry(fsm_t* ctx, fsm_state_t state);

static void srv_dm4310_can_send(uint16_t id, const uint8_t* data, uint8_t len);

/* Exported functions --------------------------------------------------------*/

void srv_dm4310_ctrl_init(void)
{
    memset(&s_motor[MOTOR_1], 0, sizeof(s_motor[MOTOR_1]));

    s_motor[MOTOR_1].id = 0x01; /* 收发 CAN ID 均为 0x01（MIT 模式：ID + 0x000） */

    /* 注册 CAN 发送回调（桥接到 drv_can 队列发送，与驱动解耦） */
    dm4310_can_send_register(srv_dm4310_can_send);

    /* FSM 配置：全连通转换，状态副作用在 entry 回调 */
    memset(s_handlers, 0, sizeof(s_handlers));
    memset(s_transitions, 0, sizeof(s_transitions));

    s_handlers[SRV_DM4310_STATE_UNINIT] = dm4310_uninit_handler;
    s_handlers[SRV_DM4310_STATE_INIT] = dm4310_init_handler;
    s_handlers[SRV_DM4310_STATE_ENABLED] = dm4310_enabled_handler;
    s_handlers[SRV_DM4310_STATE_DISABLED] = dm4310_disabled_handler;

    fsm_config_t fsm_cfg = {
        .handlers = s_handlers,
        .transitions = s_transitions,
        .state_count = SRV_DM4310_STATE_MAX,
        .entry_cb = dm4310_on_entry,
        .exit_cb = NULL,
        .state_names = s_state_names,
        .user_data = NULL,
    };
    fsm_fill(&fsm_cfg, fsm_always_true);
    (void)fsm_init(&s_fsm, SRV_DM4310_STATE_UNINIT, &fsm_cfg);

    s_init_request = false;
    s_enable_request = false;
    s_disable_request = false;

    /* 默认流程：发起初始化，随后默认使能（INIT → ENABLED） */
    s_init_request = true;
    s_enable_request = true;

    SRV_DM4310_CTRL_LOG_I("DM4310 服务初始化完成 (ID=0x%02X, MIT)",
        (unsigned)s_motor[MOTOR_1].id);

        delay_ms(2000);
}

srv_dm4310_state_t srv_dm4310_ctrl_get_state(void)
{
    return (srv_dm4310_state_t)fsm_current_state(&s_fsm);
}

void srv_dm4310_ctrl_step(void)
{
    (void)fsm_step(&s_fsm);

    /* ENABLED 状态：先确认使能帧已发出，再按周期发送 MIT 控制帧 */
    const uint32_t now_ms = millis();

    if (fsm_current_state(&s_fsm) == SRV_DM4310_STATE_ENABLED) {
        /* 使能帧未发出：等 TX 队列排空且邮箱空闲（enable 帧已上总线） */
        if (!s_enable_frame_sent) {
            if (drv_can_tx_pending(DRV_CAN_CH_1) == 0U
                && drv_can_tx_all_done(DRV_CAN_CH_1)) {
                if (s_enable_frame_ts == 0U) {
                    s_enable_frame_ts = now_ms;
                } else if ((uint32_t)(now_ms - s_enable_frame_ts)
                    >= SRV_DM4310_ENABLE_CTRL_DELAY_MS) {
                    s_enable_frame_sent = true;
                    s_ctrl_send_ts = now_ms;
                    SRV_DM4310_CTRL_LOG_I("使能帧已发送 %ums，开始下发控制帧",
                        (unsigned)SRV_DM4310_ENABLE_CTRL_DELAY_MS);
                }
            }
        } else if ((uint32_t)(now_ms - s_ctrl_send_ts) >= SRV_DM4310_CTRL_SEND_PERIOD_MS) {
            s_ctrl_send_ts = now_ms;
            dm4310_ctrl_send(&s_motor[MOTOR_1]);
        }
    }

    /* 每 100ms 上报一次电机实时状态（位置/速度/扭矩/温度） */
    // if ((uint32_t)(now_ms - s_status_report_ts) >= SRV_DM4310_STATUS_REPORT_PERIOD_MS) {
    //     s_status_report_ts = now_ms;
    //     const motor_fbpara_t* fb = &s_motor[MOTOR_1].para;
    //     SRV_DM4310_CTRL_LOG_I("状态: pos=%.2f vel=%.2f tor=%.3f Tmos=%.1f Tcoil=%.1f",
    //         (float)fb->pos, (float)fb->vel, (float)fb->tor,
    //         (float)fb->Tmos, (float)fb->Tcoil);
    // }
}

void srv_dm4310_ctrl_enable(void)
{
    if (srv_dm4310_ctrl_get_state() == SRV_DM4310_STATE_UNINIT) {
        return;
    }
    s_enable_request = true;
    s_disable_request = false;
}

void srv_dm4310_ctrl_disable(void)
{
    if (srv_dm4310_ctrl_get_state() == SRV_DM4310_STATE_UNINIT) {
        return;
    }
    s_disable_request = true;
    s_enable_request = false;
}

void srv_dm4310_ctrl_pos_step(float delta_pos)
{
    /* 仅 ENABLED 且使能帧已发出后才允许下发控制帧 */
    if (srv_dm4310_ctrl_get_state() != SRV_DM4310_STATE_ENABLED
        || !s_enable_frame_sent) {
        return;
    }

    s_motor[MOTOR_1].cmd.pos_set += delta_pos;
    dm4310_set(&s_motor[MOTOR_1]);
    dm4310_ctrl_send(&s_motor[MOTOR_1]);

    SRV_DM4310_CTRL_LOG_I("位置微调 %+.2f rad → 当前给定 %.2f rad",
        (float)delta_pos, (float)s_motor[MOTOR_1].ctrl.pos_set);
}

void srv_dm4310_ctrl_set_target(float pos, float vel, float kp, float kd, float tor)
{
    if (srv_dm4310_ctrl_get_state() == SRV_DM4310_STATE_UNINIT) {
        return;
    }

    /* 扭矩限幅 ±SRV_DM4310_TORQUE_LIMIT_NM */
    if (tor > SRV_DM4310_TORQUE_LIMIT_NM) {
        tor = SRV_DM4310_TORQUE_LIMIT_NM;
    } else if (tor < -SRV_DM4310_TORQUE_LIMIT_NM) {
        tor = -SRV_DM4310_TORQUE_LIMIT_NM;
    }

    s_motor[MOTOR_1].cmd.pos_set = pos;
    s_motor[MOTOR_1].cmd.vel_set = vel;
    s_motor[MOTOR_1].cmd.kp_set = kp;
    s_motor[MOTOR_1].cmd.kd_set = kd;
    s_motor[MOTOR_1].cmd.tor_set = tor;

    dm4310_set(&s_motor[MOTOR_1]);
}

void srv_dm4310_ctrl_send(void)
{
    /* 仅 ENABLED 且使能帧已发出后才允许下发控制帧 */
    if (srv_dm4310_ctrl_get_state() != SRV_DM4310_STATE_ENABLED
        || !s_enable_frame_sent) {
        return;
    }
    dm4310_ctrl_send(&s_motor[MOTOR_1]);
}

void srv_dm4310_ctrl_poll_rx(void)
{
    drv_can_msg_t msg;
    while (drv_can_rx_pending(DRV_CAN_CH_1) > 0U) {
        if (!drv_can_rx_pop(DRV_CAN_CH_1, &msg)) {
            break;
        }
        srv_dm4310_ctrl_feed(&msg);
    }
}

void srv_dm4310_ctrl_feed(const drv_can_msg_t* msg)
{
    if (msg == NULL) {
        return;
    }

    /* MIT 模式：反馈帧 ID = 电机 ID；按 ID 分发到对应电机槽位 */
    for (motor_num_t i = MOTOR_1; i < MOTOR_NUM; i++) {
        if (msg->id == (uint32_t)s_motor[i].id) {
            dm4310_fbdata(&s_motor[i], (uint8_t*)msg->data);

            /* RX 数据日志（限频） */
            const uint32_t now_ms = millis();
            if ((uint32_t)(now_ms - s_rx_log_ts) >= SRV_DM4310_RX_LOG_PERIOD_MS) {
                s_rx_log_ts = now_ms;
                SRV_DM4310_CTRL_LOG_D("RX id=0x%03X dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
                    (unsigned)msg->id, (unsigned)msg->dlc,
                    (unsigned)msg->data[0], (unsigned)msg->data[1],
                    (unsigned)msg->data[2], (unsigned)msg->data[3],
                    (unsigned)msg->data[4], (unsigned)msg->data[5],
                    (unsigned)msg->data[6], (unsigned)msg->data[7]);
            }
            break;
        }
    }
}

motor_t* srv_dm4310_ctrl_get_motor(void)
{
    return &s_motor[MOTOR_1];
}

/* Private functions ---------------------------------------------------------*/

/* ===== FSM handlers ===== */

static fsm_state_t dm4310_uninit_handler(fsm_t* ctx)
{
    (void)ctx;
    if (s_init_request) {
        s_init_request = false;
        return SRV_DM4310_STATE_INIT;
    }
    return SRV_DM4310_STATE_UNINIT;
}

/**
 * @brief INIT 状态：等待 SRV_DM4310_INIT_ENABLE_DELAY_MS 后，有待使能请求 → ENABLED
 */
static fsm_state_t dm4310_init_handler(fsm_t* ctx)
{
    (void)ctx;

    /* 初始化后等待电机/总线稳定再发使能命令 */
    if ((uint32_t)(millis() - s_init_entry_ts) < SRV_DM4310_INIT_ENABLE_DELAY_MS) {
        return SRV_DM4310_STATE_INIT;
    }

    if (s_enable_request) {
        s_enable_request = false;
        return SRV_DM4310_STATE_ENABLED;
    }
    return SRV_DM4310_STATE_INIT;
}

/**
 * @brief ENABLED 状态：有待禁用请求 → DISABLED
 */
static fsm_state_t dm4310_enabled_handler(fsm_t* ctx)
{
    (void)ctx;
    if (s_disable_request) {
        s_disable_request = false;
        return SRV_DM4310_STATE_DISABLED;
    }
    return SRV_DM4310_STATE_ENABLED;
}

/**
 * @brief DISABLED 状态：有待使能请求 → ENABLED
 */
static fsm_state_t dm4310_disabled_handler(fsm_t* ctx)
{
    (void)ctx;
    if (s_enable_request) {
        s_enable_request = false;
        return SRV_DM4310_STATE_ENABLED;
    }
    return SRV_DM4310_STATE_DISABLED;
}

/* ===== 状态副作用 ===== */

/**
 * @brief 状态进入回调：执行参数配置/使能/禁用等副作用
 */
static void dm4310_on_entry(fsm_t* ctx, fsm_state_t state)
{
    switch ((srv_dm4310_state_t)state) {
    case SRV_DM4310_STATE_UNINIT:
        break;

    case SRV_DM4310_STATE_INIT:
        /* 配置 MIT 初始参数：位置 0、速度 0、刚度/阻尼给定、扭矩 0 */
        s_motor[MOTOR_1].ctrl.mode = MIT_MODE;
        s_motor[MOTOR_1].cmd.mode = MIT_MODE;
        s_motor[MOTOR_1].cmd.pos_set = 0.0f;
        s_motor[MOTOR_1].cmd.vel_set = 0.0f;
        s_motor[MOTOR_1].cmd.kp_set = SRV_DM4310_DEFAULT_KP;
        s_motor[MOTOR_1].cmd.kd_set = SRV_DM4310_DEFAULT_KD;
        s_motor[MOTOR_1].cmd.tor_set = 0.0f;
        dm4310_set(&s_motor[MOTOR_1]);
        s_init_entry_ts = millis(); /* 记录进入 INIT 时间，供使能延时 */
        break;

    case SRV_DM4310_STATE_ENABLED:
        s_motor[MOTOR_1].start_flag = 1;
        /* 先发使能帧（FF...FC），确认发出后才能发控制帧 */
        dm4310_enable(&s_motor[MOTOR_1]);
        s_enable_frame_sent = false;
        s_enable_frame_ts = 0;
        s_ctrl_send_ts = 0;

        break;

    case SRV_DM4310_STATE_DISABLED:
        s_motor[MOTOR_1].start_flag = 0;
        s_enable_frame_sent = false;
        dm4310_disable(&s_motor[MOTOR_1]);
        break;

    default:
        break;
    }

    SRV_DM4310_CTRL_LOG_I("状态切换 → %s", fsm_name(ctx, state));
}

/**
 * @brief DM4310 CAN 发送回调：桥接到 drv_can 队列发送（与驱动解耦）
 */
static void srv_dm4310_can_send(uint16_t id, const uint8_t* data, uint8_t len)
{
    drv_can_msg_t msg;
    msg.id = id;
    msg.is_extended = false;
    msg.dlc = len;
    if (len > 8U) {
        len = 8U;
    }
    memcpy(msg.data, data, len);

    const drv_can_error_t err = drv_can_send(DRV_CAN_CH_1, &msg);
    const uint32_t now_ms = millis();

    if (err == DRV_CAN_OK) {
        /* TX 数据日志（限频，实时发送下防刷屏） */
        if ((uint32_t)(now_ms - s_tx_log_ts) >= SRV_DM4310_TX_LOG_PERIOD_MS) {
            s_tx_log_ts = now_ms;
            SRV_DM4310_CTRL_LOG_D("TX id=0x%03X dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X",
                (unsigned)id, (unsigned)len,
                (unsigned)msg.data[0], (unsigned)msg.data[1],
                (unsigned)msg.data[2], (unsigned)msg.data[3],
                (unsigned)msg.data[4], (unsigned)msg.data[5],
                (unsigned)msg.data[6], (unsigned)msg.data[7]);
        }
    } else {
        /* TX 失败限频（队列满 / 未初始化） */
        if ((uint32_t)(now_ms - s_tx_err_log_ts) >= SRV_DM4310_TX_ERR_LOG_PERIOD_MS) {
            s_tx_err_log_ts = now_ms;
            SRV_DM4310_CTRL_LOG_E("TX 失败: err=%d id=0x%03X dlc=%u",
                (int)err, (unsigned)id, (unsigned)len);
        }
    }
}
