/**
 * @file    srv_pa430_temp_test.c
 * @brief   Motorevo (PA430) CAN FD 伺服电机 — MIT 模式速度耐久测试实现
 *
 * MIT 速度控制：令 Kp=0、T_ref=0，则 T_out = Kd×(V_ref − V)（速度 P 环），
 * 通过广播控制帧 0x20（DLC 64，槽 (ID-1)*8，大端）下发 V_ref=目标速度。
 * 时序：正转 30s → 停 30s → 反转 30s → 停 30s 为一个循环，持续在线累计满 24h
 * 自动停止；V_ref 变化用主机侧线性斜坡软启停。
 *
 * 安全：Control Mode(Index 11) 确认为 MIT 前不下发控制帧/不使能（防上电误执行）；
 * 就绪后才使能并下发控制。通道由 SRV_PA430_TEMP_CAN_CH 决定。
 */

#include "srv_pa430_temp_test.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 日志开关 */
#define SRV_PA430_TEMP_LOG_ENABLE 1
#if SRV_PA430_TEMP_LOG_ENABLE
#define SRV_PA430_TEMP_LOG_I(...) LOG_I("pa430_temp", __VA_ARGS__)
#define SRV_PA430_TEMP_LOG_W(...) LOG_W("pa430_temp", __VA_ARGS__)
#define SRV_PA430_TEMP_LOG_E(...) LOG_E("pa430_temp", __VA_ARGS__)
#else
#define SRV_PA430_TEMP_LOG_I(...) ((void)0)
#define SRV_PA430_TEMP_LOG_W(...) ((void)0)
#define SRV_PA430_TEMP_LOG_E(...) ((void)0)
#endif

/* --- 配置 --- */

/** @brief 广播总线最多电机数（ID 1~8） */
#define SRV_PA430_TEMP_MAX_MOTORS 8U
/** @brief 实际参与测试的电机数（1~8，须与总线一致） */
#define SRV_PA430_TEMP_MOTOR_COUNT 1U
/** @brief 上电自动启动 */
#define SRV_PA430_TEMP_AUTO_START 1

/* --- 运动参数（物理单位，内部换算 raw） --- */

/** @brief 正/反转目标速度（rad/s），停发 0 */
#define SRV_PA430_TEMP_SPEED_RADPS (3.0f)
/** @brief 速度环 Kd（MIT 阻尼，Nm/(rad/s)，0~50）：Kp=0 时 T=Kd×(V_ref−V) */
#define SRV_PA430_TEMP_KD_NMPRPDS (25.0f)
/** @brief 主机侧速度指令线性斜坡时长 (ms)（软启停） */
#define SRV_PA430_TEMP_RAMP_MS 2000U
/** @brief 每个阶段时长 (ms)：正转/停/反转/停 各 30s */
#define SRV_PA430_TEMP_PHASE_MS 6000U

/* --- 周期 --- */

/** @brief 广播控制帧重发周期 (ms)（有反馈时；MIT 需持续下发保持刚度） */
#define SRV_PA430_TEMP_CTRL_PERIOD_MS 5U
/** @brief 未收到反馈时的控制帧重发周期 (ms)（避免无电机总线上错误计数快速累计） */
#define SRV_PA430_TEMP_CTRL_PERIOD_IDLE_MS 100U
/** @brief 无反馈判定周期 (ms) */
#define SRV_PA430_TEMP_NORESP_PERIOD_MS 2000U
/** @brief 使能重试周期 (ms) */
#define SRV_PA430_TEMP_ENABLE_RETRY_MS 200U
/** @brief 使能确认超时 (ms) */
#define SRV_PA430_TEMP_ENABLE_TIMEOUT_MS 2000U
/** @brief 停机：速度回 0 后保持零速时长 (ms)（随后失能） */
#define SRV_PA430_TEMP_STOP_HOLD_MS 500U
/** @brief 稳态状态日志周期 (ms) */
#define SRV_PA430_TEMP_STATUS_LOG_MS 5000U
/** @brief 停机时速度回 0 的斜坡时长 (ms) */
#define SRV_PA430_TEMP_STOP_RAMP_MS 2000U
/** @brief 耐久时长 (ms)：持续在线累计满 24h 自动停止；0=禁用 */
#define SRV_PA430_TEMP_DURATION_MS 86400000U

/* --- 模式确认（Index 11 = Control Mode）周期 --- */

/** @brief 读 Control Mode 重发周期 (ms) */
#define SRV_PA430_TEMP_MODE_READ_RETRY_MS 500U
/** @brief 读 Control Mode 单轮超时 (ms)（超时保持未就绪并重试） */
#define SRV_PA430_TEMP_MODE_READ_TIMEOUT_MS 5000U
/** @brief 写模式后等待生效 (ms) */
#define SRV_PA430_TEMP_MODE_APPLY_MS 300U
/** @brief 校验读回超时 (ms) */
#define SRV_PA430_TEMP_MODE_VERIFY_TIMEOUT_MS 1000U

/* --- Motorevo 协议 --- */

#define SRV_PA430_TEMP_ID_STATUS 0x10U /**< 广播状态/使能帧（DLC 64） */
#define SRV_PA430_TEMP_ID_CTRL 0x20U /**< 广播控制帧（DLC 64） */
#define SRV_PA430_TEMP_CMD_ENABLE 0xFCU /**< 槽 byte7：使能 */
#define SRV_PA430_TEMP_CMD_DISABLE 0xFDU /**< 槽 byte7：失能 */
#define SRV_PA430_TEMP_CMD_NONE 0xFFU /**< 槽 byte7：无命令 */
#define SRV_PA430_TEMP_PARAM_ID_BASE 0x600U /**< 单机参数读写帧基址（+ID） */
#define SRV_PA430_TEMP_PARAM_HEAD 0x67U
#define SRV_PA430_TEMP_PARAM_TAIL 0x76U
#define SRV_PA430_TEMP_PARAM_RW_WRITE 0x15U
#define SRV_PA430_TEMP_PARAM_RW_READ 0x04U
#define SRV_PA430_TEMP_PARAM_INDEX_MODE 11U /**< Control Mode */
#define SRV_PA430_TEMP_MODE_MIT 0x02000000UL /**< Control Mode = MIT(2) */

/* 缩放常量（文档 §3.1，勿改） */
#define SRV_PA430_TEMP_THETA_MIN_RAD (-12.5f)
#define SRV_PA430_TEMP_THETA_MAX_RAD (12.5f)
#define SRV_PA430_TEMP_VEL_MIN_RADPS (-10.0f)
#define SRV_PA430_TEMP_VEL_MAX_RADPS (10.0f)
#define SRV_PA430_TEMP_KD_MAX_NMPRPDS (50.0f)
#define SRV_PA430_TEMP_TQ_RAW_0 0x0800U

/* --- 反馈错误码 --- */

typedef struct {
    uint16_t mask;
    const char* name;
} srv_pa430_temp_err_desc_t;

#define SRV_PA430_TEMP_ERR_ENABLE_BIT 0x0001U
#define SRV_PA430_TEMP_ERR_BIT_MASK 0xFFFEU
#define SRV_PA430_TEMP_ERR_NUM (sizeof(s_err_map) / sizeof(s_err_map[0]))

static const srv_pa430_temp_err_desc_t s_err_map[] = {
    { 0x0002U, "母线过压" },
    { 0x0004U, "相电流过流" },
    { 0x0008U, "线圈过温" },
    { 0x0010U, "电机超速" },
    { 0x0100U, "堵转" },
    { 0x0400U, "板温过温" },
    { 0x0800U, "母线欠压" },
    { 0x1000U, "位置超限" },
    { 0x2000U, "CAN 通信超时" },
};

/* --- 阶段 --- */

typedef enum {
    SRV_PA430_TEMP_PHASE_FORWARD = 0, /**< 正转 */
    SRV_PA430_TEMP_PHASE_DWELL_F, /**< 正转后停 */
    SRV_PA430_TEMP_PHASE_REVERSE, /**< 反转 */
    SRV_PA430_TEMP_PHASE_DWELL_R, /**< 反转后停 */
    SRV_PA430_TEMP_PHASE_NUM
} srv_pa430_temp_phase_t;

/* --- 模式确认状态 --- */

typedef enum {
    SRV_PA430_TEMP_MODE_READ = 0,
    SRV_PA430_TEMP_MODE_DISABLE,
    SRV_PA430_TEMP_MODE_WRITE,
    SRV_PA430_TEMP_MODE_WAIT_APPLY,
    SRV_PA430_TEMP_MODE_VERIFY_READ,
    SRV_PA430_TEMP_MODE_VERIFY_WAIT,
} srv_pa430_temp_mode_state_t;

/* Private variables ---------------------------------------------------------*/

static bool s_running;
static bool s_stopping;
static uint32_t s_stop_start_ms;

/** @brief 控制就绪：Control Mode 确认 MIT 前不下发控制帧/不使能 */
static bool s_ctrl_ready;
static bool s_enable_sent;

static srv_pa430_temp_phase_t s_phase;
static uint32_t s_phase_start_ms;
static uint32_t s_cycle;
/** @brief 是否已真正开始运动（控制就绪并发出使能后置位；阶段计时由此开始，保证首段完整 30s） */
static bool s_motion_started;

/* 主机侧速度斜坡 */
static float s_ramp_target_radps;
static float s_ramp_prev_radps; /**< 上次斜坡目标（检测变化 → 重建斜坡起点） */
static float s_ramp_from_radps;
static uint32_t s_ramp_start_ms;
static float s_cmd_speed_radps;

/* raw 常量（start 时换算） */
static uint16_t s_pos_mid;
static uint16_t s_vel_zero_raw;
static uint16_t s_kd_raw;

/* 电机状态 */
static const uint8_t s_motor_ids[SRV_PA430_TEMP_MAX_MOTORS] = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U };
static uint16_t s_motor_theta_raw[SRV_PA430_TEMP_MAX_MOTORS];
static uint16_t s_motor_vel_raw[SRV_PA430_TEMP_MAX_MOTORS];
static uint16_t s_motor_tq_raw[SRV_PA430_TEMP_MAX_MOTORS];
static uint16_t s_motor_temp_raw[SRV_PA430_TEMP_MAX_MOTORS];
static bool s_motor_have_fb[SRV_PA430_TEMP_MAX_MOTORS];
static bool s_fb_logged[SRV_PA430_TEMP_MAX_MOTORS];
static uint16_t s_motor_err[SRV_PA430_TEMP_MAX_MOTORS];
static bool s_motor_enabled[SRV_PA430_TEMP_MAX_MOTORS];
static bool s_enabled_prev[SRV_PA430_TEMP_MAX_MOTORS];
static bool s_err_pending[SRV_PA430_TEMP_MAX_MOTORS];
static uint32_t s_motor_last_seen_ms[SRV_PA430_TEMP_MAX_MOTORS];
static bool s_motor_nresp_latch[SRV_PA430_TEMP_MAX_MOTORS];
static bool s_online_evt_pending[SRV_PA430_TEMP_MAX_MOTORS];

static uint32_t s_last_ctrl_ms;
static uint32_t s_enable_retry_last_ms;
static uint32_t s_enable_stall_since_ms;
static bool s_enable_warned;
static uint32_t s_start_ms;
static uint32_t s_online_ms;
static uint32_t s_online_last_ms;
static uint32_t s_last_status_ms;

/* 模式确认 */
static srv_pa430_temp_mode_state_t s_mode_state;
static uint32_t s_mode_wait_ms;
static uint32_t s_mode_read_ms;
static uint32_t s_mode_since_ms;
static bool s_mode_val_have;
static uint32_t s_mode_val;

/* Private function prototypes -----------------------------------------------*/

static uint8_t srv_pa430_temp_find_idx(uint8_t addr);
static bool srv_pa430_temp_all_online(void);
static uint16_t srv_pa430_temp_vel_to_raw(float radps);
static uint16_t srv_pa430_temp_gain_to_raw(float val, float max);
static uint16_t srv_pa430_temp_theta_to_raw(float rad);
static void srv_pa430_temp_recalc_raw(void);
static void srv_pa430_temp_pack_mit(uint8_t* slot, uint16_t pos, uint16_t vel,
    uint16_t kp, uint16_t kd, uint16_t tq);
static bool srv_pa430_temp_send_enable(bool enable);
static void srv_pa430_temp_send_control(void);
static void srv_pa430_temp_send_param_read(uint8_t addr, uint8_t index);
static void srv_pa430_temp_send_param_write(uint8_t addr, uint8_t index, uint32_t value);
static void srv_pa430_temp_mode_step(uint32_t now);
static float srv_pa430_temp_phase_speed(srv_pa430_temp_phase_t phase);
static void srv_pa430_temp_ramp_update(uint32_t now);
static void srv_pa430_temp_err_print(uint8_t addr, uint16_t code);
static void srv_pa430_temp_log_phase(srv_pa430_temp_phase_t phase);
static float srv_pa430_temp_vel_raw_to_radps(uint16_t raw);

/* Exported functions --------------------------------------------------------*/

void srv_pa430_temp_test_init(void)
{
#if SRV_PA430_TEMP_AUTO_START
    srv_pa430_temp_test_start();
#endif
}

void srv_pa430_temp_test_start(void)
{
    s_running = true;
    s_stopping = false;
    s_stop_start_ms = 0;
    s_ctrl_ready = false;
    s_enable_sent = false;
    s_motion_started = false;

    s_start_ms = millis();
    s_online_ms = 0;
    s_online_last_ms = s_start_ms;
    s_last_status_ms = s_start_ms;

    srv_pa430_temp_recalc_raw();

    s_phase = SRV_PA430_TEMP_PHASE_FORWARD;
    s_phase_start_ms = s_start_ms;
    s_cycle = 0;
    s_ramp_target_radps = srv_pa430_temp_phase_speed(s_phase); /* 首个正转阶段目标=+SPEED（关键：否则首段不转） */
    s_ramp_prev_radps = 0.0f;
    s_ramp_from_radps = 0.0f;
    s_ramp_start_ms = s_start_ms;
    s_cmd_speed_radps = 0.0f;
    s_last_ctrl_ms = s_start_ms;

    memset(s_motor_theta_raw, 0, sizeof(s_motor_theta_raw));
    memset(s_motor_vel_raw, 0, sizeof(s_motor_vel_raw));
    memset(s_motor_tq_raw, 0, sizeof(s_motor_tq_raw));
    memset(s_motor_temp_raw, 0, sizeof(s_motor_temp_raw));
    memset(s_motor_have_fb, 0, sizeof(s_motor_have_fb));
    memset(s_fb_logged, 0, sizeof(s_fb_logged));
    memset(s_motor_err, 0, sizeof(s_motor_err));
    memset(s_motor_enabled, 0, sizeof(s_motor_enabled));
    memset(s_enabled_prev, 0, sizeof(s_enabled_prev));
    memset(s_err_pending, 0, sizeof(s_err_pending));
    memset(s_motor_nresp_latch, 0, sizeof(s_motor_nresp_latch));
    memset(s_online_evt_pending, 0, sizeof(s_online_evt_pending));
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MAX_MOTORS; i++) {
        s_motor_last_seen_ms[i] = s_start_ms;
    }

    s_enable_retry_last_ms = s_start_ms;
    s_enable_stall_since_ms = 0;
    s_enable_warned = false;

    /* 模式确认启动 */
    s_mode_state = SRV_PA430_TEMP_MODE_READ;
    s_mode_wait_ms = 0;
    s_mode_read_ms = 0;
    s_mode_since_ms = s_start_ms;
    s_mode_val_have = false;
    s_mode_val = 0;

    /* 安全：此处不使能、不发控制帧，先确认 Control Mode 为 MIT */
    SRV_PA430_TEMP_LOG_I("PA430 MIT 速度耐久启动：%u 台电机，目标 ±%d mrad/s，周期 正/停/反/停 各 %u s，%.0f h 自动停止（先确认 MIT 模式）",
        (unsigned)SRV_PA430_TEMP_MOTOR_COUNT,
        (int)(SRV_PA430_TEMP_SPEED_RADPS * 1000.0f),
        (unsigned)(SRV_PA430_TEMP_PHASE_MS / 1000U),
        (double)(SRV_PA430_TEMP_DURATION_MS / 3600000U));
}

void srv_pa430_temp_test_stop(void)
{
    if (!s_running || s_stopping) {
        return;
    }
    s_stopping = true;
    s_stop_start_ms = millis();
    s_ramp_target_radps = 0.0f; /* 速度斜坡回 0 后失能 */
    SRV_PA430_TEMP_LOG_I("PA430 MIT 速度耐久停止：速度斜坡回 0 后失能");
}

void srv_pa430_temp_test_step(void)
{
    if (!s_running) {
        return;
    }

    const uint32_t now = millis();

    /* 停机序列：斜坡回 0 → 保持零速 → 失能 */
    if (s_stopping) {
        if ((now - s_stop_start_ms) >= (SRV_PA430_TEMP_STOP_RAMP_MS + SRV_PA430_TEMP_STOP_HOLD_MS)) {
            (void)srv_pa430_temp_send_enable(false);
            s_running = false;
            s_stopping = false;
            SRV_PA430_TEMP_LOG_I("PA430 MIT 速度耐久已停止并失能（累计在线 %lu ms，循环 %u）",
                (unsigned long)s_online_ms, (unsigned)s_cycle);
        } else if ((now - s_last_ctrl_ms) >= SRV_PA430_TEMP_CTRL_PERIOD_MS) {
            s_last_ctrl_ms = now;
            srv_pa430_temp_ramp_update(now);
            srv_pa430_temp_send_control();
        }
        return;
    }

    /* 在线计时 */
    if (srv_pa430_temp_all_online()) {
        s_online_ms += (uint32_t)(now - s_online_last_ms);
    }
    s_online_last_ms = now;

    if ((SRV_PA430_TEMP_DURATION_MS != 0U) && (s_online_ms >= SRV_PA430_TEMP_DURATION_MS)) {
        SRV_PA430_TEMP_LOG_I("累计在线 %lu ms（24h）已到，自动停止",
            (unsigned long)s_online_ms);
        srv_pa430_temp_test_stop();
        return;
    }

    /* 错误打印 */
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (s_err_pending[i]) {
            s_err_pending[i] = false;
            if (s_motor_err[i] != 0U) {
                srv_pa430_temp_err_print(s_motor_ids[i], s_motor_err[i]);
            } else {
                SRV_PA430_TEMP_LOG_W("电机 ID=%u 错误已消除", (unsigned)s_motor_ids[i]);
            }
        }
    }

    /* 首帧反馈提示 */
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (s_motor_have_fb[i] && !s_fb_logged[i]) {
            s_fb_logged[i] = true;
            SRV_PA430_TEMP_LOG_I("电机 ID=%u 已收到反馈（V=%d mrad/s，使能=%d）",
                (unsigned)s_motor_ids[i],
                (int)(srv_pa430_temp_vel_raw_to_radps(s_motor_vel_raw[i]) * 1000.0f),
                s_motor_enabled[i] ? 1 : 0);
        }
    }

    /* 模式确认：确认 MIT 前不使能、不发控制帧 */
    srv_pa430_temp_mode_step(now);
    if (!s_ctrl_ready) {
        return;
    }

    /* 就绪后发一次使能 */
    if (!s_enable_sent) {
        if (srv_pa430_temp_send_enable(true)) {
            s_enable_sent = true;
            SRV_PA430_TEMP_LOG_I("已发送使能（MIT 模式就绪）");
        }
    }

    /* 首次就绪：从此刻开始阶段计时（保证首个正转阶段完整 30s），并打印开始日志 */
    if (!s_motion_started) {
        s_motion_started = true;
        s_phase_start_ms = now;
        SRV_PA430_TEMP_LOG_I("控制开始：正转（目标 %d mrad/s，%u s）",
            (int)(srv_pa430_temp_phase_speed(s_phase) * 1000.0f),
            (unsigned)(SRV_PA430_TEMP_PHASE_MS / 1000U));
    }

    /* 阶段推进 */
    if ((now - s_phase_start_ms) >= SRV_PA430_TEMP_PHASE_MS) {
        const srv_pa430_temp_phase_t next =
            (srv_pa430_temp_phase_t)(((uint32_t)s_phase + 1U) % (uint32_t)SRV_PA430_TEMP_PHASE_NUM);
        if (next == SRV_PA430_TEMP_PHASE_FORWARD) {
            s_cycle++;
        }
        s_phase = next;
        s_phase_start_ms = now;
        s_ramp_target_radps = srv_pa430_temp_phase_speed(s_phase); /* 斜坡由 ramp_update 按目标变化重建 */
        srv_pa430_temp_log_phase(s_phase);
    }

    /* 速度斜坡 + 周期控制帧 */
    bool fb_seen = false;
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (s_motor_have_fb[i]) {
            fb_seen = true;
            break;
        }
    }
    const uint32_t ctrl_period = fb_seen ? SRV_PA430_TEMP_CTRL_PERIOD_MS : SRV_PA430_TEMP_CTRL_PERIOD_IDLE_MS;
    if ((now - s_last_ctrl_ms) >= ctrl_period) {
        s_last_ctrl_ms = now;
        srv_pa430_temp_ramp_update(now);
        srv_pa430_temp_send_control();
    }

    /* 无响应检测 */
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (!s_motor_nresp_latch[i]
            && ((now - s_motor_last_seen_ms[i]) >= SRV_PA430_TEMP_NORESP_PERIOD_MS)) {
            s_motor_nresp_latch[i] = true;
            SRV_PA430_TEMP_LOG_W("电机 ID=%u 长时间无反馈（掉线或断电）", (unsigned)s_motor_ids[i]);
        }
    }

    /* 恢复在线 → 重新使能 */
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (s_online_evt_pending[i]) {
            s_online_evt_pending[i] = false;
            srv_pa430_temp_send_enable(true);
            SRV_PA430_TEMP_LOG_W("电机 ID=%u 恢复在线，重发使能", (unsigned)s_motor_ids[i]);
        }
    }

    /* 保持使能：在线、无错误但反馈使能位=0 → 周期重发 */
    bool need_enable = false;
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (s_motor_have_fb[i] && (s_motor_err[i] == 0U) && !s_motor_enabled[i]) {
            need_enable = true;
            break;
        }
    }
    if (need_enable) {
        if (s_enable_stall_since_ms == 0U) {
            s_enable_stall_since_ms = now;
            s_enable_warned = false;
        } else if (!s_enable_warned
            && (now - s_enable_stall_since_ms) >= SRV_PA430_TEMP_ENABLE_TIMEOUT_MS) {
            s_enable_warned = true;
            SRV_PA430_TEMP_LOG_W("电机使能未确认已超时，继续周期重试");
        }
        if ((now - s_enable_retry_last_ms) >= SRV_PA430_TEMP_ENABLE_RETRY_MS) {
            if (srv_pa430_temp_send_enable(true)) {
                s_enable_retry_last_ms = now;
            }
        }
    } else {
        s_enable_stall_since_ms = 0U;
    }

    /* 使能确认日志 */
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (s_motor_enabled[i] && !s_enabled_prev[i]) {
            SRV_PA430_TEMP_LOG_I("电机 ID=%u 已确认使能", (unsigned)s_motor_ids[i]);
        }
        s_enabled_prev[i] = s_motor_enabled[i];
    }

    /* 状态日志 */
    if ((now - s_last_status_ms) >= SRV_PA430_TEMP_STATUS_LOG_MS) {
        s_last_status_ms = now;
        for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
            SRV_PA430_TEMP_LOG_I("STATUS ID=%u 阶段=%u 目标V=%d 实际V=%d mrad/s T=%d(0.01Nm) 温度=%d(0.1C) 使能=%d 错误=0x%04X 在线=%lu ms 循环=%u",
                (unsigned)s_motor_ids[i], (unsigned)s_phase,
                (int)(s_cmd_speed_radps * 1000.0f),
                (int)(srv_pa430_temp_vel_raw_to_radps(s_motor_vel_raw[i]) * 1000.0f),
                (int)s_motor_tq_raw[i], (int)s_motor_temp_raw[i],
                s_motor_enabled[i] ? 1 : 0, (unsigned)s_motor_err[i],
                (unsigned long)(now - s_motor_last_seen_ms[i]), (unsigned)s_cycle);
        }
    }
}

bool srv_pa430_temp_test_on_rx(const drv_can_msg_t* msg)
{
    if (!msg) {
        return false;
    }

    /* 参数应答帧（0x600+ID, DLC 8）：Byte0=ID, Byte1=Index, Byte2..5=值(大端) */
    if (!msg->is_extended && (msg->id > SRV_PA430_TEMP_PARAM_ID_BASE)
        && (msg->id <= (SRV_PA430_TEMP_PARAM_ID_BASE + SRV_PA430_TEMP_MAX_MOTORS))
        && (msg->dlc >= 8U)) {
        if (msg->data[1] == SRV_PA430_TEMP_PARAM_INDEX_MODE) {
            s_mode_val = ((uint32_t)msg->data[2] << 24) | ((uint32_t)msg->data[3] << 16)
                | ((uint32_t)msg->data[4] << 8) | msg->data[5];
            s_mode_val_have = true;
        }
        return true;
    }

    /* 反馈帧：ID 1~8，DLC≥8 */
    if (msg->is_extended || (msg->id < 1U) || (msg->id > SRV_PA430_TEMP_MAX_MOTORS) || (msg->dlc < 8U)) {
        return false;
    }
    const uint8_t idx = srv_pa430_temp_find_idx((uint8_t)msg->id);
    if (idx == SRV_PA430_TEMP_MOTOR_COUNT) {
        return true;
    }

    s_motor_last_seen_ms[idx] = millis();
    if (s_motor_nresp_latch[idx]) {
        s_motor_nresp_latch[idx] = false;
        s_online_evt_pending[idx] = true;
    }

    s_motor_theta_raw[idx] = (uint16_t)(((uint16_t)msg->data[0] << 8) | msg->data[1]);
    s_motor_vel_raw[idx] = (uint16_t)(((uint16_t)msg->data[2] << 4) | (msg->data[3] & 0x0FU));
    s_motor_tq_raw[idx] = (uint16_t)((((uint16_t)msg->data[3] & 0x0FU) << 8) | msg->data[4]);
    s_motor_temp_raw[idx] = msg->data[5];
    s_motor_have_fb[idx] = true;

    const uint16_t raw = (uint16_t)(((uint16_t)msg->data[6] << 8) | msg->data[7]);
    const uint16_t err = raw & SRV_PA430_TEMP_ERR_BIT_MASK;
    s_motor_enabled[idx] = (raw & SRV_PA430_TEMP_ERR_ENABLE_BIT) != 0U;
    if (err != s_motor_err[idx]) {
        s_motor_err[idx] = err;
        s_err_pending[idx] = true;
    }
    return true;
}

/* Private functions ---------------------------------------------------------*/

static uint8_t srv_pa430_temp_find_idx(uint8_t addr)
{
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (s_motor_ids[i] == addr) {
            return i;
        }
    }
    return SRV_PA430_TEMP_MOTOR_COUNT;
}

static bool srv_pa430_temp_all_online(void)
{
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        if (s_motor_nresp_latch[i]) {
            return false;
        }
    }
    return true;
}

/** @brief 反馈速度 raw(12bit) → rad/s */
static float srv_pa430_temp_vel_raw_to_radps(uint16_t raw)
{
    const float span = SRV_PA430_TEMP_VEL_MAX_RADPS - SRV_PA430_TEMP_VEL_MIN_RADPS;
    return SRV_PA430_TEMP_VEL_MIN_RADPS + ((float)raw / 4095.0f) * span;
}

static uint16_t srv_pa430_temp_vel_to_raw(float radps)
{
    const float span = SRV_PA430_TEMP_VEL_MAX_RADPS - SRV_PA430_TEMP_VEL_MIN_RADPS;
    float raw = (radps - SRV_PA430_TEMP_VEL_MIN_RADPS) / span * 4095.0f + 0.5f;
    if (raw < 0.0f) {
        raw = 0.0f;
    } else if (raw > 4095.0f) {
        raw = 4095.0f;
    }
    return (uint16_t)raw;
}

static uint16_t srv_pa430_temp_gain_to_raw(float val, float max)
{
    float raw = val / max * 4095.0f + 0.5f;
    if (raw < 0.0f) {
        raw = 0.0f;
    } else if (raw > 4095.0f) {
        raw = 4095.0f;
    }
    return (uint16_t)raw;
}

static uint16_t srv_pa430_temp_theta_to_raw(float rad)
{
    const float span = SRV_PA430_TEMP_THETA_MAX_RAD - SRV_PA430_TEMP_THETA_MIN_RAD;
    float raw = (rad - SRV_PA430_TEMP_THETA_MIN_RAD) / span * 65535.0f + 0.5f;
    if (raw < 0.0f) {
        raw = 0.0f;
    } else if (raw > 65535.0f) {
        raw = 65535.0f;
    }
    return (uint16_t)raw;
}

static void srv_pa430_temp_recalc_raw(void)
{
    s_pos_mid = srv_pa430_temp_theta_to_raw(0.0f);
    s_vel_zero_raw = srv_pa430_temp_vel_to_raw(0.0f);
    s_kd_raw = srv_pa430_temp_gain_to_raw(SRV_PA430_TEMP_KD_NMPRPDS, SRV_PA430_TEMP_KD_MAX_NMPRPDS);
}

static void srv_pa430_temp_pack_mit(uint8_t* slot, uint16_t pos, uint16_t vel,
    uint16_t kp, uint16_t kd, uint16_t tq)
{
    slot[0] = (uint8_t)(pos >> 8);
    slot[1] = (uint8_t)(pos & 0xFFU);
    slot[2] = (uint8_t)(vel >> 4);
    slot[3] = (uint8_t)(((vel & 0x0FU) << 4) | ((kp >> 8) & 0x0FU));
    slot[4] = (uint8_t)(kp & 0xFFU);
    slot[5] = (uint8_t)(kd >> 4);
    slot[6] = (uint8_t)(((kd & 0x0FU) << 4) | ((tq >> 8) & 0x0FU));
    slot[7] = (uint8_t)(tq & 0xFFU);
}

static bool srv_pa430_temp_send_enable(bool enable)
{
    if (!drv_can_tx_ready(SRV_PA430_TEMP_CAN_CH)) {
        return false;
    }
    const uint8_t cmd = enable ? SRV_PA430_TEMP_CMD_ENABLE : SRV_PA430_TEMP_CMD_DISABLE;
    drv_can_msg_t tx = {
        .id = SRV_PA430_TEMP_ID_STATUS,
        .is_extended = false,
        .is_fd = true,
        .dlc = 64,
    };
    memset(tx.data, SRV_PA430_TEMP_CMD_NONE, sizeof(tx.data));
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        tx.data[(uint8_t)((s_motor_ids[i] - 1U) * 8U) + 7U] = cmd;
    }
    drv_can_send(SRV_PA430_TEMP_CAN_CH, &tx);
    return true;
}

/** @brief MIT 速度控制帧：Kp=0、T=0、V_ref=当前斜坡速度（T=Kd×(V_ref−V)） */
static void srv_pa430_temp_send_control(void)
{
    if (!drv_can_tx_ready(SRV_PA430_TEMP_CAN_CH)) {
        return;
    }
    const uint16_t vel = srv_pa430_temp_vel_to_raw(s_cmd_speed_radps);

    drv_can_msg_t tx = {
        .id = SRV_PA430_TEMP_ID_CTRL,
        .is_extended = false,
        .is_fd = true,
        .dlc = 64,
    };
    memset(tx.data, 0, sizeof(tx.data));

    /* 未配置槽：中性包 */
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MAX_MOTORS; i++) {
        srv_pa430_temp_pack_mit(&tx.data[i * 8U], s_pos_mid, s_vel_zero_raw, 0U, 0U,
            SRV_PA430_TEMP_TQ_RAW_0);
    }
    /* 配置槽：Kp=0、Kd、V_ref、T=0 */
    for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
        srv_pa430_temp_pack_mit(&tx.data[(uint8_t)((s_motor_ids[i] - 1U) * 8U)],
            s_pos_mid, vel, 0U, s_kd_raw, SRV_PA430_TEMP_TQ_RAW_0);
    }
    drv_can_send(SRV_PA430_TEMP_CAN_CH, &tx);
}

static void srv_pa430_temp_send_param_read(uint8_t addr, uint8_t index)
{
    if (!drv_can_tx_ready(SRV_PA430_TEMP_CAN_CH)) {
        return;
    }
    drv_can_msg_t tx = {
        .id = SRV_PA430_TEMP_PARAM_ID_BASE + addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    tx.data[0] = SRV_PA430_TEMP_PARAM_HEAD;
    tx.data[1] = index;
    tx.data[2] = 0;
    tx.data[3] = 0;
    tx.data[4] = 0;
    tx.data[5] = 0;
    tx.data[6] = SRV_PA430_TEMP_PARAM_RW_READ;
    tx.data[7] = SRV_PA430_TEMP_PARAM_TAIL;
    drv_can_send(SRV_PA430_TEMP_CAN_CH, &tx);
}

static void srv_pa430_temp_send_param_write(uint8_t addr, uint8_t index, uint32_t value)
{
    if (!drv_can_tx_ready(SRV_PA430_TEMP_CAN_CH)) {
        return;
    }
    drv_can_msg_t tx = {
        .id = SRV_PA430_TEMP_PARAM_ID_BASE + addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    tx.data[0] = SRV_PA430_TEMP_PARAM_HEAD;
    tx.data[1] = index;
    tx.data[2] = (uint8_t)(value >> 24);
    tx.data[3] = (uint8_t)(value >> 16);
    tx.data[4] = (uint8_t)(value >> 8);
    tx.data[5] = (uint8_t)value;
    tx.data[6] = SRV_PA430_TEMP_PARAM_RW_WRITE;
    tx.data[7] = SRV_PA430_TEMP_PARAM_TAIL;
    drv_can_send(SRV_PA430_TEMP_CAN_CH, &tx);
}

/** @brief 模式确认状态机：确认 Index 11 = MIT(2) 之前不使能、不发控制帧 */
static void srv_pa430_temp_mode_step(uint32_t now)
{
    if (s_ctrl_ready) {
        return;
    }
    switch (s_mode_state) {
    case SRV_PA430_TEMP_MODE_READ:
        if ((now - s_mode_read_ms) >= SRV_PA430_TEMP_MODE_READ_RETRY_MS) {
            s_mode_read_ms = now;
            for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
                srv_pa430_temp_send_param_read(s_motor_ids[i], SRV_PA430_TEMP_PARAM_INDEX_MODE);
            }
        }
        if (s_mode_val_have) {
            s_mode_val_have = false;
            if (s_mode_val == SRV_PA430_TEMP_MODE_MIT) {
                s_ctrl_ready = true;
                SRV_PA430_TEMP_LOG_I("Control Mode 已确认 MIT(2)，控制就绪");
            } else {
                SRV_PA430_TEMP_LOG_W("Control Mode=0x%08lX 非 MIT，改写为 MIT（仅 RAM）",
                    (unsigned long)s_mode_val);
                s_mode_state = SRV_PA430_TEMP_MODE_DISABLE;
            }
        } else if ((now - s_mode_since_ms) >= SRV_PA430_TEMP_MODE_READ_TIMEOUT_MS) {
            s_mode_since_ms = now;
            s_mode_read_ms = now;
            SRV_PA430_TEMP_LOG_W("Control Mode 读回超时，保持未就绪（不下发控制帧），继续重试");
        }
        break;

    case SRV_PA430_TEMP_MODE_DISABLE:
        (void)srv_pa430_temp_send_enable(false);
        s_mode_state = SRV_PA430_TEMP_MODE_WRITE;
        break;

    case SRV_PA430_TEMP_MODE_WRITE:
        for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
            srv_pa430_temp_send_param_write(s_motor_ids[i], SRV_PA430_TEMP_PARAM_INDEX_MODE, 2U);
        }
        s_mode_wait_ms = now + SRV_PA430_TEMP_MODE_APPLY_MS;
        s_mode_state = SRV_PA430_TEMP_MODE_WAIT_APPLY;
        break;

    case SRV_PA430_TEMP_MODE_WAIT_APPLY:
        if (now >= s_mode_wait_ms) {
            s_mode_val_have = false;
            s_mode_state = SRV_PA430_TEMP_MODE_VERIFY_READ;
        }
        break;

    case SRV_PA430_TEMP_MODE_VERIFY_READ:
        for (uint8_t i = 0; i < SRV_PA430_TEMP_MOTOR_COUNT; i++) {
            srv_pa430_temp_send_param_read(s_motor_ids[i], SRV_PA430_TEMP_PARAM_INDEX_MODE);
        }
        s_mode_wait_ms = now + SRV_PA430_TEMP_MODE_VERIFY_TIMEOUT_MS;
        s_mode_state = SRV_PA430_TEMP_MODE_VERIFY_WAIT;
        break;

    case SRV_PA430_TEMP_MODE_VERIFY_WAIT:
        if (s_mode_val_have) {
            s_mode_val_have = false;
            if (s_mode_val == SRV_PA430_TEMP_MODE_MIT) {
                s_ctrl_ready = true;
                SRV_PA430_TEMP_LOG_I("Control Mode 改写成功，已确认为 MIT(2)，控制就绪");
            } else {
                SRV_PA430_TEMP_LOG_W("Control Mode 改写未生效，重新确认");
                s_mode_state = SRV_PA430_TEMP_MODE_READ;
                s_mode_since_ms = now;
                s_mode_read_ms = 0;
            }
        } else if (now >= s_mode_wait_ms) {
            SRV_PA430_TEMP_LOG_W("Control Mode 校验读回超时，重新确认");
            s_mode_state = SRV_PA430_TEMP_MODE_READ;
            s_mode_since_ms = now;
            s_mode_read_ms = 0;
        }
        break;

    default:
        s_mode_state = SRV_PA430_TEMP_MODE_READ;
        break;
    }
}

static float srv_pa430_temp_phase_speed(srv_pa430_temp_phase_t phase)
{
    if (phase == SRV_PA430_TEMP_PHASE_FORWARD) {
        return SRV_PA430_TEMP_SPEED_RADPS;
    }
    if (phase == SRV_PA430_TEMP_PHASE_REVERSE) {
        return -SRV_PA430_TEMP_SPEED_RADPS;
    }
    return 0.0f;
}

/** @brief 速度指令线性斜坡：目标变化时以当前指令值为起点，按 RAMP_MS 线性逼近 */
static void srv_pa430_temp_ramp_update(uint32_t now)
{
    if (s_ramp_target_radps != s_ramp_prev_radps) {
        s_ramp_prev_radps = s_ramp_target_radps;
        s_ramp_from_radps = s_cmd_speed_radps;
        s_ramp_start_ms = now;
    }
    const uint32_t elapsed = now - s_ramp_start_ms;
    if (elapsed >= SRV_PA430_TEMP_RAMP_MS) {
        s_cmd_speed_radps = s_ramp_target_radps;
        return;
    }
    const float span = s_ramp_target_radps - s_ramp_from_radps;
    s_cmd_speed_radps = s_ramp_from_radps
        + span * ((float)elapsed / (float)SRV_PA430_TEMP_RAMP_MS);
}

static void srv_pa430_temp_err_print(uint8_t addr, uint16_t code)
{
    SRV_PA430_TEMP_LOG_W("电机 ID=%u 错误：0x%04X", (unsigned)addr, (unsigned)code);
    for (uint8_t i = 0; i < SRV_PA430_TEMP_ERR_NUM; i++) {
        if ((code & s_err_map[i].mask) != 0U) {
            SRV_PA430_TEMP_LOG_W("  - %s", s_err_map[i].name);
        }
    }
}

static void srv_pa430_temp_log_phase(srv_pa430_temp_phase_t phase)
{
    const char* name;
    switch (phase) {
    case SRV_PA430_TEMP_PHASE_FORWARD:
        name = "正转";
        break;
    case SRV_PA430_TEMP_PHASE_DWELL_F:
        name = "停(正转后)";
        break;
    case SRV_PA430_TEMP_PHASE_REVERSE:
        name = "反转";
        break;
    case SRV_PA430_TEMP_PHASE_DWELL_R:
        name = "停(反转后)";
        break;
    default:
        name = "未知";
        break;
    }
    SRV_PA430_TEMP_LOG_I("阶段切换 → %s（目标 %d mrad/s，时长 %u s，循环 %u）",
        name, (int)(srv_pa430_temp_phase_speed(phase) * 1000.0f),
        (unsigned)(SRV_PA430_TEMP_PHASE_MS / 1000U), (unsigned)s_cycle);
}
