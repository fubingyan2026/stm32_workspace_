/**
 * @file    srv_pa430_torque_test.c
 * @brief   Motorevo (PA430) CAN FD 伺服电机 — MIT 力位混合模式来回运动测试实现
 *
 * 电机控制指令按 docs/Motorevo电机CAN协议文档.md 组帧发送（CAN FD 广播帧，
 * 状态帧 0x10 / 控制帧 0x20，DLC 64，每电机槽 (ID-1)*8，多字节大端）。
 * 本模块走 FDCAN2（DRV_CAN_CH_2，PB12 RX / PB13 TX）独立总线。
 *
 * MIT 公式：T_out = Kp×(θ_ref−θ) + Kd×(V_ref−V) + T_ref。本模块令 θ_ref 在
 * ±SRV_PA430_ANGLE_AMP_RAD（默认 ±2.5 rad）两端点间交替，Kp/Kd/V_ref 均按
 * docs/Motorevo电机CAN协议文档.md §3.1 缩放换算为 raw（物理单位宏可调），
 * V_ref/T_ref 默认恒为 0；每 CTRL_PERIOD_MS 重发广播控制帧（MIT 需持续下发
 * 以保持刚度）。到位反向采用位置反馈闭环：各电机以自身 ID（1~8）周期回复
 * DLC 8 反馈帧（θ 16bit，与 θ_ref 同标度），主循环 |θ_raw − 目标| ≤ 到达阈值
 * 判定到位；所有配置电机均到位后 θ_ref 翻转到另一端。
 *
 * 启动流程：对配置电机发广播使能（0x10, byte7=0xFC）；可选（SRV_PA430_CONFIGURE_MODE）
 * 经 0x600+ID 单机帧写 Control Mode=2（MIT）并保存到 Flash。累计在线满
 * DURATION_MS（30 天）自动停止并失能。
 *
 * 运行中自愈：在线但反馈使能位（Bit0）=0 的电机（断电重上电/保护下使能后）由主循环
 * 每 ENABLE_RETRY_MS 重发使能，直到反馈 Bit0=1 确认（有活动错误位时跳过）；
 * 电机恢复在线时把 θ_ref 重同步到当前反馈位置，再重启往复运动。
 *
 * RX 侧：反馈帧（CAN-ID = 电机 ID 1~8, DLC 8）由 can_task 按 CH_2 分发到
 * srv_pa430_torque_test_on_rx() 解析记录（ISR 上下文，不打日志），主循环消费
 * 到位/错误/在线标志；非 1~8 反馈帧直接丢弃（CAN2 为专用总线）。
 */

#include "srv_pa430_torque_test.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/
/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印（确认摆动正常后可关闭） */
#define SRV_PA430_TORQUE_TEST_LOG_ENABLE 1

#if SRV_PA430_TORQUE_TEST_LOG_ENABLE
#define SRV_PA430_TORQUE_TEST_LOG_I(...) LOG_I("pa430_test", __VA_ARGS__)
#define SRV_PA430_TORQUE_TEST_LOG_W(...) LOG_W("pa430_test", __VA_ARGS__)
#define SRV_PA430_TORQUE_TEST_LOG_E(...) LOG_E("pa430_test", __VA_ARGS__)
#else
#define SRV_PA430_TORQUE_TEST_LOG_I(...) ((void)0)
#define SRV_PA430_TORQUE_TEST_LOG_W(...) ((void)0)
#define SRV_PA430_TORQUE_TEST_LOG_E(...) ((void)0)
#endif

/* 模块测试开关 ----------------------------------------------------------------*/

/** @brief 电机测试模式：1=上电自动启动；0=需手动调用 srv_pa430_torque_test_start() */
#define SRV_PA430_AUTO_START 1

/**
 * @brief 是否由固件配置 Control Mode：1=启动时经 0x600+ID 写 Index 11 = 2 (MIT)
 *        并发送保存命令（触发 Flash 擦写，供电须稳定 ≥1s，见协议文档 §5.2 警告）；
 *        0=假设电机 flash 已预配置为 MIT（默认，避免 Flash 擦写风险）
 */
#define SRV_PA430_CONFIGURE_MODE 0

/**
 * @brief 经典 CAN 诊断测试开关：置 1 时 CH_2 上的帧改为经典 CAN（DLC 8，全程 1M），
 *        用于隔离「收发器未驱动」vs「5M FD 数据段问题」（经典 CAN 仲裁段与 FD 相同）。
 *        PA430 协议本身要求 CAN FD DLC 64，此开关仅作诊断用，测完置 0。
 */
#define SRV_PA430_CLASSIC_TEST 0

/* Private constants ---------------------------------------------------------*/

/* --- 总线电机配置 --- */

/** @brief 广播总线最多电机数（ID 1~8） */
#define SRV_PA430_MAX_MOTORS 8U
/** @brief 实际参与测试的电机数（1~8，须与总线一致） */
#define SRV_PA430_MOTOR_COUNT 1U

/* --- MIT 控制参数（docs/Motorevo电机CAN协议文档.md §3.1；物理单位可调） --- */

/* ===== 用户可调参数（物理单位；内部自动换算 raw） ===== */
/** @brief 反转角度半幅（rad），行程为 ±AMP（默认 ±2.5 rad） */
#define SRV_PA430_ANGLE_AMP_RAD (3.2f)
/** @brief V_ref 目标速度（rad/s），通常 0 */
#define SRV_PA430_VEL_REF_RADPS (0.0f)
/** @brief MIT 刚度 Kp（Nm/rad，0~250） */
#define SRV_PA430_KP_NMPR (5.5f)
/** @brief MIT 阻尼 Kd（Nm/(rad/s)，0~50） */
#define SRV_PA430_KD_NMPRPDS (26.75f)
/** @brief 到位判定阈值（rad） */
#define SRV_PA430_ARRIVE_THRESHOLD_RAD (0.4f)

/* ===== 协议缩放常量（docs/Motorevo电机CAN协议文档.md §3.1，勿改） ===== */
#define SRV_PA430_THETA_MIN_RAD (-12.5f)
#define SRV_PA430_THETA_MAX_RAD (12.5f)
#define SRV_PA430_VEL_MIN_RADPS (-10.0f)
#define SRV_PA430_VEL_MAX_RADPS (10.0f)
#define SRV_PA430_KP_MAX_NMPR (250.0f)
#define SRV_PA430_KD_MAX_NMPRPDS (50.0f)

/** @brief 前馈扭矩 T_ref 原始值（12bit：0x800 ↔ 0 Nm，本模块恒为 0，不参与换算） */
#define SRV_PA430_TQ_RAW_0 0x0800U

/* --- 周期 --- */

/** @brief 广播控制帧（0x20）重发周期 (ms)：对齐 can_task 5ms，MIT 需持续下发保持刚度 */
#define SRV_PA430_CTRL_PERIOD_MS 5U
/** @brief 未收到任何反馈帧时的控制帧重发周期 (ms)：避免总线上无电机时 5ms 连发
 *        使错误计数器快速累计进 BUS-OFF；收到首帧反馈后切回 CTRL_PERIOD_MS */
#define SRV_PA430_CTRL_PERIOD_IDLE_MS 100U
/** @brief 电机无反馈判定周期 (ms)：超过该时长未收到电机反馈帧视为掉线/断电 */
#define SRV_PA430_NORESP_PERIOD_MS 2000U
/** @brief 使能重试周期：恢复在线后每间隔重发一次使能，直到反馈 Bit0 确认 */
#define SRV_PA430_ENABLE_RETRY_MS 200U
/** @brief 使能确认超时：连续重试仍未确认时打印一次告警（继续重试） */
#define SRV_PA430_ENABLE_TIMEOUT_MS 2000U
/**
 * @brief 耐久运行时长 (ms)：电机持续在线累计满 30 天自动停止并失能；
 *        30 天 = 2592000000 ms（uint32 范围内）；置 0 禁用自动停止
 */
#define SRV_PA430_DURATION_MS 2592000000U

/* --- Motorevo 协议指令（docs/Motorevo电机CAN协议文档.md §2/§3/§5） --- */

#define SRV_PA430_ID_STATUS 0x10U /**< 广播状态/使能帧标识符（DLC 64，CAN FD） */
#define SRV_PA430_ID_CTRL 0x20U /**< 广播控制帧标识符（DLC 64，CAN FD） */
#define SRV_PA430_CMD_ENABLE 0xFCU /**< 槽 byte7：使能命令 */
#define SRV_PA430_CMD_DISABLE 0xFDU /**< 槽 byte7：失能命令 */
#define SRV_PA430_CMD_NONE 0xFFU /**< 槽 byte7：无命令（未配置电机槽） */
#define SRV_PA430_PARAM_ID_BASE 0x600U /**< 单机参数读写帧基址（+ Motor ID） */
#define SRV_PA430_PARAM_HEAD 0x67U /**< 参数帧帧头 */
#define SRV_PA430_PARAM_TAIL 0x76U /**< 参数帧帧尾 */
#define SRV_PA430_PARAM_RW_WRITE 0x15U /**< R/W：写命令 */
#define SRV_PA430_PARAM_INDEX_MODE 11U /**< 参数索引：Control Mode */
#define SRV_PA430_PARAM_INDEX_SAVE 0U /**< 参数索引 0：保存命令 */

/* Private types -------------------------------------------------------------*/

/** @brief 反馈帧错误码描述表项（见 docs/Motorevo电机CAN协议文档.md §4，16bit 错误码） */
typedef struct {
    uint16_t mask; /**< 错误码位 */
    const char* name; /**< 含义 */
} srv_pa430_torque_test_err_desc_t;

/** @brief 反馈帧错误字段使能指示位（Bit0）：1=已使能，非错误 */
#define SRV_PA430_ERR_ENABLE_BIT 0x0001U
/** @brief 错误码有效位掩码（排除使能指示位 Bit0） */
#define SRV_PA430_ERR_BIT_MASK 0xFFFEU

/** @brief 错误码 → 含义映射表（反馈帧 byte6-7，Bit0 为使能指示位非错误，跳过） */
static const srv_pa430_torque_test_err_desc_t s_err_map[] = {
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

/** @brief 错误码描述表项数 */
#define SRV_PA430_ERR_NUM (sizeof(s_err_map) / sizeof(s_err_map[0]))

/* Private variables ---------------------------------------------------------*/

/** @brief 测试模式运行标志 */
static bool s_running;

/** @brief 电机 ID 列表（广播槽 = (ID-1)*8；默认单台 ID=1，可扩展 1~8） */
static const uint8_t s_motor_ids[SRV_PA430_MAX_MOTORS] = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U };

/** @brief 当前方向：1=朝 RAW_POS（+），0=朝 RAW_NEG（−） */
static uint8_t s_dir;

/** @brief 当前目标 θ_ref 原始值（主循环维护，广播控制帧使用） */
static uint16_t s_target_raw;

/** @brief 物理参数宏换算出的 raw 值（start() 时由 srv_pa430_torque_test_recalc_raw 计算） */
static uint16_t s_pos_raw_pos; /* 正端点 */
static uint16_t s_pos_raw_neg; /* 负端点 */
static uint16_t s_pos_raw_mid; /* 中点（0 rad） */
static uint16_t s_vel_ref_raw; /* 配置槽 V_ref */
static uint16_t s_vel_zero_raw; /* 0 rad/s（未配置槽用） */
static uint16_t s_kp_raw;
static uint16_t s_kd_raw;
static uint16_t s_arrive_thresh_raw;

/** @brief 每电机最新反馈 θ（16bit 原始值，与 θ_ref 同标度，ISR 写主循环读） */
static uint16_t s_motor_theta_raw[SRV_PA430_MAX_MOTORS];

/** @brief 每电机是否已收到过反馈帧（未收到不参与到位判定，避免 0 值误判） */
static bool s_motor_have_fb[SRV_PA430_MAX_MOTORS];

/** @brief 每电机首帧反馈是否已打印（确认电机在总线上的提示，避免刷屏） */
static bool s_fb_logged[SRV_PA430_MAX_MOTORS];

/** @brief 每电机最新错误码（反馈帧 byte6-7，ISR 写） */
static uint16_t s_motor_err[SRV_PA430_MAX_MOTORS];

/** @brief 每电机反馈使能位（Bit0：1=已使能，ISR 写） */
static bool s_motor_enabled[SRV_PA430_MAX_MOTORS];

/** @brief 每电机上次使能位（主循环用于检测使能上升沿打印确认） */
static bool s_enabled_prev[SRV_PA430_MAX_MOTORS];

/** @brief 上次实际发出使能帧的时间 (millis) */
static uint32_t s_enable_retry_last_ms;

/** @brief 进入"需使能"状态的起始时间 (millis)，0=不在需使能状态 */
static uint32_t s_enable_stall_since_ms;

/** @brief 使能超时告警是否已打印（确认使能或离开需使能状态后复位） */
static bool s_enable_warned;

/** @brief 新错误应答待打印标志（ISR 置位，主循环清零打印） */
static bool s_err_pending[SRV_PA430_MAX_MOTORS];

/** @brief 每电机最后收到反馈帧的时间 (millis)，用于掉线检测（ISR 更新） */
static uint32_t s_motor_last_seen_ms[SRV_PA430_MAX_MOTORS];

/** @brief 电机无响应告警锁存（收到反馈帧后清除，避免重复刷屏） */
static bool s_motor_nresp_latch[SRV_PA430_MAX_MOTORS];

/** @brief 恢复在线事件待打印标志（ISR 置位，主循环清零打印 + 重新使能） */
static bool s_online_evt_pending[SRV_PA430_MAX_MOTORS];

/** @brief 上次广播控制帧发送时间 (millis) */
static uint32_t s_last_ctrl_ms;

/** @brief 测试起始时间 (millis) */
static uint32_t s_start_ms;

/** @brief 累计在线时长 (ms)：仅所有电机在线时累加，用于 DURATION 自动停止 */
static uint32_t s_online_ms;

/** @brief 上次在线时长累计时间点 (millis) */
static uint32_t s_online_last_ms;

/* Private function prototypes -----------------------------------------------*/

static uint8_t srv_pa430_torque_test_find_idx(uint8_t addr);
static bool srv_pa430_torque_test_all_online(void);
static bool srv_pa430_torque_test_is_arrived(uint8_t idx);
static bool srv_pa430_torque_test_send_enable(bool enable);
static uint16_t srv_pa430_theta_to_raw(float theta_rad);
static uint16_t srv_pa430_vel_to_raw(float vel_radps);
static uint16_t srv_pa430_gain_to_raw(float val, float max);
static uint16_t srv_pa430_delta_theta_to_raw(float delta_rad);
static void srv_pa430_torque_test_recalc_raw(void);
static void srv_pa430_torque_test_pack_mit(uint8_t* slot, uint16_t pos, uint16_t vel,
    uint16_t kp, uint16_t kd, uint16_t tq);
static void srv_pa430_torque_test_send_control(void);
#if SRV_PA430_CLASSIC_TEST
static void srv_pa430_torque_test_send_classic(void);
#endif
static void srv_pa430_torque_test_err_print(uint8_t addr, uint16_t code);
#if SRV_PA430_CONFIGURE_MODE
static void srv_pa430_torque_test_send_param_write(uint8_t addr, uint8_t index, uint32_t value);
static void srv_pa430_torque_test_configure_mode(void);
#endif

/* Exported functions --------------------------------------------------------*/

void srv_pa430_torque_test_init(void)
{
#if SRV_PA430_AUTO_START
    srv_pa430_torque_test_start(); /* 测试模式：广播使能 + MIT 来回运动驱动 */
#endif
}

/**
 * @brief 启动来回运动测试模式
 * @note  对配置电机发广播使能（0x10, byte7=0xFC）→ 持续下发 MIT 控制帧（0x20），
 *        θ_ref 在 ±SRV_PA430_ANGLE_AMP_RAD 两端点间交替，到位即反向
 */
void srv_pa430_torque_test_start(void)
{
    s_running = true;
    s_start_ms = millis(); /* 30 天自动停止计时起点 */
    s_online_ms = 0; /* 持续在线时长从 0 累计 */
    s_online_last_ms = s_start_ms;

    /* 由物理单位宏换算全部 raw 控制参数 */
    srv_pa430_torque_test_recalc_raw();

    /* 初始目标：朝 +AMP 端点 */
    s_dir = 1;
    s_target_raw = s_pos_raw_pos;
    s_last_ctrl_ms = s_start_ms;

    memset(s_motor_theta_raw, 0, sizeof(s_motor_theta_raw));
    memset(s_motor_have_fb, 0, sizeof(s_motor_have_fb));
    memset(s_fb_logged, 0, sizeof(s_fb_logged));
    memset(s_motor_err, 0, sizeof(s_motor_err));
    memset(s_err_pending, 0, sizeof(s_err_pending));
    memset(s_motor_nresp_latch, 0, sizeof(s_motor_nresp_latch));
    memset(s_online_evt_pending, 0, sizeof(s_online_evt_pending));
    memset(s_motor_enabled, 0, sizeof(s_motor_enabled));
    memset(s_enabled_prev, 0, sizeof(s_enabled_prev));
    s_enable_retry_last_ms = s_start_ms;
    s_enable_stall_since_ms = 0;
    s_enable_warned = false;
    for (uint8_t i = 0; i < SRV_PA430_MAX_MOTORS; i++) {
        s_motor_last_seen_ms[i] = s_start_ms; /* 掉线检测起点 */
    }

#if SRV_PA430_CONFIGURE_MODE
    srv_pa430_torque_test_configure_mode(); /* 写 Control Mode=2 (MIT) 并保存 */
#endif
    (void)srv_pa430_torque_test_send_enable(true); /* 广播使能全部配置电机 */

    SRV_PA430_TORQUE_TEST_LOG_I("PA430 来回运动启动：%u 台电机，θ_ref 0x%04X ↔ 0x%04X（Kp 0x%03X Kd 0x%03X）",
        (unsigned)SRV_PA430_MOTOR_COUNT, (unsigned)s_pos_raw_neg,
        (unsigned)s_pos_raw_pos, (unsigned)s_kp_raw, (unsigned)s_kd_raw);
}

/**
 * @brief 停止来回运动测试模式：θ_ref 回中 + 广播失能
 * @note  断电前必须失能 (0xFD)，命令发往所有配置电机
 */
void srv_pa430_torque_test_stop(void)
{
    s_running = false;
    s_target_raw = s_pos_raw_mid; /* 目标回中（θ=0），尽力下发一帧后失能 */
    srv_pa430_torque_test_send_control();
    (void)srv_pa430_torque_test_send_enable(false);
    SRV_PA430_TORQUE_TEST_LOG_I("PA430 来回运动停止：已回中并失能");
}

/**
 * @brief 来回运动测试模式周期步进（由 can_task 每 TASK_PERIOD_MS 调用）
 * @note  周期重发广播 MIT 控制帧；消费反馈帧到位标志，所有配置电机到位后
 *        θ_ref 翻转到另一端；打印错误码变化、掉线告警；在线但未使能的电机
 *        周期重发使能直到反馈 Bit0 确认（断电重上电/保护恢复自动重新使能）；
 *        电机恢复在线时把 θ_ref 重同步到当前位置再重启往复；
 *        累计在线满 DURATION_MS（30 天，置 0 禁用）自动停止
 */
void srv_pa430_torque_test_step(void)
{
    if (!s_running)
        return;

    const uint32_t now = millis();

    /* 持续在线计时：仅当所有电机在线时累加（掉线期间不计时，恢复后继续累计）。
       s_online_last_ms 每步都更新，保证离线/恢复后时间不跳变 */
    if (srv_pa430_torque_test_all_online()) {
        s_online_ms += (uint32_t)(now - s_online_last_ms);
    }
    s_online_last_ms = now;

#if (SRV_PA430_DURATION_MS != 0U)
    if (s_online_ms >= SRV_PA430_DURATION_MS) {
        SRV_PA430_TORQUE_TEST_LOG_I("累计在线 %lu ms 已到（30 天），PA430 测试自动停止",
            (unsigned long)s_online_ms);
        srv_pa430_torque_test_stop();
        return;
    }
#endif

    /* 打印新到达的错误码（主循环上下文，ISR 只置标志）。
       反馈帧错误码仅在变化时置位，此处直接打印当前值 */
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_err_pending[i]) {
            s_err_pending[i] = false; /* 先清标志再取值，避免 ISR 并发丢更新 */
            if (s_motor_err[i] != 0U) {
                srv_pa430_torque_test_err_print(s_motor_ids[i], s_motor_err[i]);
            } else {
                SRV_PA430_TORQUE_TEST_LOG_W("电机 ID=%u 错误已消除，恢复正常",
                    (unsigned)s_motor_ids[i]);
            }
        }
    }

    /* 首帧反馈提示：确认电机在总线上并正常回帧（仅各电机打印一次） */
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_motor_have_fb[i] && !s_fb_logged[i]) {
            s_fb_logged[i] = true;
            SRV_PA430_TORQUE_TEST_LOG_I("电机 ID=%u 已收到反馈：θ=0x%04X",
                (unsigned)s_motor_ids[i], (unsigned)s_motor_theta_raw[i]);
        }
    }

    /* 到位判定：所有已收过反馈的配置电机均 |θ−目标|≤阈值 时翻转 θ_ref。
       翻转后目标远离当前位置，天然去抖，不会连续重复翻转 */
    bool all_arrived = true;
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (!s_motor_have_fb[i] || !srv_pa430_torque_test_is_arrived(i)) {
            all_arrived = false;
            break;
        }
    }
    if (all_arrived) {
        if (s_dir == 1U) {
            s_dir = 0;
            s_target_raw = s_pos_raw_neg;
        } else {
            s_dir = 1;
            s_target_raw = s_pos_raw_pos;
        }
        SRV_PA430_TORQUE_TEST_LOG_I("全部电机到位，θ_ref 翻转为 0x%04X",
            (unsigned)s_target_raw);
    }

    /* 周期重发控制帧：经典诊断模式发经典测试帧，FD 模式发广播 MIT 控制帧
       （FD 模式未收到反馈时降低重发频率，避免无电机总线上错误计数器快速累计进 BUS-OFF） */
#if SRV_PA430_CLASSIC_TEST
    if ((now - s_last_ctrl_ms) >= SRV_PA430_CTRL_PERIOD_IDLE_MS) {
        s_last_ctrl_ms = now;
        srv_pa430_torque_test_send_classic();
    }
#else
    bool fb_seen = false;
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_motor_have_fb[i]) {
            fb_seen = true;
            break;
        }
    }
    const uint32_t ctrl_period = fb_seen ? SRV_PA430_CTRL_PERIOD_MS : SRV_PA430_CTRL_PERIOD_IDLE_MS;
    if ((now - s_last_ctrl_ms) >= ctrl_period) {
        s_last_ctrl_ms = now;
        srv_pa430_torque_test_send_control();
    }
#endif

    /* 电机无响应检测：超过 NORESP_PERIOD 未收到反馈帧视为掉线/断电（每电机只告警一次） */
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (!s_motor_nresp_latch[i] && ((now - s_motor_last_seen_ms[i]) >= SRV_PA430_NORESP_PERIOD_MS)) {
            s_motor_nresp_latch[i] = true;
            SRV_PA430_TORQUE_TEST_LOG_W("电机 ID=%u 长时间无反馈（掉线或断电）",
                (unsigned)s_motor_ids[i]);
        }
    }

    /* 恢复在线：把运动目标重同步到电机当前位置，避免旧目标/旧反馈造成不翻转；
       实际使能仍交给下方"保持使能"重试循环 */
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_online_evt_pending[i]) {
            s_online_evt_pending[i] = false;
            uint16_t theta = s_motor_theta_raw[i];
            /* 钳位到测试端点 [NEG, POS]，防止反馈异常值作为目标 */
            if (theta < s_pos_raw_neg) {
                theta = s_pos_raw_neg;
            } else if (theta > s_pos_raw_pos) {
                theta = s_pos_raw_pos;
            }
            s_target_raw = theta; /* 目标 = 当前位置 → 下一 tick 到位即翻转，从当前位置重启往复 */
            s_dir = 1;
            SRV_PA430_TORQUE_TEST_LOG_W("电机 ID=%u 恢复在线，θ=0x%04X，目标重同步 0x%04X",
                (unsigned)s_motor_ids[i], (unsigned)s_motor_theta_raw[i], (unsigned)s_target_raw);
        }
    }

    /* 保持使能：在线且无活动错误但反馈使能位=0 的电机，周期重发使能直到确认。
       有错误位（保护中）时跳过，避免反复顶撞故障；故障消除后自动重新使能 */
    bool need_enable = false;
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
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
            && (now - s_enable_stall_since_ms) >= SRV_PA430_ENABLE_TIMEOUT_MS) {
            s_enable_warned = true;
            SRV_PA430_TORQUE_TEST_LOG_W("电机使能未确认已超时，继续周期重试");
        }
        if ((now - s_enable_retry_last_ms) >= SRV_PA430_ENABLE_RETRY_MS) {
            if (srv_pa430_torque_test_send_enable(true)) {
                s_enable_retry_last_ms = now;
            }
        }
    } else {
        s_enable_stall_since_ms = 0U; /* 已全部使能或不在线，复位 */
    }

    /* 使能确认日志（反馈 Bit0 0→1 上升沿，每电机一次） */
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_motor_enabled[i] && !s_enabled_prev[i]) {
            SRV_PA430_TORQUE_TEST_LOG_I("电机 ID=%u 已确认使能", (unsigned)s_motor_ids[i]);
        }
        s_enabled_prev[i] = s_motor_enabled[i];
    }
}

/**
 * @brief 处理 Motorevo 反馈帧（由 can_task 按 CH_2 分发调用）
 * @param  msg CAN 报文指针
 * @return true=反馈帧（CAN-ID 1~8, DLC≥8），已消费；false=非反馈帧（丢弃）
 * @note   ISR 上下文，只做数据记录与标志置位，不打日志
 */
bool srv_pa430_torque_test_on_rx(const drv_can_msg_t* msg)
{
    if (!msg)
        return false;

    /* 仅消费标准帧、ID 1~8、DLC≥8 的反馈帧；其余（0x100 上位机帧等）交旧协议 */
    if (msg->is_extended || (msg->id < 1U) || (msg->id > SRV_PA430_MAX_MOTORS) || (msg->dlc < 8U))
        return false;

    /* 未配置电机的反馈帧：仍属 Motorevo 协议，消费但不记录 */
    const uint8_t idx = srv_pa430_torque_test_find_idx((uint8_t)msg->id);
    if (idx == SRV_PA430_MOTOR_COUNT)
        return true;

    /* 在线刷新：收到反馈帧即视为电机在应答。
       此前掉线锁存的电机收到帧即恢复在线，置标志由主循环重新使能（ISR 不打日志） */
    s_motor_last_seen_ms[idx] = millis();
    if (s_motor_nresp_latch[idx]) {
        s_motor_nresp_latch[idx] = false;
        s_online_evt_pending[idx] = true;
    }

    /* θ 反馈：data[0..1] = 16bit 大端，与 θ_ref 同标度（0x0000↔MIN，0xFFFF↔MAX） */
    s_motor_theta_raw[idx] = (uint16_t)(((uint16_t)msg->data[0] << 8) | msg->data[1]);
    s_motor_have_fb[idx] = true;

    /* 错误码：data[6..7] = 16bit 大端；Bit0 为使能指示位，掩掉后再判定错误（ISR 不打日志） */
    const uint16_t raw = (uint16_t)(((uint16_t)msg->data[6] << 8) | msg->data[7]);
    const uint16_t err = raw & SRV_PA430_ERR_BIT_MASK;
    s_motor_enabled[idx] = (raw & SRV_PA430_ERR_ENABLE_BIT) != 0U; /* 使能确认依据 */
    if (err != s_motor_err[idx]) {
        s_motor_err[idx] = err;
        s_err_pending[idx] = true;
    }

    return true;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 查找电机地址在配置列表中的索引
 * @param  addr 电机地址（1~8）
 * @return 索引；未找到返回 SRV_PA430_MOTOR_COUNT（哨兵值）
 */
static uint8_t srv_pa430_torque_test_find_idx(uint8_t addr)
{
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_motor_ids[i] == addr)
            return i;
    }
    return SRV_PA430_MOTOR_COUNT;
}

/**
 * @brief 所有配置电机是否在线（无任何电机进入无响应锁存）
 * @return true=全部在线
 * @note   DURATION 持续在线计时仅在全部在线时累加
 */
static bool srv_pa430_torque_test_all_online(void)
{
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_motor_nresp_latch[i])
            return false;
    }
    return true;
}

/**
 * @brief 判定电机 idx 是否到达当前目标
 * @param idx 电机索引（0 ~ MOTOR_COUNT-1）
 * @return true=|θ_raw − 目标| ≤ 到达阈值
 */
static bool srv_pa430_torque_test_is_arrived(uint8_t idx)
{
    const uint16_t theta = s_motor_theta_raw[idx];
    uint32_t diff;
    if (theta >= s_target_raw) {
        diff = (uint32_t)(theta - s_target_raw);
    } else {
        diff = (uint32_t)(s_target_raw - theta);
    }
    return diff <= (uint32_t)s_arrive_thresh_raw;
}

/**
 * @brief 位置（rad）→ 16bit raw（0x0000↔THETA_MIN，0xFFFF↔THETA_MAX）
 * @param theta_rad 目标角度（rad）
 * @return raw 值（钳位 0~0xFFFF）
 */
static uint16_t srv_pa430_theta_to_raw(float theta_rad)
{
    const float span = SRV_PA430_THETA_MAX_RAD - SRV_PA430_THETA_MIN_RAD;
    float raw = (theta_rad - SRV_PA430_THETA_MIN_RAD) / span * 65535.0f + 0.5f;
    if (raw < 0.0f) {
        raw = 0.0f;
    } else if (raw > 65535.0f) {
        raw = 65535.0f;
    }
    return (uint16_t)raw;
}

/**
 * @brief 速度（rad/s）→ 12bit raw（0x000↔VEL_MIN，0xFFF↔VEL_MAX）
 * @param vel_radps 目标速度（rad/s）
 * @return raw 值（钳位 0~0xFFF）
 */
static uint16_t srv_pa430_vel_to_raw(float vel_radps)
{
    const float span = SRV_PA430_VEL_MAX_RADPS - SRV_PA430_VEL_MIN_RADPS;
    float raw = (vel_radps - SRV_PA430_VEL_MIN_RADPS) / span * 4095.0f + 0.5f;
    if (raw < 0.0f) {
        raw = 0.0f;
    } else if (raw > 4095.0f) {
        raw = 4095.0f;
    }
    return (uint16_t)raw;
}

/**
 * @brief 增益（0~max）→ 12bit raw（0x000↔0，0xFFF↔max）
 * @param val 目标增益（Kp: Nm/rad；Kd: Nm/(rad/s)）
 * @param max 对应 CAN COM 满量程（Kp=250，Kd=50）
 * @return raw 值（钳位 0~0xFFF）
 */
static uint16_t srv_pa430_gain_to_raw(float val, float max)
{
    float raw = val / max * 4095.0f + 0.5f;
    if (raw < 0.0f) {
        raw = 0.0f;
    } else if (raw > 4095.0f) {
        raw = 4095.0f;
    }
    return (uint16_t)raw;
}

/**
 * @brief 角度差（rad）→ raw 差量（与原点无关，仅用于到位阈值）
 * @param delta_rad 角度差（rad）
 * @return 对应的 raw 计数
 */
static uint16_t srv_pa430_delta_theta_to_raw(float delta_rad)
{
    const float span = SRV_PA430_THETA_MAX_RAD - SRV_PA430_THETA_MIN_RAD;
    float raw = delta_rad / span * 65535.0f + 0.5f;
    if (raw < 0.0f) {
        raw = 0.0f;
    } else if (raw > 65535.0f) {
        raw = 65535.0f;
    }
    return (uint16_t)raw;
}

/**
 * @brief 由物理单位宏重算全部 raw 控制参数（start() 时调用）
 */
static void srv_pa430_torque_test_recalc_raw(void)
{
    s_pos_raw_pos = srv_pa430_theta_to_raw(+SRV_PA430_ANGLE_AMP_RAD);
    s_pos_raw_neg = srv_pa430_theta_to_raw(-SRV_PA430_ANGLE_AMP_RAD);
    s_pos_raw_mid = srv_pa430_theta_to_raw(0.0f);
    s_vel_ref_raw = srv_pa430_vel_to_raw(SRV_PA430_VEL_REF_RADPS);
    s_vel_zero_raw = srv_pa430_vel_to_raw(0.0f);
    s_kp_raw = srv_pa430_gain_to_raw(SRV_PA430_KP_NMPR, SRV_PA430_KP_MAX_NMPR);
    s_kd_raw = srv_pa430_gain_to_raw(SRV_PA430_KD_NMPRPDS, SRV_PA430_KD_MAX_NMPRPDS);
    s_arrive_thresh_raw = srv_pa430_delta_theta_to_raw(SRV_PA430_ARRIVE_THRESHOLD_RAD);
}

/**
 * @brief 发送广播使能/失能帧（0x10；FD 模式 DLC 64，经典测试模式 DLC 8）
 * @param enable true=使能 (byte7=0xFC)，false=失能 (byte7=0xFD)
 * @return true=帧已入 TX FIFO；false=TX 未就绪未发送（调用方可决定何时重试）
 * @note  FD 模式全槽 byte7 写入命令（未配置槽 0xFF 无命令）；
 *        经典诊断模式只写第一槽命令（DLC 8 仅够一槽）
 */
static bool srv_pa430_torque_test_send_enable(bool enable)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return false;

    drv_can_msg_t tx = {
        .id = SRV_PA430_ID_STATUS,
        .is_extended = false,
        .is_fd = !SRV_PA430_CLASSIC_TEST, /* 经典诊断模式发经典帧 */
        .dlc = SRV_PA430_CLASSIC_TEST ? 8U : 64U,
    };
    memset(tx.data, SRV_PA430_CMD_NONE, sizeof(tx.data)); /* 全槽 byte7=0xFF 无命令 */

    const uint8_t cmd = enable ? SRV_PA430_CMD_ENABLE : SRV_PA430_CMD_DISABLE;
#if SRV_PA430_CLASSIC_TEST
    tx.data[7] = cmd; /* 经典模式仅第一槽 */
#else
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        tx.data[(uint8_t)((s_motor_ids[i] - 1U) * 8U) + 7U] = cmd;
    }
#endif
    drv_can_send(DRV_CAN_CH_2, &tx);
    return true;
}

/**
 * @brief MIT 单机 8 字节封包（docs/Motorevo电机CAN协议文档.md §3.1）
 * @param slot 目标槽缓冲区（8 字节）
 * @param pos   θ_ref 16bit 原始值（0x0000↔MIN，0xFFFF↔MAX）
 * @param vel   V_ref 12bit 原始值（0x000↔MIN，0xFFF↔MAX）
 * @param kp    Kp 12bit 原始值（0x000↔0，0xFFF↔250 Nm/rad）
 * @param kd    Kd 12bit 原始值（0x000↔0，0xFFF↔50 Nm/(rad/s)）
 * @param tq    T_ref 12bit 原始值（0x000↔MIN，0xFFF↔MAX）
 */
static void srv_pa430_torque_test_pack_mit(uint8_t* slot, uint16_t pos, uint16_t vel,
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

/**
 * @brief 发送广播 MIT 控制帧（0x20, DLC 64, CAN FD）
 * @note  配置电机槽写当前目标 θ_ref + 刚度（Kp/Kd，V_ref/T_ref=0）；
 *        未配置槽发中性包（θ=0, Kp=0）避免误动其它 ID
 */
static void srv_pa430_torque_test_send_control(void)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = SRV_PA430_ID_CTRL,
        .is_extended = false,
        .is_fd = true, /* DLC 64：CAN FD */
        .dlc = 64,
    };
    memset(tx.data, 0, sizeof(tx.data));

    for (uint8_t i = 0; i < SRV_PA430_MAX_MOTORS; i++) {
        srv_pa430_torque_test_pack_mit(&tx.data[i * 8U], s_pos_raw_mid,
            s_vel_zero_raw, 0U, 0U, SRV_PA430_TQ_RAW_0); /* 未配置槽中性包 */
    }
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        srv_pa430_torque_test_pack_mit(&tx.data[(uint8_t)((s_motor_ids[i] - 1U) * 8U)],
            s_target_raw, s_vel_ref_raw, s_kp_raw, s_kd_raw,
            SRV_PA430_TQ_RAW_0);
    }
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 经典 CAN 诊断测试帧发送（仅 SRV_PA430_CLASSIC_TEST=1 时由 step 周期调用）
 * @note  经典帧 ID 0x10、DLC 8、data 填 0xAA 测试图案，全程 1M 仲裁时序；
 *        用于验证 CAN2 收发器在经典模式下的总线驱动能力（隔离 5M FD 数据段问题）
 */
#if SRV_PA430_CLASSIC_TEST
static void srv_pa430_torque_test_send_classic(void)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = SRV_PA430_ID_STATUS,
        .is_extended = false,
        .is_fd = false, /* 经典 CAN */
        .dlc = 8,
    };
    memset(tx.data, 0xAA, sizeof(tx.data));
    drv_can_send(DRV_CAN_CH_2, &tx);
}
#endif

/**
 * @brief 打印反馈错误码详情（主循环上下文调用）
 * @note  错误码可组合（多位同时置位）；WARN 打印原始码并逐位解码
 */
static void srv_pa430_torque_test_err_print(uint8_t addr, uint16_t code)
{
    SRV_PA430_TORQUE_TEST_LOG_W("电机 ID=%u 错误：0x%04X", (unsigned)addr, (unsigned)code);
    for (uint8_t i = 0; i < SRV_PA430_ERR_NUM; i++) {
        if ((code & s_err_map[i].mask) != 0U) {
            SRV_PA430_TORQUE_TEST_LOG_W("  - %s", s_err_map[i].name);
        }
    }
}

#if SRV_PA430_CONFIGURE_MODE
/**
 * @brief 发送单机参数写帧（0x600+ID, DLC 8, 经典 CAN）
 * @param addr  电机地址（1~8）
 * @param index 参数索引（见文档 §5.1）
 * @param value 写入值（4 字节大端）
 */
static void srv_pa430_torque_test_send_param_write(uint8_t addr, uint8_t index, uint32_t value)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = SRV_PA430_PARAM_ID_BASE + addr,
        .is_extended = false,
        .is_fd = false, /* DLC 8 单机参数帧 */
        .dlc = 8,
    };
    tx.data[0] = SRV_PA430_PARAM_HEAD;
    tx.data[1] = index;
    tx.data[2] = (uint8_t)(value >> 24);
    tx.data[3] = (uint8_t)(value >> 16);
    tx.data[4] = (uint8_t)(value >> 8);
    tx.data[5] = (uint8_t)value;
    tx.data[6] = SRV_PA430_PARAM_RW_WRITE;
    tx.data[7] = SRV_PA430_PARAM_TAIL;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 配置全部电机为 MIT 模式（Index 11 = 2）并保存到 Flash
 * @note  保存命令触发 Flash 擦写，供电须稳定 ≥1s（见协议文档 §5.2 警告）
 */
static void srv_pa430_torque_test_configure_mode(void)
{
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        srv_pa430_torque_test_send_param_write(s_motor_ids[i], SRV_PA430_PARAM_INDEX_MODE, 2U);
        srv_pa430_torque_test_send_param_write(s_motor_ids[i], SRV_PA430_PARAM_INDEX_SAVE, 0U);
    }
    SRV_PA430_TORQUE_TEST_LOG_W("已将 %u 台电机 Control Mode 写为 MIT(2) 并保存",
        (unsigned)SRV_PA430_MOTOR_COUNT);
}
#endif
