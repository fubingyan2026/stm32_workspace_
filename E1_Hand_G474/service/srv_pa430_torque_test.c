/**
 * @file    srv_pa430_torque_test.c
 * @brief   Motorevo (PA430) CAN FD 伺服电机 — MIT 力位混合模式来回运动测试实现
 *
 * 电机控制指令按 docs/Motorevo电机CAN协议文档.md 组帧发送（CAN FD 广播帧，
 * 状态帧 0x10 / 控制帧 0x20，DLC 64，每电机槽 (ID-1)*8，多字节大端）。
 * 本模块走 FDCAN2（DRV_CAN_CH_2，PB12 RX / PB13 TX）独立总线。
 *
 * MIT 公式：T_out = Kp×(θ_ref−θ) + Kd×(V_ref−V) + T_ref。本模块令 θ_ref 在各电机
 * 自身运动中心 ±SRV_PA430_ANGLE_AMP_RAD 两端点间交替，Kp/Kd/V_ref 均按
 * docs/Motorevo电机CAN协议文档.md §3.1 缩放换算为 raw（物理单位宏可调），
 * V_ref/T_ref 默认恒为 0；每 CTRL_PERIOD_MS 重发广播控制帧（MIT 需持续下发
 * 以保持刚度）。到位反向采用位置反馈闭环：各电机以自身 ID（1~8）周期回复
 * DLC 8 反馈帧（θ 16bit，与 θ_ref 同标度），主循环 |θ_raw − 目标| ≤ 到达阈值
 * 判定到位；所有配置电机均到位后 θ_ref 翻转到另一端。
 *
 * 运动中心：各电机首次收到反馈帧时的位置被锁存为自身中心（s_center_latched），
 * 目标 = 各自中心 ±SRV_PA430_ANGLE_AMP_RAD；电机掉线恢复时对该电机重新锁存
 * 当前反馈位置为新中心，再从当前位置重启往复。
 *
 * 启动流程：对配置电机发广播使能（0x10, byte7=0xFC）；可选（SRV_PA430_CONFIGURE_MODE）
 * 经 0x600+ID 单机帧写 Control Mode=2（MIT）。累计在线满
 * DURATION_MS（30 天）自动停止并失能。
 *
 * 运行中自愈：在线但反馈使能位（Bit0）=0 的电机（断电重上电/保护下使能后）由主循环
 * 每 ENABLE_RETRY_MS 重发使能，直到反馈 Bit0=1 确认（有活动错误位时跳过）；
 * 电机恢复在线时重新锁存中心并重启往复运动。
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
 * @brief 是否由固件自动校正 Control Mode：1=启动读回 Index 11 后，若确认非 MIT(2)，
 *        经 0x600+ID 写 Control Mode=2（仅 RAM 生效，不发送保存命令，避免触发电机
 *        Flash 擦写）；若已是 MIT 则跳过
 *        0=禁用自动校正（仅读回打印，不写）
 */
#define SRV_PA430_CONFIGURE_MODE 1

/**
 * @brief 经典 CAN 诊断测试开关：置 1 时 CH_2 上的帧改为经典 CAN（DLC 8，全程 1M），
 *        用于隔离「收发器未驱动」vs「5M FD 数据段问题」（经典 CAN 仲裁段与 FD 相同）。
 *        PA430 协议本身要求 CAN FD DLC 64，此开关仅作诊断用，测完置 0。
 */
#define SRV_PA430_CLASSIC_TEST 0

/**
 * @brief 使能/控制帧寻址方式：0=广播帧（0x10 使能 / 0x20 控制，DLC 64 CAN FD）。
 *        本电机固件 0x180426EF（2026-04-18）早于 260617，单机寻址（0x100+ID/0x200+ID）
 *        不受支持（见文档 §1.1），必须使用广播方式；置 1 仅作实验用。
 */
#define SRV_PA430_SINGLE_ADDR 0

/* Private constants ---------------------------------------------------------*/

/* --- 总线电机配置 --- */

/** @brief 广播总线最多电机数（ID 1~8） */
#define SRV_PA430_MAX_MOTORS 8U
/** @brief 实际参与测试的电机数（1~8，须与总线一致） */
#define SRV_PA430_MOTOR_COUNT 1U

/* --- MIT 控制参数（docs/Motorevo电机CAN协议文档.md §3.1；物理单位可调） --- */

/* ===== 用户可调参数（物理单位；内部自动换算 raw） ===== */
/** @brief 反转角度半幅（rad），行程为 ±AMP（默认 ±2.5 rad） */
#define SRV_PA430_ANGLE_AMP_RAD (1.9f)
/** @brief V_ref 目标速度（rad/s），通常 0 */
#define SRV_PA430_VEL_REF_RADPS (0.0f)
/** @brief MIT 刚度 Kp（Nm/rad，0~250） */
#define SRV_PA430_KP_NMPR (26.5f)
/** @brief MIT 阻尼 Kd（Nm/(rad/s)，0~50）；调大→运动更慢更平稳，不影响到位精度 */
#define SRV_PA430_KD_NMPRPDS (50.0f)
/** @brief 到位判定阈值（rad） */
#define SRV_PA430_ARRIVE_THRESHOLD_RAD (0.5f)
/**
 * @brief 目标位置递进限速（rad/s）：θ_ref 每控制周期向终点最多移动
 *        VEL×(CTRL_PERIOD_MS/1000) rad，指令速度受限→电机实际转速≈递进速度，
 *        与 Kd 阻尼正交可叠加限速；置 0 关闭递进（直接跳变到端点）
 */
#define SRV_PA430_TARGET_VEL_RADPS (2.5f)

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

/**
 * @brief 稳态状态日志周期 (ms)：周期打印 目标/反馈 θ/使能/错误，用于观察电机是否在运动。
 *        电机不动时不产生任何事件日志（到位翻转/错误/掉线均无），本日志提供低频可见性；
 *        置 0 禁用。确认摆动正常后可调大或置 0 避免刷屏。
 */
#define SRV_PA430_STATUS_LOG_MS 6000U

/* --- 启动参数读回诊断（仅读，不写 Flash） --- */

/** @brief 参数读回重发周期 (ms)：未收到应答前周期重发读请求 */
#define SRV_PA430_DIAG_READ_RETRY_MS 500U
/** @brief 参数读回总超时 (ms)：超时后按已收到部分打印 */
#define SRV_PA430_DIAG_READ_TIMEOUT_MS 5000U

/** @brief 模式校正：写命令后等待其生效的时间 (ms)（不发送保存命令，模式仅在 RAM 生效） */
#define SRV_PA430_MODE_CORRECT_APPLY_MS 300U
/** @brief 模式校正：读回校验等待超时 (ms) */
#define SRV_PA430_MODE_CORRECT_VERIFY_TIMEOUT_MS 1000U

/* --- Motorevo 协议指令（docs/Motorevo电机CAN协议文档.md §2/§3/§5） --- */

#define SRV_PA430_ID_STATUS 0x10U /**< 广播状态/使能帧标识符（DLC 64，CAN FD） */
#define SRV_PA430_ID_CTRL 0x20U /**< 广播控制帧标识符（DLC 64，CAN FD） */
#define SRV_PA430_ID_STATUS_SINGLE_BASE 0x100U /**< 单机状态/使能帧基址（+ Motor ID，DLC 8） */
#define SRV_PA430_ID_CTRL_SINGLE_BASE 0x200U /**< 单机控制帧基址（+ Motor ID，DLC 8） */
#define SRV_PA430_CMD_ENABLE 0xFCU /**< 槽 byte7：使能命令 */
#define SRV_PA430_CMD_DISABLE 0xFDU /**< 槽 byte7：失能命令 */
#define SRV_PA430_CMD_NONE 0xFFU /**< 槽 byte7：无命令（未配置电机槽） */
#define SRV_PA430_PARAM_ID_BASE 0x600U /**< 单机参数读写帧基址（+ Motor ID） */
#define SRV_PA430_PARAM_HEAD 0x67U /**< 参数帧帧头 */
#define SRV_PA430_PARAM_TAIL 0x76U /**< 参数帧帧尾 */
#define SRV_PA430_PARAM_RW_WRITE 0x15U /**< R/W：写命令 */
#define SRV_PA430_PARAM_RW_READ 0x04U /**< R/W：读命令 */
#define SRV_PA430_PARAM_INDEX_MODE 11U /**< 参数索引：Control Mode */

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

/** @brief 当前方向：1=朝中心+AMP，0=朝中心−AMP */
static uint8_t s_dir;

/** @brief 当前目标 θ_ref 原始值（主循环维护，随递进逐步逼近终点，广播控制帧使用） */
static uint16_t s_target_raw;

/** @brief 目标端点 θ_ref 原始值（递进终点；到位/翻转按此判定，s_target_raw 向它逼近） */
static uint16_t s_target_dest_raw;

/** @brief 运动中心是否已锁存并据此设置目标（首次反馈锁存后置位） */
static bool s_target_armed;

/** @brief 物理参数宏换算出的 raw 值（start() 时由 srv_pa430_torque_test_recalc_raw 计算） */
static uint16_t s_pos_raw_mid; /* 中点（0 rad），未锁存中心前的中性目标 */
static uint16_t s_amp_delta_raw; /* ±AMP 对应的 raw 幅值（中心±该值） */
static uint16_t s_vel_ref_raw; /* 配置槽 V_ref */
static uint16_t s_vel_zero_raw; /* 0 rad/s（未配置槽用） */
static uint16_t s_kp_raw;
static uint16_t s_kd_raw;
static uint16_t s_arrive_thresh_raw;
static uint16_t s_target_step_raw; /* 每控制周期目标递进步长 raw */

/** @brief 每电机运动中心（首次收到反馈时锁存的上电位置，ISR 写主循环读） */
static uint16_t s_motor_center_raw[SRV_PA430_MAX_MOTORS];

/** @brief 每电机运动中心是否已锁存 */
static bool s_center_latched[SRV_PA430_MAX_MOTORS];

/** @brief 每电机最新反馈 θ（16bit 原始值，与 θ_ref 同标度，ISR 写主循环读） */
static uint16_t s_motor_theta_raw[SRV_PA430_MAX_MOTORS];

/** @brief 每电机最新反馈速度（12bit 原始值，ISR 写） */
static uint16_t s_motor_vel_raw[SRV_PA430_MAX_MOTORS];

/** @brief 每电机最新反馈扭矩（12bit 原始值，ISR 写） */
static uint16_t s_motor_tq_raw[SRV_PA430_MAX_MOTORS];

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

/** @brief 上次状态日志打印时间 (millis) */
static uint32_t s_last_status_ms;

/** @brief 启动时读回验证的参数索引列表（11=Control Mode，21=Torque Limit，69=Protocol Type，
 *         10=Firmware Version） */
static const uint8_t s_diag_indices[4] = { 11U, 21U, 69U, 10U };
#define SRV_PA430_DIAG_READ_NUM 4U

/** @brief 参数读回值（每电机每参数，32bit 大端原始值，ISR 写主循环读） */
static uint32_t s_diag_val[SRV_PA430_MAX_MOTORS][SRV_PA430_DIAG_READ_NUM];
/** @brief 参数读回应答已收到标志（ISR 写主循环读） */
static bool s_diag_have[SRV_PA430_MAX_MOTORS][SRV_PA430_DIAG_READ_NUM];
/** @brief 参数读回起始时间 (millis)，0=不在读回阶段 */
static uint32_t s_diag_since_ms;
/** @brief 上次重发参数读请求时间 (millis) */
static uint32_t s_diag_sent_ms;

/** @brief Control Mode 改写状态机状态 */
typedef enum {
    SRV_PA430_MODE_CORRECT_IDLE = 0,
    SRV_PA430_MODE_CORRECT_DISABLE, /* 失能（部分固件使能态拒绝改写模式） */
    SRV_PA430_MODE_CORRECT_WRITE, /* 写 Index 11 = 2（不保存，仅 RAM 生效） */
    SRV_PA430_MODE_CORRECT_WAIT_APPLY, /* 等待改写生效 */
    SRV_PA430_MODE_CORRECT_VERIFY_READ, /* 发读 Index 11 校验 */
    SRV_PA430_MODE_CORRECT_VERIFY_WAIT, /* 等待校验读回 */
    SRV_PA430_MODE_CORRECT_DONE, /* 恢复使能并打印结果 */
} srv_pa430_mode_correct_state_t;

/** @brief 模式校正状态机当前状态 */
static srv_pa430_mode_correct_state_t s_mode_correct_state = SRV_PA430_MODE_CORRECT_IDLE;
/** @brief 模式校正等待截止时间 (millis) */
static uint32_t s_mode_correct_wait_ms;
/** @brief 模式校正结果：true=改写未生效/校验失败 */
static bool s_mode_correct_failed;

/* Private function prototypes -----------------------------------------------*/

static uint8_t srv_pa430_torque_test_find_idx(uint8_t addr);
static bool srv_pa430_torque_test_all_online(void);
static bool srv_pa430_torque_test_is_arrived(uint8_t idx);
static uint16_t srv_pa430_torque_test_target_endpoint(uint8_t idx, uint8_t dir);
static uint8_t srv_pa430_torque_test_first_center_idx(void);
static void srv_pa430_torque_test_ramp_target(void);
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
static void srv_pa430_torque_test_send_param_read(uint8_t addr, uint8_t index);
static void srv_pa430_torque_test_send_diag_reads(void);
#if SRV_PA430_CONFIGURE_MODE
static void srv_pa430_torque_test_mode_correct_step(uint32_t now);
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
 * @note  对配置电机发广播使能（0x10, byte7=0xFC）→ 持续下发 MIT 控制帧（0x20）。
 *        各电机首次收到反馈时锁存该位置为自身运动中心，目标 = 中心 ±SRV_PA430_ANGLE_AMP_RAD，
 *        到位即反向；中心锁存前目标保持中性（0 rad）等待
 */
void srv_pa430_torque_test_start(void)
{
    s_running = true;
    s_start_ms = millis(); /* 30 天自动停止计时起点 */
    s_online_ms = 0; /* 持续在线时长从 0 累计 */
    s_online_last_ms = s_start_ms;
    s_last_status_ms = s_start_ms;

    /* 由物理单位宏换算全部 raw 控制参数 */
    srv_pa430_torque_test_recalc_raw();

    /* 初始目标：中心锁存前保持中性等待；锁存后由 step 置终点为中心+AMP 并递进 */
    s_dir = 1;
    s_target_raw = s_pos_raw_mid;
    s_target_dest_raw = s_pos_raw_mid;
    s_target_armed = false;
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
    memset(s_motor_center_raw, 0, sizeof(s_motor_center_raw));
    memset(s_center_latched, 0, sizeof(s_center_latched));
    s_enable_retry_last_ms = s_start_ms;
    s_enable_stall_since_ms = 0;
    s_enable_warned = false;
    for (uint8_t i = 0; i < SRV_PA430_MAX_MOTORS; i++) {
        s_motor_last_seen_ms[i] = s_start_ms; /* 掉线检测起点 */
    }

    (void)srv_pa430_torque_test_send_enable(true); /* 广播使能全部配置电机 */

    /* 启动参数读回：读 Control Mode(11) / Torque Limit(21) 验证电机实际配置
       （仅读不写，无 Flash 风险；应答由 on_rx 记录，step 周期重发直到应答/超时后打印）。
       读回完成后若确认 Control Mode≠MIT 且 SRV_PA430_CONFIGURE_MODE=1，自动改写 */
    s_diag_since_ms = s_start_ms;
    s_diag_sent_ms = 0U; /* 首个 step 周期立即发送 */
    memset(s_diag_val, 0, sizeof(s_diag_val));
    memset(s_diag_have, 0, sizeof(s_diag_have));

    SRV_PA430_TORQUE_TEST_LOG_I("PA430 来回运动启动：%u 台电机，各自中心 ±%u rad（raw 幅值 0x%04X，Kp 0x%03X Kd 0x%03X）",
        (unsigned)SRV_PA430_MOTOR_COUNT, (unsigned)SRV_PA430_ANGLE_AMP_RAD,
        (unsigned)s_amp_delta_raw, (unsigned)s_kp_raw, (unsigned)s_kd_raw);
}

/**
 * @brief 停止来回运动测试模式：θ_ref 回自身中心 + 广播失能
 * @note  断电前必须失能 (0xFD)，命令发往所有配置电机
 */
void srv_pa430_torque_test_stop(void)
{
    s_running = false;
    const uint8_t idx = srv_pa430_torque_test_first_center_idx();
    s_target_dest_raw = (idx < SRV_PA430_MOTOR_COUNT) ? s_motor_center_raw[idx] : s_pos_raw_mid;
    s_target_raw = s_target_dest_raw; /* 停止时指令直接回中心，尽力下发一帧后失能 */
    srv_pa430_torque_test_send_control();
    (void)srv_pa430_torque_test_send_enable(false);
    SRV_PA430_TORQUE_TEST_LOG_I("PA430 来回运动停止：已回中心并失能");
}

/**
 * @brief 来回运动测试模式周期步进（由 can_task 每 TASK_PERIOD_MS 调用）
 * @note  周期重发广播 MIT 控制帧；各电机首次反馈锁存自身运动中心，目标 = 中心 ±AMP，
 *        所有配置电机到位后 θ_ref 翻转到另一端点；打印错误码变化、掉线告警；在线但
 *        未使能的电机周期重发使能直到反馈 Bit0 确认（断电重上电/保护恢复自动重新使能）；
 *        电机恢复在线时重新锁存其中心为当前反馈位置再重启往复；
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

    /* 首帧反馈提示：确认电机在总线上并正常回帧；首次反馈已在 on_rx 锁存运动中心
       （仅各电机打印一次） */
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_motor_have_fb[i] && !s_fb_logged[i]) {
            s_fb_logged[i] = true;
            SRV_PA430_TORQUE_TEST_LOG_I("电机 ID=%u 已收到反馈：θ=0x%04X，中心锁存 0x%04X",
                (unsigned)s_motor_ids[i], (unsigned)s_motor_theta_raw[i],
                (unsigned)s_motor_center_raw[i]);
        }
    }

    /* 目标初始化：首台电机中心锁存后，把终点设为其中心+AMP，指令从当前位置开始递进 */
    if (!s_target_armed) {
        const uint8_t idx = srv_pa430_torque_test_first_center_idx();
        if (idx < SRV_PA430_MOTOR_COUNT) {
            s_target_armed = true;
            s_dir = 1;
            s_target_dest_raw = srv_pa430_torque_test_target_endpoint(idx, 1U);
            s_target_raw = s_motor_theta_raw[idx]; /* 指令从当前位置起，避免突跳 */
            SRV_PA430_TORQUE_TEST_LOG_I("电机 ID=%u 运动中心 0x%04X，目标 0x%04X 开始往复",
                (unsigned)s_motor_ids[idx], (unsigned)s_motor_center_raw[idx],
                (unsigned)s_target_dest_raw);
        }
    }

    /* 目标递进限速：s_target_raw 每周期向终点逼近（限速宏为 0 时直接跳变） */
    srv_pa430_torque_test_ramp_target();

    /* 到位判定：指令已递进到终点 且 所有已收过反馈的配置电机均 |θ−终点|≤阈值 时翻转。
       翻转后终点远离当前位置，天然去抖，不会连续重复翻转 */
    bool all_arrived = (s_target_raw == s_target_dest_raw);
    if (all_arrived) {
        for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
            if (!s_motor_have_fb[i] || !srv_pa430_torque_test_is_arrived(i)) {
                all_arrived = false;
                break;
            }
        }
    }
    if (all_arrived) {
        const uint8_t idx = srv_pa430_torque_test_first_center_idx();
        if (idx < SRV_PA430_MOTOR_COUNT) {
            s_dir = (s_dir == 1U) ? 0U : 1U;
            s_target_dest_raw = srv_pa430_torque_test_target_endpoint(idx, s_dir);
            SRV_PA430_TORQUE_TEST_LOG_I("全部电机到位，θ_ref 翻转为 0x%04X",
                (unsigned)s_target_dest_raw);
        }
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

    /* 恢复在线：对该电机重新锁存当前反馈位置为新中心，终点=新中心+AMP，
       指令从当前位置开始递进，从当前位置重启往复；实际使能仍交给下方"保持使能"重试循环 */
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_online_evt_pending[i]) {
            s_online_evt_pending[i] = false;
            s_motor_center_raw[i] = s_motor_theta_raw[i];
            s_center_latched[i] = true;
            s_target_dest_raw = srv_pa430_torque_test_target_endpoint(i, 1U);
            s_target_raw = s_motor_theta_raw[i];
            s_dir = 1;
            s_target_armed = true;
            SRV_PA430_TORQUE_TEST_LOG_W("电机 ID=%u 恢复在线，θ=0x%04X，中心重新锁存，目标 0x%04X",
                (unsigned)s_motor_ids[i], (unsigned)s_motor_theta_raw[i],
                (unsigned)s_target_dest_raw);
        }
    }

    /* 保持使能：在线且无活动错误但反馈使能位=0 的电机，周期重发使能直到确认。
       有错误位（保护中）时跳过，避免反复顶撞故障；故障消除后自动重新使能。
       Control Mode 改写流程进行中时暂停，避免其失能步骤被此循环立刻重新使能 */
    bool need_enable = false;
#if SRV_PA430_CONFIGURE_MODE
    if (s_mode_correct_state == SRV_PA430_MODE_CORRECT_IDLE)
#endif
    {
        for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
            if (s_motor_have_fb[i] && (s_motor_err[i] == 0U) && !s_motor_enabled[i]) {
                need_enable = true;
                break;
            }
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

#if (SRV_PA430_STATUS_LOG_MS != 0U)
    /* 稳态状态日志：电机不动时无任何事件日志，此低频日志提供目标/反馈可见性 */
    if ((now - s_last_status_ms) >= SRV_PA430_STATUS_LOG_MS) {
        s_last_status_ms = now;
        for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
            SRV_PA430_TORQUE_TEST_LOG_I("STATUS 电机 ID=%u θ=0x%04X V=0x%03X T=0x%03X 目标=0x%04X 使能=%d 错误=0x%04X",
                (unsigned)s_motor_ids[i], (unsigned)s_motor_theta_raw[i],
                (unsigned)s_motor_vel_raw[i], (unsigned)s_motor_tq_raw[i],
                (unsigned)s_target_raw, s_motor_enabled[i] ? 1 : 0,
                (unsigned)s_motor_err[i]);
        }
    }
#endif

    /* 参数读回：未全部应答时周期重发；全部应答或超时后打印一次并结束读回阶段 */
    if (s_diag_since_ms != 0U) {
        if ((now - s_diag_sent_ms) >= SRV_PA430_DIAG_READ_RETRY_MS) {
            s_diag_sent_ms = now;
            srv_pa430_torque_test_send_diag_reads();
        }
        bool all_have = true;
        for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT && all_have; i++) {
            for (uint8_t k = 0; k < SRV_PA430_DIAG_READ_NUM; k++) {
                if (!s_diag_have[i][k]) {
                    all_have = false;
                    break;
                }
            }
        }
        if (all_have || ((now - s_diag_since_ms) >= SRV_PA430_DIAG_READ_TIMEOUT_MS)) {
            for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
                SRV_PA430_TORQUE_TEST_LOG_I("电机 ID=%u 参数回读（11=Control Mode 1=伺服/2=MIT/3=速度，21=Torque Limit float，69=Protocol 0=FD5M/1=FD4M/2=CAN，10=Firmware）:",
                    (unsigned)s_motor_ids[i]);
                for (uint8_t k = 0; k < SRV_PA430_DIAG_READ_NUM; k++) {
                    if (s_diag_have[i][k]) {
                        SRV_PA430_TORQUE_TEST_LOG_I("  Index %u = 0x%08lX", (unsigned)s_diag_indices[k],
                            (unsigned long)s_diag_val[i][k]);
                    } else {
                        SRV_PA430_TORQUE_TEST_LOG_W("  Index %u 读回超时", (unsigned)s_diag_indices[k]);
                    }
                }
            }
#if SRV_PA430_CONFIGURE_MODE
            srv_pa430_torque_test_configure_mode(); /* 读回确认非 MIT 时自动改写 */
#endif
            s_diag_since_ms = 0U;
        }
    }

#if SRV_PA430_CONFIGURE_MODE
    /* Control Mode 改写状态机（失能→写→校验→恢复使能） */
    srv_pa430_torque_test_mode_correct_step(now);
#endif
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

    /* 参数读回应答帧（0x600+ID, DLC 8）：Byte0=Motor ID, Byte1=Index,
       Byte2..5=值（4 字节大端）, Byte6=0x00, Byte7=0xFF（见文档 §5.2）。
       PA430 总线专用，直接消费；不在此上下文打日志 */
    if (!msg->is_extended && (msg->id > SRV_PA430_PARAM_ID_BASE)
        && (msg->id <= (SRV_PA430_PARAM_ID_BASE + SRV_PA430_MAX_MOTORS))
        && (msg->dlc >= 8U)) {
        const uint8_t idx = srv_pa430_torque_test_find_idx(
            (uint8_t)(msg->id - SRV_PA430_PARAM_ID_BASE));
        if (idx < SRV_PA430_MOTOR_COUNT) {
            const uint8_t pindex = msg->data[1];
            for (uint8_t k = 0; k < SRV_PA430_DIAG_READ_NUM; k++) {
                if (s_diag_indices[k] == pindex) {
                    s_diag_val[idx][k] = ((uint32_t)msg->data[2] << 24)
                        | ((uint32_t)msg->data[3] << 16)
                        | ((uint32_t)msg->data[4] << 8)
                        | msg->data[5];
                    s_diag_have[idx][k] = true;
                    break;
                }
            }
        }
        return true;
    }

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
    if (!s_motor_have_fb[idx]) {
        /* 首次收到反馈：锁存该位置为自身运动中心（仅一次；掉线恢复的重新锁存
           由主循环 s_online_evt_pending 处理） */
        s_motor_center_raw[idx] = s_motor_theta_raw[idx];
        s_center_latched[idx] = true;
    }
    s_motor_have_fb[idx] = true;

    /* V/T 反馈：V = Byte2[7:0]<<4 | Byte3[3:0]（12bit）；T = Byte3[7:4]<<8 | Byte4[7:0]（12bit） */
    s_motor_vel_raw[idx] = (uint16_t)(((uint16_t)msg->data[2] << 4) | (msg->data[3] & 0x0FU));
    s_motor_tq_raw[idx] = (uint16_t)((((uint16_t)msg->data[3] & 0x0FU) << 8) | msg->data[4]);

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
 * @brief 判定电机 idx 是否到达当前目标端点（终点，非递进中的中间值）
 * @param idx 电机索引（0 ~ MOTOR_COUNT-1）
 * @return true=|θ_raw − 终点| ≤ 到达阈值
 */
static bool srv_pa430_torque_test_is_arrived(uint8_t idx)
{
    const uint16_t theta = s_motor_theta_raw[idx];
    const uint16_t target = s_target_dest_raw;
    uint32_t diff;
    if (theta >= target) {
        diff = (uint32_t)(theta - target);
    } else {
        diff = (uint32_t)(target - theta);
    }
    return diff <= (uint32_t)s_arrive_thresh_raw;
}

/**
 * @brief 计算电机中心 ±AMP 端点（钳位到 0~0xFFFF）
 * @param idx 电机索引
 * @param dir 1=中心+AMP，0=中心−AMP
 * @return 端点 raw 值
 */
static uint16_t srv_pa430_torque_test_target_endpoint(uint8_t idx, uint8_t dir)
{
    const uint16_t center = s_motor_center_raw[idx];
    if (dir == 1U) {
        const uint32_t t = (uint32_t)center + s_amp_delta_raw;
        return (t > 0xFFFFU) ? 0xFFFFU : (uint16_t)t;
    }
    return (s_amp_delta_raw >= center) ? 0U : (uint16_t)(center - s_amp_delta_raw);
}

/**
 * @brief 查找首个已锁存运动中心的电机索引
 * @return 索引；均未锁存返回 SRV_PA430_MOTOR_COUNT（哨兵值）
 */
static uint8_t srv_pa430_torque_test_first_center_idx(void)
{
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (s_center_latched[i])
            return i;
    }
    return SRV_PA430_MOTOR_COUNT;
}

/**
 * @brief 目标位置递进：s_target_raw 每周期向 s_target_dest_raw 逼近 ≤s_target_step_raw
 * @note  步长为 0（SRV_PA430_TARGET_VEL_RADPS=0）时递进关闭，直接跳变到终点
 */
static void srv_pa430_torque_test_ramp_target(void)
{
    if (s_target_raw == s_target_dest_raw)
        return;
    if (s_target_step_raw == 0U) {
        s_target_raw = s_target_dest_raw;
        return;
    }
    int32_t diff = (int32_t)s_target_dest_raw - (int32_t)s_target_raw;
    const int32_t step = (int32_t)s_target_step_raw;
    if (diff > step) {
        diff = step;
    } else if (diff < -step) {
        diff = -step;
    }
    s_target_raw = (uint16_t)((int32_t)s_target_raw + diff);
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
    s_pos_raw_mid = srv_pa430_theta_to_raw(0.0f);
    s_amp_delta_raw = srv_pa430_delta_theta_to_raw(SRV_PA430_ANGLE_AMP_RAD);
    s_vel_ref_raw = srv_pa430_vel_to_raw(SRV_PA430_VEL_REF_RADPS);
    s_vel_zero_raw = srv_pa430_vel_to_raw(0.0f);
    s_kp_raw = srv_pa430_gain_to_raw(SRV_PA430_KP_NMPR, SRV_PA430_KP_MAX_NMPR);
    s_kd_raw = srv_pa430_gain_to_raw(SRV_PA430_KD_NMPRPDS, SRV_PA430_KD_MAX_NMPRPDS);
    s_arrive_thresh_raw = srv_pa430_delta_theta_to_raw(SRV_PA430_ARRIVE_THRESHOLD_RAD);
    /* 每周期目标递进步长：VEL×(周期秒)，再换算为 raw 计数 */
    s_target_step_raw = srv_pa430_delta_theta_to_raw(
        SRV_PA430_TARGET_VEL_RADPS * ((float)SRV_PA430_CTRL_PERIOD_MS / 1000.0f));
}

/**
 * @brief 发送使能/失能帧
 * @param enable true=使能 (byte7=0xFC)，false=失能 (byte7=0xFD)
 * @return true=帧已入 TX FIFO；false=TX 未就绪未发送（调用方可决定何时重试）
 * @note  单机寻址（SRV_PA430_SINGLE_ADDR=1）：每配置电机一帧 0x100+ID，DLC 8 经典
 *        （FD 与经典协议均支持，见文档 §1.1/§1.5，Byte0~6=0xFF 固定，Byte7=命令）；
 *        广播寻址（=0）：0x10 全槽 byte7 写命令（FD 模式 DLC 64，未配置槽 0xFF 无命令）
 */
static bool srv_pa430_torque_test_send_enable(bool enable)
{
    const uint8_t cmd = enable ? SRV_PA430_CMD_ENABLE : SRV_PA430_CMD_DISABLE;
#if SRV_PA430_SINGLE_ADDR
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (!drv_can_tx_ready(DRV_CAN_CH_2))
            return false;
        drv_can_msg_t tx = {
            .id = SRV_PA430_ID_STATUS_SINGLE_BASE + s_motor_ids[i],
            .is_extended = false,
            .is_fd = false,
            .dlc = 8,
        };
        memset(tx.data, 0xFF, sizeof(tx.data));
        tx.data[7] = cmd;
        drv_can_send(DRV_CAN_CH_2, &tx);
    }
#else
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return false;
    drv_can_msg_t tx = {
        .id = SRV_PA430_ID_STATUS,
        .is_extended = false,
        .is_fd = !SRV_PA430_CLASSIC_TEST, /* 经典诊断模式发经典帧 */
        .dlc = SRV_PA430_CLASSIC_TEST ? 8U : 64U,
    };
    memset(tx.data, SRV_PA430_CMD_NONE, sizeof(tx.data)); /* 全槽 byte7=0xFF 无命令 */
#if SRV_PA430_CLASSIC_TEST
    tx.data[7] = cmd; /* 经典模式仅第一槽 */
#else
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        tx.data[(uint8_t)((s_motor_ids[i] - 1U) * 8U) + 7U] = cmd;
    }
#endif
    drv_can_send(DRV_CAN_CH_2, &tx);
#endif
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
 * @brief 发送 MIT 控制帧
 * @note  单机寻址（SRV_PA430_SINGLE_ADDR=1）：每配置电机一帧 0x200+ID，DLC 8 经典，
 *        Byte0~7 直接为 8 字节 MIT 封包（文档 §3.1 单独方式，FD 与经典协议均支持）；
 *        广播寻址（=0）：0x20 一帧 DLC 64，配置槽写目标+刚度，未配置槽发中性包
 */
static void srv_pa430_torque_test_send_control(void)
{
#if SRV_PA430_SINGLE_ADDR
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (!drv_can_tx_ready(DRV_CAN_CH_2))
            return;
        drv_can_msg_t tx = {
            .id = SRV_PA430_ID_CTRL_SINGLE_BASE + s_motor_ids[i],
            .is_extended = false,
            .is_fd = false,
            .dlc = 8,
        };
        srv_pa430_torque_test_pack_mit(tx.data, s_target_raw, s_vel_ref_raw,
            s_kp_raw, s_kd_raw, SRV_PA430_TQ_RAW_0);
        drv_can_send(DRV_CAN_CH_2, &tx);
    }
#else
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
#endif
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

/**
 * @brief 发送单机参数读帧（0x600+ID, DLC 8, 经典 CAN）
 * @param addr  电机地址（1~8）
 * @param index 参数索引（见文档 §5.1，11=Control Mode，21=Torque Limit）
 * @note  电机应答帧（0x600+ID）由 srv_pa430_torque_test_on_rx 解析记录；
 *        仅读命令，不触发任何 Flash 擦写
 */
static void srv_pa430_torque_test_send_param_read(uint8_t addr, uint8_t index)
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
    tx.data[2] = 0;
    tx.data[3] = 0;
    tx.data[4] = 0;
    tx.data[5] = 0;
    tx.data[6] = SRV_PA430_PARAM_RW_READ;
    tx.data[7] = SRV_PA430_PARAM_TAIL;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 重发全部尚未收到应答的参数读请求
 */
static void srv_pa430_torque_test_send_diag_reads(void)
{
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        for (uint8_t k = 0; k < SRV_PA430_DIAG_READ_NUM; k++) {
            if (!s_diag_have[i][k]) {
                srv_pa430_torque_test_send_param_read(s_motor_ids[i], s_diag_indices[k]);
            }
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
 * @brief 启动 Control Mode 改写流程（由启动参数读回完成后调用）
 * @note  仅当 Index 11 读回已应答且确认非 MIT 时触发状态机；已是 MIT 或读回超时的
 *        电机跳过（不盲写 Flash）。状态机流程：失能→写 2→等待生效→读回校验→恢复使能。
 *        不发送保存命令：模式仅在 RAM 生效，电机断电后恢复 Flash 内原模式
 */
static void srv_pa430_torque_test_configure_mode(void)
{
    bool need_correct = false;
    for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
        if (!s_diag_have[i][0]) {
            SRV_PA430_TORQUE_TEST_LOG_W("电机 ID=%u Control Mode 读回超时，跳过自动改写",
                (unsigned)s_motor_ids[i]);
            continue;
        }
        if (s_diag_val[i][0] == 0x02000000UL) {
            SRV_PA430_TORQUE_TEST_LOG_I("电机 ID=%u Control Mode 已是 MIT(2)，跳过配置",
                (unsigned)s_motor_ids[i]);
            continue;
        }
        SRV_PA430_TORQUE_TEST_LOG_W("电机 ID=%u Control Mode=0x%08lX 非 MIT，启动改写流程（失能→写2→校验→恢复使能，仅 RAM 生效不保存）",
            (unsigned)s_motor_ids[i], (unsigned long)s_diag_val[i][0]);
        need_correct = true;
    }
    if (need_correct) {
        s_mode_correct_state = SRV_PA430_MODE_CORRECT_DISABLE;
        s_mode_correct_failed = false;
    }
}

/**
 * @brief Control Mode 改写状态机步进（由 step 每周期调用，仅在非 IDLE 时动作）
 * @param now 当前毫秒时间
 * @note  流程：失能 → 写 Control Mode=2 → 等待生效 → 读回校验 → 恢复使能并打印结果。
 *        不发送保存命令（仅 RAM 生效，断电恢复原模式）；改写失败（未生效/校验超时）
 *        仅打印告警，电机保持使能但不响应 MIT 帧，需改用伺服模式驱动或人工检查
 */
static void srv_pa430_torque_test_mode_correct_step(uint32_t now)
{
    switch (s_mode_correct_state) {
    case SRV_PA430_MODE_CORRECT_DISABLE:
        (void)srv_pa430_torque_test_send_enable(false); /* 部分固件使能态拒绝改写模式 */
        s_mode_correct_state = SRV_PA430_MODE_CORRECT_WRITE;
        break;

    case SRV_PA430_MODE_CORRECT_WRITE:
        for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
            srv_pa430_torque_test_send_param_write(s_motor_ids[i], SRV_PA430_PARAM_INDEX_MODE, 2U);
        }
        s_mode_correct_wait_ms = now + SRV_PA430_MODE_CORRECT_APPLY_MS;
        s_mode_correct_state = SRV_PA430_MODE_CORRECT_WAIT_APPLY;
        break;

    case SRV_PA430_MODE_CORRECT_WAIT_APPLY:
        if (now < s_mode_correct_wait_ms) {
            break; /* 等待改写生效 */
        }
        s_mode_correct_state = SRV_PA430_MODE_CORRECT_VERIFY_READ;
        break;

    case SRV_PA430_MODE_CORRECT_VERIFY_READ:
        for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
            s_diag_have[i][0] = false; /* 清标志，等待校验读回 */
            srv_pa430_torque_test_send_param_read(s_motor_ids[i], s_diag_indices[0]); /* Index 11 */
        }
        s_mode_correct_wait_ms = now + SRV_PA430_MODE_CORRECT_VERIFY_TIMEOUT_MS;
        s_mode_correct_state = SRV_PA430_MODE_CORRECT_VERIFY_WAIT;
        break;

    case SRV_PA430_MODE_CORRECT_VERIFY_WAIT: {
        bool answered = true;
        bool all_ok = true;
        for (uint8_t i = 0; i < SRV_PA430_MOTOR_COUNT; i++) {
            if (!s_diag_have[i][0]) {
                answered = false;
                all_ok = false;
                break;
            }
            if (s_diag_val[i][0] != 0x02000000UL) {
                all_ok = false;
            }
        }
        if (answered) {
            s_mode_correct_failed = !all_ok;
            s_mode_correct_state = SRV_PA430_MODE_CORRECT_DONE;
        } else if (now >= s_mode_correct_wait_ms) {
            s_mode_correct_failed = true; /* 校验读回超时 */
            s_mode_correct_state = SRV_PA430_MODE_CORRECT_DONE;
        }
        break;
    }

    case SRV_PA430_MODE_CORRECT_DONE:
        (void)srv_pa430_torque_test_send_enable(true); /* 恢复使能 */
        if (s_mode_correct_failed) {
            SRV_PA430_TORQUE_TEST_LOG_W("Control Mode 改写未生效或校验失败，电机仍非 MIT（供电不稳/固件拒绝？）；"
                "MIT 控制帧将不被执行，需改用伺服模式驱动或人工检查");
        } else {
            SRV_PA430_TORQUE_TEST_LOG_I("Control Mode 已确认为 MIT(2)，PA430 MIT 控制生效");
        }
        s_mode_correct_state = SRV_PA430_MODE_CORRECT_IDLE;
        break;

    default:
        s_mode_correct_state = SRV_PA430_MODE_CORRECT_IDLE;
        break;
    }
}
#endif
