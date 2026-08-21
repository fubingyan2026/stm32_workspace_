/**
 * @file    srv_tongzhi_torque_test.c
 * @brief   良志(ODrive) 伺服执行器 CAN 控制协议服务实现 — 位置模式梯形轨迹往复耐久测试
 *
 * 电机控制指令按 docs/良志电机can协议.md 组帧发送（经典 CAN 2.0A, 1 Mbps,
 * CAN-ID = (node_id<<5)|cmd_id，8 字节帧，小端，IEEE-754 单精度浮点）。
 *
 * 本模块走 FDCAN1（DRV_CAN_CH_1），与苇熠(HT) 测试通过 srv_motor_test_select.h
 * 的 SRV_MOTOR_TEST_SELECT 三选一；PA430(Motorevo) 在 FDCAN2 独立并行，互不干扰。
 *
 * 控制方式：位置模式 + 梯形轨迹。对发现的电机下发 清错 → Set_Axis_State(8=闭环) →
 * Set_Controller_Mode(control=3 position, input=5 trap_traj) → Set_Traj_Vel_Limit /
 * Set_Traj_Accel_Limits；之后 Set_Input_Pos 在「初始化零点」±SRV_TONGZHI_POS_AMP_TURNS
 * 两端点间交替（零点 = 各电机初始化完成、首次进入闭环时的编码器位置，即读回的当前
 * 位置；掉线恢复后重新锁存；未锁存前暂按绝对 ±AMP 下发）。
 * ODrive 的 Set_Input_Pos 为锁存式目标（梯形规划器自动跑到位并保持），无需像 MIT 模式
 * 那样高频持续下发保持刚度；到位判定靠电机周期推送的编码器位置(0x09)，全部已闭环电机
 * 到位后目标翻转（未闭环/掉线/无编码器反馈的电机不参与判定，镜像 PA430）。
 *
 * 多电机寻址：ODrive 无主机握手命令，但电机上电后周期主动推送心跳(0x01, 默认 100ms)；
 * 本模块靠监听心跳被动发现 node_id，动态收录并初始化新电机（掉线电机仍在列表中，
 * 恢复在线时由主循环重发闭环与目标）。
 *
 * RX 侧：心跳/编码器帧由 can_task 按 CH_1 分发到 on_rx() 解析记录（ISR 上下文不打日志），
 * 主循环消费到位/错误/在线标志；FDCAN1 在本模块使能时为专用总线。
 */

#include "srv_tongzhi_torque_test.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/
/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印（耐久测试确认摆动正常后可关闭） */
#define SRV_TONGZHI_TORQUE_TEST_LOG_ENABLE 0

#if SRV_TONGZHI_TORQUE_TEST_LOG_ENABLE
#define SRV_TONGZHI_TORQUE_TEST_LOG_I(...) LOG_I("tongzhi_test", __VA_ARGS__)
#define SRV_TONGZHI_TORQUE_TEST_LOG_W(...) LOG_W("tongzhi_test", __VA_ARGS__)
#define SRV_TONGZHI_TORQUE_TEST_LOG_E(...) LOG_E("tongzhi_test", __VA_ARGS__)
#else
#define SRV_TONGZHI_TORQUE_TEST_LOG_I(...) ((void)0)
#define SRV_TONGZHI_TORQUE_TEST_LOG_W(...) ((void)0)
#define SRV_TONGZHI_TORQUE_TEST_LOG_E(...) ((void)0)
#endif

/* 模块测试开关 ----------------------------------------------------------------*/

/** @brief 电机测试模式：1=上电自动启动（心跳被动发现 + 往复驱动）；0=需手动调用 start() */
#define SRV_TONGZHI_AUTO_START 1

/* Private constants ---------------------------------------------------------*/

/* --- 总线电机配置 --- */

/** @brief 总线最多可管理电机数（node_id 协议范围 0~63，本模块限 8） */
#define SRV_TONGZHI_MAX_MOTORS 8U

/* --- 往复运动参数（物理单位可调） --- */

/** @brief 往复半幅（转）：目标在 初始化零点 ±AMP 两端点间交替（零点=初始化完成时的当前位置） */
#define SRV_TONGZHI_POS_AMP_TURNS (6.5f)
/** @brief 到位判定阈值（转）：|编码器位置−目标| ≤ 该值视为到位（随后翻转目标） */
#define SRV_TONGZHI_ARRIVE_THRESH_TURNS (0.25f)
/** @brief 梯形轨迹最大速度（转/s） */
#define SRV_TONGZHI_TRAJ_VEL_LIMIT_TPS (12.0f)
/** @brief 梯形轨迹加速度（转/s²） */
#define SRV_TONGZHI_TRAJ_ACCEL_TPS2 (2.0f)
/** @brief 梯形轨迹减速度（转/s²） */
#define SRV_TONGZHI_TRAJ_DECEL_TPS2 (2.0f)

/* --- 周期 --- */

/** @brief 电机无心跳判定周期 (ms)：超过该时长未收到任何帧视为掉线/断电（心跳默认 100ms，2s≈20 帧） */
#define SRV_TONGZHI_NORESP_PERIOD_MS 2000U
/** @brief 未闭环电机重发 清错+闭环+模式+梯形参数 的周期 (ms) */
#define SRV_TONGZHI_ENABLE_RETRY_MS 200U
/** @brief 闭环确认超时：连续重试仍未确认时打印一次告警（继续重试） */
#define SRV_TONGZHI_ENABLE_TIMEOUT_MS 2000U
/** @brief 周期重发当前目标周期 (ms)：锁存式目标，重发同目标为安全 no-op，防首帧/丢帧漏目标 */
#define SRV_TONGZHI_TARGET_RESEND_MS 500U
/** @brief 主动探测周期 (ms)：对下一个候选 node 发 Get_Error(0x03)，静默电机靠回包被发现 */
#define SRV_TONGZHI_PROBE_INTERVAL_MS 200U
/** @brief 发现窗口 (ms)：启动后该时长内主动探测 node 0~MAX-1；结束后仍无发现则回退盲发 */
#define SRV_TONGZHI_FALLBACK_MS 2000U
/** @brief 回退默认 node_id：探测无果后按该 ID 盲发（镜像 HT 的 DEFAULT_MOTOR_ADDR） */
#define SRV_TONGZHI_FALLBACK_NODE 0U
/** @brief 假定闭环宽限 (ms)：init 下发后该时长内无心跳 axis_state=8 确认 → 假定已闭环 */
#define SRV_TONGZHI_INIT_GRACE_MS 1000U
/** @brief 无位置反馈时的单程移动估计时间 (ms)：定时翻转用（按 amp/vel/accel 粗调） */
#define SRV_TONGZHI_TRAVEL_EST_MS 2500U
/** @brief 位置到位判定超时 (ms)：超过该时长仍未到位则强制翻转，避免反馈冻结/到位偏置导致永久停摆 */
#define SRV_TONGZHI_ARRIVE_TIMEOUT_MS 60000U
/** @brief 周期状态诊断日志间隔 (ms)：打印受控电机 enc/target/axis_state/error/在线年龄 */
#define SRV_TONGZHI_STATUS_LOG_MS 5000U
/** @brief 未发现电机时周期告警日志间隔 (ms) */
#define SRV_TONGZHI_NO_MOTOR_LOG_MS 5000U
/**
 * @brief 耐久运行时长 (ms)：电机持续在线累计满 30 天自动停止；
 *        30 天 = 2592000000 ms（uint32 范围内）；置 0 禁用自动停止
 */
#define SRV_TONGZHI_DURATION_MS 2592000000U

/* --- 良志(ODrive) 协议指令（docs/良志电机can协议.md §3） --- */

#define SRV_TONGZHI_CMD_HEARTBEAT 0x01U /**< 心跳：电机→主机，周期推送（默认 100ms），DLC 8 */
#define SRV_TONGZHI_CMD_SET_AXIS_STATE 0x07U /**< 设置轴状态：data[0..3]=axis_state uint32 小端 */
#define SRV_TONGZHI_CMD_ENCODER_ESTIMATES 0x09U /**< 编码器估计：电机→主机，周期推送（默认 10ms），DLC 8 */
#define SRV_TONGZHI_CMD_SET_CONTROLLER_MODE 0x0BU /**< 设置控制模式：control_mode+input_mode 各 uint32 */
#define SRV_TONGZHI_CMD_SET_INPUT_POS 0x0CU /**< 设置目标位置：pos float32(转)+vel_ff int16+torque_ff int16 */
#define SRV_TONGZHI_CMD_SET_TRAJ_VEL_LIMIT 0x11U /**< 梯形速度限制：float32(转/s) */
#define SRV_TONGZHI_CMD_SET_TRAJ_ACCEL_LIMITS 0x12U /**< 梯形加速度/减速度：两个 float32 */
#define SRV_TONGZHI_CMD_CLEAR_ERRORS 0x18U /**< 清除错误：8×0 */
#define SRV_TONGZHI_CMD_GET_ERROR 0x03U /**< 获取错误：data[0]=error_type；电机回错误码帧（主动探测用） */

#define SRV_TONGZHI_AXIS_STATE_IDLE 1U /**< 空闲 */
#define SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP 8U /**< 闭环控制 */
#define SRV_TONGZHI_CONTROL_MODE_POSITION 3U /**< 位置控制 */
#define SRV_TONGZHI_INPUT_MODE_TRAP_TRAJ 5U /**< 梯形轨迹输入 */

/* Private types -------------------------------------------------------------*/

/** @brief 错误码描述表项（见 docs/良志电机can协议.md §5.3，ODrive axis.error 位） */
typedef struct {
    uint32_t mask; /**< 错误码位 */
    const char* name; /**< 含义 */
} srv_tongzhi_torque_test_err_desc_t;

/** @brief 错误码 → 含义映射表（0x03 回包为 motor.error，error_type=0；心跳 axis_error 为 axis.error，
 *        两者低 8 位含义不同，此处以 motor.error 为主并保留文档 §5.3 的常见位；始终打印原始 hex） */
static const srv_tongzhi_torque_test_err_desc_t s_err_map[] = {
    { 0x00000001U, "超速(OVERSPEED)" },
    { 0x00000002U, "母线欠压(DC_BUS_UNDER_VOLTAGE)" },
    { 0x00000004U, "母线过压(DC_BUS_OVER_VOLTAGE)" },
    { 0x00000040U, "编码器速度超限(ENCODER_VELOCITY_LIMIT)" },
    { 0x00000080U, "编码器索引未找到(INDEX_NOT_FOUND)" },
    { 0x00000400U, "电流采样饱和(CURRENT_SENSE_SATURATION)" },
    { 0x00001000U, "下限位触发(MIN_ENDSTOP_PRESSED)" },
    { 0x00004000U, "紧急停止(ESTOP_REQUESTED)" },
    { 0x00020000U, "电机过热(MOTOR_THERMISTOR_OVER_TEMP)" },
    { 0x00040000U, "驱动过热(FET_THERMISTOR_OVER_TEMP)" },
};

/** @brief 错误码描述表项数 */
#define SRV_TONGZHI_ERR_NUM (sizeof(s_err_map) / sizeof(s_err_map[0]))

/* Private variables ---------------------------------------------------------*/

/** @brief 测试模式运行标志 */
static bool s_running;

/** @brief 已发现电机 node_id 列表（心跳 0x01 被动收录，ISR 写入主循环读取） */
static uint8_t s_motor_ids[SRV_TONGZHI_MAX_MOTORS];

/** @brief 已发现电机数量（ISR 写入，主循环读取） */
static uint8_t s_motor_cnt;

/** @brief 已打印日志的电机数（避免重复打印发现日志） */
static uint8_t s_scan_log_cnt;

/** @brief 当前目标方向：+1=朝 +AMP 端点，-1=朝 -AMP 端点（所有电机同向摆动） */
static int8_t s_dir;

/** @brief 每电机往复中心（转）：各电机初始化完成（首次进入闭环）时的读回位置为自身 0 点，
 *        目标在 各自中心±AMP 间交替（多电机各以自身零点摆动） */
static float s_center_turns[SRV_TONGZHI_MAX_MOTORS];

/** @brief 每电机往复中心是否已锁存（init 完成且已有编码器反馈后锁存；掉线恢复后重新锁存） */
static bool s_center_latched[SRV_TONGZHI_MAX_MOTORS];

/** @brief 每电机最新心跳轴状态（ISR 写） */
static uint8_t s_motor_axis_state[SRV_TONGZHI_MAX_MOTORS];

/** @brief 每电机上次轴状态（主循环检测 0/非闭环 → 8 上升沿打印闭环确认） */
static uint8_t s_axis_state_prev[SRV_TONGZHI_MAX_MOTORS];

/** @brief 每电机最新错误码（心跳 axis_error，ISR 写） */
static uint32_t s_motor_err[SRV_TONGZHI_MAX_MOTORS];

/** @brief 新错误应答待打印标志（ISR 置位，主循环清零打印） */
static bool s_err_pending[SRV_TONGZHI_MAX_MOTORS];

/** @brief 每电机最新编码器位置（转，ISR 写主循环读） */
static float s_motor_encoder_turns[SRV_TONGZHI_MAX_MOTORS];

/** @brief 每电机是否已收到过编码器帧（未收到不参与到位判定，避免初始 0 值误判） */
static bool s_motor_have_encoder[SRV_TONGZHI_MAX_MOTORS];

/** @brief 每电机最后收到任何帧的时间 (millis)，用于掉线检测（ISR 更新） */
static uint32_t s_motor_last_seen_ms[SRV_TONGZHI_MAX_MOTORS];

/** @brief 电机无响应告警锁存（收到任何帧后清除，避免重复刷屏） */
static bool s_motor_nresp_latch[SRV_TONGZHI_MAX_MOTORS];

/** @brief 恢复在线事件待处理标志（ISR 置位，主循环清零后重发闭环+目标） */
static bool s_online_evt_pending[SRV_TONGZHI_MAX_MOTORS];

/** @brief 上次重发 清错+闭环+模式+梯形参数 的时间 (millis) */
static uint32_t s_last_enable_retry_ms;

/** @brief 进入"需闭环"状态的起始时间 (millis)，0=不在需闭环状态 */
static uint32_t s_enable_stall_since_ms;

/** @brief 闭环确认超时告警是否已打印（确认闭环或离开需闭环状态后复位） */
static bool s_enable_warned;

/** @brief 上次周期重发目标的时间 (millis) */
static uint32_t s_last_target_resend_ms;

/** @brief 主动探测：下一个候选 node（0~MAX-1 轮转） */
static uint8_t s_probe_idx;

/** @brief 错误回查：下一个待回查的已发现电机索引（发现窗口后轮转） */
static uint8_t s_err_query_idx;

/** @brief 主动探测上次发送时间 (millis) */
static uint32_t s_last_probe_ms;

/** @brief 发现窗口结束标志（探测停止，进入回退判定） */
static bool s_probe_done;

/** @brief 已回退到默认 node_id 盲发标志 */
static bool s_fallback_active;

/** @brief 每电机最近一次首次 init 下发时间 (millis)，0=未 init 过（宽限判定用，不随重发刷新） */
static uint32_t s_last_init_ms[SRV_TONGZHI_MAX_MOTORS];

/** @brief 每电机假定闭环标志（无心跳确认时置位，避免反复重发 init 且不阻塞翻转） */
static bool s_assumed_closed[SRV_TONGZHI_MAX_MOTORS];

/**
 * @brief 每电机 init 步进序号（0..4=待发帧序号，5=已全部发完）
 * @note  motor_init 拆成 5 帧逐次下发（每次 ENABLE_RETRY_MS 发 1 帧），避免一次连发 5 帧
 *        超出 FDCAN TX FIFO 深度（G4 固定 3）导致 Set_Traj_Vel_Limit/Set_Traj_Accel_Limits
 *        被静默丢弃——那是梯形限速/限加从未生效、电机按默认高速跑导致超速(0x40)的根因。
 */
static uint8_t s_init_step[SRV_TONGZHI_MAX_MOTORS];

/** @brief 最近一次目标翻转时间 (millis)，定时翻转用 */
static uint32_t s_last_flip_ms;

/** @brief 上次「仍未发现电机」日志时间 (millis) */
static uint32_t s_last_no_motor_log_ms;

/** @brief 上次周期状态诊断日志时间 (millis) */
static uint32_t s_last_status_log_ms;

/** @brief 测试起始时间 (millis) */
static uint32_t s_start_ms;

/** @brief 累计在线时长 (ms)：仅所有电机在线时累加，用于 DURATION 自动停止 */
static uint32_t s_online_ms;

/** @brief 上次在线时长累计时间点 (millis) */
static uint32_t s_online_last_ms;

/* Private function prototypes -----------------------------------------------*/

static uint8_t srv_tongzhi_torque_test_find_idx(uint8_t node);
static bool srv_tongzhi_torque_test_all_online(void);
static uint32_t srv_tongzhi_torque_test_can_id(uint8_t node, uint8_t cmd);
static void srv_tongzhi_torque_test_pack_float_le(uint8_t* dst, float v);
static void srv_tongzhi_torque_test_pack_u32_le(uint8_t* dst, uint32_t v);
static void srv_tongzhi_torque_test_pack_i16_le(uint8_t* dst, int16_t v);
static float srv_tongzhi_torque_test_unpack_float_le(const uint8_t* src);
static uint32_t srv_tongzhi_torque_test_unpack_u32_le(const uint8_t* src);
static void srv_tongzhi_torque_test_send_axis_state(uint8_t node, uint8_t state);
static void srv_tongzhi_torque_test_send_controller_mode(uint8_t node);
static void srv_tongzhi_torque_test_send_traj_vel_limit(uint8_t node);
static void srv_tongzhi_torque_test_send_traj_accel_limits(uint8_t node);
static void srv_tongzhi_torque_test_send_input_pos(uint8_t node, float turns);
static void srv_tongzhi_torque_test_send_clear_errors(uint8_t node);
static void srv_tongzhi_torque_test_send_get_error(uint8_t node);
static void srv_tongzhi_torque_test_motor_init_step(uint8_t node, uint8_t step);
static void srv_tongzhi_torque_test_probe_step(uint32_t now);
static void srv_tongzhi_torque_test_fallback(uint32_t now);
static float srv_tongzhi_torque_test_motor_target(uint8_t idx);
static bool srv_tongzhi_torque_test_in_control(uint8_t idx);
static bool srv_tongzhi_torque_test_target_ready(uint8_t idx);
static void srv_tongzhi_torque_test_scan_record(uint8_t node);
static void srv_tongzhi_torque_test_scan_log_new(void);
static void srv_tongzhi_torque_test_err_print(uint8_t node, uint32_t code);

/* Exported functions --------------------------------------------------------*/

void srv_tongzhi_torque_test_init(void)
{
    delay_ms(2000);
#if SRV_TONGZHI_AUTO_START
    srv_tongzhi_torque_test_start(); /* 测试模式：心跳被动发现 + 往复驱动 */
#endif
}

/**
 * @brief 启动往复耐久测试模式
 * @note   ODride 电机可能不发周期帧（heartbeat_rate_ms=0）：本模块在发现窗口内
 *         主动发 Get_Error(0x03) 探测 node 0~MAX-1，靠回包被动收录；同时继续监听
 *         心跳被动发现。无任何发现时回退默认 node_id 盲发。
 */
void srv_tongzhi_torque_test_start(void)
{
    s_running = true;
    s_start_ms = millis(); /* 30 天自动停止计时起点 */
    s_online_ms = 0; /* 持续在线时长从 0 累计 */
    s_online_last_ms = s_start_ms;

    /* 初始方向：朝 +AMP 端点；零点未锁存时暂按绝对 ±AMP 下发，锁存后自动校正为 各自中心±AMP */
    s_dir = 1;
    memset(s_center_turns, 0, sizeof(s_center_turns));
    memset(s_center_latched, 0, sizeof(s_center_latched));
    s_last_enable_retry_ms = s_start_ms;
    s_enable_stall_since_ms = 0;
    s_enable_warned = false;
    s_last_target_resend_ms = s_start_ms;
    s_probe_idx = 0;
    s_err_query_idx = 0;
    s_last_probe_ms = s_start_ms;
    s_probe_done = false;
    s_fallback_active = false;
    memset(s_last_init_ms, 0, sizeof(s_last_init_ms));
    memset(s_assumed_closed, 0, sizeof(s_assumed_closed));
    memset(s_init_step, 0, sizeof(s_init_step));
    s_last_flip_ms = s_start_ms;
    s_last_no_motor_log_ms = s_start_ms;
    s_last_status_log_ms = s_start_ms;

    memset(s_motor_ids, 0, sizeof(s_motor_ids));
    memset(s_motor_axis_state, 0, sizeof(s_motor_axis_state));
    memset(s_axis_state_prev, 0, sizeof(s_axis_state_prev));
    memset(s_motor_err, 0, sizeof(s_motor_err));
    memset(s_err_pending, 0, sizeof(s_err_pending));
    memset(s_motor_encoder_turns, 0, sizeof(s_motor_encoder_turns));
    memset(s_motor_have_encoder, 0, sizeof(s_motor_have_encoder));
    memset(s_motor_nresp_latch, 0, sizeof(s_motor_nresp_latch));
    memset(s_online_evt_pending, 0, sizeof(s_online_evt_pending));
    s_motor_cnt = 0;
    s_scan_log_cnt = 0;
    for (uint8_t i = 0; i < SRV_TONGZHI_MAX_MOTORS; i++) {
        s_motor_last_seen_ms[i] = s_start_ms; /* 掉线检测起点 */
    }

    SRV_TONGZHI_TORQUE_TEST_LOG_I("良志(ODrive) 往复启动：心跳被动发现 + 主动探测 node 0~%u（初始化零点 ±%d 毫转，梯形限速 %d 毫转/s）",
        (unsigned)(SRV_TONGZHI_MAX_MOTORS - 1U),
        (int)(SRV_TONGZHI_POS_AMP_TURNS * 1000.0f), (int)(SRV_TONGZHI_TRAJ_VEL_LIMIT_TPS * 1000.0f));
}

/**
 * @brief 停止往复耐久测试模式：电机回 IDLE（发往所有已发现电机）
 * @note  IDLE 后电机失力，可安全断电
 */
void srv_tongzhi_torque_test_stop(void)
{
    s_running = false;
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_tongzhi_torque_test_send_axis_state(s_motor_ids[i], SRV_TONGZHI_AXIS_STATE_IDLE);
    }
    SRV_TONGZHI_TORQUE_TEST_LOG_I("良志(ODrive) 往复停止：已向 %u 台电机发 IDLE",
        (unsigned)s_motor_cnt);
}

/**
 * @brief 往复耐久测试模式周期步进（由 can_task 每 TASK_PERIOD_MS 调用）
 * @note  主动探测(Get_Error)+被动心跳双发现：新电机初始化、错误变化打印、闭环保持重试、
 *        受控电机到位翻转（全有反馈→位置判定，否则定时）、掉线告警与恢复重发闭环、
 *        累计在线满 DURATION_MS（30 天，置 0 禁用）自动停止
 */
void srv_tongzhi_torque_test_step(void)
{
    if (!s_running)
        return;

    const uint32_t now = millis();

    /* 持续在线计时：仅当所有已发现电机在线时累加（掉线期间不计时，恢复后继续累计）。
       s_online_last_ms 每步都更新，保证离线/恢复后时间不跳变 */
    if (srv_tongzhi_torque_test_all_online()) {
        s_online_ms += (uint32_t)(now - s_online_last_ms);
    }
    s_online_last_ms = now;

#if (SRV_TONGZHI_DURATION_MS != 0U)
    if (s_online_ms >= SRV_TONGZHI_DURATION_MS) {
        SRV_TONGZHI_TORQUE_TEST_LOG_I("累计在线 %lu ms 已到（30 天），良志(ODrive) 测试自动停止",
            (unsigned long)s_online_ms);
        srv_tongzhi_torque_test_stop();
        return;
    }
#endif

    /* 主动探测：对不发周期帧的静默电机发 Get_Error，靠回包被动收录（发现窗口内轮转 node 0~MAX-1） */
    srv_tongzhi_torque_test_probe_step(now);
    /* 探测窗口结束仍无发现：回退到默认 node_id 盲发 */
    srv_tongzhi_torque_test_fallback(now);

    /* 未发现电机周期告警（区分"探测中"与"窗口已结束"，提示核对电机 CAN 使能/波特率/供电） */
    if (s_motor_cnt == 0U && ((now - s_last_no_motor_log_ms) >= SRV_TONGZHI_NO_MOTOR_LOG_MS)) {
        s_last_no_motor_log_ms = now;
        SRV_TONGZHI_TORQUE_TEST_LOG_W("仍未发现电机：已主动探测 node 0~%u（%s），请核对电机 CAN 使能/波特率/供电",
            (unsigned)(SRV_TONGZHI_MAX_MOTORS - 1U),
            s_probe_done ? "探测窗口已结束" : "探测中");
    }

    /* 打印新发现的电机（主循环上下文，避免 ISR 打日志） */
    srv_tongzhi_torque_test_scan_log_new();

    /* 打印新到达的错误码（主循环上下文，ISR 只置标志）。心跳 axis_error 变化时打印 */
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (s_err_pending[i]) {
            s_err_pending[i] = false; /* 先清标志再取值，避免 ISR 并发丢更新 */
            if (s_motor_err[i] != 0U) {
                srv_tongzhi_torque_test_err_print(s_motor_ids[i], s_motor_err[i]);
            } else {
                SRV_TONGZHI_TORQUE_TEST_LOG_W("电机 node=%u 错误已消除，恢复正常",
                    (unsigned)s_motor_ids[i]);
            }
        }
    }

    /* 闭环确认日志（心跳 axis_state 0/非闭环 → 8 上升沿，每电机一次） */
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if ((s_motor_axis_state[i] == SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP)
            && (s_axis_state_prev[i] != SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP)) {
            SRV_TONGZHI_TORQUE_TEST_LOG_I("电机 node=%u 已确认闭环",
                (unsigned)s_motor_ids[i]);
        }
        s_axis_state_prev[i] = s_motor_axis_state[i];
    }

    /* 假定闭环：已下发 init 但无心跳确认的电机，宽限后假定已闭环，
       避免无反馈静默电机一直阻塞翻转/反复重发 init */
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (!s_assumed_closed[i] && (s_last_init_ms[i] != 0U)
            && (s_motor_axis_state[i] != SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP)
            && ((now - s_last_init_ms[i]) >= SRV_TONGZHI_INIT_GRACE_MS)) {
            s_assumed_closed[i] = true;
            SRV_TONGZHI_TORQUE_TEST_LOG_I("电机 node=%u 无心跳闭环确认，假定已闭环（继续下发目标）",
                (unsigned)s_motor_ids[i]);
        }
    }

    /* 锁存往复零点：电机初始化完成（init 序列走完 + 已有编码器反馈）时，
       以读回的当前位置为各自 0 点，此后目标在 各自中心±AMP 两端点间交替，
       而非绝对 ±AMP。盲发回退（无反馈）保持绝对目标 */
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (!s_center_latched[i] && srv_tongzhi_torque_test_target_ready(i) && s_motor_have_encoder[i]) {
            s_center_turns[i] = s_motor_encoder_turns[i];
            s_center_latched[i] = true;
            SRV_TONGZHI_TORQUE_TEST_LOG_I("已锁存往复零点：电机 node=%u 当前 %d 毫转为 0 点，目标区间 [%d, %d] 毫转",
                (unsigned)s_motor_ids[i],
                (int)(s_center_turns[i] * 1000.0f),
                (int)((s_center_turns[i] - SRV_TONGZHI_POS_AMP_TURNS) * 1000.0f),
                (int)((s_center_turns[i] + SRV_TONGZHI_POS_AMP_TURNS) * 1000.0f));
            /* 锁存前可能已按绝对目标下发过，立即重发校正后的 中心±AMP 目标 */
            if (srv_tongzhi_torque_test_target_ready(i)) {
                srv_tongzhi_torque_test_send_input_pos(s_motor_ids[i], srv_tongzhi_torque_test_motor_target(i));
            }
        }
    }

    /* 到位翻转双模式：
       - 全部受控电机均有编码器反馈 → 位置到位判定（受控=心跳确认闭环或假定闭环）；
       - 存在无反馈的受控电机 → 定时翻转（按 TRAVEL_EST_MS 粗估单程移动时间） */
    bool all_have_encoder = true;
    uint8_t in_ctrl_cnt = 0;
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (srv_tongzhi_torque_test_in_control(i)) {
            in_ctrl_cnt++;
            if (!s_motor_have_encoder[i]) {
                all_have_encoder = false;
            }
        }
    }
    bool should_flip = false;
    bool flip_timeout = false;
    if (in_ctrl_cnt > 0U) {
        if (all_have_encoder) {
            bool all_arrived = true;
            for (uint8_t i = 0; i < s_motor_cnt; i++) {
                if (!srv_tongzhi_torque_test_in_control(i)) {
                    continue; /* 未受控电机不参与到位判定 */
                }
                float diff = s_motor_encoder_turns[i] - srv_tongzhi_torque_test_motor_target(i);
                if (diff < 0.0f) {
                    diff = -diff;
                }
                if (diff > SRV_TONGZHI_ARRIVE_THRESH_TURNS) {
                    all_arrived = false;
                    break;
                }
            }
            /* 位置到位 + 超时兜底：反馈冻结/到位偏置导致 |enc−target| 恒 > 阈值时，
               超时强制翻转，避免往复永久停摆 */
            should_flip = all_arrived || ((now - s_last_flip_ms) >= SRV_TONGZHI_ARRIVE_TIMEOUT_MS);
            flip_timeout = should_flip && !all_arrived;
        } else {
            should_flip = ((now - s_last_flip_ms) >= SRV_TONGZHI_TRAVEL_EST_MS);
        }
    }
    if (should_flip) {
        if (flip_timeout) {
            SRV_TONGZHI_TORQUE_TEST_LOG_W("目标到达超时，强制翻转（可能反馈冻结/到位偏置）");
        }
        if (s_dir > 0) {
            s_dir = -1;
        } else {
            s_dir = 1;
        }
        s_last_flip_ms = now;
        for (uint8_t i = 0; i < s_motor_cnt; i++) {
            if (srv_tongzhi_torque_test_target_ready(i)) {
                srv_tongzhi_torque_test_send_input_pos(s_motor_ids[i], srv_tongzhi_torque_test_motor_target(i));
            }
        }
        SRV_TONGZHI_TORQUE_TEST_LOG_I("目标翻转 → %s 端（%s）",
            (s_dir > 0) ? "+AMP" : "-AMP",
            flip_timeout ? "到达超时" : (all_have_encoder ? "位置到位" : "定时"));
    }

    /* 周期重发当前目标（锁存式目标，重发同目标为安全 no-op，防首帧/丢帧漏目标） */
    if ((now - s_last_target_resend_ms) >= SRV_TONGZHI_TARGET_RESEND_MS) {
        s_last_target_resend_ms = now;
        for (uint8_t i = 0; i < s_motor_cnt; i++) {
            if (srv_tongzhi_torque_test_target_ready(i)) {
                srv_tongzhi_torque_test_send_input_pos(s_motor_ids[i], srv_tongzhi_torque_test_motor_target(i));
            }
        }
    }

    /* 周期状态诊断日志：打印受控电机 编码器/目标/轴状态/错误/在线年龄，
       停摆时用于区分「反馈冻结/掉出闭环/到位偏置」等根因 */
    if ((now - s_last_status_log_ms) >= SRV_TONGZHI_STATUS_LOG_MS) {
        s_last_status_log_ms = now;
        for (uint8_t i = 0; i < s_motor_cnt; i++) {
            if (!srv_tongzhi_torque_test_in_control(i)) {
                continue;
            }
            SRV_TONGZHI_TORQUE_TEST_LOG_I("状态 node=%u axis=%u err=0x%08lX enc=%d 毫转 tgt=%d 毫转 ctr=%d 毫转 enc_ok=%u last_seen=%lu ms",
                (unsigned)s_motor_ids[i],
                (unsigned)s_motor_axis_state[i],
                (unsigned long)s_motor_err[i],
                (int)(s_motor_encoder_turns[i] * 1000.0f),
                (int)(srv_tongzhi_torque_test_motor_target(i) * 1000.0f),
                (int)(s_center_turns[i] * 1000.0f),
                s_motor_have_encoder[i] ? 1U : 0U,
                (unsigned long)(now - s_motor_last_seen_ms[i]));
        }
    }

    /* 电机无响应检测：超过 NORESP_PERIOD 未收到任何帧视为掉线/断电（每电机只告警一次） */
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (!s_motor_nresp_latch[i] && ((now - s_motor_last_seen_ms[i]) >= SRV_TONGZHI_NORESP_PERIOD_MS)) {
            s_motor_nresp_latch[i] = true;
            SRV_TONGZHI_TORQUE_TEST_LOG_W("电机 node=%u 长时间无响应（掉线或断电）",
                (unsigned)s_motor_ids[i]);
        }
    }

    /* 恢复在线：撤销假定闭环，重置 init 步进让 keep-alive 重新按序下发
       （掉线期间电机可能已复位为 IDLE，需完整重走 清错→模式→梯形限制→闭环） */
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (s_online_evt_pending[i]) {
            s_online_evt_pending[i] = false;
            s_assumed_closed[i] = false;
            s_last_init_ms[i] = 0U; /* 强制重新走完整 init 序列 */
            s_init_step[i] = 0U;
            s_center_latched[i] = false; /* 重新以该电机恢复后的当前位置为往复零点 */
            SRV_TONGZHI_TORQUE_TEST_LOG_W("电机 node=%u 恢复在线，重新按序下发闭环/模式/梯形参数",
                (unsigned)s_motor_ids[i]);
        }
    }

    /* 保持闭环/补发配置：未假定闭环的电机逐帧推进 init 序列。
       motor_init 拆成 5 帧（清错→控制模式→梯形限速→梯形限加→闭环）逐次下发，
       每 ENABLE_RETRY_MS 发 1 帧——避免一次连发 5 帧超出 FDCAN TX FIFO 深度（G4=3）
       导致 Set_Traj_Vel_Limit/Set_Traj_Accel_Limits 被静默丢弃（那会让电机按默认高速
       梯形跑 → 编码器超速 0x40）。序列完成后若仍未闭环则从头重发；已闭环则停止。
       注意：不带活动错误门控——ODrive 锁存错误必须靠 Clear_Errors(0x18) 才能清除，
       带错误的电机也要收到 init（其第一步即清错），否则错误永远清不掉、永远进不了闭环。 */
    bool need_enable = false;
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (!s_assumed_closed[i]
            && ((s_motor_axis_state[i] != SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP)
                || (s_last_init_ms[i] == 0U)
                || (s_init_step[i] < 5U))) {
            need_enable = true;
            break;
        }
    }
    if (need_enable) {
        if (s_enable_stall_since_ms == 0U) {
            s_enable_stall_since_ms = now;
            s_enable_warned = false;
        } else if (!s_enable_warned
            && (now - s_enable_stall_since_ms) >= SRV_TONGZHI_ENABLE_TIMEOUT_MS) {
            s_enable_warned = true;
            SRV_TONGZHI_TORQUE_TEST_LOG_W("电机闭环未确认已超时，继续周期重试");
        }
        if ((now - s_last_enable_retry_ms) >= SRV_TONGZHI_ENABLE_RETRY_MS) {
            s_last_enable_retry_ms = now;
            /* 每个重试周期只推进 1 帧（全局轮转），避免多台电机同 tick 连发超 FIFO 深度 3 */
            for (uint8_t i = 0; i < s_motor_cnt; i++) {
                if (!s_assumed_closed[i]
                    && ((s_motor_axis_state[i] != SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP)
                        || (s_last_init_ms[i] == 0U)
                        || (s_init_step[i] < 5U))) {
                    if (s_last_init_ms[i] == 0U) {
                        s_last_init_ms[i] = now; /* 首次 init 计时起点（宽限判定用，不随重发刷新） */
                        SRV_TONGZHI_TORQUE_TEST_LOG_I("电机 node=%u 首次下发初始化（清错/闭环/位置+梯形模式/梯形限速限加）",
                            (unsigned)s_motor_ids[i]);
                    }
                    if (s_init_step[i] >= 5U) {
                        s_init_step[i] = 0U; /* 序列已发完仍未闭环 → 从头重发 */
                    }
                    srv_tongzhi_torque_test_motor_init_step(s_motor_ids[i], s_init_step[i]);
                    s_init_step[i]++;
                    break; /* 本轮只发一帧，下一轮再推进下一台/下一帧 */
                }
            }
        }
    } else {
        s_enable_stall_since_ms = 0U; /* 已全部闭环/假定闭环或不在线，复位 */
    }
}

/**
 * @brief 处理良志(ODrive)心跳/编码器帧（由 can_task 按 CH_1 分发调用）
 * @param  msg CAN 报文指针
 * @return true=ODrive 帧（node_id 0~63, DLC≥8），已消费；false=非本协议帧
 * @note   ISR 上下文，只做数据记录与标志置位，不打日志
 */
bool srv_tongzhi_torque_test_on_rx(const drv_can_msg_t* msg)
{
    if (!msg)
        return false;

    /* 仅消费标准帧、DLC≥8（良志协议帧均为 8 字节） */
    if (msg->is_extended || (msg->dlc < 8U))
        return false;

    const uint8_t node = (uint8_t)((msg->id >> 5) & 0x3FU);
    const uint8_t cmd = (uint8_t)(msg->id & 0x1FU);

    /* node_id 超出本模块管理上限：仍属 ODrive 帧，消费但不记录 */
    if (node >= SRV_TONGZHI_MAX_MOTORS)
        return true;

    /* 未知 node 的帧：被动收录进列表，之后按索引处理 */
    uint8_t idx = srv_tongzhi_torque_test_find_idx(node);
    if (idx == SRV_TONGZHI_MAX_MOTORS) {
        srv_tongzhi_torque_test_scan_record(node);
        idx = srv_tongzhi_torque_test_find_idx(node);
        if (idx == SRV_TONGZHI_MAX_MOTORS)
            return true; /* 列表已满，忽略 */
    }

    /* 在线刷新：收到任意帧都视为"电机在应答"。
       此前掉线锁存的电机收到帧即恢复在线，置标志由主循环重发闭环（ISR 不打日志） */
    s_motor_last_seen_ms[idx] = millis();
    if (s_motor_nresp_latch[idx]) {
        s_motor_nresp_latch[idx] = false;
        s_online_evt_pending[idx] = true;
    }

    /* 心跳：data[0..3]=axis_error uint32 小端，data[4]=axis_state，
       data[5]=flags(bit7=轨迹完成)，data[6]=temp，data[7]=life */
    if (cmd == SRV_TONGZHI_CMD_HEARTBEAT) {
        s_motor_axis_state[idx] = msg->data[4];
        if (s_assumed_closed[idx]) {
            /* 心跳证明电机真实在线（如控制板先上电、电机后上电的 fallback 盲发场景）：
               撤销假定并强制重新走完整 init 序列，避免电机停在默认速度模式导致不能运行 */
            s_assumed_closed[idx] = false;
            s_last_init_ms[idx] = 0U;
            s_init_step[idx] = 0U;
        }
        if (msg->data[4] != SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP) {
            s_assumed_closed[idx] = false; /* 心跳证实未闭环，撤销假定 */
        }
        const uint32_t err = srv_tongzhi_torque_test_unpack_u32_le(&msg->data[0]);
        if (err != s_motor_err[idx]) {
            s_motor_err[idx] = err;
            s_err_pending[idx] = true;
        }
        return true;
    }

    /* Get_Error 回包（主动探测用）：data[0..3]=错误码 uint32 小端，兼作在线/发现信号 */
    if (cmd == SRV_TONGZHI_CMD_GET_ERROR) {
        const uint32_t err = srv_tongzhi_torque_test_unpack_u32_le(&msg->data[0]);
        if (err != s_motor_err[idx]) {
            s_motor_err[idx] = err;
            s_err_pending[idx] = true;
        }
        return true;
    }

    /* 编码器估计：data[0..3]=pos float32 小端（转），data[4..7]=vel float32（转/s）。
       仅在闭环状态输出有效数据；主循环以 axis_state==8 为前提使用 */
    if (cmd == SRV_TONGZHI_CMD_ENCODER_ESTIMATES) {
        s_motor_encoder_turns[idx] = srv_tongzhi_torque_test_unpack_float_le(&msg->data[0]);
        s_motor_have_encoder[idx] = true;
        return true;
    }

    /* 其他应答帧：已刷新 last_seen，消费之 */
    return true;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 查找 node_id 在已发现列表中的索引
 * @param  node 电机 node_id（0~63）
 * @return 索引；未找到返回 SRV_TONGZHI_MAX_MOTORS（哨兵值）
 */
static uint8_t srv_tongzhi_torque_test_find_idx(uint8_t node)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (s_motor_ids[i] == node)
            return i;
    }
    return SRV_TONGZHI_MAX_MOTORS;
}

/**
 * @brief 所有已发现电机是否在线（无任何电机进入无响应锁存）
 * @return true=全部在线
 * @note   DURATION 持续在线计时仅在全部在线时累加；未发现电机视为不在线
 */
static bool srv_tongzhi_torque_test_all_online(void)
{
    if (s_motor_cnt == 0U)
        return false;
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (s_motor_nresp_latch[i])
            return false;
    }
    return true;
}

/**
 * @brief 电机是否处于"受控"状态（参与到位判定/目标下发）
 * @param idx 电机索引
 * @return true=心跳确认闭环 或 假定闭环
 */
static bool srv_tongzhi_torque_test_in_control(uint8_t idx)
{
    return (s_motor_axis_state[idx] == SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP) || s_assumed_closed[idx];
}

/**
 * @brief 电机是否可下发目标位置（init 序列已完整执行）
 * @param idx 电机索引
 * @return true=受控且已假定闭环 或 init 序列完成（5 帧全部发出，含梯形限速/限加）
 * @note   梯形限速/限加未就位前不发 Set_Input_Pos，避免电机按默认高速限制跑导致超速(0x40)
 */
static bool srv_tongzhi_torque_test_target_ready(uint8_t idx)
{
    if (!srv_tongzhi_torque_test_in_control(idx))
        return false;
    return s_assumed_closed[idx] || (s_init_step[idx] >= 5U);
}

/**
 * @brief 主动探测步进：发现窗口内对未发现的候选 node 周期发 Get_Error(0x03)
 * @param now 当前时间 (millis)
 * @note   Get_Error 回包由 on_rx 自动收录 node（任意帧即记录），因此无需改发现路径。
 *         发现窗口结束时置 s_probe_done 停止，限制失败帧数（≤10 次 × TEC+8）避免把主机打进 BUS-OFF；
 *         窗口后转入对已发现电机的周期错误回查（兼作静默电机的在线保活）
 */
static void srv_tongzhi_torque_test_probe_step(uint32_t now)
{
    if ((now - s_last_probe_ms) < SRV_TONGZHI_PROBE_INTERVAL_MS)
        return;

    /* 发现窗口：轮转探测未发现的候选 node 0~MAX-1 */
    if (!s_probe_done) {
        if ((now - s_start_ms) >= SRV_TONGZHI_FALLBACK_MS) {
            s_probe_done = true; /* 窗口结束，转入错误回查/回退判定 */
        } else {
            s_last_probe_ms = now;
            for (uint8_t pass = 0; pass < SRV_TONGZHI_MAX_MOTORS; pass++) {
                const uint8_t node = s_probe_idx;
                s_probe_idx++;
                if (s_probe_idx >= SRV_TONGZHI_MAX_MOTORS) {
                    s_probe_idx = 0;
                }
                if (srv_tongzhi_torque_test_find_idx(node) == SRV_TONGZHI_MAX_MOTORS) {
                    srv_tongzhi_torque_test_send_get_error(node);
                    return;
                }
            }
            return;
        }
    }

    /* 发现窗口后：周期回查已发现电机的错误状态（on_rx 按 0x03 回包更新 err 并刷新 last_seen，
       兼作不发周期帧电机的在线保活，避免误报掉线） */
    if (s_motor_cnt == 0U)
        return;
    s_last_probe_ms = now;
    const uint8_t idx = s_err_query_idx;
    s_err_query_idx++;
    if (s_err_query_idx >= s_motor_cnt) {
        s_err_query_idx = 0;
    }
    srv_tongzhi_torque_test_send_get_error(s_motor_ids[idx]);
}

/**
 * @brief 探测窗口结束仍无发现的回退：按默认 node_id 盲发（镜像 HT 的默认地址回退）
 * @param now 当前时间 (millis)
 * @note   盲发后直接假定受控并持续下发目标；若总线上无 ACK（波特率/使能不匹配），
 *         drv_can 的 tx fail / EP / BUS-OFF 日志即为诊断信号
 */
static void srv_tongzhi_torque_test_fallback(uint32_t now)
{
    if (s_fallback_active || !s_probe_done)
        return;
    if (s_motor_cnt != 0U)
        return;

    s_fallback_active = true;
    srv_tongzhi_torque_test_scan_record(SRV_TONGZHI_FALLBACK_NODE);
    const uint8_t idx = srv_tongzhi_torque_test_find_idx(SRV_TONGZHI_FALLBACK_NODE);
    if (idx == SRV_TONGZHI_MAX_MOTORS)
        return; /* 列表已满，放弃回退 */

    s_last_init_ms[idx] = now;
    s_assumed_closed[idx] = true; /* 盲发：直接假定受控，避免反复重发 */
    s_init_step[idx] = 5U; /* 假定已闭环 → init 序列视为已完成（盲发节点无真实反馈） */
    srv_tongzhi_torque_test_send_input_pos(SRV_TONGZHI_FALLBACK_NODE, srv_tongzhi_torque_test_motor_target(idx));
    SRV_TONGZHI_TORQUE_TEST_LOG_W("探测 %u 个 node 均无应答，回退默认 node_id=%u 盲发（若总线无 ACK 请核对波特率/使能）",
        (unsigned)SRV_TONGZHI_MAX_MOTORS, (unsigned)SRV_TONGZHI_FALLBACK_NODE);
}

/**
 * @brief 计算指定电机的当前目标位置（转）
 * @param idx 电机索引
 * @return 目标位置：已锁存零点时为其自身中心±AMP（各电机以自身零点摆动）；
 *         未锁存（盲发回退/启动初期）为绝对 ±AMP
 */
static float srv_tongzhi_torque_test_motor_target(uint8_t idx)
{
    if (s_center_latched[idx]) {
        return (s_dir > 0) ? (s_center_turns[idx] + SRV_TONGZHI_POS_AMP_TURNS)
                           : (s_center_turns[idx] - SRV_TONGZHI_POS_AMP_TURNS);
    }
    return (s_dir > 0) ? SRV_TONGZHI_POS_AMP_TURNS : -SRV_TONGZHI_POS_AMP_TURNS;
}

/**
 * @brief 计算良志(ODrive) CAN ID：CAN ID = (node_id<<5) | cmd_id
 * @param node 电机 node_id（0~63）
 * @param cmd  命令码（0~31）
 * @return 标准 11 位 CAN ID
 */
static uint32_t srv_tongzhi_torque_test_can_id(uint8_t node, uint8_t cmd)
{
    return (uint32_t)(((uint32_t)(node & 0x3FU) << 5) | (uint32_t)(cmd & 0x1FU));
}

/* --- 小端打包/解包（Cortex-M4 小端，memcpy 避免别名/对齐问题） --- */

static void srv_tongzhi_torque_test_pack_float_le(uint8_t* dst, float v)
{
    memcpy(dst, &v, 4);
}

static void srv_tongzhi_torque_test_pack_u32_le(uint8_t* dst, uint32_t v)
{
    memcpy(dst, &v, 4);
}

static void srv_tongzhi_torque_test_pack_i16_le(uint8_t* dst, int16_t v)
{
    memcpy(dst, &v, 2);
}

static float srv_tongzhi_torque_test_unpack_float_le(const uint8_t* src)
{
    float v;
    memcpy(&v, src, 4);
    return v;
}

static uint32_t srv_tongzhi_torque_test_unpack_u32_le(const uint8_t* src)
{
    uint32_t v;
    memcpy(&v, src, 4);
    return v;
}

/* --- 命令发送（经典 CAN 2.0A, DLC 8, 小端） --- */

/**
 * @brief 发送设置轴状态帧 (0x07)
 * @param node  电机 node_id
 * @param state 目标轴状态（1=IDLE，8=闭环）
 */
static void srv_tongzhi_torque_test_send_axis_state(uint8_t node, uint8_t state)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = srv_tongzhi_torque_test_can_id(node, SRV_TONGZHI_CMD_SET_AXIS_STATE),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tongzhi_torque_test_pack_u32_le(&tx.data[0], (uint32_t)state);
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送设置控制模式帧 (0x0B)：位置控制(3) + 梯形轨迹输入(5)
 * @param node 电机 node_id
 */
static void srv_tongzhi_torque_test_send_controller_mode(uint8_t node)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = srv_tongzhi_torque_test_can_id(node, SRV_TONGZHI_CMD_SET_CONTROLLER_MODE),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tongzhi_torque_test_pack_u32_le(&tx.data[0], SRV_TONGZHI_CONTROL_MODE_POSITION);
    srv_tongzhi_torque_test_pack_u32_le(&tx.data[4], SRV_TONGZHI_INPUT_MODE_TRAP_TRAJ);
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送梯形速度限制帧：Set_Traj_Vel_Limit (0x11)
 * @param node 电机 node_id
 */
static void srv_tongzhi_torque_test_send_traj_vel_limit(uint8_t node)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = srv_tongzhi_torque_test_can_id(node, SRV_TONGZHI_CMD_SET_TRAJ_VEL_LIMIT),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tongzhi_torque_test_pack_float_le(&tx.data[0], SRV_TONGZHI_TRAJ_VEL_LIMIT_TPS);
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送梯形加速度/减速度限制帧：Set_Traj_Accel_Limits (0x12)
 * @param node 电机 node_id
 */
static void srv_tongzhi_torque_test_send_traj_accel_limits(uint8_t node)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = srv_tongzhi_torque_test_can_id(node, SRV_TONGZHI_CMD_SET_TRAJ_ACCEL_LIMITS),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tongzhi_torque_test_pack_float_le(&tx.data[0], SRV_TONGZHI_TRAJ_ACCEL_TPS2);
    srv_tongzhi_torque_test_pack_float_le(&tx.data[4], SRV_TONGZHI_TRAJ_DECEL_TPS2);
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送设置目标位置帧 (0x0C)
 * @param node  电机 node_id
 * @param turns 目标位置（转）；速度/力矩前馈恒为 0
 */
static void srv_tongzhi_torque_test_send_input_pos(uint8_t node, float turns)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = srv_tongzhi_torque_test_can_id(node, SRV_TONGZHI_CMD_SET_INPUT_POS),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tongzhi_torque_test_pack_float_le(&tx.data[0], turns); /* 目标位置（转） */
    srv_tongzhi_torque_test_pack_i16_le(&tx.data[4], 0); /* 速度前馈 0（0.001 转/s） */
    srv_tongzhi_torque_test_pack_i16_le(&tx.data[6], 0); /* 力矩前馈 0（0.001 Nm） */
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送清除错误帧 (0x18)
 * @param node 电机 node_id
 * @note   ODrive 标准：data[0..3]=clear_errors(1=清本轴错误)，data[4..7]=
 *         clear_errors_on_other_axis(0=不清其它轴)。良志文档写「8 字节 0」，但 ODrive
 *         实现要求标志置 1 才真正清错，按 ODrive 标准执行；若现场无效再回退试 8×0
 */
static void srv_tongzhi_torque_test_send_clear_errors(uint8_t node)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = srv_tongzhi_torque_test_can_id(node, SRV_TONGZHI_CMD_CLEAR_ERRORS),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tongzhi_torque_test_pack_u32_le(&tx.data[0], 1U); /* clear_errors=true */
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送获取错误帧 (0x03)：data[0]=error_type(0=电机错误)
 * @param node 电机 node_id
 * @note   ODrive 系电机收到会回错误码帧；该回包被 on_rx 当作"在线/发现"信号，
 *         用于 heartbeat_rate_ms=0（不发周期帧）的静默电机的主动探测
 */
static void srv_tongzhi_torque_test_send_get_error(uint8_t node)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = srv_tongzhi_torque_test_can_id(node, SRV_TONGZHI_CMD_GET_ERROR),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    tx.data[0] = 0U; /* error_type=0：电机错误 */
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 电机初始化逐帧下发（step=0..4，每次只发 1 帧）
 * @param node 电机 node_id
 * @param step 帧序号：0=清错，1=控制模式(位置+梯形)，2=梯形限速，3=梯形限加，4=闭环
 * @note   按 ODrive 推荐顺序：先配置 模式/梯形限制 再进闭环，避免进闭环后短暂按默认
 *         高速限制运行导致编码器超速(0x40)。每帧由 keep-alive 以 ENABLE_RETRY_MS 间隔
 *         逐次调用，确保 5 帧全部进入 FDCAN TX FIFO（G4 深度 3），不被静默丢弃。
 */
static void srv_tongzhi_torque_test_motor_init_step(uint8_t node, uint8_t step)
{
    switch (step) {
    case 0:
        srv_tongzhi_torque_test_send_clear_errors(node); /* 清残留错误，避免闭环被旧错误阻塞 */
        break;
    case 1:
        srv_tongzhi_torque_test_send_controller_mode(node);
        break;
    case 2:
        srv_tongzhi_torque_test_send_traj_vel_limit(node);
        break;
    case 3:
        srv_tongzhi_torque_test_send_traj_accel_limits(node);
        break;
    case 4:
        srv_tongzhi_torque_test_send_axis_state(node, SRV_TONGZHI_AXIS_STATE_CLOSED_LOOP);
        break;
    default:
        break;
    }
}

/**
 * @brief 记录发现的电机 node_id（去重，越界忽略）
 * @note   在 ISR 中调用，只做数组操作，不打日志
 */
static void srv_tongzhi_torque_test_scan_record(uint8_t node)
{
    if (node >= SRV_TONGZHI_MAX_MOTORS)
        return;

    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (s_motor_ids[i] == node)
            return; /* 已记录 */
    }
    if (s_motor_cnt < SRV_TONGZHI_MAX_MOTORS) {
        const uint8_t idx = s_motor_cnt;
        s_motor_ids[idx] = node;
        s_motor_last_seen_ms[idx] = millis();
        s_motor_axis_state[idx] = 0U; /* 尚未闭环 */
        s_motor_have_encoder[idx] = false;
        s_center_latched[idx] = false; /* 新电机：未锁存往复零点 */
        s_center_turns[idx] = 0.0f;
        s_init_step[idx] = 0U; /* 新电机：从 init 序列头开始（逐帧下发） */
        s_motor_cnt++;
    }
}

/**
 * @brief 打印扫描期间新发现的电机（主循环调用，避免 ISR 打日志）
 */
static void srv_tongzhi_torque_test_scan_log_new(void)
{
    while (s_scan_log_cnt < s_motor_cnt) {
        SRV_TONGZHI_TORQUE_TEST_LOG_I("  发现电机：node_id = %u",
            (unsigned)s_motor_ids[s_scan_log_cnt]);
        s_scan_log_cnt++;
    }
}

/**
 * @brief 打印错误详情（主循环上下文调用）
 * @note   错误码可组合（多位同时置位）；WARN 打印原始码并逐位解码
 */
static void srv_tongzhi_torque_test_err_print(uint8_t node, uint32_t code)
{
    SRV_TONGZHI_TORQUE_TEST_LOG_W("电机 node=%u 错误：0x%08X", (unsigned)node, (unsigned)code);
    for (uint8_t i = 0; i < SRV_TONGZHI_ERR_NUM; i++) {
        if ((code & s_err_map[i].mask) != 0U) {
            SRV_TONGZHI_TORQUE_TEST_LOG_W("  - %s", s_err_map[i].name);
        }
    }
}
