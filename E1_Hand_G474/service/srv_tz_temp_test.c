/**
 * @file    srv_tz_temp_test.c
 * @brief   良志(ODrive) 伺服执行器 CAN 控制协议服务实现 — 耐久测试（多实例，速度/位置双模式）
 *
 * 电机控制指令按 docs/良志电机can协议.md 组帧发送（经典 CAN 2.0A, 1 Mbps,
 * CAN-ID = (node_id<<5)|cmd_id，8 字节帧，小端，IEEE-754 单精度浮点）。
 *
 * 控制模式由编译期宏 SRV_TZ_TEMP_CTRL_MODE 切换（见 srv_tz_temp_test.h）：
 *   - SPEED    速度模式：Set_Input_Vel 按 正转→停→反转→停 定时循环，主机侧斜坡软启停；
 *   - POSITION 位置模式：以各电机初始化位置为 0°，通过反馈位置做 P 控制
 *     （速度指令 = Kp × 位置误差，限幅 + 加减速限制）驱动电机在 0°±pos_amp_turns
 *     （默认 6.5 转）之间往复，到位即翻转目标。
 * 两种模式均走速度环输出（Set_Input_Vel，PASS_THROUGH 输入）。
 *
 * 本服务支持多个实例：每个实例绑定一条 CAN 通道（inst->config.can_ch），
 * 各自独立 发现/初始化/管理 电机并运行耐久循环，互不干扰。
 *
 * 控制方式：两种模式均仅所有已发现电机在线时累计在线时长，满 duration_ms
 * 自动软停止（先速度回 0 再发 IDLE）。
 *
 * 多电机寻址：ODrive 无主机握手命令，但电机上电后周期主动推送心跳(0x01, 默认 100ms)；
 * 本模块靠监听心跳被动发现 node_id，动态收录并初始化新电机（掉线电机仍在列表中，
 * 恢复在线时由主循环重发闭环与速度）。对不发周期帧的静默电机在发现窗口内主动发
 * Get_Error(0x03) 探测，靠回包被动收录；无任何发现时回退默认 node_id 盲发。
 *
 * RX 侧：驱动层把收到的帧入 msg_fifo 队列（ISR 只入队、零协议解析开销），本模块
 * step() 内用 drv_can_rx_pop() 出队并交给 on_rx() 解析记录（主循环上下文）；TX 侧
 * 全部经 drv_can_tx_enqueue() 入软件队列，由 drv_can_poll_status 周期排空到 FDCAN。
 * 所有实例共用 err_map 等只读常量表，运行状态全部在实例句柄内（禁止模块级单例）。
 */

#include "srv_tz_temp_test.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/
/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印（耐久测试确认运转正常后可关闭） */
#define SRV_TZ_TEMP_LOG_ENABLE 1

#if SRV_TZ_TEMP_LOG_ENABLE
#define SRV_TZ_TEMP_TEST_LOG_I(inst, fmt, ...) \
    LOG_I("tz_temp_test", "[ch%u] " fmt, (unsigned)((inst)->config.can_ch) + 1U, ##__VA_ARGS__)
#define SRV_TZ_TEMP_TEST_LOG_W(inst, fmt, ...) \
    LOG_W("tz_temp_test", "[ch%u] " fmt, (unsigned)((inst)->config.can_ch) + 1U, ##__VA_ARGS__)
#define SRV_TZ_TEMP_TEST_LOG_E(inst, fmt, ...) \
    LOG_E("tz_temp_test", "[ch%u] " fmt, (unsigned)((inst)->config.can_ch) + 1U, ##__VA_ARGS__)
#else
#define SRV_TZ_TEMP_TEST_LOG_I(inst, fmt, ...) ((void)0)
#define SRV_TZ_TEMP_TEST_LOG_W(inst, fmt, ...) ((void)0)
#define SRV_TZ_TEMP_TEST_LOG_E(inst, fmt, ...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/
/* --- 控制模式选择（编译期宏，改后需重新编译） --- */

#define SRV_TZ_TEMP_CTRL_MODE_SPEED 0 /**< 速度模式：正转→停→反转→停 定时循环（现有功能） */
#define SRV_TZ_TEMP_CTRL_MODE_POSITION 1 /**< 位置模式：初始化位置为 0°，中心±pos_amp_turns 往复 */
#ifndef SRV_TZ_TEMP_CTRL_MODE
#define SRV_TZ_TEMP_CTRL_MODE SRV_TZ_TEMP_CTRL_MODE_POSITION
#endif

/* --- 默认配置（srv_tz_temp_test_config_default 用） --- */

/** @brief 默认目标速度（转/s）：正转发 +SPEED，反转发 -SPEED，停发 0 */
#define SRV_TZ_TEMP_DEF_SPEED_TPS (18.0f)
/** @brief 默认主机侧软斜坡时长 (ms) */
#define SRV_TZ_TEMP_DEF_RAMP_TIME_MS 2000U
/** @brief 默认速度命令帧发送周期 (ms) */
#define SRV_TZ_TEMP_DEF_CMD_PERIOD_MS 20U
/** @brief 默认每个阶段时长 (ms)：正转/停/反转/停 各 30s */
#define SRV_TZ_TEMP_DEF_PHASE_MS 30000U
/** @brief 默认停止序列斜坡回 0 后保持零速时长 (ms) */
#define SRV_TZ_TEMP_DEF_STOP_HOLD_MS 1000U
/** @brief 默认电机无心跳判定周期 (ms) */
#define SRV_TZ_TEMP_DEF_NORESP_PERIOD_MS 2000U
/** @brief 默认未闭环电机重发 init 周期 (ms) */
#define SRV_TZ_TEMP_DEF_ENABLE_RETRY_MS 200U
/** @brief 默认闭环确认超时 (ms) */
#define SRV_TZ_TEMP_DEF_ENABLE_TIMEOUT_MS 2000U
/** @brief 默认主动探测周期 (ms) */
#define SRV_TZ_TEMP_DEF_PROBE_INTERVAL_MS 200U
/** @brief 默认发现窗口时长 (ms) */
#define SRV_TZ_TEMP_DEF_FALLBACK_MS 2000U
/** @brief 默认回退 node_id */
#define SRV_TZ_TEMP_DEF_FALLBACK_NODE 0U
/** @brief 默认假定闭环宽限 (ms) */
#define SRV_TZ_TEMP_DEF_INIT_GRACE_MS 1000U
/** @brief 默认周期状态诊断日志间隔 (ms) */
#define SRV_TZ_TEMP_DEF_STATUS_LOG_MS 5000U
/** @brief 默认未发现电机周期告警间隔 (ms) */
#define SRV_TZ_TEMP_DEF_NO_MOTOR_LOG_MS 5000U
/** @brief 默认耐久运行时长（速度模式）：24h = 86400000 ms（uint32 范围内）；0=禁用 */
#define SRV_TZ_TEMP_DEF_DURATION_SPEED_MS 86400000U
/** @brief 默认耐久运行时长（位置模式）：30 天 = 2592000000 ms（uint32 范围内）；0=禁用 */
#define SRV_TZ_TEMP_DEF_DURATION_POSITION_MS 2592000000U

/* --- 良志(ODrive) 协议指令（docs/良志电机can协议.md §3） --- */

#define SRV_TZ_TEMP_CMD_HEARTBEAT 0x01U /**< 心跳：电机→主机，周期推送（默认 100ms），DLC 8 */
#define SRV_TZ_TEMP_CMD_SET_AXIS_STATE 0x07U /**< 设置轴状态：data[0..3]=axis_state uint32 小端 */
#define SRV_TZ_TEMP_CMD_ENCODER_ESTIMATES 0x09U /**< 编码器估计：电机→主机，周期推送（默认 10ms），DLC 8 */
#define SRV_TZ_TEMP_CMD_SET_CONTROLLER_MODE 0x0BU /**< 设置控制模式：control_mode+input_mode 各 uint32 */
#define SRV_TZ_TEMP_CMD_SET_INPUT_VEL 0x0DU /**< 设置目标速度：vel float32(转/s)+torque_ff float32(Nm) */
#define SRV_TZ_TEMP_CMD_CLEAR_ERRORS 0x18U /**< 清除错误：8×0 */
#define SRV_TZ_TEMP_CMD_GET_ERROR 0x03U /**< 获取错误：data[0]=error_type；电机回错误码帧（主动探测用） */

#define SRV_TZ_TEMP_AXIS_STATE_IDLE 1U /**< 空闲 */
#define SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP 8U /**< 闭环控制 */
#define SRV_TZ_TEMP_CONTROL_MODE_VELOCITY 2U /**< 速度控制 */
#define SRV_TZ_TEMP_INPUT_MODE_PASS_THROUGH 1U /**< 直通输入（速度指令直通速度环） */

/* --- 位置模式默认参数（srv_tz_temp_test_config_default 用；每实例可经 config 独立覆盖） --- */

/** @brief 默认往复半幅（转）：目标在 初始化零点(0°) ±6.5 转 两端点间交替 */
#define SRV_TZ_TEMP_DEF_POS_AMP_TURNS (5.6f)
/** @brief 默认位置环 P 增益（1/s）：速度指令 = Kp × 位置误差（转） */
#define SRV_TZ_TEMP_DEF_POS_KP (2.0f)
/** @brief 默认位置模式速度指令限幅（转/s） */
#define SRV_TZ_TEMP_DEF_POS_MAX_VEL_TPS (4.0f)
/** @brief 默认速度指令加减速限制（转/s²）：换向/到位时速度指令按该斜率渐变，
 *        避免 换向瞬间速度阶跃（如 +3 转/s 突变 -3 转/s） */
#define SRV_TZ_TEMP_DEF_POS_ACCEL_TPS2 (2.0f)
/** @brief 默认到位判定阈值（转）：|位置−目标| ≤ 该值视为到位（随后翻转目标） */
#define SRV_TZ_TEMP_DEF_POS_ARRIVE_THRESH_TURNS (0.025f)
/** @brief 默认位置到位判定超时 (ms)：超过该时长仍未到位则强制翻转（反馈冻结/到位偏置兜底） */
#define SRV_TZ_TEMP_DEF_POS_ARRIVE_TIMEOUT_MS 12000U
/** @brief 停止序列回 0°（中心）超时 (ms)：超时仍未全部到位则强制进入保持+IDLE */
#define SRV_TZ_TEMP_POS_RETURN_TIMEOUT_MS 60000U

/* Private types -------------------------------------------------------------*/

/** @brief 错误码描述表项（见 docs/良志电机can协议.md §5.3，ODrive axis.error 位） */
typedef struct {
    uint32_t mask; /**< 错误码位 */
    const char* name; /**< 含义 */
} srv_tz_temp_test_err_desc_t;

/** @brief 错误码 → 含义映射表（0x03 回包为 motor.error，error_type=0；心跳 axis_error 为 axis.error，
 *        两者低 8 位含义不同，此处以 motor.error 为主并保留文档 §5.3 的常见位；始终打印原始 hex）。
 *        只读常量表，所有实例共享。 */
static const srv_tz_temp_test_err_desc_t s_err_map[] = {
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
#define SRV_TZ_TEMP_ERR_NUM (sizeof(s_err_map) / sizeof(s_err_map[0]))

/* Private function prototypes -----------------------------------------------*/

static uint8_t srv_tz_temp_test_find_idx(const srv_tz_temp_test_inst_t* inst, uint8_t node);
static bool srv_tz_temp_test_all_online(const srv_tz_temp_test_inst_t* inst);
static bool srv_tz_temp_test_in_control(const srv_tz_temp_test_inst_t* inst, uint8_t idx);
static bool srv_tz_temp_test_target_ready(const srv_tz_temp_test_inst_t* inst, uint8_t idx);
static uint32_t srv_tz_temp_test_can_id(uint8_t node, uint8_t cmd);
static void srv_tz_temp_test_pack_float_le(uint8_t* dst, float v);
static void srv_tz_temp_test_pack_u32_le(uint8_t* dst, uint32_t v);
static float srv_tz_temp_test_unpack_float_le(const uint8_t* src);
static uint32_t srv_tz_temp_test_unpack_u32_le(const uint8_t* src);
static void srv_tz_temp_test_send_axis_state(srv_tz_temp_test_inst_t* inst, uint8_t node, uint8_t state);
static void srv_tz_temp_test_send_controller_mode(srv_tz_temp_test_inst_t* inst, uint8_t node);
static void srv_tz_temp_test_send_input_vel(srv_tz_temp_test_inst_t* inst, uint8_t node, float tps);
static void srv_tz_temp_test_send_clear_errors(srv_tz_temp_test_inst_t* inst, uint8_t node);
static void srv_tz_temp_test_send_get_error(srv_tz_temp_test_inst_t* inst, uint8_t node);
static void srv_tz_temp_test_cmd_velocity_all(srv_tz_temp_test_inst_t* inst, float tps);
static void srv_tz_temp_test_motor_init_step(srv_tz_temp_test_inst_t* inst, uint8_t node, uint8_t step);
static void srv_tz_temp_test_probe_step(srv_tz_temp_test_inst_t* inst, uint32_t now);
static void srv_tz_temp_test_fallback(srv_tz_temp_test_inst_t* inst, uint32_t now);
#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_SPEED)
static float srv_tz_temp_test_phase_target(const srv_tz_temp_test_inst_t* inst, srv_tz_temp_test_phase_t phase);
static float srv_tz_temp_test_ramp_next(srv_tz_temp_test_inst_t* inst, uint32_t now, float target_tps);
static void srv_tz_temp_test_log_phase(srv_tz_temp_test_inst_t* inst, srv_tz_temp_test_phase_t phase);
#endif
#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_POSITION)
static float srv_tz_temp_test_pos_target_turns(const srv_tz_temp_test_inst_t* inst, uint8_t idx);
static void srv_tz_temp_test_pos_step(srv_tz_temp_test_inst_t* inst, uint32_t now);
static void srv_tz_temp_test_pos_return_center(srv_tz_temp_test_inst_t* inst, uint32_t now);
#endif
static void srv_tz_temp_test_scan_record(srv_tz_temp_test_inst_t* inst, uint8_t node);
static void srv_tz_temp_test_scan_log_new(srv_tz_temp_test_inst_t* inst);
static void srv_tz_temp_test_err_print(srv_tz_temp_test_inst_t* inst, uint8_t node, uint32_t code);

/* Exported functions --------------------------------------------------------*/

void srv_tz_temp_test_config_default(srv_tz_temp_test_config_t* cfg)
{
    if (!cfg)
        return;
    cfg->can_ch = DRV_CAN_CH_1;
    cfg->max_motors = SRV_TZ_TEMP_MAX_MOTORS;
    cfg->speed_tps = SRV_TZ_TEMP_DEF_SPEED_TPS;
    cfg->ramp_time_ms = SRV_TZ_TEMP_DEF_RAMP_TIME_MS;
    cfg->cmd_period_ms = SRV_TZ_TEMP_DEF_CMD_PERIOD_MS;
    cfg->phase_ms = SRV_TZ_TEMP_DEF_PHASE_MS;
    cfg->stop_hold_ms = SRV_TZ_TEMP_DEF_STOP_HOLD_MS;
    cfg->nresp_period_ms = SRV_TZ_TEMP_DEF_NORESP_PERIOD_MS;
    cfg->enable_retry_ms = SRV_TZ_TEMP_DEF_ENABLE_RETRY_MS;
    cfg->enable_timeout_ms = SRV_TZ_TEMP_DEF_ENABLE_TIMEOUT_MS;
    cfg->probe_interval_ms = SRV_TZ_TEMP_DEF_PROBE_INTERVAL_MS;
    cfg->fallback_ms = SRV_TZ_TEMP_DEF_FALLBACK_MS;
    cfg->fallback_node = SRV_TZ_TEMP_DEF_FALLBACK_NODE;
    cfg->init_grace_ms = SRV_TZ_TEMP_DEF_INIT_GRACE_MS;
    cfg->status_log_ms = SRV_TZ_TEMP_DEF_STATUS_LOG_MS;
    cfg->no_motor_log_ms = SRV_TZ_TEMP_DEF_NO_MOTOR_LOG_MS;
    cfg->duration_ms = SRV_TZ_TEMP_DEF_DURATION_SPEED_MS;
    cfg->auto_start = true;
#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_POSITION)
    cfg->duration_ms = SRV_TZ_TEMP_DEF_DURATION_POSITION_MS; /* 位置模式默认 30 天 */
#endif
    cfg->pos_amp_turns = SRV_TZ_TEMP_DEF_POS_AMP_TURNS;
    cfg->pos_kp = SRV_TZ_TEMP_DEF_POS_KP;
    cfg->pos_max_vel_tps = SRV_TZ_TEMP_DEF_POS_MAX_VEL_TPS;
    cfg->pos_accel_tps2 = SRV_TZ_TEMP_DEF_POS_ACCEL_TPS2;
    cfg->pos_arrive_thresh_turns = SRV_TZ_TEMP_DEF_POS_ARRIVE_THRESH_TURNS;
    cfg->pos_arrive_timeout_ms = SRV_TZ_TEMP_DEF_POS_ARRIVE_TIMEOUT_MS;
}

void srv_tz_temp_test_init(srv_tz_temp_test_inst_t* inst,
    const srv_tz_temp_test_config_t* cfg)
{
    if (!inst)
        return;

    if (cfg) {
        inst->config = *cfg;
    } else {
        srv_tz_temp_test_config_default(&inst->config);
    }

    /* 参数校验与兜底 */
    if (inst->config.can_ch >= DRV_CAN_CH_NUM) {
        inst->config.can_ch = DRV_CAN_CH_1;
    }
    if ((inst->config.max_motors == 0U) || (inst->config.max_motors > SRV_TZ_TEMP_MAX_MOTORS)) {
        inst->config.max_motors = SRV_TZ_TEMP_MAX_MOTORS;
    }

    /* 复位全部运行状态（保留配置副本） */
    const srv_tz_temp_test_config_t cfg_saved = inst->config;
    memset(inst, 0, sizeof(*inst));
    inst->config = cfg_saved;

    if (inst->config.auto_start) {
        srv_tz_temp_test_start(inst);
    }
}

/**
 * @brief 启动速度模式耐久测试
 * @param inst 实例句柄
 * @note   ODride 电机可能不发周期帧（heartbeat_rate_ms=0）：本模块在发现窗口内
 *         主动发 Get_Error(0x03) 探测 node 0~max_motors-1，靠回包被动收录；
 *         同时继续监听心跳被动发现。无任何发现时回退默认 node_id 盲发。
 */
void srv_tz_temp_test_start(srv_tz_temp_test_inst_t* inst)
{
    if (!inst)
        return;

    const uint32_t now = millis();
    inst->running = true;
    inst->stopping = false;
    inst->return_center = false;
    inst->return_center_start_ms = 0;
    inst->start_ms = now; /* 自动停止计时起点 */
    inst->online_ms = 0; /* 持续在线时长从 0 累计 */
    inst->online_last_ms = now;

    /* 阶段循环从正转开始；斜坡从 0 起步 */
    inst->phase = SRV_TZ_TEMP_PHASE_FORWARD;
    inst->phase_start_ms = now;
    inst->cycle = 0;
    inst->ramp_target_tps = 0.0f;
    inst->ramp_from_tps = 0.0f;
    inst->ramp_start_ms = now;
    inst->last_sent_tps = 0.0f;
    inst->last_cmd_ms = now;

    inst->last_enable_retry_ms = now;
    inst->enable_stall_since_ms = 0;
    inst->enable_warned = false;
    inst->probe_idx = 0;
    inst->err_query_idx = 0;
    inst->last_probe_ms = now;
    inst->probe_done = false;
    inst->fallback_active = false;
    memset(inst->last_init_ms, 0, sizeof(inst->last_init_ms));
    memset(inst->assumed_closed, 0, sizeof(inst->assumed_closed));
    memset(inst->init_step, 0, sizeof(inst->init_step));
    inst->last_no_motor_log_ms = now;
    inst->last_status_log_ms = now;

    memset(inst->motor_ids, 0, sizeof(inst->motor_ids));
    memset(inst->motor_axis_state, 0, sizeof(inst->motor_axis_state));
    memset(inst->axis_state_prev, 0, sizeof(inst->axis_state_prev));
    memset(inst->motor_err, 0, sizeof(inst->motor_err));
    memset(inst->err_pending, 0, sizeof(inst->err_pending));
    memset(inst->motor_encoder_turns, 0, sizeof(inst->motor_encoder_turns));
    memset(inst->motor_encoder_vel_tps, 0, sizeof(inst->motor_encoder_vel_tps));
    memset(inst->motor_have_encoder, 0, sizeof(inst->motor_have_encoder));
    memset(inst->motor_nresp_latch, 0, sizeof(inst->motor_nresp_latch));
    memset(inst->online_evt_pending, 0, sizeof(inst->online_evt_pending));
    memset(inst->center_turns, 0, sizeof(inst->center_turns));
    memset(inst->center_latched, 0, sizeof(inst->center_latched));
    memset(inst->pos_sign, 0, sizeof(inst->pos_sign));
    memset(inst->pos_last_flip_ms, 0, sizeof(inst->pos_last_flip_ms));
    memset(inst->pos_cmd_vel_tps, 0, sizeof(inst->pos_cmd_vel_tps));
    inst->motor_cnt = 0;
    inst->scan_log_cnt = 0;
    for (uint8_t i = 0; i < SRV_TZ_TEMP_MAX_MOTORS; i++) {
        inst->motor_last_seen_ms[i] = now; /* 掉线检测起点 */
    }

#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_SPEED)
    SRV_TZ_TEMP_TEST_LOG_I(inst,
        "速度耐久启动：心跳被动发现 + 主动探测 node 0~%u（目标 %d 毫转/s，正转/停/反转/停 各 %u s，%lu ms 自动停止）",
        (unsigned)(inst->config.max_motors - 1U),
        (int)(inst->config.speed_tps * 1000.0f),
        (unsigned)(inst->config.phase_ms / 1000U),
        (unsigned long)inst->config.duration_ms);
#else
    SRV_TZ_TEMP_TEST_LOG_I(inst,
        "位置耐久启动：初始化位置为 0°，±%d 毫转 往复（心跳被动发现 + 主动探测 node 0~%u，%lu ms 自动停止）",
        (int)(inst->config.pos_amp_turns * 1000.0f),
        (unsigned)(inst->config.max_motors - 1U),
        (unsigned long)inst->config.duration_ms);
#endif
}

/**
 * @brief 停止速度模式耐久测试（软停止：斜坡回 0 保持制动，随后发 IDLE）
 * @param inst 实例句柄
 * @note   IDLE 后电机失力，可安全断电；停止序列由 step() 完成，避免高转速下
 *         直接 IDLE 造成惯性空转
 */
void srv_tz_temp_test_stop(srv_tz_temp_test_inst_t* inst)
{
    if (!inst)
        return;
    if (!inst->running || inst->stopping)
        return;
    inst->stopping = true;
    inst->stop_start_ms = millis();
#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_POSITION)
    /* 位置模式：停止前先回 0°（中心），到位后再保持零速并发 IDLE */
    inst->return_center = true;
    inst->return_center_start_ms = inst->stop_start_ms;
#endif
    SRV_TZ_TEMP_TEST_LOG_I(inst, "耐久停止：速度回 0（位置模式先回 0°），随后发 IDLE");
}

/**
 * @brief 速度模式耐久测试周期步进（由 can_task 每 TASK_PERIOD_MS 调用，每实例各一次）
 * @param inst 实例句柄
 * @note  主动探测(Get_Error)+被动心跳双发现：新电机初始化、错误变化打印、闭环保持重试、
 *        阶段切换与主机侧斜坡下发、周期状态诊断日志、掉线告警与恢复重发闭环、
 *        累计在线满 duration_ms 自动软停止
 */
void srv_tz_temp_test_step(srv_tz_temp_test_inst_t* inst)
{
    if (!inst || !inst->running)
        return;

    const uint32_t now = millis();

    /* RX 队列消费（主循环上下文）：can_task 不直连 on_rx，本模块从驱动接收队列出队处理。
       驱动 HAL 回调只把帧入队，ISR 侧零协议解析；on_rx 保持 ISR 安全写法 */
    {
        drv_can_msg_t rmsg;
        while (drv_can_rx_pop(inst->config.can_ch, &rmsg)) {
            (void)srv_tz_temp_test_on_rx(inst, &rmsg);
        }
    }

    /* 停止序列：速度模式先斜坡回 0；位置模式先回 0°（中心）；随后保持零速并发 IDLE 完成停止 */
    if (inst->stopping) {
#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_SPEED)
        if ((now - inst->stop_start_ms)
            >= (inst->config.ramp_time_ms + inst->config.stop_hold_ms)) {
            for (uint8_t i = 0; i < inst->motor_cnt; i++) {
                srv_tz_temp_test_send_axis_state(inst, inst->motor_ids[i], SRV_TZ_TEMP_AXIS_STATE_IDLE);
            }
            inst->stopping = false;
            inst->running = false;
            SRV_TZ_TEMP_TEST_LOG_I(inst,
                "速度耐久已停止：已向 %u 台电机发 IDLE（累计在线 %lu ms，循环 %u 次）",
                (unsigned)inst->motor_cnt, (unsigned long)inst->online_ms, (unsigned)inst->cycle);
        } else {
            if ((now - inst->last_cmd_ms) >= inst->config.cmd_period_ms) {
                inst->last_cmd_ms = now;
                const float send = srv_tz_temp_test_ramp_next(inst, now, 0.0f);
                srv_tz_temp_test_cmd_velocity_all(inst, send);
            }
        }
#else
        /* 位置模式：先 P 控制回 0°，全部到位（或超时）后保持零速，随后发 IDLE */
        if (inst->return_center) {
            if ((now - inst->last_cmd_ms) >= inst->config.cmd_period_ms) {
                inst->last_cmd_ms = now;
                srv_tz_temp_test_pos_return_center(inst, now);
            }
            return;
        }
        if ((now - inst->stop_start_ms) >= inst->config.stop_hold_ms) {
            for (uint8_t i = 0; i < inst->motor_cnt; i++) {
                srv_tz_temp_test_send_axis_state(inst, inst->motor_ids[i], SRV_TZ_TEMP_AXIS_STATE_IDLE);
            }
            inst->stopping = false;
            inst->running = false;
            SRV_TZ_TEMP_TEST_LOG_I(inst,
                "位置耐久已停止：已回 0°并向 %u 台电机发 IDLE（累计在线 %lu ms）",
                (unsigned)inst->motor_cnt, (unsigned long)inst->online_ms);
        } else {
            if ((now - inst->last_cmd_ms) >= inst->config.cmd_period_ms) {
                inst->last_cmd_ms = now;
                srv_tz_temp_test_cmd_velocity_all(inst, 0.0f);
            }
        }
#endif
        return;
    }

    /* 持续在线计时：仅当所有已发现电机在线时累加（掉线期间不计时，恢复后继续累计）。
       online_last_ms 每步都更新，保证离线/恢复后时间不跳变 */
    if (srv_tz_temp_test_all_online(inst)) {
        inst->online_ms += (uint32_t)(now - inst->online_last_ms);
    }
    inst->online_last_ms = now;

    if ((inst->config.duration_ms != 0U) && (inst->online_ms >= inst->config.duration_ms)) {
        SRV_TZ_TEMP_TEST_LOG_I(inst, "累计在线 %lu ms 已到，耐久自动停止",
            (unsigned long)inst->online_ms);
        srv_tz_temp_test_stop(inst);
        return;
    }

    /* 主动探测：对不发周期帧的静默电机发 Get_Error，靠回包被动收录（发现窗口内轮转 node 0~max-1） */
    srv_tz_temp_test_probe_step(inst, now);
    /* 探测窗口结束仍无发现：回退到默认 node_id 盲发 */
    srv_tz_temp_test_fallback(inst, now);

    /* 未发现电机周期告警（区分"探测中"与"窗口已结束"，提示核对电机 CAN 使能/波特率/供电） */
    if (inst->motor_cnt == 0U && ((now - inst->last_no_motor_log_ms) >= inst->config.no_motor_log_ms)) {
        inst->last_no_motor_log_ms = now;
        SRV_TZ_TEMP_TEST_LOG_W(inst, "仍未发现电机：已主动探测 node 0~%u（%s），请核对电机 CAN 使能/波特率/供电",
            (unsigned)(inst->config.max_motors - 1U),
            inst->probe_done ? "探测窗口已结束" : "探测中");
    }

    /* 打印新发现的电机（主循环上下文，避免 ISR 打日志） */
    srv_tz_temp_test_scan_log_new(inst);

    /* 打印新到达的错误码（主循环上下文，ISR 只置标志）。心跳 axis_error 变化时打印 */
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (inst->err_pending[i]) {
            inst->err_pending[i] = false; /* 先清标志再取值，避免并发丢更新 */
            if (inst->motor_err[i] != 0U) {
                srv_tz_temp_test_err_print(inst, inst->motor_ids[i], inst->motor_err[i]);
            } else {
                SRV_TZ_TEMP_TEST_LOG_W(inst, "电机 node=%u 错误已消除，恢复正常",
                    (unsigned)inst->motor_ids[i]);
            }
        }
    }

    /* 闭环确认日志（心跳 axis_state 0/非闭环 → 8 上升沿，每电机一次） */
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if ((inst->motor_axis_state[i] == SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP)
            && (inst->axis_state_prev[i] != SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP)) {
            SRV_TZ_TEMP_TEST_LOG_I(inst, "电机 node=%u 已确认闭环",
                (unsigned)inst->motor_ids[i]);
        }
        inst->axis_state_prev[i] = inst->motor_axis_state[i];
    }

    /* 假定闭环：已下发 init 但无心跳确认的电机，宽限后假定已闭环，
       避免无反馈静默电机一直阻塞发速度/反复重发 init */
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (!inst->assumed_closed[i] && (inst->last_init_ms[i] != 0U)
            && (inst->motor_axis_state[i] != SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP)
            && ((now - inst->last_init_ms[i]) >= inst->config.init_grace_ms)) {
            inst->assumed_closed[i] = true;
            SRV_TZ_TEMP_TEST_LOG_I(inst, "电机 node=%u 无心跳闭环确认，假定已闭环（继续下发速度）",
                (unsigned)inst->motor_ids[i]);
        }
    }

    /* 保持闭环/补发配置：未假定闭环的电机逐帧推进 init 序列。
       init 拆成 3 帧（清错→速度模式→闭环）逐次下发，每 enable_retry_ms 发 1 帧——
       虽然 TX 已走 drv_can_tx_enqueue 软件队列（不再受硬件 FIFO 深度限制），
       仍保持逐帧间隔，避免瞬时堆满队列且保证电机侧处理顺序。序列完成后若仍未闭环
       则从头重发；已闭环则停止。带活动错误的电机也要收到 init（第一步即清错），
       否则锁存错误永远清不掉、永远进不了闭环。 */
    bool need_enable = false;
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (!inst->assumed_closed[i]
            && ((inst->motor_axis_state[i] != SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP)
                || (inst->last_init_ms[i] == 0U)
                || (inst->init_step[i] < 3U))) {
            need_enable = true;
            break;
        }
    }
    if (need_enable) {
        if (inst->enable_stall_since_ms == 0U) {
            inst->enable_stall_since_ms = now;
            inst->enable_warned = false;
        } else if (!inst->enable_warned
            && (now - inst->enable_stall_since_ms) >= inst->config.enable_timeout_ms) {
            inst->enable_warned = true;
            SRV_TZ_TEMP_TEST_LOG_W(inst, "电机闭环未确认已超时，继续周期重试");
        }
        if ((now - inst->last_enable_retry_ms) >= inst->config.enable_retry_ms) {
            inst->last_enable_retry_ms = now;
            /* 每个重试周期只推进 1 帧（全局轮转），避免多台电机同 tick 连发堆队列 */
            for (uint8_t i = 0; i < inst->motor_cnt; i++) {
                if (!inst->assumed_closed[i]
                    && ((inst->motor_axis_state[i] != SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP)
                        || (inst->last_init_ms[i] == 0U)
                        || (inst->init_step[i] < 3U))) {
                    if (inst->last_init_ms[i] == 0U) {
                        inst->last_init_ms[i] = now; /* 首次 init 计时起点（宽限判定用，不随重发刷新） */
                        SRV_TZ_TEMP_TEST_LOG_I(inst, "电机 node=%u 首次下发初始化（清错/速度模式/闭环）",
                            (unsigned)inst->motor_ids[i]);
                    }
                    if (inst->init_step[i] >= 3U) {
                        inst->init_step[i] = 0U; /* 序列已发完仍未闭环 → 从头重发 */
                    }
                    srv_tz_temp_test_motor_init_step(inst, inst->motor_ids[i], inst->init_step[i]);
                    inst->init_step[i]++;
                    break; /* 本轮只发一帧，下一轮再推进下一台/下一帧 */
                }
            }
        }
    } else {
        inst->enable_stall_since_ms = 0U; /* 已全部闭环/假定闭环或不在线，复位 */
    }

#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_SPEED)
    /* 阶段切换：正转 → 停 → 反转 → 停，每 phase_ms 推进；DWELL_R 结束回 FORWARD 计一个完整循环 */
    if ((now - inst->phase_start_ms) >= inst->config.phase_ms) {
        const srv_tz_temp_test_phase_t next =
            (srv_tz_temp_test_phase_t)(((uint32_t)inst->phase + 1U) % (uint32_t)SRV_TZ_TEMP_PHASE_NUM);
        if (next == SRV_TZ_TEMP_PHASE_FORWARD) {
            inst->cycle++;
        }
        inst->phase = next;
        inst->phase_start_ms = now;
        inst->last_cmd_ms = now; /* 立即进入下一速度段斜坡 */
        srv_tz_temp_test_log_phase(inst, inst->phase);
    }

    /* 主机侧线性斜坡下发：每 cmd_period_ms 计算一次目标段内插值速度并发送。
       目标变化时以当前下发速度为起点重新计时（软启停），锁存式目标同速重发为安全 no-op。 */
    if ((now - inst->last_cmd_ms) >= inst->config.cmd_period_ms) {
        inst->last_cmd_ms = now;
        const float target = srv_tz_temp_test_phase_target(inst, inst->phase);
        const float send = srv_tz_temp_test_ramp_next(inst, now, target);
        srv_tz_temp_test_cmd_velocity_all(inst, send);
    }
#else
    /* 位置模式控制：以初始化位置为 0°，±pos_amp_turns 往复；P 控制速度指令，到位即翻转（每电机独立） */
    if ((now - inst->last_cmd_ms) >= inst->config.cmd_period_ms) {
        inst->last_cmd_ms = now;
        srv_tz_temp_test_pos_step(inst, now);
    }
#endif

    /* 周期状态诊断日志：打印受控电机 编码器速度/位置/轴状态/错误/在线年龄 + 总体进度，
       停摆时用于区分「反馈冻结/掉出闭环/速度未到位」等根因 */
    if ((now - inst->last_status_log_ms) >= inst->config.status_log_ms) {
        inst->last_status_log_ms = now;
        for (uint8_t i = 0; i < inst->motor_cnt; i++) {
            if (!srv_tz_temp_test_in_control(inst, i)) {
                continue;
            }
#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_SPEED)
            SRV_TZ_TEMP_TEST_LOG_I(inst,
                "状态 node=%u axis=%u err=0x%08lX vel=%d enc=%d 毫转/s 毫转 tgt=%d 毫转/s enc_ok=%u last_seen=%lu ms",
                (unsigned)inst->motor_ids[i],
                (unsigned)inst->motor_axis_state[i],
                (unsigned long)inst->motor_err[i],
                (int)(inst->motor_encoder_vel_tps[i] * 1000.0f),
                (int)(inst->motor_encoder_turns[i] * 1000.0f),
                (int)(inst->last_sent_tps * 1000.0f),
                inst->motor_have_encoder[i] ? 1U : 0U,
                (unsigned long)(now - inst->motor_last_seen_ms[i]));
#else
            SRV_TZ_TEMP_TEST_LOG_I(inst,
                "状态 node=%u axis=%u err=0x%08lX vel=%d enc=%d 毫转/s 毫转 tgt_enc=%d 毫转 ctr=%d 毫转 vel_cmd=%d 毫转/s enc_ok=%u last_seen=%lu ms",
                (unsigned)inst->motor_ids[i],
                (unsigned)inst->motor_axis_state[i],
                (unsigned long)inst->motor_err[i],
                (int)(inst->motor_encoder_vel_tps[i] * 1000.0f),
                (int)(inst->motor_encoder_turns[i] * 1000.0f),
                (int)(srv_tz_temp_test_pos_target_turns(inst, i) * 1000.0f),
                (int)(inst->center_turns[i] * 1000.0f),
                (int)(inst->pos_cmd_vel_tps[i] * 1000.0f),
                inst->motor_have_encoder[i] ? 1U : 0U,
                (unsigned long)(now - inst->motor_last_seen_ms[i]));
#endif
        }
        SRV_TZ_TEMP_TEST_LOG_I(inst, "总览 在线时长=%lu ms（耐久=%lu ms） 循环=%u 电机=%u",
            (unsigned long)inst->online_ms,
            (unsigned long)inst->config.duration_ms,
            (unsigned)inst->cycle,
            (unsigned)inst->motor_cnt);
    }

    /* 电机无响应检测：超过 nresp_period 未收到任何帧视为掉线/断电（每电机只告警一次） */
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (!inst->motor_nresp_latch[i]
            && ((now - inst->motor_last_seen_ms[i]) >= inst->config.nresp_period_ms)) {
            inst->motor_nresp_latch[i] = true;
            SRV_TZ_TEMP_TEST_LOG_W(inst, "电机 node=%u 长时间无响应（掉线或断电）",
                (unsigned)inst->motor_ids[i]);
        }
    }

    /* 恢复在线：撤销假定闭环，重置 init 步进让 keep-alive 重新按序下发
       （掉线期间电机可能已复位为 IDLE，需完整重走 清错→速度模式→闭环） */
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (inst->online_evt_pending[i]) {
            inst->online_evt_pending[i] = false;
            inst->assumed_closed[i] = false;
            inst->last_init_ms[i] = 0U; /* 强制重新走完整 init 序列 */
            inst->init_step[i] = 0U;
            inst->center_latched[i] = false; /* 位置模式：恢复后重新以新位置为 0° */
            SRV_TZ_TEMP_TEST_LOG_W(inst, "电机 node=%u 恢复在线，重新按序下发闭环/速度模式",
                (unsigned)inst->motor_ids[i]);
        }
    }
}

/**
 * @brief 处理良志(ODrive)心跳/编码器帧（由 srv_tz_temp_test_step 从驱动接收队列出队后调用）
 * @param inst 实例句柄
 * @param msg  CAN 报文指针
 * @return true=ODrive 帧（node_id 0~max_motors-1, DLC≥8），已消费；false=非本协议帧
 * @note   主循环上下文调用（驱动 HAL 回调只把帧入 msg_fifo 队列），保持 ISR 安全写法：
 *         只做数据记录与标志置位，不打日志
 */
bool srv_tz_temp_test_on_rx(srv_tz_temp_test_inst_t* inst, const drv_can_msg_t* msg)
{
    if (!inst || !msg)
        return false;

    /* 仅消费标准帧、DLC≥8（良志协议帧均为 8 字节） */
    if (msg->is_extended || (msg->dlc < 8U))
        return false;

    const uint8_t node = (uint8_t)((msg->id >> 5) & 0x3FU);
    const uint8_t cmd = (uint8_t)(msg->id & 0x1FU);

    /* node_id 超出本实例管理上限：仍属 ODrive 帧，消费但不记录 */
    if (node >= inst->config.max_motors)
        return true;

    /* 未知 node 的帧：被动收录进列表，之后按索引处理 */
    uint8_t idx = srv_tz_temp_test_find_idx(inst, node);
    if (idx == SRV_TZ_TEMP_MAX_MOTORS) {
        srv_tz_temp_test_scan_record(inst, node);
        idx = srv_tz_temp_test_find_idx(inst, node);
        if (idx == SRV_TZ_TEMP_MAX_MOTORS)
            return true; /* 列表已满，忽略 */
    }

    /* 在线刷新：收到任意帧都视为"电机在应答"。
       此前掉线锁存的电机收到帧即恢复在线，置标志由主循环重发闭环（不打日志） */
    inst->motor_last_seen_ms[idx] = millis();
    if (inst->motor_nresp_latch[idx]) {
        inst->motor_nresp_latch[idx] = false;
        inst->online_evt_pending[idx] = true;
    }

    /* 心跳：data[0..3]=axis_error uint32 小端，data[4]=axis_state，
       data[5]=flags(bit7=轨迹完成)，data[6]=temp，data[7]=life */
    if (cmd == SRV_TZ_TEMP_CMD_HEARTBEAT) {
        inst->motor_axis_state[idx] = msg->data[4];
        if (inst->assumed_closed[idx]) {
            /* 心跳证明电机真实在线：撤销假定并强制重新走完整 init 序列，
               避免电机停在默认位置/速度模式导致不能运行 */
            inst->assumed_closed[idx] = false;
            inst->last_init_ms[idx] = 0U;
            inst->init_step[idx] = 0U;
        }
        if (msg->data[4] != SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP) {
            inst->assumed_closed[idx] = false; /* 心跳证实未闭环，撤销假定 */
        }
        const uint32_t err = srv_tz_temp_test_unpack_u32_le(&msg->data[0]);
        if (err != inst->motor_err[idx]) {
            inst->motor_err[idx] = err;
            inst->err_pending[idx] = true;
        }
        return true;
    }

    /* Get_Error 回包（主动探测用）：data[0..3]=错误码 uint32 小端，兼作在线/发现信号 */
    if (cmd == SRV_TZ_TEMP_CMD_GET_ERROR) {
        const uint32_t err = srv_tz_temp_test_unpack_u32_le(&msg->data[0]);
        if (err != inst->motor_err[idx]) {
            inst->motor_err[idx] = err;
            inst->err_pending[idx] = true;
        }
        return true;
    }

    /* 编码器估计：data[0..3]=pos float32 小端（转），data[4..7]=vel float32（转/s）。
       仅在闭环状态输出有效数据；主循环以 axis_state==8 为前提使用 */
    if (cmd == SRV_TZ_TEMP_CMD_ENCODER_ESTIMATES) {
        inst->motor_encoder_turns[idx] = srv_tz_temp_test_unpack_float_le(&msg->data[0]);
        inst->motor_encoder_vel_tps[idx] = srv_tz_temp_test_unpack_float_le(&msg->data[4]);
        inst->motor_have_encoder[idx] = true;
        return true;
    }

    /* 其他应答帧：已刷新 last_seen，消费之 */
    return true;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 查找 node_id 在已发现列表中的索引
 * @param inst 实例句柄
 * @param node 电机 node_id（0~63）
 * @return 索引；未找到返回 SRV_TZ_TEMP_MAX_MOTORS（哨兵值）
 */
static uint8_t srv_tz_temp_test_find_idx(const srv_tz_temp_test_inst_t* inst, uint8_t node)
{
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (inst->motor_ids[i] == node)
            return i;
    }
    return SRV_TZ_TEMP_MAX_MOTORS;
}

/**
 * @brief 所有已发现电机是否在线（无任何电机进入无响应锁存）
 * @param inst 实例句柄
 * @return true=全部在线
 * @note   duration 持续在线计时仅在全部在线时累加；未发现电机视为不在线
 */
static bool srv_tz_temp_test_all_online(const srv_tz_temp_test_inst_t* inst)
{
    if (inst->motor_cnt == 0U)
        return false;
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (inst->motor_nresp_latch[i])
            return false;
    }
    return true;
}

/**
 * @brief 电机是否处于"受控"状态（参与发速度/状态统计）
 * @param inst 实例句柄
 * @param idx  电机索引
 * @return true=心跳确认闭环 或 假定闭环
 */
static bool srv_tz_temp_test_in_control(const srv_tz_temp_test_inst_t* inst, uint8_t idx)
{
    return (inst->motor_axis_state[idx] == SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP)
        || inst->assumed_closed[idx];
}

/**
 * @brief 电机是否可下发速度（init 序列已完整执行）
 * @param inst 实例句柄
 * @param idx  电机索引
 * @return true=受控且已假定闭环 或 init 序列完成（3 帧全部发出）
 * @note   速度模式/清错未就位前不发 Set_Input_Vel，避免电机按错误模式运行
 */
static bool srv_tz_temp_test_target_ready(const srv_tz_temp_test_inst_t* inst, uint8_t idx)
{
    if (!srv_tz_temp_test_in_control(inst, idx))
        return false;
    return inst->assumed_closed[idx] || (inst->init_step[idx] >= 3U);
}

/**
 * @brief 主动探测步进：发现窗口内对未发现的候选 node 周期发 Get_Error(0x03)
 * @param inst 实例句柄
 * @param now  当前时间 (millis)
 * @note   Get_Error 回包由 on_rx 自动收录 node（任意帧即记录），因此无需改发现路径。
 *         发现窗口结束时置 probe_done 停止，限制失败帧数避免把主机打进 BUS-OFF；
 *         窗口后转入对已发现电机的周期错误回查（兼作静默电机的在线保活）
 */
static void srv_tz_temp_test_probe_step(srv_tz_temp_test_inst_t* inst, uint32_t now)
{
    if ((now - inst->last_probe_ms) < inst->config.probe_interval_ms)
        return;

    /* 发现窗口：轮转探测未发现的候选 node 0~max_motors-1 */
    if (!inst->probe_done) {
        if ((now - inst->start_ms) >= inst->config.fallback_ms) {
            inst->probe_done = true; /* 窗口结束，转入错误回查/回退判定 */
        } else {
            inst->last_probe_ms = now;
            for (uint8_t pass = 0; pass < inst->config.max_motors; pass++) {
                const uint8_t node = inst->probe_idx;
                inst->probe_idx++;
                if (inst->probe_idx >= inst->config.max_motors) {
                    inst->probe_idx = 0;
                }
                if (srv_tz_temp_test_find_idx(inst, node) == SRV_TZ_TEMP_MAX_MOTORS) {
                    srv_tz_temp_test_send_get_error(inst, node);
                    return;
                }
            }
            return;
        }
    }

    /* 发现窗口后：周期回查已发现电机的错误状态（on_rx 按 0x03 回包更新 err 并刷新 last_seen，
       兼作不发周期帧电机的在线保活，避免误报掉线） */
    if (inst->motor_cnt == 0U)
        return;
    inst->last_probe_ms = now;
    const uint8_t idx = inst->err_query_idx;
    inst->err_query_idx++;
    if (inst->err_query_idx >= inst->motor_cnt) {
        inst->err_query_idx = 0;
    }
    srv_tz_temp_test_send_get_error(inst, inst->motor_ids[idx]);
}

/**
 * @brief 探测窗口结束仍无发现的回退：按默认 node_id 盲发
 * @param inst 实例句柄
 * @param now  当前时间 (millis)
 * @note   盲发后直接假定受控并持续下发速度；若总线上无 ACK（波特率/使能不匹配），
 *         drv_can 的 tx fail / EP / BUS-OFF 日志即为诊断信号
 */
static void srv_tz_temp_test_fallback(srv_tz_temp_test_inst_t* inst, uint32_t now)
{
    if (inst->fallback_active || !inst->probe_done)
        return;
    if (inst->motor_cnt != 0U)
        return;

    inst->fallback_active = true;
    srv_tz_temp_test_scan_record(inst, inst->config.fallback_node);
    const uint8_t idx = srv_tz_temp_test_find_idx(inst, inst->config.fallback_node);
    if (idx == SRV_TZ_TEMP_MAX_MOTORS)
        return; /* 列表已满，放弃回退 */

    inst->last_init_ms[idx] = now;
    inst->assumed_closed[idx] = true; /* 盲发：直接假定受控，避免反复重发 */
    inst->init_step[idx] = 3U; /* 假定已闭环 → init 序列视为已完成（盲发节点无真实反馈） */
    SRV_TZ_TEMP_TEST_LOG_W(inst,
        "探测 %u 个 node 均无应答，回退默认 node_id=%u 盲发（若总线无 ACK 请核对波特率/使能）",
        (unsigned)inst->config.max_motors, (unsigned)inst->config.fallback_node);
}

#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_SPEED)
/**
 * @brief 当前阶段的标称目标速度（转/s）
 * @param inst  实例句柄
 * @param phase 阶段
 * @return 正转=+speed_tps，反转=-speed_tps，停=0
 */
static float srv_tz_temp_test_phase_target(const srv_tz_temp_test_inst_t* inst, srv_tz_temp_test_phase_t phase)
{
    if (phase == SRV_TZ_TEMP_PHASE_FORWARD)
        return inst->config.speed_tps;
    if (phase == SRV_TZ_TEMP_PHASE_REVERSE)
        return -inst->config.speed_tps;
    return 0.0f;
}

/**
 * @brief 主机侧线性斜坡：按 ramp_time_ms 从当前下发速度线性插值到目标速度
 * @param inst        实例句柄
 * @param now         当前时间 (millis)
 * @param target_tps  目标速度（转/s），与上一目标不同时以当前下发速度为起点重新计时
 * @return 本周期应下发的速度（转/s）
 * @note   软启停：正转/反转/停之间均经斜坡过渡，避免 0→目标 阶跃过流
 */
static float srv_tz_temp_test_ramp_next(srv_tz_temp_test_inst_t* inst, uint32_t now, float target_tps)
{
    if (target_tps != inst->ramp_target_tps) {
        inst->ramp_target_tps = target_tps;
        inst->ramp_from_tps = inst->last_sent_tps;
        inst->ramp_start_ms = now;
    }

    float send;
    const uint32_t elapsed = now - inst->ramp_start_ms;
    if (elapsed >= inst->config.ramp_time_ms) {
        send = target_tps; /* 斜坡完成，保持目标 */
    } else {
        const float span = target_tps - inst->ramp_from_tps;
        send = inst->ramp_from_tps + (span * ((float)elapsed / (float)inst->config.ramp_time_ms));
    }
    inst->last_sent_tps = send;
    return send;
}
#endif /* SPEED */

#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_POSITION)
/**
 * @brief 当前目标位置（转）：已锁存零点时为 中心 ±pos_amp_turns，未锁存返回 0
 * @param inst 实例句柄
 * @param idx  电机索引
 * @return 目标位置（转）
 */
static float srv_tz_temp_test_pos_target_turns(const srv_tz_temp_test_inst_t* inst, uint8_t idx)
{
    if (!inst->center_latched[idx]) {
        return 0.0f;
    }
    return (inst->pos_sign[idx] > 0)
        ? (inst->center_turns[idx] + inst->config.pos_amp_turns)
        : (inst->center_turns[idx] - inst->config.pos_amp_turns);
}

/**
 * @brief 位置模式控制步进（P 控制 + 加减速限幅的速度指令）
 * @param inst 实例句柄
 * @param now  当前时间 (millis)
 * @note   以各电机初始化位置为 0°，目标在 中心 ±pos_amp_turns 两端点间交替（每电机独立，
 *        到位即翻转）。期望速度 = Kp × 位置误差（限幅 ±MAX_VEL），下发速度按
 *        ACCEL_TPS2 斜率渐变逼近期望值——换向/到位时先减速过零再反向加速，无阶跃。
 *        未锁存零点（无编码器反馈/盲发）的电机发 0 速保持不动。
 *        到位判定：|位置−目标| ≤ ARRIVE_THRESH；反馈冻结/到位偏置超时强制翻转。
 */
static void srv_tz_temp_test_pos_step(srv_tz_temp_test_inst_t* inst, uint32_t now)
{
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (!srv_tz_temp_test_target_ready(inst, i)) {
            continue;
        }

        /* 锁存往复零点：进闭环且有编码器反馈时，以当前位置为 0° */
        if (!inst->center_latched[i] && inst->motor_have_encoder[i]) {
            inst->center_turns[i] = inst->motor_encoder_turns[i];
            inst->center_latched[i] = true;
            inst->pos_sign[i] = 1;
            inst->pos_last_flip_ms[i] = now;
            inst->pos_cmd_vel_tps[i] = 0.0f; /* 从静止开始 */
            SRV_TZ_TEMP_TEST_LOG_I(inst,
                "已锁存往复零点：电机 node=%u 当前 %d 毫转为 0°，目标区间 [%d, %d] 毫转",
                (unsigned)inst->motor_ids[i],
                (int)(inst->center_turns[i] * 1000.0f),
                (int)((inst->center_turns[i] - inst->config.pos_amp_turns) * 1000.0f),
                (int)((inst->center_turns[i] + inst->config.pos_amp_turns) * 1000.0f));
        }

        if (!inst->center_latched[i]) {
            /* 未锁存零点（无编码器反馈/盲发）：保持当前位置不动 */
            srv_tz_temp_test_send_input_vel(inst, inst->motor_ids[i], 0.0f);
            continue;
        }

        /* 到位翻转：|位置−目标| ≤ 阈值 或 超时强制翻转 */
        const float cur = inst->motor_encoder_turns[i];
        float target = srv_tz_temp_test_pos_target_turns(inst, i);
        float err = target - cur;
        if (err < 0.0f) {
            err = -err;
        }
        bool flip = false;
        if (err <= inst->config.pos_arrive_thresh_turns) {
            flip = true;
        } else if ((now - inst->pos_last_flip_ms[i]) >= inst->config.pos_arrive_timeout_ms) {
            SRV_TZ_TEMP_TEST_LOG_W(inst, "电机 node=%u 位置到位超时，强制翻转",
                (unsigned)inst->motor_ids[i]);
            flip = true;
        }
        if (flip) {
            inst->pos_sign[i] = (int8_t)-inst->pos_sign[i];
            inst->pos_last_flip_ms[i] = now;
            target = srv_tz_temp_test_pos_target_turns(inst, i);
        }

        /* 期望速度 = Kp × 位置误差，限幅到 ±max_vel */
        float desired = (target - cur) * inst->config.pos_kp;
        if (desired > inst->config.pos_max_vel_tps) {
            desired = inst->config.pos_max_vel_tps;
        } else if (desired < -inst->config.pos_max_vel_tps) {
            desired = -inst->config.pos_max_vel_tps;
        }

        /* 加减速限幅：下发速度每控制周期最多变化 accel × dt，换向平滑过零 */
        const float dt = (float)inst->config.cmd_period_ms / 1000.0f;
        const float dv_max = inst->config.pos_accel_tps2 * dt;
        float vel = inst->pos_cmd_vel_tps[i];
        if (desired > vel) {
            vel += dv_max;
            if (vel > desired) {
                vel = desired;
            }
        } else {
            vel -= dv_max;
            if (vel < desired) {
                vel = desired;
            }
        }
        inst->pos_cmd_vel_tps[i] = vel;

        srv_tz_temp_test_send_input_vel(inst, inst->motor_ids[i], vel);
    }
}

/**
 * @brief 停止序列回 0°（中心）：P 控制把受控电机带回初始化零点
 * @param inst 实例句柄
 * @param now  当前时间 (millis)
 * @note   全部已锁存电机到位（|位置−中心| ≤ arrive_thresh）或超时后，
 *        置 return_center=false 并重置 stop_start_ms 进入保持零速+IDLE 阶段；
 *        未锁存（无反馈/盲发）电机发 0 速保持，不参与到位等待。
 */
static void srv_tz_temp_test_pos_return_center(srv_tz_temp_test_inst_t* inst, uint32_t now)
{
    bool all_at_center = true;
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (!srv_tz_temp_test_in_control(inst, i)) {
            continue;
        }
        if (!inst->center_latched[i]) {
            /* 无编码器反馈（盲发/静默）：无法回中，发 0 速保持 */
            srv_tz_temp_test_send_input_vel(inst, inst->motor_ids[i], 0.0f);
            continue;
        }

        const float cur = inst->motor_encoder_turns[i];
        const float err_s = inst->center_turns[i] - cur;
        float err_abs = err_s;
        if (err_abs < 0.0f) {
            err_abs = -err_abs;
        }

        if (err_abs <= inst->config.pos_arrive_thresh_turns) {
            /* 已到中心：下发速度按加减速限制渐变回 0 */
            const float dt = (float)inst->config.cmd_period_ms / 1000.0f;
            const float dv_max = inst->config.pos_accel_tps2 * dt;
            float vel = inst->pos_cmd_vel_tps[i];
            if (vel > dv_max) {
                vel -= dv_max;
            } else if (vel < -dv_max) {
                vel += dv_max;
            } else {
                vel = 0.0f;
            }
            inst->pos_cmd_vel_tps[i] = vel;
            srv_tz_temp_test_send_input_vel(inst, inst->motor_ids[i], vel);
            continue;
        }

        all_at_center = false;
        float desired = err_s * inst->config.pos_kp;
        if (desired > inst->config.pos_max_vel_tps) {
            desired = inst->config.pos_max_vel_tps;
        } else if (desired < -inst->config.pos_max_vel_tps) {
            desired = -inst->config.pos_max_vel_tps;
        }
        const float dt = (float)inst->config.cmd_period_ms / 1000.0f;
        const float dv_max = inst->config.pos_accel_tps2 * dt;
        float vel = inst->pos_cmd_vel_tps[i];
        if (desired > vel) {
            vel += dv_max;
            if (vel > desired) {
                vel = desired;
            }
        } else {
            vel -= dv_max;
            if (vel < desired) {
                vel = desired;
            }
        }
        inst->pos_cmd_vel_tps[i] = vel;
        srv_tz_temp_test_send_input_vel(inst, inst->motor_ids[i], vel);
    }

    if (all_at_center) {
        inst->return_center = false;
        inst->stop_start_ms = now; /* 回 0° 完成，进入保持零速+IDLE 计时 */
        SRV_TZ_TEMP_TEST_LOG_I(inst, "电机均已回到 0°，准备停止");
    } else if ((now - inst->return_center_start_ms) >= SRV_TZ_TEMP_POS_RETURN_TIMEOUT_MS) {
        inst->return_center = false;
        inst->stop_start_ms = now;
        SRV_TZ_TEMP_TEST_LOG_W(inst, "回 0° 超时，强制停止");
    }
}
#endif /* POSITION */

/**
 * @brief 计算良志(ODrive) CAN ID：CAN ID = (node_id<<5) | cmd_id
 * @param node 电机 node_id（0~63）
 * @param cmd  命令码（0~31）
 * @return 标准 11 位 CAN ID
 */
static uint32_t srv_tz_temp_test_can_id(uint8_t node, uint8_t cmd)
{
    return (uint32_t)(((uint32_t)(node & 0x3FU) << 5) | (uint32_t)(cmd & 0x1FU));
}

/* --- 小端打包/解包（Cortex-M4 小端，memcpy 避免别名/对齐问题） --- */

static void srv_tz_temp_test_pack_float_le(uint8_t* dst, float v)
{
    memcpy(dst, &v, 4);
}

static void srv_tz_temp_test_pack_u32_le(uint8_t* dst, uint32_t v)
{
    memcpy(dst, &v, 4);
}

static float srv_tz_temp_test_unpack_float_le(const uint8_t* src)
{
    float v;
    memcpy(&v, src, 4);
    return v;
}

static uint32_t srv_tz_temp_test_unpack_u32_le(const uint8_t* src)
{
    uint32_t v;
    memcpy(&v, src, 4);
    return v;
}

/* --- 命令发送（经典 CAN 2.0A, DLC 8, 小端） --- */

/**
 * @brief 发送设置轴状态帧 (0x07)
 * @param inst  实例句柄
 * @param node  电机 node_id
 * @param state 目标轴状态（1=IDLE，8=闭环）
 */
static void srv_tz_temp_test_send_axis_state(srv_tz_temp_test_inst_t* inst, uint8_t node, uint8_t state)
{
    drv_can_msg_t tx = {
        .id = srv_tz_temp_test_can_id(node, SRV_TZ_TEMP_CMD_SET_AXIS_STATE),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tz_temp_test_pack_u32_le(&tx.data[0], (uint32_t)state);
    (void)drv_can_tx_enqueue(inst->config.can_ch, &tx);
}

/**
 * @brief 发送设置控制模式帧 (0x0B)：速度控制(2) + 直通输入(1)
 * @param inst 实例句柄
 * @param node 电机 node_id
 */
static void srv_tz_temp_test_send_controller_mode(srv_tz_temp_test_inst_t* inst, uint8_t node)
{
    drv_can_msg_t tx = {
        .id = srv_tz_temp_test_can_id(node, SRV_TZ_TEMP_CMD_SET_CONTROLLER_MODE),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tz_temp_test_pack_u32_le(&tx.data[0], SRV_TZ_TEMP_CONTROL_MODE_VELOCITY);
    srv_tz_temp_test_pack_u32_le(&tx.data[4], SRV_TZ_TEMP_INPUT_MODE_PASS_THROUGH);
    (void)drv_can_tx_enqueue(inst->config.can_ch, &tx);
}

/**
 * @brief 发送设置目标速度帧 (0x0D)
 * @param inst 实例句柄
 * @param node 电机 node_id
 * @param tps  目标速度（转/s，正=正转，负=反转，0=停）；力矩前馈恒为 0
 */
static void srv_tz_temp_test_send_input_vel(srv_tz_temp_test_inst_t* inst, uint8_t node, float tps)
{
    drv_can_msg_t tx = {
        .id = srv_tz_temp_test_can_id(node, SRV_TZ_TEMP_CMD_SET_INPUT_VEL),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tz_temp_test_pack_float_le(&tx.data[0], tps); /* 目标速度（转/s） */
    srv_tz_temp_test_pack_float_le(&tx.data[4], 0.0f); /* 力矩前馈 0（Nm） */
    (void)drv_can_tx_enqueue(inst->config.can_ch, &tx);
}

/**
 * @brief 发送清除错误帧 (0x18)
 * @param inst 实例句柄
 * @param node 电机 node_id
 * @note   ODrive 标准：data[0..3]=clear_errors(1=清本轴错误)，data[4..7]=
 *         clear_errors_on_other_axis(0=不清其它轴)。良志文档写「8 字节 0」，但 ODrive
 *         实现要求标志置 1 才真正清错，按 ODrive 标准执行；若现场无效再回退试 8×0
 */
static void srv_tz_temp_test_send_clear_errors(srv_tz_temp_test_inst_t* inst, uint8_t node)
{
    drv_can_msg_t tx = {
        .id = srv_tz_temp_test_can_id(node, SRV_TZ_TEMP_CMD_CLEAR_ERRORS),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    srv_tz_temp_test_pack_u32_le(&tx.data[0], 1U); /* clear_errors=true */
    (void)drv_can_tx_enqueue(inst->config.can_ch, &tx);
}

/**
 * @brief 发送获取错误帧 (0x03)：data[0]=error_type(0=电机错误)
 * @param inst 实例句柄
 * @param node 电机 node_id
 * @note   ODrive 系电机收到会回错误码帧；该回包被 on_rx 当作"在线/发现"信号，
 *         用于 heartbeat_rate_ms=0（不发周期帧）的静默电机的主动探测
 */
static void srv_tz_temp_test_send_get_error(srv_tz_temp_test_inst_t* inst, uint8_t node)
{
    drv_can_msg_t tx = {
        .id = srv_tz_temp_test_can_id(node, SRV_TZ_TEMP_CMD_GET_ERROR),
        .is_extended = false,
        .is_fd = false,
        .dlc = 8,
    };
    memset(tx.data, 0, sizeof(tx.data));
    tx.data[0] = 0U; /* error_type=0：电机错误 */
    (void)drv_can_tx_enqueue(inst->config.can_ch, &tx);
}

/**
 * @brief 对全部可下发速度的电机发送目标速度帧（逐台串行入队）
 * @param inst 实例句柄
 * @param tps  目标速度（转/s）
 */
static void srv_tz_temp_test_cmd_velocity_all(srv_tz_temp_test_inst_t* inst, float tps)
{
    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (srv_tz_temp_test_target_ready(inst, i)) {
            srv_tz_temp_test_send_input_vel(inst, inst->motor_ids[i], tps);
        }
    }
}

/**
 * @brief 电机初始化逐帧下发（step=0..2，每次只发 1 帧）
 * @param inst 实例句柄
 * @param node 电机 node_id
 * @param step 帧序号：0=清错，1=速度模式(速度控制+直通)，2=闭环
 * @note   按 ODrive 推荐顺序：先配置 模式 再进闭环，避免进闭环后按默认位置模式运行。
 *         每帧由 keep-alive 以 enable_retry_ms 间隔逐次调用，经 drv_can_tx_enqueue
 *         入软件队列后由驱动周期排空，保证 3 帧不被丢失。
 */
static void srv_tz_temp_test_motor_init_step(srv_tz_temp_test_inst_t* inst, uint8_t node, uint8_t step)
{
    switch (step) {
    case 0:
        srv_tz_temp_test_send_clear_errors(inst, node); /* 清残留错误，避免闭环被旧错误阻塞 */
        break;
    case 1:
        srv_tz_temp_test_send_controller_mode(inst, node); /* 速度控制 + 直通输入 */
        break;
    case 2:
        srv_tz_temp_test_send_axis_state(inst, node, SRV_TZ_TEMP_AXIS_STATE_CLOSED_LOOP);
        break;
    default:
        break;
    }
}

/**
 * @brief 记录发现的电机 node_id（去重，越界忽略）
 * @param inst 实例句柄
 * @param node 电机 node_id
 * @note   主循环上下文调用（RX 队列消费路径），只做数组操作，不打日志
 */
static void srv_tz_temp_test_scan_record(srv_tz_temp_test_inst_t* inst, uint8_t node)
{
    if (node >= inst->config.max_motors)
        return;

    for (uint8_t i = 0; i < inst->motor_cnt; i++) {
        if (inst->motor_ids[i] == node)
            return; /* 已记录 */
    }
    if (inst->motor_cnt < inst->config.max_motors) {
        const uint8_t idx = inst->motor_cnt;
        inst->motor_ids[idx] = node;
        inst->motor_last_seen_ms[idx] = millis();
        inst->motor_axis_state[idx] = 0U; /* 尚未闭环 */
        inst->motor_have_encoder[idx] = false;
        inst->motor_encoder_turns[idx] = 0.0f;
        inst->motor_encoder_vel_tps[idx] = 0.0f;
        inst->init_step[idx] = 0U; /* 新电机：从 init 序列头开始（逐帧下发） */
        inst->center_latched[idx] = false; /* 位置模式：新电机未锁存 0° 中心 */
        inst->center_turns[idx] = 0.0f;
        inst->pos_sign[idx] = 1;
        inst->pos_last_flip_ms[idx] = millis();
        inst->pos_cmd_vel_tps[idx] = 0.0f;
        inst->motor_cnt++;
    }
}

/**
 * @brief 打印扫描期间新发现的电机（主循环调用，避免 ISR 打日志）
 * @param inst 实例句柄
 */
static void srv_tz_temp_test_scan_log_new(srv_tz_temp_test_inst_t* inst)
{
    while (inst->scan_log_cnt < inst->motor_cnt) {
        SRV_TZ_TEMP_TEST_LOG_I(inst, "  发现电机：node_id = %u",
            (unsigned)inst->motor_ids[inst->scan_log_cnt]);
        inst->scan_log_cnt++;
    }
}

/**
 * @brief 打印错误详情（主循环上下文调用）
 * @param inst 实例句柄
 * @param node 电机 node_id
 * @param code 错误码
 * @note   错误码可组合（多位同时置位）；WARN 打印原始码并逐位解码
 */
static void srv_tz_temp_test_err_print(srv_tz_temp_test_inst_t* inst, uint8_t node, uint32_t code)
{
    SRV_TZ_TEMP_TEST_LOG_W(inst, "电机 node=%u 错误：0x%08X", (unsigned)node, (unsigned)code);
    for (uint8_t i = 0; i < SRV_TZ_TEMP_ERR_NUM; i++) {
        if ((code & s_err_map[i].mask) != 0U) {
            SRV_TZ_TEMP_TEST_LOG_W(inst, "  - %s", s_err_map[i].name);
        }
    }
}

#if (SRV_TZ_TEMP_CTRL_MODE == SRV_TZ_TEMP_CTRL_MODE_SPEED)
/**
 * @brief 打印阶段切换日志
 * @param inst  实例句柄
 * @param phase 新阶段
 */
static void srv_tz_temp_test_log_phase(srv_tz_temp_test_inst_t* inst, srv_tz_temp_test_phase_t phase)
{
    const char* name;
    switch (phase) {
    case SRV_TZ_TEMP_PHASE_FORWARD:
        name = "正转";
        break;
    case SRV_TZ_TEMP_PHASE_DWELL_F:
        name = "停(正转后)";
        break;
    case SRV_TZ_TEMP_PHASE_REVERSE:
        name = "反转";
        break;
    case SRV_TZ_TEMP_PHASE_DWELL_R:
        name = "停(反转后)";
        break;
    default:
        name = "未知";
        break;
    }

    const float tps = srv_tz_temp_test_phase_target(inst, phase);
    SRV_TZ_TEMP_TEST_LOG_I(inst, "阶段切换 → %s（目标速度 %d 毫转/s，时长 %u s，循环 %u）",
        name,
        (int)(tps * 1000.0f),
        (unsigned)(inst->config.phase_ms / 1000U),
        (unsigned)inst->cycle);
}
#endif /* SPEED */
