/**
 * @file    srv_tz_temp_test.h
 * @brief   良志(ODrive) 伺服执行器 CAN 控制协议服务 — 耐久测试（多实例，速度/位置双模式）
 *
 * 电机控制指令按 docs/良志电机can协议.md 组帧发送（经典 CAN 2.0A, 1 Mbps,
 * CAN-ID = (node_id<<5) | cmd_id，8 字节帧，小端，IEEE-754 单精度浮点）。
 *
 * 控制模式由编译期宏 SRV_TZ_TEMP_CTRL_MODE 切换：
 *   - SPEED    速度模式：正转→停→反转→停 定时循环（现有功能），主机侧斜坡软启停；
 *   - POSITION 位置模式：以各电机初始化位置为 0°，通过反馈位置 P 控制
 *     （速度指令 = Kp × 位置误差，限幅 + 加减速限制）在 0°±pos_amp_turns
 *     （默认 6.5 转）之间往复，到位即翻转目标。
 *
 * 本服务支持多个实例，每个实例绑定一条 CAN 通道（config.can_ch），各自独立
 * 发现/管理电机并运行耐久循环；两种模式均持续在线累计满 config.duration_ms 自动软停止
 * （先速度回 0 再发 IDLE）。
 *
 * RX 侧：驱动层把收到的帧入 msg_fifo 队列（ISR 只入队），本模块 step() 内用
 * drv_can_rx_pop(inst->config.can_ch) 出队并交给 on_rx() 解析记录；TX 侧全部经
 * drv_can_tx_enqueue() 入软件队列、由 drv_can_poll_status 周期排空。
 */

#ifndef __SRV_TZ_TEMP_TEST_H
#define __SRV_TZ_TEMP_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "drv_can.h"

/* Exported constants --------------------------------------------------------*/

/** @brief 单个实例最多可管理电机数（node_id 协议范围 0~63，本模块每实例上限） */
#define SRV_TZ_TEMP_MAX_MOTORS 16U


/* Exported types ------------------------------------------------------------*/

/** @brief 测试循环阶段：正转 → 停 → 反转 → 停，一个循环 */
typedef enum {
    SRV_TZ_TEMP_PHASE_FORWARD = 0, /**< 正转 */
    SRV_TZ_TEMP_PHASE_DWELL_F, /**< 正转后停 */
    SRV_TZ_TEMP_PHASE_REVERSE, /**< 反转 */
    SRV_TZ_TEMP_PHASE_DWELL_R, /**< 反转后停 */
    SRV_TZ_TEMP_PHASE_NUM, /**< 阶段总数 */
} srv_tz_temp_test_phase_t;

/**
 * @brief 实例配置
 * @note   srv_tz_temp_test_config_default() 填充默认值，调用方按需覆盖字段
 */
typedef struct {
    drv_can_channel_t can_ch; /**< 绑定的 CAN 通道（DRV_CAN_CH_1 / CH_2） */
    uint8_t max_motors; /**< 本实例最多管理电机数（≤ SRV_TZ_TEMP_MAX_MOTORS） */
    float speed_tps; /**< 正转/反转目标速度（转/s），停发 0 */
    uint32_t ramp_time_ms; /**< 主机侧软斜坡时长 (ms)：目标速度变化线性插值到达 */
    uint32_t cmd_period_ms; /**< 速度命令帧发送周期 (ms)：斜坡步长 + 锁存目标重发周期 */
    uint32_t phase_ms; /**< 每个阶段时长 (ms)：正转/停/反转/停 */
    uint32_t stop_hold_ms; /**< 停止序列斜坡回 0 后保持零速时长 (ms)，随后发 IDLE */
    uint32_t nresp_period_ms; /**< 电机无响应判定周期 (ms)：超过视为掉线/断电 */
    uint32_t enable_retry_ms; /**< 未闭环电机重发 init 的周期 (ms) */
    uint32_t enable_timeout_ms; /**< 闭环确认超时 (ms)：超时打印一次告警（继续重试） */
    uint32_t probe_interval_ms; /**< 主动探测周期 (ms)：对候选 node 发 Get_Error */
    uint32_t fallback_ms; /**< 发现窗口时长 (ms)：结束仍无发现则回退盲发 */
    uint8_t fallback_node; /**< 回退默认 node_id：探测无果后按该 ID 盲发 */
    uint32_t init_grace_ms; /**< 假定闭环宽限 (ms)：init 下发后无确认 → 假定已闭环 */
    uint32_t status_log_ms; /**< 周期状态诊断日志间隔 (ms) */
    uint32_t no_motor_log_ms; /**< 未发现电机周期告警日志间隔 (ms) */
    uint32_t duration_ms; /**< 耐久运行时长 (ms)：仅全部电机在线时累计，0=禁用自动停止 */
    bool auto_start; /**< true=init 后自动 start() */

    /* —— 位置模式参数（SRV_TZ_TEMP_CTRL_MODE == POSITION 时生效，每实例独立） —— */
    float pos_amp_turns; /**< 往复半幅（转）：目标在 初始化零点(0°) ±pos_amp_turns 两端点交替（180° = 0.5 转） */
    float pos_kp; /**< 位置环 P 增益（1/s）：速度指令 = Kp × 位置误差（转） */
    float pos_max_vel_tps; /**< 位置模式速度指令限幅（转/s） */
    float pos_accel_tps2; /**< 位置模式速度指令加减速限制（转/s²）：换向/到位时按该斜率渐变，无速度阶跃 */
    float pos_arrive_thresh_turns; /**< 到位判定阈值（转）：|位置−目标| ≤ 该值视为到位（随后翻转目标） */
    uint32_t pos_arrive_timeout_ms; /**< 位置到位判定超时 (ms)：超时仍未到位则强制翻转（反馈冻结兜底） */
} srv_tz_temp_test_config_t;

/**
 * @brief 实例句柄（调用者静态分配；运行时字段由 init/start 初始化，勿直接操作）
 */
typedef struct {
    srv_tz_temp_test_config_t config; /**< 实例配置（副本） */

    /* —— 运行时状态（内部） —— */
    bool running; /**< 测试模式运行标志 */
    bool stopping; /**< 停止序列进行中：先斜坡回 0 保持制动，随后发 IDLE */
    uint32_t stop_start_ms; /**< 停止序列起始时间 (millis) */
    uint8_t motor_ids[SRV_TZ_TEMP_MAX_MOTORS]; /**< 已发现电机 node_id 列表 */
    uint8_t motor_cnt; /**< 已发现电机数量 */
    uint8_t scan_log_cnt; /**< 已打印日志的电机数 */
    srv_tz_temp_test_phase_t phase; /**< 当前阶段 */
    uint32_t phase_start_ms; /**< 当前阶段起始时间 (millis) */
    uint32_t cycle; /**< 已完成的完整循环次数（正转→停→反转→停 计 1 次） */
    uint8_t motor_axis_state[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机最新心跳轴状态 */
    uint8_t axis_state_prev[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机上次轴状态 */
    uint32_t motor_err[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机最新错误码 */
    bool err_pending[SRV_TZ_TEMP_MAX_MOTORS]; /**< 新错误应答待打印标志 */
    float motor_encoder_turns[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机最新编码器位置（转） */
    float motor_encoder_vel_tps[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机最新编码器速度（转/s） */
    bool motor_have_encoder[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机是否已收到过编码器帧 */
    uint32_t motor_last_seen_ms[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机最后收到帧时间 (millis) */
    bool motor_nresp_latch[SRV_TZ_TEMP_MAX_MOTORS]; /**< 电机无响应告警锁存 */
    bool online_evt_pending[SRV_TZ_TEMP_MAX_MOTORS]; /**< 恢复在线事件待处理标志 */

    /* —— 位置模式状态（SRV_TZ_TEMP_CTRL_MODE == POSITION 时使用） —— */
    float center_turns[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机往复中心（转）：初始化位置为 0° */
    bool center_latched[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机往复中心是否已锁存 */
    int8_t pos_sign[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机目标方向：+1=朝 +180°，-1=朝 -180° */
    uint32_t pos_last_flip_ms[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机最近目标翻转时间 (millis) */
    float pos_cmd_vel_tps[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机当前下发速度（转/s，按加减速限制渐变） */
    bool return_center; /**< 位置模式停止序列：正在回 0°（中心）标志 */
    uint32_t return_center_start_ms; /**< 位置模式回 0° 起始时间 (millis) */
    uint32_t last_enable_retry_ms; /**< 上次重发 init 的时间 (millis) */
    uint32_t enable_stall_since_ms; /**< 进入"需闭环"状态起始时间，0=不在该状态 */
    bool enable_warned; /**< 闭环确认超时告警是否已打印 */
    uint8_t probe_idx; /**< 主动探测：下一个候选 node（0~max-1 轮转） */
    uint8_t err_query_idx; /**< 错误回查：下一个待回查的已发现电机索引 */
    uint32_t last_probe_ms; /**< 主动探测上次发送时间 (millis) */
    bool probe_done; /**< 发现窗口结束标志 */
    bool fallback_active; /**< 已回退到默认 node_id 盲发标志 */
    uint32_t last_init_ms[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机最近首次 init 下发时间 */
    bool assumed_closed[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机假定闭环标志 */
    uint8_t init_step[SRV_TZ_TEMP_MAX_MOTORS]; /**< 每电机 init 步进序号（0..2=待发帧，3=完成） */
    uint32_t last_cmd_ms; /**< 上次速度命令发送时间 (millis) */
    float ramp_target_tps; /**< 当前斜坡段目标速度（转/s） */
    float ramp_from_tps; /**< 当前斜坡段起点速度（转/s） */
    uint32_t ramp_start_ms; /**< 当前斜坡段起始时间 (millis) */
    float last_sent_tps; /**< 最近一次实际下发的速度（转/s） */
    uint32_t last_no_motor_log_ms; /**< 上次「仍未发现电机」日志时间 (millis) */
    uint32_t last_status_log_ms; /**< 上次周期状态诊断日志时间 (millis) */
    uint32_t start_ms; /**< 测试起始时间 (millis) */
    uint32_t online_ms; /**< 累计在线时长 (ms)：仅全部电机在线时累加 */
    uint32_t online_last_ms; /**< 上次在线时长累计时间点 (millis) */
} srv_tz_temp_test_inst_t;

/* Exported functions prototypes ---------------------------------------------*/

/* --- 配置 --- */

/**
 * @brief 填充实例默认配置（调用方随后按需覆盖字段，如 can_ch / speed_tps / duration_ms）
 * @param cfg 配置指针（非空）
 */
void srv_tz_temp_test_config_default(srv_tz_temp_test_config_t* cfg);

/* --- 生命周期 --- */

/**
 * @brief 初始化一个实例
 * @param inst 实例句柄（调用者静态分配）
 * @param cfg  配置指针；NULL=使用默认配置
 * @note   config.auto_start=true 时初始化后自动进入测试模式；
 *         不阻塞（无 delay），启动时序由调用方安排
 */
void srv_tz_temp_test_init(srv_tz_temp_test_inst_t* inst,
    const srv_tz_temp_test_config_t* cfg);

/**
 * @brief 启动该实例的速度耐久测试
 * @param inst 实例句柄
 * @note  等待电机周期心跳(0x01)被动发现 node_id（辅以 Get_Error(0x03) 主动探测），
 *        对发现的电机下发 清错 → 闭环 → 速度+PASS_THROUGH 模式；
 *        Set_Input_Vel 按 正转+SPEED→停0→反转-SPEED→停0 每 phase_ms 切换，
 *        主机侧 ramp_time_ms 线性斜坡软启停；持续在线累计满 duration_ms 自动停止
 */
void srv_tz_temp_test_start(srv_tz_temp_test_inst_t* inst);

/**
 * @brief 停止该实例的耐久测试（软停止：速度回 0；位置模式先回 0° 中心，随后发 IDLE）
 * @param inst 实例句柄
 */
void srv_tz_temp_test_stop(srv_tz_temp_test_inst_t* inst);

/**
 * @brief 周期步进（主循环每 TASK_PERIOD_MS 调用，每实例各调一次）
 * @param inst 实例句柄
 * @note  从 inst->config.can_ch 的驱动接收队列出队消费反馈；阶段切换与斜坡下发、
 *        闭环保持重试、状态诊断日志、掉线告警与恢复、累计在线满 duration_ms 软停止
 */
void srv_tz_temp_test_step(srv_tz_temp_test_inst_t* inst);

/* --- RX 帧处理 --- */

/**
 * @brief 处理良志(ODrive)心跳/编码器帧（由 srv_tz_temp_test_step 从驱动接收队列出队后调用）
 * @param inst 实例句柄
 * @param msg  CAN 报文指针
 * @return true=ODrive 帧（node_id 0~max_motors-1, DLC≥8），已消费；
 *         false=非本协议帧（不应出现在该实例专用总线）
 * @note   主循环上下文调用（驱动 HAL 回调只把帧入 msg_fifo 队列），保持 ISR 安全写法：
 *         只做数据记录与标志置位，不打日志
 */
bool srv_tz_temp_test_on_rx(srv_tz_temp_test_inst_t* inst, const drv_can_msg_t* msg);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_TZ_TEMP_TEST_H */
