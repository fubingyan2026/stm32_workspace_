/**
 * @file    srv_ht_can2_torque_test.c
 * @brief   苇熠(HT) 伺服执行器 CAN 控制协议服务实现 — 速度模式多圈往复耐久测试（CAN2 版）
 *
 * 本模块是 srv_ht_torque_test（CAN1/FDCAN1 版）在 CAN2/FDCAN2 上的完整克隆：
 * 运行逻辑与功能完全一致，唯一区别是全部指令经 DRV_CAN_CH_2（PB12 RX / PB13 TX）
 * 发送，供 CAN1 已被占用时在独立总线上控制另一组苇熠电机。
 *
 * 电机控制指令按 docs/苇熠电机can协议文档.md 组帧发送（经典 CAN 2.0A, 1 Mbps,
 * CAN-ID 低 8 位 = 设备地址，data[0]=指令，data[1..]=参数，多字节大端）。
 *
 * 与 srv_ht_temp_test（速度模式时间相位）的区别：本模块用速度模式（0x07 02）
 * 连续旋转，令电机在「初始化位置」±POS_LIMIT_DEG 之间多圈往复（初始化位置即
 * 各电机启动后首次读回的位置，作为自身 0 点，掉线恢复后重新锁存）。方向变化时用
 * RAMP_MS 线性斜坡平滑加减速（消除卡顿/突变）；每 POS_POLL_PERIOD_MS 用 0x06
 * 读当前位置，到达端点（中心±POS_LIMIT）即反向。累计在线运行满 30 天自动停止并失能。
 *
 * 启动流程：先扫描总线电机 ID（握手指令 0x00 逐地址探测，0x01~0x3F），
 * 对检测到的电机下发使能/速度模式；未检测到任何电机时回退到
 * 默认设备地址（SRV_HT_CAN2_TORQUE_TEST_DEFAULT_MOTOR_ADDR）继续控制。
 *
 * RX 侧：srv_ht_can2_torque_test_on_rx() 由 can_task 按 CH_2 分发，只消费测试
 * 协议帧（扫描应答/位置/报警/电压/在线心跳），返回 true 表示已处理；CAN2 为
 * 独立专用总线，非测试帧直接丢弃。
 *
 * 速度发送策略：速度模式（0x09）电机侧锁存速度值（非零启动、零停止），方向
 * 变化时由固件 RAMP_MS 斜坡在 CMD_PERIOD_MS 周期重发，斜坡完成后保持目标转速。
 */

#include "srv_ht_can2_torque_test.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/
/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印（30 天耐久测试确认摆动正常后可关闭） */
#define SRV_HT_CAN2_TORQUE_TEST_LOG_ENABLE 1

#if SRV_HT_CAN2_TORQUE_TEST_LOG_ENABLE
#define SRV_HT_CAN2_TORQUE_TEST_LOG_I(...) //LOG_I("srv_ht_can2_torque_test", __VA_ARGS__)
#define SRV_HT_CAN2_TORQUE_TEST_LOG_W(...) LOG_W("torque_test_can2", __VA_ARGS__)
#define SRV_HT_CAN2_TORQUE_TEST_LOG_E(...) LOG_E("torque_test_can2", __VA_ARGS__)
#else
#define SRV_HT_CAN2_TORQUE_TEST_LOG_I(...) ((void)0)
#define SRV_HT_CAN2_TORQUE_TEST_LOG_W(...) ((void)0)
#define SRV_HT_CAN2_TORQUE_TEST_LOG_E(...) ((void)0)
#endif
/* 模块测试开关 ----------------------------------------------------------------*/

/** @brief 电机测试模式：1=上电自动启动（扫描 + 速度模式多圈往复循环）；0=需手动调用 srv_ht_can2_torque_test_start() */
#define SRV_HT_CAN2_TORQUE_TEST_AUTO_START 1

/* Private constants ---------------------------------------------------------*/

/* --- 苇熠伺服执行器 CAN 测试参数 --- */

/** @brief 往复半幅 (deg)：电机以初始化位置为中心，在 中心±该角度（15 圈 = 5400°，1 转=360°）间摆动 */
#define SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_DEG (6 * 360)
/**
 * @brief 往复半幅换算为 IQ24 位置值（单位 R）：15R = 0x00F00000
 * @note  IQ = deg/360 × 2^24（文档 §4.2：位置 IQ24 值即为实际转数，满量程 ±127R）
 */
const int32_t SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_IQ = ((SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_DEG / 360) << 24);

/** @brief 到达端点判定阈值 (deg)：|位置-端点| ≤ 该角度视为到达端点（随后反向） */
#define SRV_HT_CAN2_TORQUE_TEST_REACH_DEG 3
/** @brief 到达端点判定阈值换算为 IQ24（同上换算公式） */
#define SRV_HT_CAN2_TORQUE_TEST_REACH_IQ \
    ((int32_t)(((int64_t)SRV_HT_CAN2_TORQUE_TEST_REACH_DEG << 24) / 360))
/** @brief 位置反馈轮询周期 (ms)：0x06 读取当前电机位置，用于端点反向判定 */
#define SRV_HT_CAN2_TORQUE_TEST_POS_POLL_PERIOD_MS 20U

/* --- 速度模式参数（速度模式 0x02 连续旋转，固件手动斜坡平滑加减速） --- */

/** @brief 巡航转速 (RPM)：连续旋转的目标转速，正负表示方向（与 CAN1 版 srv_ht_torque_test 一致） */
#define SRV_HT_CAN2_TORQUE_TEST_SPEED_RPM 300
/** @brief 斜坡时长 (ms)：速度目标变化时在该时长内线性爬升/下降（平滑加减速、消除卡顿） */
#define SRV_HT_CAN2_TORQUE_TEST_RAMP_MS 2000U
/** @brief 速度帧重发周期 (ms)：斜坡期间按该周期重发速度（巡航期速度已锁存） */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_PERIOD_MS 100U
/** @brief 报警查询周期 (ms)：电机报警需主动查询（0xFF），1s 一次 */
#define SRV_HT_CAN2_TORQUE_TEST_ALARM_PERIOD_MS 1000U
/** @brief 报警查询相对位置轮询的时间偏移 (ms)：错开查询与位置帧，避免多包冲突丢应答 */
#define SRV_HT_CAN2_TORQUE_TEST_ALARM_OFFSET_MS (SRV_HT_CAN2_TORQUE_TEST_ALARM_PERIOD_MS / 2U)
/** @brief 电机无响应判定周期 (ms)：超过该时长未收到电机任何帧视为掉线/断电。
 *        取 10s：电机速度模式自主运行，短时丢帧不影响运转，放大阈值可减少
 *        偶发误判触发的「恢复在线→重新使能」循环（该循环会重置电机参考点） */
#define SRV_HT_CAN2_TORQUE_TEST_NORESP_PERIOD_MS 10000U
/**
 * @brief 耐久运行时长 (ms)：电机持续在线累计满 30 天自动停止并失能。
 *        30 天 = 30×24×3600×1000 ms = 2,592,000,000 ms（uint32 范围内）；掉线期间不计时
 */
#define SRV_HT_CAN2_TORQUE_TEST_DURATION_MS 2592000000U
/** @brief 扫描完成后 1s 内补发使能/速度模式周期 (ms)：首帧可能因 TX FIFO 未就绪被丢弃 */
#define SRV_HT_CAN2_TORQUE_TEST_STARTUP_RETRY_MS 100U
/** @brief 端点反向超时兜底 (ms)：超过该时长未发生端点反向（位置反馈冻结/0x06 丢帧/到位偏置）
 *        时强制反向，防止电机只往一个方向跑（同良志排查文档 §3.3 到位超时强制翻转）。
 *        单程约 7.5s（9R@100RPM+斜坡 2s），60s ≈ 8× 留足裕量 */
#define SRV_HT_CAN2_TORQUE_TEST_FLIP_TIMEOUT_MS 60000U
/** @brief 周期状态诊断日志 (ms)：打印各电机 位置/方向/转速/报警/在线年龄。
 *        60s 一次避免刷屏；确认耐久运行正常后可置大或关闭 */
#define SRV_HT_CAN2_TORQUE_TEST_STATUS_LOG_MS 60000U
/** @brief 1=记录并打印所有收到的 CAN 帧（确认 0x06 位置应答真实格式）；0=关闭 */
#define SRV_HT_CAN2_TORQUE_TEST_LOG_RAW_RX_ENABLE 0
/** @brief 原始帧环形缓冲深度 */
#define SRV_HT_CAN2_TORQUE_TEST_RAW_RX_DEPTH 16U
/** @brief 1=周期读取供电电压（0x87）；0=关闭。实测 0x87 仅返回 [0x87][状态]（写命令应答），不含电压数据 */
#define SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE 0
#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
/** @brief 供电电压读取周期 (ms)：0x87 读取（×100，单位 V，仅使能时刷新） */
#define SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_PERIOD_MS 3000U
/** @brief 电压读取相对位置轮询的时间偏移 (ms)：错开与位置/报警查询同时发送 */
#define SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_OFFSET_MS 250U
#endif

/* --- 总线电机扫描参数 --- */

/** @brief 扫描地址范围（含）：协议推荐 0x01~0x3F，0x00 为广播且无返回 */
#define SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MIN 0x01U
#define SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MAX 0x3FU
/** @brief 每地址探测间隔 (ms)：留出电机握手应答时间 */
#define SRV_HT_CAN2_TORQUE_TEST_SCAN_PROBE_PERIOD_MS 20U
/** @brief 总线重扫周期 (ms)：电机晚于控制板上电（热插拔）时错过启动扫描，
 *        周期重扫握手探测 0x01~0x3F 发现新电机并接管使能 */
#define SRV_HT_CAN2_TORQUE_TEST_RESCAN_PERIOD_MS 3000U
/** @brief 最多可同时控制的电机数 */
#define SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS 63U
/** @brief 未扫描到电机时使用的默认设备地址（CAN-ID 低 8 位） */
#define SRV_HT_CAN2_TORQUE_TEST_DEFAULT_MOTOR_ADDR 0x21U

/* --- 苇熠协议指令符（docs/苇熠电机can协议文档.md §6） --- */

#define SRV_HT_CAN2_TORQUE_TEST_CMD_HANDSHAKE 0x00U /**< 握手：发送 1 字节，电机返回 [0x00][状态] */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_ENABLE 0x2AU /**< 使能/失能：参数 0x01 使能，0x00 失能 */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_MODE 0x07U /**< 设置模式 */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_SPEED 0x09U /**< 设置速度值：IQ24，归一化 × 速度满量程（正=正转，负=反转，0=停止） */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_POS_READ 0x06U /**< 读取当前位置值：返回 [0x06][4B 大端 IQ24]（5B） */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_ALARM 0xFFU /**< 查询报警：发送 1 字节，实测返回 [0xFF][4B 大端报警码]（5B） */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_VOLTAGE 0x87U /**< 读取供电电压：返回 [0x87][V高][V低]，×100 单位 V（仅使能时刷新） */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_ENABLE_STATE 0x2BU /**< 查询使能/失能状态：返回 [0x2B][0x01/0x00]（2B） */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_MODE_QUERY 0x55U /**< 查询当前模式：返回 [0x55][模式值]（2B，读取指令1） */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_SPEED_READ 0x05U /**< 读取当前速度值：返回 [0x05][4B 大端 IQ24]（5B，×6000） */
#define SRV_HT_CAN2_TORQUE_TEST_MODE_SPEED 0x02U /**< 速度模式 */
#define SRV_HT_CAN2_TORQUE_TEST_SPEED_FULL_SCALE 6000 /**< 速度满量程 (RPM)：IQ24 = 值/6000 × 2^24（文档 §4.2） */
#define SRV_HT_CAN2_TORQUE_TEST_CMD_CUR_LIMIT 0x58U /**< 设置电流限制（写入指令3）：归一化 IQ24，× 满量程电流 */
/** @brief 电流限制归一化值 (IQ24)：1.0 = 满量程电流（型号相关，如 45A），速度模式扭矩输出上限由此决定。
 *        出厂默认限制偏小导致扭矩不够时保持 0x01000000（满量程）即拉到最大扭矩；可按需下调 */
#define SRV_HT_CAN2_TORQUE_TEST_CUR_LIMIT_IQ 0x01000000U
/** @brief 使能保持补发周期 (ms)：电机在线但查询到未使能时，按该周期补发使能+速度模式。
 *        修「电机晚于控制板上电、错过启动 1s 补发窗口后永久失能」问题（同良志排查文档 §2.2）。
 *        起始偏移取 11ms（非 20/100ms 整数倍），与位置轮询(0x06)/速度(0x09)/报警(0xFF)
 *        分时错开，保证任一 tick 帧数不超 FDCAN TX FIFO 深度 3 */
#define SRV_HT_CAN2_TORQUE_TEST_ENABLE_KEEPALIVE_MS 2000U
#define SRV_HT_CAN2_TORQUE_TEST_ENABLE_KEEPALIVE_OFFSET_MS 11U

/* Private types -------------------------------------------------------------*/

/** @brief 报警码描述表项（见 docs/苇熠电机can协议文档.md §8，含 24 位错误码） */
typedef struct {
    uint32_t mask; /**< 错误码位 */
    const char* name; /**< 含义 */
} srv_ht_can2_torque_test_alarm_desc_t;

/** @brief 报警码 → 含义映射表（协议文档 §8；0xFF 应答为 [0xFF][4B 大端]，支持全部 24 位码） */
static const srv_ht_can2_torque_test_alarm_desc_t s_alarm_map[] = {
    { 0x00000001U, "过压" },
    { 0x00000002U, "欠压" },
    { 0x00000004U, "堵转" },
    { 0x00000008U, "过热" },
    { 0x00000010U, "读写参数异常" },
    { 0x00000020U, "EtherCAT通信异常" },
    { 0x00000040U, "位置超差" },
    { 0x00000080U, "CAN通信异常" },
    { 0x00000100U, "速度超差" },
    { 0x00000200U, "阶跃过大" },
    { 0x00000400U, "DRV保护" },
    { 0x00000800U, "编码器故障" },
    { 0x00001000U, "多圈错误" },
    { 0x00002000U, "抱闸故障" },
    { 0x00004000U, "末端编码器故障" },
    { 0x00008000U, "初始化失败" },
    { 0x00010000U, "电机缺相" },
    { 0x00020000U, "母线过流" },
    { 0x00080000U, "校准错误" },
    { 0x00100000U, "逆变器过温" },
    { 0x00200000U, "电机过温" },
    { 0x00400000U, "IMU初始化失败" },
    { 0x00800000U, "IAP错误" },
};

/** @brief 报警码描述表项数 */
#define SRV_HT_CAN2_TORQUE_TEST_ALARM_NUM (sizeof(s_alarm_map) / sizeof(s_alarm_map[0]))

#if SRV_HT_CAN2_TORQUE_TEST_LOG_RAW_RX_ENABLE
/** @brief 原始接收帧记录（调试用，ISR 写入主循环打印） */
typedef struct {
    uint32_t id; /**< CAN ID */
    uint8_t dlc; /**< 数据长度 */
    uint8_t data[8]; /**< 前 8 字节 */
} srv_ht_can2_torque_test_raw_rx_t;
#endif

/* Private variables ---------------------------------------------------------*/

/** @brief 测试模式运行标志 */
static bool s_running;

/** @brief 总线扫描进行中标志（true=只发握手探测，不发控制命令） */
static bool s_scanning;

/** @brief 扫描当前探测地址 */
static uint8_t s_scan_addr;

/** @brief 扫描阶段上次探测时间 (millis) */
static uint32_t s_scan_last_probe_ms;

/** @brief 重扫进行中标志（true=周期重扫握手探测中，发现新电机） */
static bool s_rescanning;

/** @brief 重扫当前探测地址 */
static uint8_t s_rescan_addr;

/** @brief 重扫阶段上次探测时间 (millis) */
static uint32_t s_rescan_last_probe_ms;

/** @brief 上次重扫结束时间 (millis)：按 RESCAN_PERIOD 周期重扫，发现晚到电机 */
static uint32_t s_last_rescan_ms;

/** @brief 回退占位标志：启动扫描未发现电机时回退到默认地址并置位；
 *        热插拔重扫前若仍占位则清空列表，由重扫重新发现真实电机 */
static bool s_fallback_active;

/** @brief 重扫开始前的电机数：重扫完成后与当前数比较，识别新发现的电机 */
static uint8_t s_rescan_start_cnt;

/** @brief 已检测到的电机 CAN-ID 列表（握手 0x00 应答） */
static uint8_t s_motor_ids[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 已检测电机数量（ISR 写入，主循环读取） */
static uint8_t s_motor_cnt;

/** @brief 已打印日志的电机数（避免重复打印） */
static uint8_t s_scan_log_cnt;

/** @brief 控制阶段起始时间 (millis)，扫描完成后置位 */
static uint32_t s_ctrl_start_ms;

/** @brief 测试起始时间 (millis)，用于 30 天自动停止 */
static uint32_t s_start_ms;

/** @brief 累计在线时长 (ms)：仅电机持续在线时累加，用于 DURATION 自动停止 */
static uint32_t s_online_ms;

/** @brief 上次在线时长累计时间点 (millis) */
static uint32_t s_online_last_ms;

/* --- 速度模式连续旋转控制 --- */

/** @brief 全局旋转方向：+1=向 +50R（正转），-1=向 0（反转） */
static int8_t s_dir;

/** @brief 每电机最新读回位置 (IQ24，单位 R，ISR 写主循环读) */
static int32_t s_motor_pos_iq[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 新位置应答待处理标志（ISR 置位，主循环清零后做端点反向判定） */
static bool s_motor_pos_pending[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 每电机往复中心 (IQ24，单位 R)：各电机初始化（首次读到位置）时的位置作为自身 0 点，
 *        目标在 各自中心 ± POS_LIMIT 两端点间交替（多电机各以自身初始化位置摆动） */
static int32_t s_center_iq[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 每电机往复中心是否已锁存（首个位置应答后锁存；恢复在线后重新锁存） */
static bool s_center_latched[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 每电机最新使能状态（0x2B 应答更新，0=失能，1=使能，ISR 写） */
static bool s_motor_enabled[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 每电机使能状态是否已确认（收到过 0x2B 应答；未确认时不补发使能） */
static bool s_motor_enable_known[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 每电机最新模式（0x55 应答更新，ISR 写） */
static uint8_t s_motor_mode[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 每电机最新实际速度 IQ24（0x05 应答更新，×6000 RPM，ISR 写） */
static int32_t s_motor_speed_iq[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 上次使能保持补发时间 (millis) */
static uint32_t s_last_keepalive_ms;

/** @brief 使能保持补发相位：0=查询0x2B，1=补发使能，2=补发模式，3=复查（每相位占一个 keepalive 周期，
 *        保证任一 tick 对单电机至多发 1 帧，规避 FDCAN TX FIFO 深度 3 突爆丢帧） */
static uint8_t s_keepalive_phase[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 未使能补发日志锁存：同一次失能期间只打印一次（电机确认使能后清除） */
static bool s_enable_log_latch[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 状态日志查询相位：0=查模式0x55，1=查速度0x05（每 5s 周期只发 1 帧） */
static bool s_status_q_phase;

/** @brief 当前实际下发转速 (RPM)：斜坡插值结果 */
static int16_t s_speed_rpm;

/** @brief 斜坡段目标转速 (RPM)：方向变化时重新计时 */
static int16_t s_ramp_target_rpm;

/** @brief 斜坡段起点转速 (RPM)：目标变化瞬间的下发转速 */
static int16_t s_ramp_from_rpm;

/** @brief 斜坡段起始时间 (millis) */
static uint32_t s_ramp_start_ms;

/** @brief 上次速度帧发送时间 (millis) */
static uint32_t s_last_cmd_ms;

/** @brief 上次位置轮询时间 (millis) */
static uint32_t s_last_pos_poll_ms;

/** @brief 每电机最新报警码（0xFF 应答更新，24 位，ISR 写） */
static uint32_t s_motor_alarm[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 新报警应答待打印标志（ISR 置位，主循环清零后打印） */
static bool s_alarm_pending[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 上一轮已处理的报警码（主循环维护，用于状态变化检测） */
static uint32_t s_alarm_last[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 每电机最后收到任何帧的时间 (millis)，用于掉线检测（ISR 更新） */
static uint32_t s_motor_last_seen_ms[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 电机无响应告警锁存（收到任何帧后清除，避免重复刷屏） */
static bool s_motor_nresp_latch[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 恢复在线事件待打印标志（ISR 置位，主循环清零打印） */
static bool s_online_evt_pending[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 控制阶段上次补发时间 (millis) */
static uint32_t s_last_retry_ms;

/** @brief 上次报警查询时间 (millis) */
static uint32_t s_last_alarm_ms;

/** @brief 上次方向翻转时间 (millis)：用于端点反向超时兜底判定 */
static uint32_t s_last_flip_ms;

/** @brief 启动补发相位：0=补发使能，1=补发速度模式，2=补发电流限制（交错，避免单 tick 突爆 >3 帧） */
static uint8_t s_retry_phase;

/** @brief 周期状态日志上次打印时间 (millis) */
static uint32_t s_last_status_ms;

#if SRV_HT_CAN2_TORQUE_TEST_LOG_RAW_RX_ENABLE
/** @brief 原始接收帧环形缓冲（ISR 写 head，主循环读 tail） */
static srv_ht_can2_torque_test_raw_rx_t s_raw_rx[SRV_HT_CAN2_TORQUE_TEST_RAW_RX_DEPTH];
static volatile uint8_t s_raw_rx_head;
static volatile uint8_t s_raw_rx_tail;
#endif

#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
/** @brief 每电机最新供电电压（0x87 应答更新，×100 单位 V，ISR 写） */
static uint16_t s_motor_volt[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 新电压应答待打印标志（ISR 置位，主循环清零） */
static bool s_volt_pending[SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS];

/** @brief 上次电压读取时间 (millis) */
static uint32_t s_last_volt_ms;
#endif

/* Private function prototypes -----------------------------------------------*/

static uint32_t srv_ht_can2_torque_test_find_idx(uint8_t addr);
static bool srv_ht_can2_torque_test_all_online(void);
static void srv_ht_can2_torque_test_scan_record(uint8_t addr);
static void srv_ht_can2_torque_test_scan_log_new(void);
static void srv_ht_can2_torque_test_scan_done(void);
static void srv_ht_can2_torque_test_rescan_done(uint32_t now);
static void srv_ht_can2_torque_test_send_handshake(uint8_t addr);
static void srv_ht_can2_torque_test_send_enable(uint8_t addr, bool enable);
static void srv_ht_can2_torque_test_set_speed_mode(uint8_t addr);
static void srv_ht_can2_torque_test_send_cur_limit(uint8_t addr);
static void srv_ht_can2_torque_test_send_speed(uint8_t addr, int16_t rpm);
static void srv_ht_can2_torque_test_send_query_position(uint8_t addr);
static void srv_ht_can2_torque_test_cmd_enable_all(bool enable);
static void srv_ht_can2_torque_test_cmd_set_mode_all(void);
static void srv_ht_can2_torque_test_cmd_set_cur_limit_all(void);
static void srv_ht_can2_torque_test_cmd_speed_all(int16_t rpm);
static void srv_ht_can2_torque_test_query_position_all(void);
static void srv_ht_can2_torque_test_send_query_alarm(uint8_t addr);
static void srv_ht_can2_torque_test_send_query_enable_state(uint8_t addr);
static void srv_ht_can2_torque_test_send_query_mode(uint8_t addr);
static void srv_ht_can2_torque_test_send_query_speed(uint8_t addr);
static void srv_ht_can2_torque_test_query_alarm_all(void);
static void srv_ht_can2_torque_test_alarm_print(uint8_t addr, uint32_t code);
#if SRV_HT_CAN2_TORQUE_TEST_LOG_RAW_RX_ENABLE
static void srv_ht_can2_torque_test_raw_rx_push(const drv_can_msg_t* msg);
static void srv_ht_can2_torque_test_raw_rx_drain(void);
#endif
#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
static void srv_ht_can2_torque_test_send_query_voltage(uint8_t addr);
static void srv_ht_can2_torque_test_query_voltage_all(void);
static void srv_ht_can2_torque_test_voltage_print(uint8_t addr, uint16_t volt);
#endif

/* Exported functions --------------------------------------------------------*/

void srv_ht_can2_torque_test_init(void)
{
#if SRV_HT_CAN2_TORQUE_TEST_AUTO_START
    srv_ht_can2_torque_test_start(); /* 测试模式：先扫描总线电机 ID，再下发控制命令 */
#endif
}

/**
 * @brief 启动往复耐久测试模式
 * @note  先扫描总线电机 ID（握手 0x00，探测 0x01~0x3F），扫描完成后
 *        对检测到的电机发送：使能 (0x2A 01) → 速度模式 (0x07 02)；之后按
 *        位置监控反向，速度用固件斜坡平滑加减速，连续旋转不停顿
 */
void srv_ht_can2_torque_test_start(void)
{
    s_running = true;
    s_start_ms = millis(); /* 30 天自动停止计时起点 */
    s_online_ms = 0; /* 持续在线时长从 0 累计 */
    s_online_last_ms = s_start_ms;

    /* 重置速度斜坡状态：从 0 起步 */
    s_dir = 1;
    s_speed_rpm = 0;
    s_ramp_target_rpm = 0;
    s_ramp_from_rpm = 0;
    s_ramp_start_ms = s_start_ms;
    s_last_flip_ms = s_start_ms;
    s_retry_phase = 0;
    s_last_status_ms = s_start_ms;
    s_last_keepalive_ms = s_start_ms;
    s_status_q_phase = false;

    /* 阶段 1：扫描总线电机 ID */
    s_scanning = true;
    s_scan_addr = SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MIN;
    s_motor_cnt = 0;
    s_scan_log_cnt = 0;
    s_scan_last_probe_ms = s_start_ms - SRV_HT_CAN2_TORQUE_TEST_SCAN_PROBE_PERIOD_MS;
    s_rescanning = false;
    s_fallback_active = false;
    s_rescan_start_cnt = 0;
    s_last_rescan_ms = s_start_ms;
    memset(s_motor_pos_iq, 0, sizeof(s_motor_pos_iq));
    memset(s_motor_pos_pending, 0, sizeof(s_motor_pos_pending));
    memset(s_center_iq, 0, sizeof(s_center_iq));
    memset(s_center_latched, 0, sizeof(s_center_latched));
    memset(s_motor_enabled, 0, sizeof(s_motor_enabled));
    memset(s_motor_enable_known, 0, sizeof(s_motor_enable_known));
    memset(s_motor_mode, 0, sizeof(s_motor_mode));
    memset(s_motor_speed_iq, 0, sizeof(s_motor_speed_iq));
    memset(s_keepalive_phase, 0, sizeof(s_keepalive_phase));
    memset(s_enable_log_latch, 0, sizeof(s_enable_log_latch));
    memset(s_motor_alarm, 0, sizeof(s_motor_alarm));
    memset(s_alarm_pending, 0, sizeof(s_alarm_pending));
    memset(s_alarm_last, 0, sizeof(s_alarm_last));
    memset(s_motor_last_seen_ms, 0, sizeof(s_motor_last_seen_ms));
    memset(s_motor_nresp_latch, 0, sizeof(s_motor_nresp_latch));
    memset(s_online_evt_pending, 0, sizeof(s_online_evt_pending));
#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
    memset(s_motor_volt, 0, sizeof(s_motor_volt));
    memset(s_volt_pending, 0, sizeof(s_volt_pending));
#endif

    SRV_HT_CAN2_TORQUE_TEST_LOG_I("耐久测试启动：正在扫描总线电机 ID 0x%02X~0x%02X（每 %u ms 探测一个地址）",
        SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MIN, SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MAX,
        (unsigned)SRV_HT_CAN2_TORQUE_TEST_SCAN_PROBE_PERIOD_MS);
}

/**
 * @brief 停止往复耐久测试模式：先发速度 0 停下，再失能
 * @note  断电前必须失能 (0x2A 00)，否则零位可能丢失；命令发往所有检测到的电机
 */
void srv_ht_can2_torque_test_stop(void)
{
    s_running = false;
    s_scanning = false;
    s_speed_rpm = 0;
    s_ramp_target_rpm = 0;
    s_ramp_from_rpm = 0;
    s_ramp_start_ms = 0;
    srv_ht_can2_torque_test_cmd_speed_all(0); /* 速度 0 停下 */
    srv_ht_can2_torque_test_cmd_enable_all(false); /* 失能 */
    SRV_HT_CAN2_TORQUE_TEST_LOG_I("耐久测试停止：已向 %u 台电机发送速度 0 + 失能", (unsigned)s_motor_cnt);
}

/**
 * @brief 往复耐久测试模式周期步进（由 can_task 每 TASK_PERIOD_MS 调用）
 * @note  阶段 1 扫描总线电机 ID；阶段 2 在 初始化位置±POS_LIMIT_DEG 之间
 *        多圈往复（速度模式连续旋转，各电机以自身初始化位置为中心）：
 *        - 方向变化时用 RAMP_MS 线性斜坡平滑加减速（消除卡顿）；
 *        - 每 POS_POLL_PERIOD_MS 用 0x06 轮询当前位置，到达端点（中心±POS_LIMIT）即反向；
 *        命令仅发往检测到的电机；电机持续在线累计满 DURATION_MS（30 天）自动停止。
 */
void srv_ht_can2_torque_test_step(void)
{
    if (!s_running)
        return;

    uint32_t now = millis();

    /* 持续在线计时：仅当所有电机在线时累加（扫描期间/掉线期间不计时），
       累计满 DURATION_MS（30 天在线时长）自动停止；掉线恢复后继续累计不重置。
       s_online_last_ms 每步都更新，保证离线/扫描后恢复时时间不跳变。 */
    if (!s_scanning && srv_ht_can2_torque_test_all_online()) {
        s_online_ms += (uint32_t)(now - s_online_last_ms);
    }
    s_online_last_ms = now;

    if (s_online_ms >= SRV_HT_CAN2_TORQUE_TEST_DURATION_MS) {
        SRV_HT_CAN2_TORQUE_TEST_LOG_I("电机持续在线累计 %lu ms 已到（30 天），耐久测试自动停止",
            (unsigned long)s_online_ms);
        srv_ht_can2_torque_test_stop();
        return;
    }

    /* 阶段 1：扫描总线电机 ID（逐地址发送握手 0x00，按应答 CAN-ID 记录） */
    if (s_scanning) {
        srv_ht_can2_torque_test_scan_log_new(); /* 打印新探测到的电机 */

        if ((now - s_scan_last_probe_ms) >= SRV_HT_CAN2_TORQUE_TEST_SCAN_PROBE_PERIOD_MS) {
            s_scan_last_probe_ms = now;
            srv_ht_can2_torque_test_send_handshake(s_scan_addr);
            s_scan_addr++;
            if (s_scan_addr > SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MAX) {
                srv_ht_can2_torque_test_scan_done();
            }
        }
        return;
    }

#if SRV_HT_CAN2_TORQUE_TEST_LOG_RAW_RX_ENABLE
    srv_ht_can2_torque_test_raw_rx_drain(); /* 调试：打印原始接收帧（确认 0x06 位置应答格式） */
#endif

    /* 周期总线重扫（热插拔支持）：只要还没有任何电机（含回退占位），每 RESCAN_PERIOD
     * 重扫握手探测 0x01~0x3F，发现真实电机即接管并使能；找到电机后自动停止不再打扰。
     * 重扫期间只发握手探测（暂停控制帧，电机速度模式锁存转速继续运行），规避 TX FIFO 突爆 */
    if (!s_rescanning && ((s_motor_cnt == 0U) || s_fallback_active) &&
        ((now - s_last_rescan_ms) >= SRV_HT_CAN2_TORQUE_TEST_RESCAN_PERIOD_MS)) {
        s_rescanning = true;
        s_rescan_addr = SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MIN;
        s_rescan_last_probe_ms = now - SRV_HT_CAN2_TORQUE_TEST_SCAN_PROBE_PERIOD_MS;
        s_rescan_start_cnt = s_motor_cnt;
        if (s_fallback_active) {
            s_motor_cnt = 0; /* 清空回退占位，由重扫重新发现真实电机 */
            s_fallback_active = false;
            s_rescan_start_cnt = 0;
            SRV_HT_CAN2_TORQUE_TEST_LOG_W("热插拔重扫：清除回退占位地址，重新探测总线");
        }
    }
    if (s_rescanning) {
        if ((now - s_rescan_last_probe_ms) >= SRV_HT_CAN2_TORQUE_TEST_SCAN_PROBE_PERIOD_MS) {
            s_rescan_last_probe_ms = now;
            srv_ht_can2_torque_test_send_handshake(s_rescan_addr);
            s_rescan_addr++;
            if (s_rescan_addr > SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MAX) {
                s_rescanning = false;
                s_last_rescan_ms = now;
                srv_ht_can2_torque_test_rescan_done(now);
            }
        }
        return; /* 重扫期间只发握手探测 */
    }

    /* 阶段 2：正常往复循环。先处理新到达的报警应答（主循环上下文，ISR 只置标志）。
     * 只在报警状态「变化」时打印：报警出现/变化/消除各打印一次，持续报警与稳态无报警静默 */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_alarm_pending[i]) {
            s_alarm_pending[i] = false; /* 先清标志再取值，避免 ISR 并发丢更新 */
            const uint32_t code = s_motor_alarm[i];
            const uint32_t prev = s_alarm_last[i];

            if (code != prev) { /* 报警状态变化 */
                if (code != 0U) {
                    srv_ht_can2_torque_test_alarm_print(s_motor_ids[i], code); /* 报警出现/变化 */
                } else {
                    SRV_HT_CAN2_TORQUE_TEST_LOG_W("电机 0x%02X 报警已消除，恢复正常", (unsigned)s_motor_ids[i]);
                }
                s_alarm_last[i] = code;
            }
            /* 报警状态未变化：不打印 */
        }
    }

#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
    /* 打印新到达的电压应答（主循环上下文，ISR 只置标志） */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_volt_pending[i]) {
            s_volt_pending[i] = false;
            srv_ht_can2_torque_test_voltage_print(s_motor_ids[i], s_motor_volt[i]);
        }
    }
#endif

    /* 控制开始 1s 内补发使能 + 速度模式 + 电流限制（首帧可能被丢弃）。
     * 交错补发：三个指令按 3×STARTUP_RETRY 周期轮流，每 tick 只发 1 帧，
     * 避免与位置轮询(0x06)/速度(0x09)/报警(0xFF)同 tick 突爆超过 FDCAN TX FIFO
     * 深度 3 导致末尾帧被静默丢弃（同良志排查文档 §2.1） */
    if (((now - s_ctrl_start_ms) < 1000U) && ((now - s_last_retry_ms) >= SRV_HT_CAN2_TORQUE_TEST_STARTUP_RETRY_MS)) {
        s_last_retry_ms = now;
        switch (s_retry_phase) {
        case 0:
            srv_ht_can2_torque_test_cmd_enable_all(true);
            break;
        case 1:
            srv_ht_can2_torque_test_cmd_set_mode_all();
            break;
        default:
            srv_ht_can2_torque_test_cmd_set_cur_limit_all();
            break;
        }
        s_retry_phase = (uint8_t)((s_retry_phase + 1U) % 3U);
    }

    /* 周期轮询电机当前位置（0x06 读取，用于端点反向判定） */
    if ((now - s_last_pos_poll_ms) >= SRV_HT_CAN2_TORQUE_TEST_POS_POLL_PERIOD_MS) {
        s_last_pos_poll_ms = now;
        srv_ht_can2_torque_test_query_position_all();
    }

    /* 锁存往复中心：各电机初始化（首次读到位置）时的位置作为自身 0 点，
       此后目标在 各自中心±POS_LIMIT 两端点间交替；盲发回退（无位置应答）保持旧行为 */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (!s_center_latched[i] && s_motor_pos_pending[i]) {
            s_center_iq[i] = s_motor_pos_iq[i];
            s_center_latched[i] = true;
            SRV_HT_CAN2_TORQUE_TEST_LOG_I("已锁存往复中心：电机 0x%02X 初始位置 %ld 为 0 点，区间 [%ld, %ld]",
                (unsigned)s_motor_ids[i], (long)s_center_iq[i],
                (long)((int64_t)s_center_iq[i] - (int64_t)SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_IQ),
                (long)((int64_t)s_center_iq[i] + (int64_t)SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_IQ));
        }
    }

    /* 端点反向判定：任一台电机到达端点（中心−POS_LIMIT / 中心+POS_LIMIT）即翻转全局方向。
     * 同时记录翻转时间，供下方 FLIP_TIMEOUT 兜底判定 */
    bool s_flipped = false;
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (!s_motor_pos_pending[i]) {
            continue;
        }
        s_motor_pos_pending[i] = false; /* 先清标志再取值，避免 ISR 并发丢更新 */

        const int64_t center = s_center_latched[i] ? (int64_t)s_center_iq[i] : (int64_t)0;
        const int64_t hi = center + (int64_t)SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_IQ;
        const int64_t lo = center - (int64_t)SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_IQ;
        if (s_dir > 0) {
            if ((int64_t)s_motor_pos_iq[i] >= (hi - (int64_t)SRV_HT_CAN2_TORQUE_TEST_REACH_IQ)) {
                s_dir = -1;
                s_flipped = true;
            }
        } else {
            if ((int64_t)s_motor_pos_iq[i] <= (lo + (int64_t)SRV_HT_CAN2_TORQUE_TEST_REACH_IQ)) {
                s_dir = 1;
                s_flipped = true;
            }
        }
    }
    if (s_flipped) {
        s_last_flip_ms = now;
    }

    /* 端点反向超时兜底：位置反馈冻结/丢帧/到位偏置导致端点判定长期不触发时强制反向，
       避免电机只往一个方向跑（同良志排查文档 §3.3 到位超时强制翻转） */
    if ((now - s_last_flip_ms) >= SRV_HT_CAN2_TORQUE_TEST_FLIP_TIMEOUT_MS) {
        s_dir = (s_dir > 0) ? -1 : 1;
        s_last_flip_ms = now;
        SRV_HT_CAN2_TORQUE_TEST_LOG_W("端点反向超时（%lu ms 未翻转），强制反向",
            (unsigned long)SRV_HT_CAN2_TORQUE_TEST_FLIP_TIMEOUT_MS);
    }

    /* 速度平滑斜坡：方向变化时在 RAMP_MS 内线性爬升/下降，消除加减速突变/卡顿 */
    if ((now - s_last_cmd_ms) >= SRV_HT_CAN2_TORQUE_TEST_CMD_PERIOD_MS) {
        s_last_cmd_ms = now;

        const int16_t target_rpm = (s_dir > 0) ? SRV_HT_CAN2_TORQUE_TEST_SPEED_RPM
                                               : -SRV_HT_CAN2_TORQUE_TEST_SPEED_RPM;
        if (target_rpm != s_ramp_target_rpm) {
            s_ramp_target_rpm = target_rpm;
            s_ramp_from_rpm = s_speed_rpm;
            s_ramp_start_ms = now;
        }

        int16_t send_rpm;
        const uint32_t elapsed = now - s_ramp_start_ms;
        if (elapsed >= SRV_HT_CAN2_TORQUE_TEST_RAMP_MS) {
            send_rpm = target_rpm; /* 斜坡完成，保持目标转速 */
        } else {
            send_rpm = (int16_t)((int32_t)s_ramp_from_rpm
                + ((int32_t)(target_rpm - s_ramp_from_rpm) * (int32_t)elapsed)
                    / (int32_t)SRV_HT_CAN2_TORQUE_TEST_RAMP_MS);
        }
        s_speed_rpm = send_rpm;
        srv_ht_can2_torque_test_cmd_speed_all(send_rpm);
    }

    /* 周期查询电机报警（1s 一次，0xFF 主动查询，应答由 on_rx 记录） */
    if ((now - s_last_alarm_ms) >= SRV_HT_CAN2_TORQUE_TEST_ALARM_PERIOD_MS) {
        s_last_alarm_ms = now;
        srv_ht_can2_torque_test_query_alarm_all();
    }

    /* 使能保持补发（修「电机晚于控制板上电，错过启动 1s 补发窗口后永久失能」）：
     * 每 ENABLE_KEEPALIVE_MS 走一个相位（每相位单电机至多发 1 帧，规避 TX FIFO 突爆）：
     *   0=查询 0x2B 使能状态；1=未使能则补发使能；2=补发速度模式；3=复查。
     *   电机确认使能后相位归 0。速度由下方斜坡逻辑持续下发，使能后即恢复往复 */
    if ((now - s_last_keepalive_ms) >= SRV_HT_CAN2_TORQUE_TEST_ENABLE_KEEPALIVE_MS) {
        s_last_keepalive_ms = now;
        for (uint32_t i = 0; i < s_motor_cnt; i++) {
            switch (s_keepalive_phase[i]) {
            case 0: /* 查询使能状态 */
                srv_ht_can2_torque_test_send_query_enable_state(s_motor_ids[i]);
                break;
            case 1: /* 已确认未使能：补发使能 */
                if (s_motor_enable_known[i] && !s_motor_enabled[i]) {
                    srv_ht_can2_torque_test_send_enable(s_motor_ids[i], true);
                    if (!s_enable_log_latch[i]) {
                        s_enable_log_latch[i] = true;
                        SRV_HT_CAN2_TORQUE_TEST_LOG_W("电机 0x%02X 未使能，已补发使能", (unsigned)s_motor_ids[i]);
                    }
                }
                break;
            case 2: /* 补发速度模式 */
                if (s_motor_enable_known[i] && !s_motor_enabled[i]) {
                    srv_ht_can2_torque_test_set_speed_mode(s_motor_ids[i]);
                    if (!s_enable_log_latch[i]) {
                        s_enable_log_latch[i] = true;
                        SRV_HT_CAN2_TORQUE_TEST_LOG_W("电机 0x%02X 未使能，已补发速度模式", (unsigned)s_motor_ids[i]);
                    }
                }
                break;
            case 3: /* 复查使能状态 */
                srv_ht_can2_torque_test_send_query_enable_state(s_motor_ids[i]);
                break;
            default:
                break;
            }
            s_keepalive_phase[i] = (uint8_t)((s_keepalive_phase[i] + 1U) % 4U);
        }
    }

#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
    /* 周期读取电机供电电压（0x87，默认 5s 一次，与位置/报警查询错开） */
    if ((now - s_last_volt_ms) >= SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_PERIOD_MS) {
        s_last_volt_ms = now;
        srv_ht_can2_torque_test_query_voltage_all();
    }
#endif

    /* 周期状态诊断日志：打印各电机 位置/方向/下发转速/使能/模式/实际转速/报警/在线年龄，
     * 定位「单向/停摆」根因（未使能、模式不对、实际不转一查便知）。
     * 每 5s tick 对单电机至多发 1 帧查询（模式/速度交替），避免叠加位置+速度帧超 TX FIFO 深度 3 */
    if ((now - s_last_status_ms) >= SRV_HT_CAN2_TORQUE_TEST_STATUS_LOG_MS) {
        s_last_status_ms = now;
        for (uint32_t i = 0; i < s_motor_cnt; i++) {
            if (s_status_q_phase) {
                srv_ht_can2_torque_test_send_query_mode(s_motor_ids[i]);
            } else {
                srv_ht_can2_torque_test_send_query_speed(s_motor_ids[i]);
            }
            const int32_t act_rpm = (int32_t)(((int64_t)s_motor_speed_iq[i] * SRV_HT_CAN2_TORQUE_TEST_SPEED_FULL_SCALE) >> 24);
            SRV_HT_CAN2_TORQUE_TEST_LOG_W("状态: 电机 0x%02X pos=%ld ctr=%ld dir=%d 下发=%dRPM 使能=%d 模式=%u 实际=%ldRPM 报警=0x%lX 在线年龄=%lu ms",
                (unsigned)s_motor_ids[i], (long)s_motor_pos_iq[i], (long)s_center_iq[i], (int)s_dir, (int)s_speed_rpm,
                (int)s_motor_enabled[i], (unsigned)s_motor_mode[i], (long)act_rpm,
                (unsigned long)s_motor_alarm[i], (unsigned long)(now - s_motor_last_seen_ms[i]));
        }
        s_status_q_phase = !s_status_q_phase; /* 模式/速度查询交替，每 tick 只发 1 帧 */
    }

    /* 电机无响应检测：超过 NORESP_PERIOD 未收到电机任何帧视为掉线/断电（每电机只告警一次） */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (!s_motor_nresp_latch[i] && ((now - s_motor_last_seen_ms[i]) >= SRV_HT_CAN2_TORQUE_TEST_NORESP_PERIOD_MS)) {
            s_motor_nresp_latch[i] = true;
            SRV_HT_CAN2_TORQUE_TEST_LOG_W("电机 0x%02X 长时间无响应（掉线或断电）", (unsigned)s_motor_ids[i]);
        }
    }

    /* 打印恢复在线事件（ISR 置位，主循环消费），与上方掉线日志成对出现。
     * 恢复时重新下发使能/速度模式；速度由斜坡逻辑在 step() 中继续下发 */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_online_evt_pending[i]) {
            s_online_evt_pending[i] = false;
            s_center_latched[i] = false; /* 重新以该电机恢复后的当前位置为往复中心 */
            srv_ht_can2_torque_test_send_enable(s_motor_ids[i], true);
            srv_ht_can2_torque_test_set_speed_mode(s_motor_ids[i]);
            srv_ht_can2_torque_test_send_cur_limit(s_motor_ids[i]);
            SRV_HT_CAN2_TORQUE_TEST_LOG_W("电机 0x%02X 恢复在线，已重新使能/设速度模式/电流限制",
                (unsigned)s_motor_ids[i]);
        }
    }
}

/**
 * @brief 处理测试协议接收帧（由 can_task 按 CH_2 分发调用）
 * @param  msg CAN 报文指针
 * @return true=测试协议帧，已消费；false=非测试帧（CAN2 专用总线，直接丢弃）
 * @note   ISR 上下文，只做数据记录与标志置位，不打日志
 */
bool srv_ht_can2_torque_test_on_rx(const drv_can_msg_t* msg)
{
    if (!msg)
        return false;

#if SRV_HT_CAN2_TORQUE_TEST_LOG_RAW_RX_ENABLE
    srv_ht_can2_torque_test_raw_rx_push(msg); /* 调试：记录原始接收帧 */
#endif

    /* 扫描阶段或热插拔重扫阶段：应答帧 CAN-ID 即电机地址，记录（越界由 record 过滤） */
    if (s_scanning || s_rescanning) {
        srv_ht_can2_torque_test_scan_record((uint8_t)(msg->id & 0xFFU));
        return true;
    }

    const uint8_t addr = (uint8_t)(msg->id & 0xFFU);

    /* 非已知电机地址的帧不属本协议（CAN2 专用总线，直接丢弃） */
    const uint32_t idx = srv_ht_can2_torque_test_find_idx(addr);
    if (idx == SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS)
        return false;

    /* 在线刷新：收到电机地址的任意帧都视为"电机在应答"。
       此前处于掉线锁存的电机收到帧即恢复在线，置标志由主循环打印（ISR 不打日志） */
    s_motor_last_seen_ms[idx] = millis();
    if (s_motor_nresp_latch[idx]) {
        s_motor_nresp_latch[idx] = false;
        s_online_evt_pending[idx] = true;
    }

    /* 位置应答：data[0]=0x06，data[1..4]=当前位置 IQ24（单位 R，大端，带符号）。
     * 用于到达判定：主循环 |位置-目标|≤阈值 即反向（ISR 不打日志） */
    if ((msg->dlc >= 5U) && (msg->data[0] == SRV_HT_CAN2_TORQUE_TEST_CMD_POS_READ)) {
        const uint32_t u = ((uint32_t)msg->data[1] << 24) | ((uint32_t)msg->data[2] << 16) | ((uint32_t)msg->data[3] << 8) | (uint32_t)msg->data[4];
        s_motor_pos_iq[idx] = (int32_t)u;
        s_motor_pos_pending[idx] = true;
        return true;
    }

    /* 报警应答：data[0]=0xFF，data[1..4]=24 位报警码（大端）。
     * 实测 0xFF 应答为 5 字节（读取指令3 的 IQ24 格式），非文档 §6.1.2 写的 3 字节。
     * 只记录状态（ISR 不打日志——log 共享格式缓冲非可重入），由主循环打印 */
    if ((msg->dlc >= 5U) && (msg->data[0] == SRV_HT_CAN2_TORQUE_TEST_CMD_ALARM)) {
        const uint32_t code = ((uint32_t)msg->data[1] << 24) | ((uint32_t)msg->data[2] << 16) | ((uint32_t)msg->data[3] << 8) | (uint32_t)msg->data[4];
        s_motor_alarm[idx] = code;
        s_alarm_pending[idx] = true;
        return true;
    }

    /* 使能状态应答：data[0]=0x2B，data[1]=0x01 使能 / 0x00 失能（读取指令1，2B） */
    if ((msg->dlc >= 2U) && (msg->data[0] == SRV_HT_CAN2_TORQUE_TEST_CMD_ENABLE_STATE)) {
        s_motor_enabled[idx] = (msg->data[1] == 0x01U);
        s_motor_enable_known[idx] = true;
        if (s_motor_enabled[idx]) {
            s_enable_log_latch[idx] = false; /* 使能成功，允许下次失能再告警 */
        }
        return true;
    }

    /* 当前模式应答：data[0]=0x55，data[1]=模式值（读取指令1，2B） */
    if ((msg->dlc >= 2U) && (msg->data[0] == SRV_HT_CAN2_TORQUE_TEST_CMD_MODE_QUERY)) {
        s_motor_mode[idx] = msg->data[1];
        return true;
    }

    /* 实际速度应答：data[0]=0x05，data[1..4]=IQ24（×6000 RPM，读取指令3，5B） */
    if ((msg->dlc >= 5U) && (msg->data[0] == SRV_HT_CAN2_TORQUE_TEST_CMD_SPEED_READ)) {
        const uint32_t u = ((uint32_t)msg->data[1] << 24) | ((uint32_t)msg->data[2] << 16) | ((uint32_t)msg->data[3] << 8) | (uint32_t)msg->data[4];
        s_motor_speed_iq[idx] = (int32_t)u;
        return true;
    }

#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
    /* 电压应答：data[0]=0x87，data[1..2]=供电电压（×100，单位 V） */
    if ((msg->dlc >= 3U) && (msg->data[0] == SRV_HT_CAN2_TORQUE_TEST_CMD_VOLTAGE)) {
        const uint16_t volt = (uint16_t)(((uint16_t)msg->data[1] << 8) | msg->data[2]);
        s_motor_volt[idx] = volt;
        s_volt_pending[idx] = true;
        return true;
    }
#endif

    /* 已知电机的其他应答帧：视为在线心跳，已刷新 last_seen，消费之 */
    return true;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 查找电机地址在检测列表中的索引
 * @param  addr 电机地址（CAN-ID 低 8 位）
 * @return 索引；未找到返回 SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS（哨兵值）
 */
static uint32_t srv_ht_can2_torque_test_find_idx(uint8_t addr)
{
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_motor_ids[i] == addr)
            return i;
    }
    return SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS;
}

/**
 * @brief 所有已检测电机是否在线（无任何电机进入无响应锁存）
 * @return true=全部在线
 * @note  DURATION 持续在线计时仅在全部在线时累加；扫描中或 s_motor_cnt==0 视为不在线
 */
static bool srv_ht_can2_torque_test_all_online(void)
{
    if (s_motor_cnt == 0U)
        return false;
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_motor_nresp_latch[i])
            return false;
    }
    return true;
}

/**
 * @brief 记录检测到的电机地址（去重，越界忽略）
 * @note  在 ISR 中调用，只做数组操作，不打日志
 */
static void srv_ht_can2_torque_test_scan_record(uint8_t addr)
{
    if ((addr < SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MIN) || (addr > SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MAX))
        return;

    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (s_motor_ids[i] == addr)
            return; /* 已记录 */
    }
    if (s_motor_cnt < SRV_HT_CAN2_TORQUE_TEST_MAX_MOTORS) {
        s_motor_ids[s_motor_cnt++] = addr;
    }
}

/**
 * @brief 打印扫描期间新检测到的电机（主循环调用，避免 ISR 打日志）
 */
static void srv_ht_can2_torque_test_scan_log_new(void)
{
    while (s_scan_log_cnt < s_motor_cnt) {
        SRV_HT_CAN2_TORQUE_TEST_LOG_I("  探测到电机：CAN ID = 0x%02X", (unsigned)s_motor_ids[s_scan_log_cnt]);
        s_scan_log_cnt++;
    }
}

/**
 * @brief 扫描结束：打印结果，对检测到的电机下发使能 + 速度模式，
 *        进入 初始化位置±POS_LIMIT_DEG 速度模式多圈往复
 */
static void srv_ht_can2_torque_test_scan_done(void)
{
    s_scanning = false;
    s_ctrl_start_ms = millis();
    s_last_retry_ms = s_ctrl_start_ms;
    s_last_pos_poll_ms = s_ctrl_start_ms;
    s_last_flip_ms = s_ctrl_start_ms;
    s_last_status_ms = s_ctrl_start_ms;
    /* 报警查询/电压读取与位置帧错开：避免同节拍多包同时发送导致丢应答 */
    s_last_alarm_ms = s_ctrl_start_ms - SRV_HT_CAN2_TORQUE_TEST_ALARM_OFFSET_MS;
    /* 使能保持补发同样错开：起始偏移 11ms（非 20/100ms 整数倍对齐），
       与位置/速度/报警帧分时，保证 keepalive tick 帧数不超 TX FIFO 深度 3 */
    s_last_keepalive_ms = s_ctrl_start_ms + SRV_HT_CAN2_TORQUE_TEST_ENABLE_KEEPALIVE_OFFSET_MS - SRV_HT_CAN2_TORQUE_TEST_ENABLE_KEEPALIVE_MS;
#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
    s_last_volt_ms = s_ctrl_start_ms - SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_OFFSET_MS;
#endif

    /* 未扫描到电机：回退到默认设备地址继续测试（总线可能因电机未应答握手而探测失败）；
     * 置 s_fallback_active 标记：热插拔重扫前若仍无应答则清空占位，由重扫重新发现真实电机 */
    if (s_motor_cnt == 0U) {
        SRV_HT_CAN2_TORQUE_TEST_LOG_W("扫描完成：未检测到电机（0x%02X~0x%02X），回退到默认 CAN ID 0x%02X",
            SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MIN, SRV_HT_CAN2_TORQUE_TEST_SCAN_ADDR_MAX, SRV_HT_CAN2_TORQUE_TEST_DEFAULT_MOTOR_ADDR);
        s_motor_ids[0] = SRV_HT_CAN2_TORQUE_TEST_DEFAULT_MOTOR_ADDR;
        s_motor_cnt = 1;
        s_fallback_active = true;
    } else {
        SRV_HT_CAN2_TORQUE_TEST_LOG_I("扫描完成：检测到 %u 台电机", (unsigned)s_motor_cnt);
        s_fallback_active = false;
    }

    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        SRV_HT_CAN2_TORQUE_TEST_LOG_I("  电机[%u] CAN ID = 0x%02X", (unsigned)i, (unsigned)s_motor_ids[i]);
    }

    /* 初始化电机"最后收到帧"时间，作为掉线检测的起点 */
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        s_motor_last_seen_ms[i] = s_ctrl_start_ms;
    }

    srv_ht_can2_torque_test_cmd_enable_all(true); /* 1. 使能 */
    srv_ht_can2_torque_test_cmd_set_mode_all(); /* 2. 速度模式 */
    srv_ht_can2_torque_test_cmd_set_cur_limit_all(); /* 3. 提高电流限制（扭矩输出上限，速度模式由 0x58 决定） */
    /* 4. 速度由斜坡逻辑在 step() 中下发（初始方向 +50R，斜坡从 0 平滑爬升） */
    SRV_HT_CAN2_TORQUE_TEST_LOG_I("开始多圈往复：初始化中心 ±%d 圈（±%d°），速度模式连续旋转，累计在线 %lu ms（30 天）后停止",
        (int)(SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_DEG / 360), (int)SRV_HT_CAN2_TORQUE_TEST_POS_LIMIT_DEG,
        (unsigned long)SRV_HT_CAN2_TORQUE_TEST_DURATION_MS);
}

/**
 * @brief 热插拔重扫结束：对新发现的电机初始化在线时间并补发使能+速度模式
 * @param now 当前时间 (millis)
 * @note  仅在启动扫描未发现电机（回退占位）后按 RESCAN_PERIOD 触发；
 *        新电机从索引 s_rescan_start_cnt 开始，逐个接管使能。速度由斜坡逻辑持续下发
 */
static void srv_ht_can2_torque_test_rescan_done(uint32_t now)
{
    if (s_motor_cnt > s_rescan_start_cnt) {
        SRV_HT_CAN2_TORQUE_TEST_LOG_I("热插拔重扫完成：检测到 %u 台电机", (unsigned)(s_motor_cnt - s_rescan_start_cnt));
    }

    for (uint8_t i = s_rescan_start_cnt; i < s_motor_cnt; i++) {
        s_motor_last_seen_ms[i] = now;
        s_motor_enable_known[i] = false;
        s_motor_enabled[i] = false;
        s_enable_log_latch[i] = false;
        s_center_latched[i] = false; /* 新电机：以接管后读回的位置为往复中心 */
        srv_ht_can2_torque_test_send_enable(s_motor_ids[i], true);
        srv_ht_can2_torque_test_set_speed_mode(s_motor_ids[i]);
        srv_ht_can2_torque_test_send_cur_limit(s_motor_ids[i]);
        SRV_HT_CAN2_TORQUE_TEST_LOG_W("热插拔：接管电机 0x%02X，已使能+速度模式+电流限制", (unsigned)s_motor_ids[i]);
    }

    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        SRV_HT_CAN2_TORQUE_TEST_LOG_I("  电机[%u] CAN ID = 0x%02X", (unsigned)i, (unsigned)s_motor_ids[i]);
    }
}

/**
 * @brief 发送握手探测帧 (0x00, 经典 CAN 1B)
 * @param addr 待探测的电机地址（CAN-ID）
 */
static void srv_ht_can2_torque_test_send_handshake(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false, /* 苇熠协议：经典 CAN 2.0A，1 Mbps */
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_HANDSHAKE;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 发送使能/失能帧 (经典 CAN 2B)
 * @param addr   电机地址（CAN-ID）
 * @param enable true=使能 (0x2A 01)，false=失能 (0x2A 00)
 */
static void srv_ht_can2_torque_test_send_enable(uint8_t addr, bool enable)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 2,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_ENABLE;
    tx.data[1] = enable ? 0x01U : 0x00U;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 发送设置速度模式帧 (0x07 02, 经典 CAN 2B)
 * @param addr 电机地址（CAN-ID）
 */
static void srv_ht_can2_torque_test_set_speed_mode(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 2,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_MODE;
    tx.data[1] = SRV_HT_CAN2_TORQUE_TEST_MODE_SPEED;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 发送电流限制设置帧 (0x58 + IQ24, 经典 CAN 5B)
 * @param addr 电机地址（CAN-ID）
 * @note  IQ24 = 限制电流 / 满量程电流 × 2^24（归一化，满量程型号相关，如 45A）；
 *        速度模式扭矩输出上限由此决定，随使能/速度模式一起下发，拉满扭矩
 */
static void srv_ht_can2_torque_test_send_cur_limit(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 5,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_CUR_LIMIT;
    tx.data[1] = (uint8_t)(SRV_HT_CAN2_TORQUE_TEST_CUR_LIMIT_IQ >> 24);
    tx.data[2] = (uint8_t)(SRV_HT_CAN2_TORQUE_TEST_CUR_LIMIT_IQ >> 16);
    tx.data[3] = (uint8_t)(SRV_HT_CAN2_TORQUE_TEST_CUR_LIMIT_IQ >> 8);
    tx.data[4] = (uint8_t)(SRV_HT_CAN2_TORQUE_TEST_CUR_LIMIT_IQ);
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 发送速度设定帧 (0x09 + IQ24, 经典 CAN 5B)
 * @param addr 电机地址（CAN-ID）
 * @param rpm  目标转速 (RPM)，正=正转，负=反转，0=停止（不失能）
 * @note  IQ24 = rpm / 速度满量程(6000) × 2^24，4 字节大端（高字节在前）
 */
static void srv_ht_can2_torque_test_send_speed(uint8_t addr, int16_t rpm)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    const int32_t iq = (int32_t)(((int64_t)rpm << 24) / SRV_HT_CAN2_TORQUE_TEST_SPEED_FULL_SCALE);

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 5,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_SPEED;
    tx.data[1] = (uint8_t)((uint32_t)iq >> 24);
    tx.data[2] = (uint8_t)((uint32_t)iq >> 16);
    tx.data[3] = (uint8_t)((uint32_t)iq >> 8);
    tx.data[4] = (uint8_t)(uint32_t)iq;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 发送位置读取帧 (0x06, 经典 CAN 1B)
 * @param addr 电机地址（CAN-ID）
 * @note  电机返回 [0x06][4B 大端 IQ24]（5B，单位 R），由 on_rx 记录，主循环用于端点反向判定
 */
static void srv_ht_can2_torque_test_send_query_position(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_POS_READ;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/* --- 批量下发（对全部检测到的电机） --- */

static void srv_ht_can2_torque_test_cmd_enable_all(bool enable)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_can2_torque_test_send_enable(s_motor_ids[i], enable);
    }
}

static void srv_ht_can2_torque_test_cmd_set_mode_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_can2_torque_test_set_speed_mode(s_motor_ids[i]);
    }
}

static void srv_ht_can2_torque_test_cmd_set_cur_limit_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_can2_torque_test_send_cur_limit(s_motor_ids[i]);
    }
}

static void srv_ht_can2_torque_test_cmd_speed_all(int16_t rpm)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_can2_torque_test_send_speed(s_motor_ids[i], rpm);
    }
}

static void srv_ht_can2_torque_test_query_position_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_can2_torque_test_send_query_position(s_motor_ids[i]);
    }
}

/**
 * @brief 发送报警查询帧 (0xFF, 经典 CAN 1B)
 * @param addr 电机地址（CAN-ID）
 * @note  电机返回 [0xFF][4B 大端报警码]（5B），由 on_rx 记录
 */
static void srv_ht_can2_torque_test_send_query_alarm(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_ALARM;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 发送使能状态查询帧 (0x2B, 经典 CAN 1B)
 * @param addr 电机地址（CAN-ID）
 * @note  电机返回 [0x2B][0x01/0x00]（2B），由 on_rx 记录用于使能保持补发判定
 */
static void srv_ht_can2_torque_test_send_query_enable_state(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_ENABLE_STATE;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 发送当前模式查询帧 (0x55, 经典 CAN 1B)
 * @param addr 电机地址（CAN-ID）
 * @note  电机返回 [0x55][模式值]（2B，读取指令1），由 on_rx 记录
 */
static void srv_ht_can2_torque_test_send_query_mode(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_MODE_QUERY;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 发送实际速度读取帧 (0x05, 经典 CAN 1B)
 * @param addr 电机地址（CAN-ID）
 * @note  电机返回 [0x05][4B 大端 IQ24]（5B，×6000 RPM），由 on_rx 记录
 */
static void srv_ht_can2_torque_test_send_query_speed(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_SPEED_READ;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 对所有检测到的电机发送报警查询帧
 */
static void srv_ht_can2_torque_test_query_alarm_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_can2_torque_test_send_query_alarm(s_motor_ids[i]);
    }
}

/**
 * @brief 打印报警详情（主循环上下文调用）
 * @note  报警码可组合（多位同时置位）；WARN 打印原始码并逐位解码
 */
static void srv_ht_can2_torque_test_alarm_print(uint8_t addr, uint32_t code)
{
    if (code == 0U) {
        SRV_HT_CAN2_TORQUE_TEST_LOG_I("电机 0x%02X 报警查询：无报警", (unsigned)addr);
        return;
    }

    SRV_HT_CAN2_TORQUE_TEST_LOG_W("电机 0x%02X 报警：0x%08X", (unsigned)addr, (unsigned)code);
    for (uint32_t i = 0; i < SRV_HT_CAN2_TORQUE_TEST_ALARM_NUM; i++) {
        if ((code & s_alarm_map[i].mask) != 0U) {
            SRV_HT_CAN2_TORQUE_TEST_LOG_W("  - %s", s_alarm_map[i].name);
        }
    }
}

#if SRV_HT_CAN2_TORQUE_TEST_VOLTAGE_QUERY_ENABLE
/**
 * @brief 发送电压读取帧 (0x87, 经典 CAN 1B)
 * @param addr 电机地址（CAN-ID）
 * @note  电机返回 [0x87][V高][V低]（3B，×100 单位 V），由 on_rx 记录
 */
static void srv_ht_can2_torque_test_send_query_voltage(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_2))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_CAN2_TORQUE_TEST_CMD_VOLTAGE;
    drv_can_send(DRV_CAN_CH_2, &tx);
}

/**
 * @brief 对所有检测到的电机发送电压读取帧
 */
static void srv_ht_can2_torque_test_query_voltage_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_can2_torque_test_send_query_voltage(s_motor_ids[i]);
    }
}

/**
 * @brief 打印电压读取结果（主循环上下文调用）
 * @param volt 电压值 ×100（如 2400 = 24.00V）
 */
static void srv_ht_can2_torque_test_voltage_print(uint8_t addr, uint16_t volt)
{
    SRV_HT_CAN2_TORQUE_TEST_LOG_I("电机 0x%02X 供电电压：%u.%02u V",
        (unsigned)addr, (unsigned)(volt / 100U), (unsigned)(volt % 100U));
}
#endif

#if SRV_HT_CAN2_TORQUE_TEST_LOG_RAW_RX_ENABLE
/**
 * @brief 记录收到的原始 CAN 帧到环形缓冲（ISR 中调用，不打日志）
 */
static void srv_ht_can2_torque_test_raw_rx_push(const drv_can_msg_t* msg)
{
    uint8_t head = s_raw_rx_head;
    srv_ht_can2_torque_test_raw_rx_t* slot = &s_raw_rx[head];
    slot->id = msg->id;
    slot->dlc = msg->dlc;
    memset(slot->data, 0, sizeof(slot->data));
    uint8_t n = (msg->dlc < 8U) ? msg->dlc : 8U;
    memcpy(slot->data, msg->data, n);

    head = (uint8_t)((head + 1U) % SRV_HT_CAN2_TORQUE_TEST_RAW_RX_DEPTH);
    if (head == s_raw_rx_tail) {
        /* 环形满：丢弃最旧，保留最新 */
        s_raw_rx_tail = (uint8_t)((s_raw_rx_tail + 1U) % SRV_HT_CAN2_TORQUE_TEST_RAW_RX_DEPTH);
    }
    s_raw_rx_head = head;
}

/**
 * @brief 打印环形缓冲中的原始接收帧（主循环上下文调用）
 * @note  诊断用：确认 0x06 位置应答真实格式（协议文档写 5B，实测可能不同，
 *        与 0xFF 报警「文档 3B、实测 5B」同类）；格式确认后可置 0 关闭
 */
static void srv_ht_can2_torque_test_raw_rx_drain(void)
{
    while (s_raw_rx_tail != s_raw_rx_head) {
        const srv_ht_can2_torque_test_raw_rx_t* slot = &s_raw_rx[s_raw_rx_tail];
        s_raw_rx_tail = (uint8_t)((s_raw_rx_tail + 1U) % SRV_HT_CAN2_TORQUE_TEST_RAW_RX_DEPTH);
        SRV_HT_CAN2_TORQUE_TEST_LOG_W("RX id=0x%03lX dlc=%u %02X %02X %02X %02X %02X %02X %02X %02X",
            (unsigned long)slot->id, (unsigned)slot->dlc,
            slot->data[0], slot->data[1], slot->data[2], slot->data[3],
            slot->data[4], slot->data[5], slot->data[6], slot->data[7]);
    }
}
#endif
