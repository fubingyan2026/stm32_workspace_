/**
 * @file    srv_ht_temp_test.c
 * @brief   苇熠(HT) 伺服执行器 CAN 控制协议服务实现 — 电机测试模式
 *
 * 电机控制指令按 docs/苇熠电机can协议文档.md 组帧发送（经典 CAN 2.0A, 1 Mbps,
 * CAN-ID 低 8 位 = 设备地址，data[0]=指令，data[1..]=参数，多字节大端）。
 *
 * 启动流程：先扫描总线电机 ID（握手指令 0x00 逐地址探测，0x01~0x3F），
 * 对检测到的电机下发使能/模式/速度控制命令；未检测到任何电机时回退到
 * 默认设备地址（SRV_HT_TEMP_TEST_DEFAULT_MOTOR_ADDR）继续控制。
 *
 * RX 侧：srv_ht_temp_test_on_rx() 只消费测试协议帧（扫描应答/报警/电压/在线心跳），
 * 返回 true 表示已处理；旧 CAN FD 上位机控制帧（0x100）由 srv_can_on_rx() 解析。
 */

#include "srv_ht_temp_test.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/
/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define  SRV_HT_TEMP_LOG_ENABLE 1

#if SRV_HT_TEMP_LOG_ENABLE
#define SRV_HT_TEMP_TEST_LOG_I(...) ((void)0)//LOG_I("srv_ht_temp_test", __VA_ARGS__)
#define SRV_HT_TEMP_TEST_LOG_W(...) LOG_W("srv_ht_temp_test", __VA_ARGS__)
#define SRV_HT_TEMP_TEST_LOG_E(...) LOG_E("srv_ht_temp_test", __VA_ARGS__)
#else
#define SRV_HT_TEMP_TEST_LOG_I(...) ((void)0)
#define SRV_HT_TEMP_TEST_LOG_W(...) ((void)0)
#define SRV_HT_TEMP_TEST_LOG_E(...) ((void)0)
#endif
/* 模块测试开关 ----------------------------------------------------------------*/

/** @brief 电机测试模式：1=上电自动启动（扫描 + 正转/停留/反转/停留 循环）；0=需手动调用 srv_ht_temp_test_start() */
#define SRV_HT_TEMP_TEST_AUTO_START 1

/* Private constants ---------------------------------------------------------*/

/* --- 苇熠伺服执行器 CAN 测试参数 --- */

/** @brief 正转/反转转速 (RPM)。速度满量程 6000 RPM，IQ24 归一化 */
#define SRV_HT_TEMP_TEST_SPEED_RPM 500
/**
 * @brief 缓启动斜坡时长 (ms)：无论目标转速大小，都在该时长内线性到达目标
 * @note  应 ≤ 阶段时长/2（停留阶段才能真正停下来）。固定步长方案在目标大、
 *        命令周期短时无法在阶段内爬满（如 100RPM/s 到 2000RPM 需 20s > 阶段 6s），
 *        导致电机始终在斜坡中途、没有停留时间。按时间斜坡自适应目标大小。
 */
#define SRV_HT_TEMP_TEST_RAMP_TIME_MS 2000U
/** @brief 命令帧发送周期 (ms) */
#define SRV_HT_TEMP_TEST_CMD_PERIOD_MS 100U
/** @brief 报警查询周期 (ms)：电机报警需主动查询（0xFF），1s 一次 */
#define SRV_HT_TEMP_TEST_ALARM_PERIOD_MS 1000U
/** @brief 报警查询相对控制帧的时间偏移 (ms)：错开与速度/控制帧同时发送，避免多包冲突丢应答 */
#define SRV_HT_TEMP_TEST_ALARM_OFFSET_MS (SRV_HT_TEMP_TEST_ALARM_PERIOD_MS / 2U)
/** @brief 电机无响应判定周期 (ms)：超过该时长未收到电机任何帧视为掉线/断电 */
#define SRV_HT_TEMP_TEST_NORESP_PERIOD_MS 3000U

/** @brief 1=周期读取供电电压（0x87）；0=关闭。实测 0x87 仅返回 [0x87][状态]（写命令应答），不含电压数据 */
#define SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE 1
#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
/** @brief 供电电压读取周期 (ms)：0x87 读取（×100，单位 V，仅使能时刷新） */
#define SRV_HT_TEMP_TEST_VOLTAGE_PERIOD_MS 5000U
/** @brief 电压读取相对控制帧的时间偏移 (ms)：错开与速度/报警查询同时发送 */
#define SRV_HT_TEMP_TEST_VOLTAGE_OFFSET_MS 250U
#endif

/* --- 原始接收帧调试（临时诊断用，定位电机应答格式） --- */

/** @brief 1=记录并打印所有收到的 CAN 帧（确认电机是否有应答/应答格式）；0=关闭 */
#define SRV_HT_TEMP_TEST_LOG_RAW_RX_ENABLE 1
/** @brief 原始帧环形缓冲深度 */
#define SRV_HT_TEMP_TEST_RAW_RX_DEPTH 16U
/** @brief 每个阶段时长 (ms)：正转/停留/反转/停留 */
#define SRV_HT_TEMP_TEST_PHASE_MS 6000U
/** @brief 持续在线时长上限 (ms)：电机一直在线累计满 24h 自动停止，掉线期间不计时 */
#define SRV_HT_TEMP_TEST_DURATION_MS 86400000U
/** @brief 扫描完成后 1s 内补发使能/模式周期 (ms)：首帧可能因 TX FIFO 未就绪被丢弃 */
#define SRV_HT_TEMP_TEST_STARTUP_RETRY_MS 100U
/** @brief 1=使能后/进入旋转阶段时打开抱闸 (0xF4 0100)；0=不处理（无抱闸电机） */
#define SRV_HT_TEMP_TEST_RELEASE_BRAKE 0

/* --- 总线电机扫描参数 --- */

/** @brief 扫描地址范围（含）：协议推荐 0x01~0x3F，0x00 为广播且无返回 */
#define SRV_HT_TEMP_TEST_SCAN_ADDR_MIN 0x01U
#define SRV_HT_TEMP_TEST_SCAN_ADDR_MAX 0x3FU
/** @brief 每地址探测间隔 (ms)：留出电机握手应答时间 */
#define SRV_HT_TEMP_TEST_SCAN_PROBE_PERIOD_MS 20U
/** @brief 最多可同时控制的电机数 */
#define SRV_HT_TEMP_TEST_MAX_MOTORS 63U
/** @brief 未扫描到电机时使用的默认设备地址（CAN-ID 低 8 位） */
#define SRV_HT_TEMP_TEST_DEFAULT_MOTOR_ADDR 0x21U

/* --- 苇熠协议指令符（docs/苇熠电机can协议文档.md §6） --- */

#define SRV_HT_TEMP_TEST_CMD_HANDSHAKE 0x00U /**< 握手：发送 1 字节，电机返回 [0x00][状态] */
#define SRV_HT_TEMP_TEST_CMD_ENABLE 0x2AU /**< 使能/失能：参数 0x01 使能，0x00 失能 */
#define SRV_HT_TEMP_TEST_CMD_MODE 0x07U   /**< 设置模式 */
#define SRV_HT_TEMP_TEST_CMD_SPEED 0x09U  /**< 设置速度值：IQ24，归一化值 × 6000 */
#define SRV_HT_TEMP_TEST_CMD_BRAKE 0xF4U  /**< 抱闸操作：参数 0x0100 打开，0x0000 锁定 */
#define SRV_HT_TEMP_TEST_CMD_ALARM 0xFFU  /**< 查询报警：发送 1 字节，实测返回 [0xFF][4B 大端报警码]（5B） */
#define SRV_HT_TEMP_TEST_CMD_VOLTAGE 0x87U /**< 读取供电电压：返回 [0x87][V高][V低]，×100 单位 V（仅使能时刷新） */
#define SRV_HT_TEMP_TEST_MODE_SPEED 0x02U /**< 速度模式 */
#define SRV_HT_TEMP_TEST_SPEED_FULL_SCALE 6000 /**< 速度满量程 (RPM) */

/* Private types -------------------------------------------------------------*/

/** @brief 测试循环阶段：正转 → 停留 → 反转 → 停留，来回一个循环 */
typedef enum {
    SRV_HT_TEMP_TEST_PHASE_FORWARD = 0, /**< 正转 */
    SRV_HT_TEMP_TEST_PHASE_DWELL_F, /**< 正转后停留 */
    SRV_HT_TEMP_TEST_PHASE_REVERSE, /**< 反转 */
    SRV_HT_TEMP_TEST_PHASE_DWELL_R, /**< 反转后停留 */
    SRV_HT_TEMP_TEST_PHASE_NUM, /**< 阶段总数 */
} srv_ht_temp_test_phase_t;

/** @brief 报警码描述表项（见 docs/苇熠电机can协议文档.md §8，含 24 位错误码） */
typedef struct {
    uint32_t mask;      /**< 错误码位 */
    const char* name;   /**< 含义 */
} srv_ht_temp_test_alarm_desc_t;

/** @brief 报警码 → 含义映射表（协议文档 §8；0xFF 应答为 [0xFF][4B 大端]，支持全部 24 位码） */
static const srv_ht_temp_test_alarm_desc_t s_alarm_map[] = {
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
#define SRV_HT_TEMP_TEST_ALARM_NUM (sizeof(s_alarm_map) / sizeof(s_alarm_map[0]))

/** @brief 原始接收帧记录（调试用，ISR 写入主循环打印） */
typedef struct {
    uint32_t id;      /**< CAN ID */
    uint8_t  dlc;     /**< 数据长度 */
    uint8_t  data[8]; /**< 前 8 字节 */
} srv_ht_temp_test_raw_rx_t;

/* Private variables ---------------------------------------------------------*/

/** @brief 测试模式运行标志 */
static bool s_running;

/** @brief 总线扫描进行中标志（true=只发握手探测，不发控制命令） */
static bool s_scanning;

/** @brief 扫描当前探测地址 */
static uint8_t s_scan_addr;

/** @brief 扫描阶段上次探测时间 (millis) */
static uint32_t s_scan_last_probe_ms;

/** @brief 已检测到的电机 CAN-ID 列表（握手 0x00 应答） */
static uint8_t s_motor_ids[SRV_HT_TEMP_TEST_MAX_MOTORS];

/** @brief 已检测电机数量（ISR 写入，主循环读取） */
static uint8_t s_motor_cnt;

/** @brief 已打印日志的电机数（避免重复打印） */
static uint8_t s_scan_log_cnt;

/** @brief 测试当前阶段 */
static srv_ht_temp_test_phase_t s_phase;

/** @brief 当前阶段起始时间 (millis) */
static uint32_t s_phase_start_ms;

/** @brief 控制阶段起始时间 (millis)，扫描完成后置位 */
static uint32_t s_ctrl_start_ms;

/** @brief 上次命令帧发送时间 (millis) */
static uint32_t s_last_cmd_ms;

/** @brief 控制阶段上次补发时间 (millis) */
static uint32_t s_last_retry_ms;

/** @brief 测试起始时间 (millis)，用于 24h 自动停止 */
static uint32_t s_start_ms;

/** @brief 累计在线时长 (ms)：仅电机持续在线时累加，用于 DURATION 自动停止 */
static uint32_t s_online_ms;

/** @brief 上次在线时长累计时间点 (millis) */
static uint32_t s_online_last_ms;

/** @brief 当前实际下发转速 (RPM)，缓启动斜坡插值结果 */
static int16_t s_speed_rpm;

/** @brief 当前斜坡段目标转速 (RPM)：目标变化时重新开始计时 */
static int16_t s_ramp_target_rpm;

/** @brief 当前斜坡段起点转速 (RPM)（目标变化瞬间的下发转速） */
static int16_t s_ramp_from_rpm;

/** @brief 当前斜坡段起始时间 (millis) */
static uint32_t s_ramp_start_ms;

/** @brief 每电机最新报警码（0xFF 应答更新，24 位，ISR 写） */
static uint32_t s_motor_alarm[SRV_HT_TEMP_TEST_MAX_MOTORS];

/** @brief 新报警应答待打印标志（ISR 置位，主循环清零后打印） */
static bool s_alarm_pending[SRV_HT_TEMP_TEST_MAX_MOTORS];

/** @brief 上一轮已处理的报警码（主循环维护，用于状态变化检测） */
static uint32_t s_alarm_last[SRV_HT_TEMP_TEST_MAX_MOTORS];

/** @brief 每电机最后收到任何帧的时间 (millis)，用于掉线检测（ISR 更新） */
static uint32_t s_motor_last_seen_ms[SRV_HT_TEMP_TEST_MAX_MOTORS];

/** @brief 电机无响应告警锁存（收到任何帧后清除，避免重复刷屏） */
static bool s_motor_nresp_latch[SRV_HT_TEMP_TEST_MAX_MOTORS];

/** @brief 恢复在线事件待打印标志（ISR 置位，主循环清零打印） */
static bool s_online_evt_pending[SRV_HT_TEMP_TEST_MAX_MOTORS];

#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
/** @brief 每电机最新供电电压（0x87 应答更新，×100 单位 V，ISR 写） */
static uint16_t s_motor_volt[SRV_HT_TEMP_TEST_MAX_MOTORS];

/** @brief 新电压应答待打印标志（ISR 置位，主循环清零） */
static bool s_volt_pending[SRV_HT_TEMP_TEST_MAX_MOTORS];

/** @brief 上次电压读取时间 (millis) */
static uint32_t s_last_volt_ms;
#endif

#if SRV_HT_TEMP_TEST_LOG_RAW_RX_ENABLE
/** @brief 原始接收帧环形缓冲（ISR 写 head，主循环读 tail） */
static srv_ht_temp_test_raw_rx_t s_raw_rx[SRV_HT_TEMP_TEST_RAW_RX_DEPTH];
static volatile uint8_t s_raw_rx_head;
static volatile uint8_t s_raw_rx_tail;
#endif

/** @brief 上次报警查询时间 (millis) */
static uint32_t s_last_alarm_ms;

/* Private function prototypes -----------------------------------------------*/

static uint32_t srv_ht_temp_test_find_idx(uint8_t addr);
static bool srv_ht_temp_test_all_online(void);
static void srv_ht_temp_test_scan_record(uint8_t addr);
static void srv_ht_temp_test_scan_log_new(void);
static void srv_ht_temp_test_scan_done(void);
static void srv_ht_temp_test_send_handshake(uint8_t addr);
static void srv_ht_temp_test_send_enable(uint8_t addr, bool enable);
static void srv_ht_temp_test_set_speed_mode(uint8_t addr);
static void srv_ht_temp_test_send_speed(uint8_t addr, int16_t rpm);
static void srv_ht_temp_test_cmd_enable_all(bool enable);
static void srv_ht_temp_test_cmd_set_mode_all(void);
static void srv_ht_temp_test_cmd_speed_all(int16_t rpm);
static void srv_ht_temp_test_send_query_alarm(uint8_t addr);
static void srv_ht_temp_test_query_alarm_all(void);
static void srv_ht_temp_test_alarm_print(uint8_t addr, uint32_t code);
#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
static void srv_ht_temp_test_send_query_voltage(uint8_t addr);
static void srv_ht_temp_test_query_voltage_all(void);
static void srv_ht_temp_test_voltage_print(uint8_t addr, uint16_t volt);
#endif
static void srv_ht_temp_test_log_phase(srv_ht_temp_test_phase_t phase);

#if SRV_HT_TEMP_TEST_LOG_RAW_RX_ENABLE
static void srv_ht_temp_test_raw_rx_push(const drv_can_msg_t* msg);
static void srv_ht_temp_test_raw_rx_drain(void);
#endif

#if SRV_HT_TEMP_TEST_RELEASE_BRAKE
static void srv_ht_temp_test_send_release_brake(uint8_t addr);
static void srv_ht_temp_test_cmd_brake_all(void);
#endif

/* Exported functions --------------------------------------------------------*/

void srv_ht_temp_test_init(void)
{
#if SRV_HT_TEMP_TEST_AUTO_START
    srv_ht_temp_test_start(); /* 测试模式：先扫描总线电机 ID，再下发控制命令 */
#endif
}

/**
 * @brief 启动电机测试模式
 * @note  先扫描总线电机 ID（握手 0x00，探测 0x01~0x3F），扫描完成后
 *        对检测到的电机发送：使能 (0x2A 01) → 速度模式 (0x07 02) → 正转速度 (0x09)
 */
void srv_ht_temp_test_start(void)
{
    s_running = true;
    s_start_ms = millis(); /* 24h 自动停止计时起点 */
    s_online_ms = 0; /* 持续在线时长从 0 累计 */
    s_online_last_ms = s_start_ms;
    s_speed_rpm = 0; /* 缓启动从 0 开始 */
    s_ramp_target_rpm = 0;
    s_ramp_from_rpm = 0;
    s_ramp_start_ms = s_start_ms;

    /* 阶段 1：扫描总线电机 ID */
    s_scanning = true;
    s_scan_addr = SRV_HT_TEMP_TEST_SCAN_ADDR_MIN;
    s_motor_cnt = 0;
    s_scan_log_cnt = 0;
    s_scan_last_probe_ms = s_start_ms - SRV_HT_TEMP_TEST_SCAN_PROBE_PERIOD_MS;
    memset(s_motor_alarm, 0, sizeof(s_motor_alarm));
    memset(s_alarm_pending, 0, sizeof(s_alarm_pending));
    memset(s_alarm_last, 0, sizeof(s_alarm_last));
    memset(s_motor_last_seen_ms, 0, sizeof(s_motor_last_seen_ms));
    memset(s_motor_nresp_latch, 0, sizeof(s_motor_nresp_latch));
    memset(s_online_evt_pending, 0, sizeof(s_online_evt_pending));
#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
    memset(s_motor_volt, 0, sizeof(s_motor_volt));
    memset(s_volt_pending, 0, sizeof(s_volt_pending));
#endif

    SRV_HT_TEMP_TEST_LOG_I("测试启动：正在扫描总线电机 ID 0x%02X~0x%02X（每 %u ms 探测一个地址）",
        SRV_HT_TEMP_TEST_SCAN_ADDR_MIN, SRV_HT_TEMP_TEST_SCAN_ADDR_MAX,
        (unsigned)SRV_HT_TEMP_TEST_SCAN_PROBE_PERIOD_MS);
}

/**
 * @brief 停止电机测试模式并发送速度 0 帧 + 失能
 * @note  断电前必须失能 (0x2A 00)，否则零位可能丢失；命令发往所有检测到的电机
 */
void srv_ht_temp_test_stop(void)
{
    s_running = false;
    s_scanning = false;
    s_speed_rpm = 0;               /* 重置斜坡，下次启动从 0 开始 */
    s_ramp_target_rpm = 0;
    s_ramp_from_rpm = 0;
    s_ramp_start_ms = 0;
    srv_ht_temp_test_cmd_speed_all(0);      /* 速度 0 停下 */
    srv_ht_temp_test_cmd_enable_all(false); /* 失能 */
    SRV_HT_TEMP_TEST_LOG_I("测试停止：已向 %u 台电机发送速度 0 + 失能", (unsigned)s_motor_cnt);
}

/**
 * @brief 测试模式周期步进（由 can_task 每 10ms 调用）
 * @note  阶段 1 扫描总线电机 ID；阶段 2 循环：正转 → 停留(速度 0) → 反转 → 停留(速度 0)，
 *        命令仅发往检测到的电机；电机持续在线累计满 DURATION_MS 自动停止
 */
void srv_ht_temp_test_step(void)
{
    if (!s_running)
        return;

    uint32_t now = millis();

    /* 持续在线计时：仅当所有电机在线时累加（扫描期间/掉线期间不计时），
       累计满 DURATION_MS（24h 在线时长）自动停止；掉线恢复后继续累计不重置。
       s_online_last_ms 每步都更新，保证离线/扫描后恢复时时间不跳变。 */
    if (!s_scanning && srv_ht_temp_test_all_online()) {
        s_online_ms += (uint32_t)(now - s_online_last_ms);
    }
    s_online_last_ms = now;

    if (s_online_ms >= SRV_HT_TEMP_TEST_DURATION_MS) {
        SRV_HT_TEMP_TEST_LOG_I("电机持续在线累计 %lu ms 已到，测试自动停止",
            (unsigned long)s_online_ms);
        srv_ht_temp_test_stop();
        return;
    }

    /* 阶段 1：扫描总线电机 ID（逐地址发送握手 0x00，按应答 CAN-ID 记录） */
    if (s_scanning) {
        srv_ht_temp_test_scan_log_new(); /* 打印新探测到的电机 */

        if ((now - s_scan_last_probe_ms) >= SRV_HT_TEMP_TEST_SCAN_PROBE_PERIOD_MS) {
            s_scan_last_probe_ms = now;
            srv_ht_temp_test_send_handshake(s_scan_addr);
            s_scan_addr++;
            if (s_scan_addr > SRV_HT_TEMP_TEST_SCAN_ADDR_MAX) {
                srv_ht_temp_test_scan_done();
            }
        }
        return;
    }

#if SRV_HT_TEMP_TEST_LOG_RAW_RX_ENABLE
    srv_ht_temp_test_raw_rx_drain(); /* 调试：打印原始接收帧 */
#endif

    /* 阶段 2：正常循环。先处理新到达的报警应答（主循环上下文，ISR 只置标志）。
     * 只在报警状态「变化」时打印：报警出现/变化/消除各打印一次，持续报警与稳态无报警静默 */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_alarm_pending[i]) {
            s_alarm_pending[i] = false; /* 先清标志再取值，避免 ISR 并发丢更新 */
            const uint32_t code = s_motor_alarm[i];
            const uint32_t prev = s_alarm_last[i];

            if (code != prev) { /* 报警状态变化 */
                if (code != 0U) {
                    srv_ht_temp_test_alarm_print(s_motor_ids[i], code); /* 报警出现/变化 */
                } else {
                    SRV_HT_TEMP_TEST_LOG_W("电机 0x%02X 报警已消除，恢复正常", (unsigned)s_motor_ids[i]);
                }
                s_alarm_last[i] = code;
            }
            /* 报警状态未变化：不打印 */
        }
    }

#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
    /* 打印新到达的电压应答（主循环上下文，ISR 只置标志） */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_volt_pending[i]) {
            s_volt_pending[i] = false;
            srv_ht_temp_test_voltage_print(s_motor_ids[i], s_motor_volt[i]);
        }
    }
#endif

    /* 控制开始 1s 内补发使能 + 速度模式（首帧可能被丢弃） */
    if (((now - s_ctrl_start_ms) < 1000U) &&
        ((now - s_last_retry_ms) >= SRV_HT_TEMP_TEST_STARTUP_RETRY_MS)) {
        s_last_retry_ms = now;
        srv_ht_temp_test_cmd_enable_all(true);
        srv_ht_temp_test_cmd_set_mode_all();
    }

    /* 阶段切换 */
    if ((now - s_phase_start_ms) >= SRV_HT_TEMP_TEST_PHASE_MS) {
        s_phase = (srv_ht_temp_test_phase_t)(((uint32_t)s_phase + 1U) % (uint32_t)SRV_HT_TEMP_TEST_PHASE_NUM);
        s_phase_start_ms = now;
        s_last_cmd_ms = now;
        s_last_alarm_ms = now - SRV_HT_TEMP_TEST_ALARM_OFFSET_MS; /* 阶段切换后同样错开 */
#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
        s_last_volt_ms = now - SRV_HT_TEMP_TEST_VOLTAGE_OFFSET_MS;
#endif
        srv_ht_temp_test_log_phase(s_phase);

        /* 进入旋转阶段时补发使能 + 速度模式（停留期间可能已失能/掉模式） */
        if ((s_phase == SRV_HT_TEMP_TEST_PHASE_FORWARD) ||
            (s_phase == SRV_HT_TEMP_TEST_PHASE_REVERSE)) {
            srv_ht_temp_test_cmd_enable_all(true);
            srv_ht_temp_test_cmd_set_mode_all();
#if SRV_HT_TEMP_TEST_RELEASE_BRAKE
            srv_ht_temp_test_cmd_brake_all();
#endif
        }
    }

    /* 周期查询电机报警（1s 一次，0xFF 主动查询，应答由 on_rx 记录） */
    if ((now - s_last_alarm_ms) >= SRV_HT_TEMP_TEST_ALARM_PERIOD_MS) {
        s_last_alarm_ms = now;
        srv_ht_temp_test_query_alarm_all();
    }

#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
    /* 周期读取电机供电电压（0x87，默认 5s 一次，与速度/报警查询错开） */
    if ((now - s_last_volt_ms) >= SRV_HT_TEMP_TEST_VOLTAGE_PERIOD_MS) {
        s_last_volt_ms = now;
        srv_ht_temp_test_query_voltage_all();
    }
#endif

    /* 电机无响应检测：超过 NORESP_PERIOD 未收到电机任何帧视为掉线/断电（每电机只告警一次） */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (!s_motor_nresp_latch[i] &&
            ((now - s_motor_last_seen_ms[i]) >= SRV_HT_TEMP_TEST_NORESP_PERIOD_MS)) {
            s_motor_nresp_latch[i] = true;
            SRV_HT_TEMP_TEST_LOG_W("电机 0x%02X 长时间无响应（掉线或断电）", (unsigned)s_motor_ids[i]);
        }
    }

    /* 打印恢复在线事件（ISR 置位，主循环消费），与上方掉线日志成对出现 */
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_online_evt_pending[i]) {
            s_online_evt_pending[i] = false;
            SRV_HT_TEMP_TEST_LOG_W("电机 0x%02X 恢复在线", (unsigned)s_motor_ids[i]);
        }
    }

    /* 正转/反转发方向速度帧；停留发速度 0 帧 (IQ24=0)，不失能 */
    int16_t target_rpm = 0;
    if (s_phase == SRV_HT_TEMP_TEST_PHASE_FORWARD) {
        target_rpm = SRV_HT_TEMP_TEST_SPEED_RPM;
    } else if (s_phase == SRV_HT_TEMP_TEST_PHASE_REVERSE) {
        target_rpm = -SRV_HT_TEMP_TEST_SPEED_RPM;
    }

    if ((now - s_last_cmd_ms) >= SRV_HT_TEMP_TEST_CMD_PERIOD_MS) {
        s_last_cmd_ms = now;

        /* 缓启动：按时间线性斜坡，RAMP_TIME_MS 内从当前转速线性到达目标。
           目标变化（阶段切换）时以当前下发转速为新段起点重新计时，
           保证正/反转与停留都能真正到达并保持（停留 = 转速 0 保持）。 */
        if (target_rpm != s_ramp_target_rpm) {
            s_ramp_target_rpm = target_rpm;
            s_ramp_from_rpm = s_speed_rpm;
            s_ramp_start_ms = now;
        }

        int16_t send_rpm;
        const uint32_t elapsed = now - s_ramp_start_ms;
        if (elapsed >= SRV_HT_TEMP_TEST_RAMP_TIME_MS) {
            send_rpm = target_rpm; /* 斜坡完成，保持目标 */
        } else {
            const int32_t span = (int32_t)target_rpm - (int32_t)s_ramp_from_rpm;
            const int32_t delta = (int32_t)(((int64_t)span * elapsed)
                / (int32_t)SRV_HT_TEMP_TEST_RAMP_TIME_MS);
            send_rpm = (int16_t)((int32_t)s_ramp_from_rpm + delta);
        }
        s_speed_rpm = send_rpm;

        srv_ht_temp_test_cmd_speed_all(send_rpm);
    }
}

/**
 * @brief 处理测试协议接收帧（由 srv_can_on_rx 在旧 CAN FD 解析前调用）
 * @param  msg CAN 报文指针
 * @return true=测试协议帧，已消费；false=非测试帧，调用方继续解析 0x100
 * @note   ISR 上下文，只做数据记录与标志置位，不打日志
 */
bool srv_ht_temp_test_on_rx(const drv_can_msg_t* msg)
{
    if (!msg)
        return false;

    /* 扫描阶段：应答帧 CAN-ID 即电机地址，记录（越界由 record 过滤） */
    if (s_scanning) {
        srv_ht_temp_test_scan_record((uint8_t)(msg->id & 0xFFU));
        return true;
    }

    const uint8_t addr = (uint8_t)(msg->id & 0xFFU);

    /* 非已知电机地址的帧不属本协议（如上位机 0x100），交由旧 CAN FD 协议解析 */
    const uint32_t idx = srv_ht_temp_test_find_idx(addr);
    if (idx == SRV_HT_TEMP_TEST_MAX_MOTORS)
        return false;

    /* 在线刷新：收到电机地址的任意帧都视为"电机在应答"。
       此前处于掉线锁存的电机收到帧即恢复在线，置标志由主循环打印（ISR 不打日志） */
    s_motor_last_seen_ms[idx] = millis();
    if (s_motor_nresp_latch[idx]) {
        s_motor_nresp_latch[idx] = false;
        s_online_evt_pending[idx] = true;
    }

#if SRV_HT_TEMP_TEST_LOG_RAW_RX_ENABLE
    srv_ht_temp_test_raw_rx_push(msg); /* 调试：记录原始接收帧 */
#endif

    /* 报警应答：data[0]=0xFF，data[1..4]=24 位报警码（大端）。
     * 实测 0xFF 应答为 5 字节（读取指令3 的 IQ24 格式），非文档 §6.1.2 写的 3 字节。
     * 只记录状态（ISR 不打日志——log 共享格式缓冲非可重入），由主循环打印 */
    if ((msg->dlc >= 5U) && (msg->data[0] == SRV_HT_TEMP_TEST_CMD_ALARM)) {
        const uint32_t code = ((uint32_t)msg->data[1] << 24) | ((uint32_t)msg->data[2] << 16) |
                              ((uint32_t)msg->data[3] << 8) | (uint32_t)msg->data[4];
        s_motor_alarm[idx] = code;
        s_alarm_pending[idx] = true;
        return true;
    }

#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
    /* 电压应答：data[0]=0x87，data[1..2]=供电电压（×100，单位 V） */
    if ((msg->dlc >= 3U) && (msg->data[0] == SRV_HT_TEMP_TEST_CMD_VOLTAGE)) {
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
 * @return 索引；未找到返回 SRV_HT_TEMP_TEST_MAX_MOTORS（哨兵值）
 */
static uint32_t srv_ht_temp_test_find_idx(uint8_t addr)
{
    for (uint32_t i = 0; i < s_motor_cnt; i++) {
        if (s_motor_ids[i] == addr)
            return i;
    }
    return SRV_HT_TEMP_TEST_MAX_MOTORS;
}

/**
 * @brief 所有已检测电机是否在线（无任何电机进入无响应锁存）
 * @return true=全部在线
 * @note  DURATION 持续在线计时仅在全部在线时累加；扫描中或 s_motor_cnt==0 视为不在线
 */
static bool srv_ht_temp_test_all_online(void)
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
static void srv_ht_temp_test_scan_record(uint8_t addr)
{
    if ((addr < SRV_HT_TEMP_TEST_SCAN_ADDR_MIN) || (addr > SRV_HT_TEMP_TEST_SCAN_ADDR_MAX))
        return;

    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        if (s_motor_ids[i] == addr)
            return; /* 已记录 */
    }
    if (s_motor_cnt < SRV_HT_TEMP_TEST_MAX_MOTORS) {
        s_motor_ids[s_motor_cnt++] = addr;
    }
}

/**
 * @brief 打印扫描期间新检测到的电机（主循环调用，避免 ISR 打日志）
 */
static void srv_ht_temp_test_scan_log_new(void)
{
    while (s_scan_log_cnt < s_motor_cnt) {
        SRV_HT_TEMP_TEST_LOG_I("  探测到电机：CAN ID = 0x%02X", (unsigned)s_motor_ids[s_scan_log_cnt]);
        s_scan_log_cnt++;
    }
}

/**
 * @brief 扫描结束：打印结果，对检测到的电机下发使能 + 速度模式 + 正转，进入正转阶段
 */
static void srv_ht_temp_test_scan_done(void)
{
    s_scanning = false;
    s_ctrl_start_ms = millis();
    s_last_cmd_ms = s_ctrl_start_ms;
    s_last_retry_ms = s_ctrl_start_ms;
    /* 报警查询/电压读取与速度帧错开：避免同节拍多包同时发送导致丢应答 */
    s_last_alarm_ms = s_ctrl_start_ms - SRV_HT_TEMP_TEST_ALARM_OFFSET_MS;
#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
    s_last_volt_ms = s_ctrl_start_ms - SRV_HT_TEMP_TEST_VOLTAGE_OFFSET_MS;
#endif

    /* 未扫描到电机：回退到默认设备地址继续测试（总线可能因电机未应答握手而探测失败） */
    if (s_motor_cnt == 0U) {
        SRV_HT_TEMP_TEST_LOG_W("扫描完成：未检测到电机（0x%02X~0x%02X），回退到默认 CAN ID 0x%02X",
            SRV_HT_TEMP_TEST_SCAN_ADDR_MIN, SRV_HT_TEMP_TEST_SCAN_ADDR_MAX, SRV_HT_TEMP_TEST_DEFAULT_MOTOR_ADDR);
        s_motor_ids[0] = SRV_HT_TEMP_TEST_DEFAULT_MOTOR_ADDR;
        s_motor_cnt = 1;
    } else {
        SRV_HT_TEMP_TEST_LOG_I("扫描完成：检测到 %u 台电机", (unsigned)s_motor_cnt);
    }

    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        SRV_HT_TEMP_TEST_LOG_I("  电机[%u] CAN ID = 0x%02X", (unsigned)i, (unsigned)s_motor_ids[i]);
    }

    s_phase = SRV_HT_TEMP_TEST_PHASE_FORWARD;
    s_phase_start_ms = s_ctrl_start_ms;

    /* 初始化电机"最后收到帧"时间，作为掉线检测的起点 */
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        s_motor_last_seen_ms[i] = s_ctrl_start_ms;
    }

    srv_ht_temp_test_cmd_enable_all(true); /* 1. 使能 */
    srv_ht_temp_test_cmd_set_mode_all();   /* 2. 速度模式 */
#if SRV_HT_TEMP_TEST_RELEASE_BRAKE
    srv_ht_temp_test_cmd_brake_all();      /* 3. 打开抱闸（若支持） */
#endif
    /* 不直接发全速：由 srv_ht_temp_test_step 的缓启动斜坡从 0 线性爬升至目标，
       避免扫描完成瞬间 0→SPEED 阶跃过大报警 */
    srv_ht_temp_test_log_phase(s_phase);
}

/**
 * @brief 发送握手探测帧 (0x00, 经典 CAN 1B)
 * @param addr 待探测的电机地址（CAN-ID）
 */
static void srv_ht_temp_test_send_handshake(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false, /* 苇熠协议：经典 CAN 2.0A，1 Mbps */
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_TEMP_TEST_CMD_HANDSHAKE;
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送使能/失能帧 (经典 CAN 2B)
 * @param addr   电机地址（CAN-ID）
 * @param enable true=使能 (0x2A 01)，false=失能 (0x2A 00)
 */
static void srv_ht_temp_test_send_enable(uint8_t addr, bool enable)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 2,
    };
    tx.data[0] = SRV_HT_TEMP_TEST_CMD_ENABLE;
    tx.data[1] = enable ? 0x01U : 0x00U;
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送设置速度模式帧 (0x07 02, 经典 CAN 2B)
 * @param addr 电机地址（CAN-ID）
 */
static void srv_ht_temp_test_set_speed_mode(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 2,
    };
    tx.data[0] = SRV_HT_TEMP_TEST_CMD_MODE;
    tx.data[1] = SRV_HT_TEMP_TEST_MODE_SPEED;
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送速度设定帧 (0x09 + IQ24, 经典 CAN 5B)
 * @param addr 电机地址（CAN-ID）
 * @param rpm  目标转速 (RPM)，正=正转，负=反转，0=停止（不失能）
 * @note  IQ24 = rpm / 6000 × 2^24，4 字节大端（高字节在前）
 */
static void srv_ht_temp_test_send_speed(uint8_t addr, int16_t rpm)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    /* 实际转速 → IQ24：IQ = rpm / SRV_HT_TEMP_TEST_SPEED_FULL_SCALE × 2^24（取整） */
    const int32_t iq = (int32_t)(((int64_t)rpm << 24) / SRV_HT_TEMP_TEST_SPEED_FULL_SCALE);

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 5,
    };
    tx.data[0] = SRV_HT_TEMP_TEST_CMD_SPEED;
    tx.data[1] = (uint8_t)((uint32_t)iq >> 24);
    tx.data[2] = (uint8_t)((uint32_t)iq >> 16);
    tx.data[3] = (uint8_t)((uint32_t)iq >> 8);
    tx.data[4] = (uint8_t)(uint32_t)iq;
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/* --- 批量下发（对全部检测到的电机） --- */

static void srv_ht_temp_test_cmd_enable_all(bool enable)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_temp_test_send_enable(s_motor_ids[i], enable);
    }
}

static void srv_ht_temp_test_cmd_set_mode_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_temp_test_set_speed_mode(s_motor_ids[i]);
    }
}

static void srv_ht_temp_test_cmd_speed_all(int16_t rpm)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_temp_test_send_speed(s_motor_ids[i], rpm);
    }
}

/**
 * @brief 发送报警查询帧 (0xFF, 经典 CAN 1B)
 * @param addr 电机地址（CAN-ID）
 * @note  电机返回 [0xFF][4B 大端报警码]（5B），由 on_rx 记录
 */
static void srv_ht_temp_test_send_query_alarm(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_TEMP_TEST_CMD_ALARM;
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 对所有检测到的电机发送报警查询帧
 */
static void srv_ht_temp_test_query_alarm_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_temp_test_send_query_alarm(s_motor_ids[i]);
    }
}

/**
 * @brief 打印报警详情（主循环上下文调用）
 * @note  报警码可组合（多位同时置位）；WARN 打印原始码并逐位解码
 */
static void srv_ht_temp_test_alarm_print(uint8_t addr, uint32_t code)
{
    if (code == 0U) {
        SRV_HT_TEMP_TEST_LOG_I("电机 0x%02X 报警查询：无报警", (unsigned)addr);
        return;
    }

    SRV_HT_TEMP_TEST_LOG_W("电机 0x%02X 报警：0x%08X", (unsigned)addr, (unsigned)code);
    for (uint32_t i = 0; i < SRV_HT_TEMP_TEST_ALARM_NUM; i++) {
        if ((code & s_alarm_map[i].mask) != 0U) {
            SRV_HT_TEMP_TEST_LOG_W("  - %s", s_alarm_map[i].name);
        }
    }
}

#if SRV_HT_TEMP_TEST_VOLTAGE_QUERY_ENABLE
/**
 * @brief 发送电压读取帧 (0x87, 经典 CAN 1B)
 * @param addr 电机地址（CAN-ID）
 * @note  电机返回 [0x87][V高][V低]（3B，×100 单位 V），由 on_rx 记录
 */
static void srv_ht_temp_test_send_query_voltage(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 1,
    };
    tx.data[0] = SRV_HT_TEMP_TEST_CMD_VOLTAGE;
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 对所有检测到的电机发送电压读取帧
 */
static void srv_ht_temp_test_query_voltage_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_temp_test_send_query_voltage(s_motor_ids[i]);
    }
}

/**
 * @brief 打印电压读取结果（主循环上下文调用）
 * @param volt 电压值 ×100（如 2400 = 24.00V）
 */
static void srv_ht_temp_test_voltage_print(uint8_t addr, uint16_t volt)
{
    SRV_HT_TEMP_TEST_LOG_I("电机 0x%02X 供电电压：%u.%02u V",
        (unsigned)addr, (unsigned)(volt / 100U), (unsigned)(volt % 100U));
}
#endif

#if SRV_HT_TEMP_TEST_LOG_RAW_RX_ENABLE
/**
 * @brief 记录收到的原始 CAN 帧到环形缓冲（ISR 中调用，不打日志）
 */
static void srv_ht_temp_test_raw_rx_push(const drv_can_msg_t* msg)
{
    if (!msg)
        return;

    uint8_t head = s_raw_rx_head;
    srv_ht_temp_test_raw_rx_t* slot = &s_raw_rx[head];
    slot->id = msg->id;
    slot->dlc = msg->dlc;
    memset(slot->data, 0, sizeof(slot->data));
    uint8_t n = (msg->dlc < 8U) ? msg->dlc : 8U;
    memcpy(slot->data, msg->data, n);

    head = (uint8_t)((head + 1U) % SRV_HT_TEMP_TEST_RAW_RX_DEPTH);
    if (head == s_raw_rx_tail) {
        /* 环形满：丢弃最旧，保留最新 */
        s_raw_rx_tail = (uint8_t)((s_raw_rx_tail + 1U) % SRV_HT_TEMP_TEST_RAW_RX_DEPTH);
    }
    s_raw_rx_head = head;
}

/**
 * @brief 打印环形缓冲中的原始接收帧（主循环上下文调用）
 */
static void srv_ht_temp_test_raw_rx_drain(void)
{
    while (s_raw_rx_tail != s_raw_rx_head) {
        const srv_ht_temp_test_raw_rx_t* slot = &s_raw_rx[s_raw_rx_tail];
        s_raw_rx_tail = (uint8_t)((s_raw_rx_tail + 1U) % SRV_HT_TEMP_TEST_RAW_RX_DEPTH);
        // SRV_HT_TEMP_TEST_LOG_D("RX id=0x%03lX dlc=%u %02X %02X %02X %02X %02X %02X %02X %02X",
        //     (unsigned long)slot->id, (unsigned)slot->dlc,
        //     slot->data[0], slot->data[1], slot->data[2], slot->data[3],
        //     slot->data[4], slot->data[5], slot->data[6], slot->data[7]);
    }
}
#endif

/**
 * @brief 打印阶段切换日志
 */
static void srv_ht_temp_test_log_phase(srv_ht_temp_test_phase_t phase)
{
    const char* name;
    int16_t rpm = 0;

    switch (phase) {
    case SRV_HT_TEMP_TEST_PHASE_FORWARD:
        name = "正转";
        rpm = SRV_HT_TEMP_TEST_SPEED_RPM;
        break;
    case SRV_HT_TEMP_TEST_PHASE_DWELL_F:
        name = "停留(正转后)";
        break;
    case SRV_HT_TEMP_TEST_PHASE_REVERSE:
        name = "反转";
        rpm = -SRV_HT_TEMP_TEST_SPEED_RPM;
        break;
    case SRV_HT_TEMP_TEST_PHASE_DWELL_R:
        name = "停留(反转后)";
        break;
    default:
        name = "未知";
        break;
    }

    SRV_HT_TEMP_TEST_LOG_I("阶段切换 -> %s（速度 %d RPM，时长 %u ms）",
        name, (int)rpm, (unsigned)SRV_HT_TEMP_TEST_PHASE_MS);
}

#if SRV_HT_TEMP_TEST_RELEASE_BRAKE
/**
 * @brief 发送打开抱闸帧 (0xF4 0100, 经典 CAN 3B)
 * @param addr 电机地址（CAN-ID）
 */
static void srv_ht_temp_test_send_release_brake(uint8_t addr)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = addr,
        .is_extended = false,
        .is_fd = false,
        .dlc = 3,
    };
    tx.data[0] = SRV_HT_TEMP_TEST_CMD_BRAKE;
    tx.data[1] = 0x01U; /* 0x0100 大端 */
    tx.data[2] = 0x00U;
    drv_can_send(DRV_CAN_CH_1, &tx);
}

static void srv_ht_temp_test_cmd_brake_all(void)
{
    for (uint8_t i = 0; i < s_motor_cnt; i++) {
        srv_ht_temp_test_send_release_brake(s_motor_ids[i]);
    }
}
#endif
