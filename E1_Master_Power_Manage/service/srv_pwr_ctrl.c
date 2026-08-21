/**
 * @file    srv_pwr_ctrl.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-07-2
 * @brief   电源控制服务实现 — 电源 FSM + 电机预充电软启动 FSM（双状态机）
 *
 * 直接驱动 drv_power / drv_pwm / drv_status / srv_adc（Style B）。
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_ctrl.h"

#include <string.h>

#include "drv_power.h"
#include "drv_pwm.h"
#include "drv_status.h"
#include "fsm.h"
#include "log.h"
#include "srv_adc.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_PWR_CTRL_LOG_ENABLE 1

#if SRV_PWR_CTRL_LOG_ENABLE
#define SRV_PWR_CTRL_LOG_E(...) LOG_E("srv_pwr_ctrl", __VA_ARGS__)
#define SRV_PWR_CTRL_LOG_W(...) LOG_W("srv_pwr_ctrl", __VA_ARGS__)
#define SRV_PWR_CTRL_LOG_I(...) LOG_I("srv_pwr_ctrl", __VA_ARGS__)
#define SRV_PWR_CTRL_LOG_D(...) LOG_D("srv_pwr_ctrl", __VA_ARGS__)
#else
#define SRV_PWR_CTRL_LOG_E(...) ((void)0)
#define SRV_PWR_CTRL_LOG_W(...) ((void)0)
#define SRV_PWR_CTRL_LOG_I(...) ((void)0)
#define SRV_PWR_CTRL_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define STEADY_TIME_MS (50U)

/** @brief MOTOR 移交：母线达到 VIN 该比例后关闭预充电半桥 */
#define PWR_MOTOR_HANDOVER_PCT (98U)
/** @brief MOTOR 移交超时兜底 (ms)：母线未抬到阈值也强制关闭预充电 */
#define PWR_MOTOR_HANDOVER_TIMEOUT_MS (200U)

/* ── 预充电软启动参数（详见 docs/motor_power_charge_step.md） ── */
/** @brief 阶段一：EN 保持低电平清除 OCP 锁存的时长 (ms) */
#define PWR_PRECHARGE_EN_CLEAR_MS (20U)
/** @brief 阶段二：50kHz 恒频脉宽爬升时长 (ms) */
#define PWR_PRECHARGE_PHASE2_MS (25U)
/** @brief 阶段三：50k→600k 变频时长 (ms) */
#define PWR_PRECHARGE_PHASE3_MS (25U)
/** @brief 阶段四：600kHz 恒频占空比爬升时长 (ms) */
#define PWR_PRECHARGE_PHASE4_MS (450U)
/** @brief NO_LOAD 检查时点：阶段四内部第 50ms（全局 100ms）处判 bus_mv<1000mV */
#define PWR_PRECHARGE_NO_LOAD_CHECK_MS (50U)
/** @brief 低频端频率 (Hz) */
#define PWR_PRECHARGE_FREQ_LOW_HZ (50000U)
/** @brief 高频端频率 (Hz) */
#define PWR_PRECHARGE_FREQ_HIGH_HZ (600000U)
/** @brief 阶段二起始占空比（‰）：50kHz 下 Ton 5.95ns≈0.3‰，取整为 0 */
#define PWR_PRECHARGE_DUTY_P2_START (0U)
/** @brief 阶段二结束占空比（‰）：50kHz 下 Ton 167ns≈8.35‰ */
#define PWR_PRECHARGE_DUTY_P2_END (8U)
/** @brief 阶段三结束占空比（‰）：600kHz 下 Ton 167ns≈100.2‰ */
#define PWR_PRECHARGE_DUTY_P3_END (100U)
/** @brief 阶段四结束占空比（‰）：90% */
#define PWR_PRECHARGE_DUTY_P4_END (900U)
/** @brief 稳态占空比（‰）：90% */
#define PWR_PRECHARGE_DUTY_STEADY (900U)
/** @brief OCP 恢复延时（ms）：关输出等待硬件锁存清除 */
#define PWR_PRECHARGE_OCP_RECOVER_MS (10U)
/** @brief 单次启动最大 OCP 重试次数 */
#define PWR_PRECHARGE_OCP_RETRY_MAX (3U)
/** @brief NO_LOAD 判定电压阈值 (mV) */
#define PWR_PRECHARGE_NO_LOAD_MV (1000U)

/** @brief 冷机判定：预偏置比 < 10%（100‰）→ 走全流程 */
#define PWR_PRECHARGE_COLD_RATIO_PERMILLE (100U)

/** @brief 初始电压跟随开关：1=按初始 bus/vin 比值跳过前序阶段（预偏置），0=恒冷机全流程 */
#define PWR_PRECHARGE_PREBIAS_EN (0U)
/** @brief 提前转稳态优化开关：1=输出≥输入母线 95% 即转稳态，0=固定跑满 500ms */
#define PWR_PRECHARGE_EARLY_STEADY_EN (0U)
/** @brief 提前转稳态电压比例（95%） */
#define PWR_PRECHARGE_EARLY_STEADY_PCT (95U)

/* FSM 状态 -----------------------------------------------------------------*/

/** @brief 电源上电时序状态 */
enum {
    PWR_STATE_IDLE = 0,
    PWR_STATE_AUX,
    PWR_STATE_PRECHARGE,
    PWR_STATE_MOTOR,
    PWR_STATE_DONE,
    PWR_STATE_COUNT,
};

/** @brief 电机预充电软启动状态 */
typedef enum {
    PRECHARGE_STATE_IDLE = 0, /**< 空闲：等待启动，输出全关 */
    PRECHARGE_STATE_EN_CLEAR, /**< 阶段一：EN 保持低 5ms 清除 OCP 锁存 */
    PRECHARGE_STATE_PREBIAS, /**< 阶段0：预偏置检测（EN 低采样，决定冷/暖/近满） */
    PRECHARGE_STATE_RAMP_TON, /**< 阶段二：50kHz 恒频，脉宽爬升 */
    PRECHARGE_STATE_RAMP_FREQ, /**< 阶段三：50k→600k 变频（脉宽恒） */
    PRECHARGE_STATE_RAMP_DUTY, /**< 阶段四：600kHz 恒频，占空比爬升 */
    PRECHARGE_STATE_STEADY, /**< 稳态：600kHz 90%，Precharge_Done */
    PRECHARGE_STATE_OCP_RECOVER, /**< OCP 恢复：关输出延时 10ms */
    PRECHARGE_STATE_FAULT, /**< 锁存故障：短路/未接负载，需 reset */
    PRECHARGE_STATE_COUNT,
} precharge_state_t;

/** @brief 预充电故障码 */
typedef enum {
    PRECHARGE_FAULT_NONE = 0, /**< 无故障 */
    PRECHARGE_FAULT_SHORT_CIRCUIT, /**< OCP 重试 ≥3 次，判定后级短路 */
    PRECHARGE_FAULT_NO_LOAD, /**< 阶段四未建立电压，未接负载/上电故障 */
} precharge_fault_t;

/* Private types -------------------------------------------------------------*/

typedef struct {
    bool power_on_requested;
    uint16_t steady_ms;
    bool precharge_off_done; /**< MOTOR 态是否已完成预充电移交关闭 */
    bool aux_en;   /**< AUX_POWER_EN 驱动状态（PGD 判定门控） */
    bool motor_en; /**< MOTOR_POWER_EN 驱动状态（PGD 判定门控） */
} power_ctrl_ctx_t;

/** @brief 预充电 FSM 上下文（单实例静态） */
typedef struct {
    uint32_t phase_elapsed_ms; /**< 当前阶段已走时间 (ms) */
    uint32_t total_elapsed_ms; /**< 软启动累计时间 (ms) */
    uint32_t last_motor_bus_mv; /**< 最近一次读到的 Buck 输出电压 (mV) */
    uint32_t last_vin_mv; /**< 最近一次读到的输入母线电压 VIN (mV) */
    uint16_t elapsed_ms; /**< 最近一次 step 周期 (ms)，handler 内使用 */
    uint8_t ocp_retry; /**< OCP 重试计数 */
    uint32_t ramp_seed; /**< 暖机切入阶段四的虚拟已走时间 (ms)，冷机为 0 */
    precharge_fault_t fault; /**< 故障码 */
    bool start_requested; /**< 启动请求标志 */
    bool done; /**< 预充电完成标志 */
} precharge_ctx_t;

/* Private variables ---------------------------------------------------------*/

static fsm_t s_fsm;
static power_ctrl_ctx_t s_ctx;

static fsm_handler_t s_pwr_handlers[PWR_STATE_COUNT];
static fsm_guard_t s_pwr_transitions[PWR_STATE_COUNT * PWR_STATE_COUNT];
static const char* s_pwr_state_names[PWR_STATE_COUNT] = {
    "IDLE",
    "AUX",
    "PRECHARGE",
    "MOTOR",
    "DONE",
};

static fsm_t s_precharge_fsm;
static precharge_ctx_t s_precharge;

static fsm_handler_t s_precharge_handlers[PRECHARGE_STATE_COUNT];
static fsm_guard_t s_precharge_transitions[PRECHARGE_STATE_COUNT * PRECHARGE_STATE_COUNT];
static const char* s_precharge_state_names[PRECHARGE_STATE_COUNT] = {
    [PRECHARGE_STATE_IDLE] = "IDLE",
    [PRECHARGE_STATE_EN_CLEAR] = "EN_CLEAR",
    [PRECHARGE_STATE_PREBIAS] = "PREBIAS",
    [PRECHARGE_STATE_RAMP_TON] = "RAMP_TON",
    [PRECHARGE_STATE_RAMP_FREQ] = "RAMP_FREQ",
    [PRECHARGE_STATE_RAMP_DUTY] = "RAMP_DUTY",
    [PRECHARGE_STATE_STEADY] = "STEADY",
    [PRECHARGE_STATE_OCP_RECOVER] = "OCP_RECOVER",
    [PRECHARGE_STATE_FAULT] = "FAULT",
};

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t pwr_state_idle(fsm_t* ctx);
static fsm_state_t pwr_state_aux(fsm_t* ctx);
static fsm_state_t pwr_state_precharge(fsm_t* ctx);
static fsm_state_t pwr_state_motor(fsm_t* ctx);
static fsm_state_t pwr_state_done(fsm_t* ctx);
static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state);

static void precharge_init(void);
static void precharge_step(uint16_t elapsed_ms);
static void precharge_start(void);
static void precharge_reset(void);
static void precharge_begin(void);
static void precharge_sample_voltages(void);

static void precharge_entry_cb(fsm_t* fsm, fsm_state_t state);
static fsm_state_t precharge_state_idle(fsm_t* fsm);
static fsm_state_t precharge_state_en_clear(fsm_t* fsm);
static fsm_state_t precharge_state_prebias(fsm_t* fsm);
static fsm_state_t precharge_state_ramp_ton(fsm_t* fsm);
static fsm_state_t precharge_state_ramp_freq(fsm_t* fsm);
static fsm_state_t precharge_state_ramp_duty(fsm_t* fsm);
static fsm_state_t precharge_state_steady(fsm_t* fsm);
static fsm_state_t precharge_state_ocp_recover(fsm_t* fsm);
static fsm_state_t precharge_state_fault(fsm_t* fsm);

static void precharge_phase_ramp(uint32_t freq_start, uint32_t freq_end,
    uint16_t duty_start, uint16_t duty_end, uint32_t duration_ms);

static const char* precharge_fault_name(precharge_fault_t f);

/* Exported functions --------------------------------------------------------*/

void srv_pwr_ctrl_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    drv_power_init();
    drv_pwm_init();

    s_pwr_handlers[PWR_STATE_IDLE] = pwr_state_idle;
    s_pwr_handlers[PWR_STATE_AUX] = pwr_state_aux;
    s_pwr_handlers[PWR_STATE_PRECHARGE] = pwr_state_precharge;
    s_pwr_handlers[PWR_STATE_MOTOR] = pwr_state_motor;
    s_pwr_handlers[PWR_STATE_DONE] = pwr_state_done;

    fsm_config_t config = {
        .handlers = s_pwr_handlers,
        .transitions = s_pwr_transitions,
        .state_count = PWR_STATE_COUNT,
        .entry_cb = pwr_entry_cb,
        .exit_cb = NULL,
        .state_names = s_pwr_state_names,
        .user_data = &s_ctx,
    };

    fsm_fill(&config, fsm_always_true);
    fsm_init(&s_fsm, PWR_STATE_IDLE, &config);

    precharge_init();

    SRV_PWR_CTRL_LOG_I("电源控制服务初始化完成 (电源FSM %u 状态 + 预充电FSM %u 状态)",
        (unsigned)PWR_STATE_COUNT, (unsigned)PRECHARGE_STATE_COUNT);
}

void srv_pwr_ctrl_step(uint16_t elapsed_ms)
{
    /* 1. 预充电 FSM 步进（power_task 以 1ms 调用，保持软启动 1ms 分辨率） */
    precharge_step(elapsed_ms);

    /* 2. 电源 FSM 步进 */
    s_ctx.steady_ms += elapsed_ms;
    fsm_step(&s_fsm);

    // drv_power_set(DRV_POWER_RAIL_DC_DC_EN, true);
}

void srv_pwr_ctrl_request_on(void)
{
    if (s_ctx.power_on_requested == false) {
        s_ctx.power_on_requested = true;
        SRV_PWR_CTRL_LOG_I("上电请求已登记");
    }
}

void srv_pwr_ctrl_emergency_off(void)
{
    if (s_ctx.power_on_requested == true) {
        s_ctx.power_on_requested = false;
        s_ctx.steady_ms = 0;
        s_ctx.aux_en = false;
        s_ctx.motor_en = false;

        fsm_goto(&s_fsm, PWR_STATE_IDLE);

        for (uint32_t i = 0; i <= DRV_POWER_RAIL_DBR_LSD_EN; i++) {
            drv_power_set((drv_power_rail_t)i, false);
        }

        /* 急停时预充电 EN/PWM 立即关断 */
        precharge_reset();
        SRV_PWR_CTRL_LOG_E("紧急断电触发: 全部电源轨关闭");
    }
}

bool srv_pwr_ctrl_is_powered_on(void)
{
    return fsm_current_state(&s_fsm) == PWR_STATE_DONE;
}

bool srv_pwr_ctrl_is_aux_enabled(void)
{
    return s_ctx.aux_en;
}

bool srv_pwr_ctrl_is_motor_enabled(void)
{
    return s_ctx.motor_en;
}

/* Private functions ---------------------------------------------------------*/

static fsm_state_t pwr_state_idle(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);
    return p->power_on_requested ? PWR_STATE_AUX : PWR_STATE_IDLE;
}

static fsm_state_t pwr_state_aux(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);
    return (p->steady_ms >= STEADY_TIME_MS) ? PWR_STATE_PRECHARGE : PWR_STATE_AUX;
}

static fsm_state_t pwr_state_precharge(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);

    /* 预充电软启动失败：中止上电，等下次 request_on 重试 */
    if (s_precharge.fault != PRECHARGE_FAULT_NONE) {
        SRV_PWR_CTRL_LOG_E("预充电故障: %s (code=%d)，中止上电",
            precharge_fault_name(s_precharge.fault), (int)s_precharge.fault);
        p->power_on_requested = false;
        return PWR_STATE_IDLE;
    }

    /* 预充电完成：进入电机上电 */
    if (s_precharge.done) {
        SRV_PWR_CTRL_LOG_I("预充电完成，进入电机上电");
        return PWR_STATE_MOTOR;
    }

    return PWR_STATE_PRECHARGE;
}

/**
 * @brief 预充电故障码 → 中文原因文本
 */
static const char* precharge_fault_name(precharge_fault_t f)
{
    switch (f) {
    case PRECHARGE_FAULT_SHORT_CIRCUIT:
        return "后级短路 (SHORT_CIRCUIT)";
    case PRECHARGE_FAULT_NO_LOAD:
        return "未接负载/上电故障 (NO_LOAD)";
    default:
        return "未知故障";
    }
}

static fsm_state_t pwr_state_motor(fsm_t* ctx)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);

    /* 移交：主回路就绪（母线接近 VIN）后再关闭预充电半桥，避免 MOTOR_POWER_PGD 跌落 */
    if (!p->precharge_off_done) {
        const bool handover_ok = (s_precharge.last_vin_mv > 0
            && s_precharge.last_motor_bus_mv
                >= s_precharge.last_vin_mv * PWR_MOTOR_HANDOVER_PCT / 100U);

        if (handover_ok || p->steady_ms >= PWR_MOTOR_HANDOVER_TIMEOUT_MS) {
            SRV_PWR_CTRL_LOG_I("MOTOR 移交: %s 关闭预充电 (bus=%umV, vin=%umV, steady=%ums)",
                handover_ok ? "电压条件满足" : "超时兜底",
                (unsigned)s_precharge.last_motor_bus_mv, (unsigned)s_precharge.last_vin_mv,
                (unsigned)p->steady_ms);
            precharge_reset();
            p->precharge_off_done = true;
        }
    }

    /* 移交完成且母线稳定后进入 DONE */
    return (p->precharge_off_done && p->steady_ms >= STEADY_TIME_MS)
        ? PWR_STATE_DONE : PWR_STATE_MOTOR;
}

static fsm_state_t pwr_state_done(fsm_t* ctx)
{
    (void)ctx;
    return PWR_STATE_DONE;
}

/* --- FSM Entry Callbacks --- */

static void pwr_entry_cb(fsm_t* ctx, fsm_state_t state)
{
    power_ctrl_ctx_t* p = (power_ctrl_ctx_t*)fsm_user_data(ctx);
    p->steady_ms = 0;

    /* 仅在状态进入时打印（FSM 转换边沿），覆盖 IDLE/AUX/PRECHARGE/MOTOR/DONE */
    SRV_PWR_CTRL_LOG_I("电源 FSM 进入状态: %s", s_pwr_state_names[state]);

    switch (state) {
    case PWR_STATE_AUX:
        p->aux_en = true;
        drv_power_set(DRV_POWER_RAIL_DC_DC_EN, true);
        drv_power_set(DRV_POWER_RAIL_AUX_EN, true);
        break;
    case PWR_STATE_PRECHARGE:
        /* 启动预充电软启动（内部先复位清锁存再 start） */
        precharge_begin();
        drv_power_set(DRV_POWER_RAIL_HSD1_12V_DIAG, true);
        drv_power_set(DRV_POWER_RAIL_HSD1_24V_DIAG, true);
        drv_power_set(DRV_POWER_RAIL_HSD2_24V_DIAG, true);
        break;
    case PWR_STATE_MOTOR:
        p->precharge_off_done = false;
        p->motor_en = true;
        drv_power_set(DRV_POWER_RAIL_MOTOR_EN, true);
        drv_power_set(DRV_POWER_RAIL_HSD1_12V, true);
        drv_power_set(DRV_POWER_RAIL_HSD1_24V, true);
        drv_power_set(DRV_POWER_RAIL_HSD2_24V, true);
        /* 预充电保持导通，等母线抬到接近 VIN 后再关闭（见 pwr_state_motor） */
        break;
    default:
        break;
    }
}

/* ===== 预充电软启动（内部实现，直接驱动） ===== */

/**
 * @brief 初始化预充电 FSM：填表 + 置初始输出全关
 */
static void precharge_init(void)
{
    memset(&s_precharge, 0, sizeof(s_precharge));

    s_precharge_handlers[PRECHARGE_STATE_IDLE] = precharge_state_idle;
    s_precharge_handlers[PRECHARGE_STATE_EN_CLEAR] = precharge_state_en_clear;
    s_precharge_handlers[PRECHARGE_STATE_PREBIAS] = precharge_state_prebias;
    s_precharge_handlers[PRECHARGE_STATE_RAMP_TON] = precharge_state_ramp_ton;
    s_precharge_handlers[PRECHARGE_STATE_RAMP_FREQ] = precharge_state_ramp_freq;
    s_precharge_handlers[PRECHARGE_STATE_RAMP_DUTY] = precharge_state_ramp_duty;
    s_precharge_handlers[PRECHARGE_STATE_STEADY] = precharge_state_steady;
    s_precharge_handlers[PRECHARGE_STATE_OCP_RECOVER] = precharge_state_ocp_recover;
    s_precharge_handlers[PRECHARGE_STATE_FAULT] = precharge_state_fault;

    fsm_config_t fsm_cfg = {
        .handlers = s_precharge_handlers,
        .transitions = s_precharge_transitions,
        .state_count = PRECHARGE_STATE_COUNT,
        .entry_cb = precharge_entry_cb,
        .exit_cb = NULL,
        .state_names = s_precharge_state_names,
        .user_data = &s_precharge,
    };
    fsm_fill(&fsm_cfg, fsm_always_true);
    fsm_init(&s_precharge_fsm, PRECHARGE_STATE_IDLE, &fsm_cfg);

    /* 初始输出全关（PWM 0%、EN 低） */
    drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, false);
    drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 0);
}

/**
 * @brief 预充电 FSM 步进：OCP 全局检测 + 推进状态
 */
static void precharge_step(uint16_t elapsed_ms)
{
    s_precharge.elapsed_ms = elapsed_ms;

    /* 软启动计时（仅活动带 EN_CLEAR..STEADY 累计，FAULT/IDLE 不计） */
    const fsm_state_t st = fsm_current_state(&s_precharge_fsm);
    if (st >= PRECHARGE_STATE_EN_CLEAR
        && st <= PRECHARGE_STATE_STEADY) {
        s_precharge.total_elapsed_ms += elapsed_ms;
    }

    /* 采样母线电压（每拍刷新，供 NO_LOAD/提前转稳态判定） */
    precharge_sample_voltages();

    /* OCP 全局检测：任意活动态检出过流 → 立即关输出并进入恢复态 */
    if ((st == PRECHARGE_STATE_EN_CLEAR
            || st == PRECHARGE_STATE_RAMP_TON
            || st == PRECHARGE_STATE_RAMP_FREQ
            || st == PRECHARGE_STATE_RAMP_DUTY)
        && drv_status_read(DRV_STATUS_MOTOR_CHG_OCP)) {
        SRV_PWR_CTRL_LOG_E("检出电机充电过流 (OCP)，关输出进入恢复态");
        fsm_goto(&s_precharge_fsm, PRECHARGE_STATE_OCP_RECOVER);
    }

    fsm_step(&s_precharge_fsm);
}

/**
 * @brief 请求启动预充电（置标志，IDLE 态 handler 消费后进入 EN_CLEAR）
 */
static void precharge_start(void)
{
    s_precharge.start_requested = true;
    s_precharge.done = false;
}

/**
 * @brief 复位预充电：清锁存/计数并立即关断输出回 IDLE
 */
static void precharge_reset(void)
{
    s_precharge.start_requested = false;
    s_precharge.ocp_retry = 0;
    s_precharge.fault = PRECHARGE_FAULT_NONE;
    s_precharge.done = false;
    s_precharge.total_elapsed_ms = 0;
    s_precharge.ramp_seed = 0;

    /* 立即关断输出（急停语义：不依赖下一个 1ms tick 的 entry_cb） */
    drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, false);
    drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 0);

    fsm_goto(&s_precharge_fsm, PRECHARGE_STATE_IDLE);
}

/**
 * @brief 预充电开始：复位（清 FAULT/STEADY 锁存）后启动
 */
static void precharge_begin(void)
{
    precharge_reset();
    precharge_start();
}

/**
 * @brief 采样输出电压与输入母线电压（一次 ADC 快照取两路）
 */
static void precharge_sample_voltages(void)
{
    srv_adc_data_t adc;
    if (srv_adc_get_latest(&adc)) {
        s_precharge.last_motor_bus_mv = adc.motor_power_mv; /* Buck 输出（NO_LOAD/状态用） */
        s_precharge.last_vin_mv = adc.vin_mv; /* 主输入母线（提前稳态参考） */
    }
}

/* --- 预充电 FSM Entry Callback --- */

static void precharge_entry_cb(fsm_t* fsm, fsm_state_t state)
{
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);
    /* 仅在状态进入时打印（FSM 转换边沿），覆盖 IDLE/AUX/PRECHARGE/MOTOR/DONE */
    SRV_PWR_CTRL_LOG_I("电机电源预充 FSM 进入状态: %s", s_precharge_state_names[state]);
    c->phase_elapsed_ms = 0;

    switch (state) {
    case PRECHARGE_STATE_IDLE:
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, false);
        drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 0);
        break;
    case PRECHARGE_STATE_EN_CLEAR:
        /* EN 保持低 5ms 清除硬件 OCP 锁存 */
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, false);
        drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 0);
        break;
    case PRECHARGE_STATE_PREBIAS:
        /* 阶段0：保持 EN 低 + PWM 0，仅采样残余电压，不驱动输出 */
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, false);
        drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 0);
        break;
    case PRECHARGE_STATE_RAMP_TON:
        /* 阶段二起点：50kHz、最小占空比，拉高 EN 启动 PWM */
        c->total_elapsed_ms = 0;
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, true);
        drv_pwm_set_frequency(DRV_PWM_CH_MOTOR_CHG_IN, PWR_PRECHARGE_FREQ_LOW_HZ);
        drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, PWR_PRECHARGE_DUTY_P2_START);
        break;
    case PRECHARGE_STATE_RAMP_FREQ:
        /* 延续阶段二终点输出（频率/占空比由 handler 逐拍更新） */
        break;
    case PRECHARGE_STATE_RAMP_DUTY:
        /* 暖机切入可能跳过 RAMP_TON，这里补拉高 EN；按折算的虚拟已走时间预置进度 */
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, true);
        drv_pwm_set_frequency(DRV_PWM_CH_MOTOR_CHG_IN, PWR_PRECHARGE_FREQ_HIGH_HZ);
        c->phase_elapsed_ms = c->ramp_seed; /* 覆盖 entry_cb 顶部的 =0 */
        break;
    case PRECHARGE_STATE_STEADY:
        drv_pwm_set_frequency(DRV_PWM_CH_MOTOR_CHG_IN, PWR_PRECHARGE_FREQ_HIGH_HZ);
        drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, PWR_PRECHARGE_DUTY_STEADY);
        c->done = true;
        SRV_PWR_CTRL_LOG_I("预充电完成 (Precharge_Done): 600kHz 90%%");
        break;
    case PRECHARGE_STATE_OCP_RECOVER:
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, false);
        drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 0);
        break;
    case PRECHARGE_STATE_FAULT:
        drv_power_set(DRV_POWER_RAIL_MOTOR_CHG_EN, false);
        drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, 0);
        break;
    default:
        break;
    }
}

/* --- 预充电 FSM 状态处理器 --- */

/**
 * @brief IDLE：等待启动请求，消费后清计数/故障并进入 EN_CLEAR
 */
static fsm_state_t precharge_state_idle(fsm_t* fsm)
{
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);

    if (c->start_requested) {
        c->start_requested = false;
        c->ocp_retry = 0;
        c->fault = PRECHARGE_FAULT_NONE;
        c->done = false;
        SRV_PWR_CTRL_LOG_I("预充电启动 (EN 清窗 5ms)");
        return PRECHARGE_STATE_EN_CLEAR;
    }
    return PRECHARGE_STATE_IDLE;
}

/**
 * @brief 阶段一：EN 低 5ms 清除 OCP 锁存后进入预偏置检测
 */
static fsm_state_t precharge_state_en_clear(fsm_t* fsm)
{
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);

    c->phase_elapsed_ms += c->elapsed_ms;

    if (c->phase_elapsed_ms >= PWR_PRECHARGE_EN_CLEAR_MS) {
        return PRECHARGE_STATE_PREBIAS;
    }
    return PRECHARGE_STATE_EN_CLEAR;
}

/**
 * @brief 阶段0：预偏置检测——按 motor_power_mv/vin 比值决定冷机/暖机/近满
 */
static fsm_state_t precharge_state_prebias(fsm_t* fsm)
{
#if PWR_PRECHARGE_PREBIAS_EN
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);

    /* 初始电压跟随：按 motor_power_mv/vin 比值自适应冷机/暖机/近满 */
    const uint32_t vin = c->last_vin_mv;
    if (vin == 0U) {
        /* 母线电压无效：按冷机走全流程（安全兜底，避免除零） */
        SRV_PWR_CTRL_LOG_W("预偏置检测: VIN 无效 → 冷机全流程");
        return PRECHARGE_STATE_RAMP_TON;
    }

    const uint32_t ratio_permille = c->last_motor_bus_mv * 1000U / vin;

    if (ratio_permille < PWR_PRECHARGE_COLD_RATIO_PERMILLE) {
        SRV_PWR_CTRL_LOG_I("预偏置检测: 冷机 (ratio=%u‰<10%%) → 全流程",
            (unsigned)ratio_permille);
        return PRECHARGE_STATE_RAMP_TON;
    }

    if (ratio_permille >= PWR_PRECHARGE_EARLY_STEADY_PCT * 10U) {
        SRV_PWR_CTRL_LOG_I("预偏置检测: 近满 (ratio=%u‰≥95%%) → 直接稳态",
            (unsigned)ratio_permille);
        return PRECHARGE_STATE_STEADY;
    }

    /* 暖机：切入阶段四，起始 duty = ratio（钳位 100~950‰），按同斜率折算虚拟已走时间 */
    uint32_t duty = ratio_permille;
    if (duty < PWR_PRECHARGE_DUTY_P3_END) {
        duty = PWR_PRECHARGE_DUTY_P3_END;
    } else if (duty > PWR_PRECHARGE_DUTY_P4_END) {
        duty = PWR_PRECHARGE_DUTY_P4_END;
    }
    /* 阶段四斜坡 duty(t) = P3_END + (P4_END-P3_END)*t/PHASE4 → t = (duty-P3_END)*PHASE4/(P4_END-P3_END) */
    c->ramp_seed = (duty - PWR_PRECHARGE_DUTY_P3_END) * PWR_PRECHARGE_PHASE4_MS
        / (PWR_PRECHARGE_DUTY_P4_END - PWR_PRECHARGE_DUTY_P3_END);
    SRV_PWR_CTRL_LOG_I("预偏置检测: 暖机 (ratio=%u‰) → 切入阶段四 duty=%u‰",
        (unsigned)ratio_permille, (unsigned)duty);
    return PRECHARGE_STATE_RAMP_DUTY;
#else
    /* 不使用初始电压跟随：恒冷机走全流程（完整斜坡时长全部生效） */
    SRV_PWR_CTRL_LOG_D("预偏置检测: 电压跟随已禁用 → 恒冷机全流程");
    return PRECHARGE_STATE_RAMP_TON;
#endif
}

/**
 * @brief 阶段二：50kHz 恒频，脉宽（占空比）0→8‰ 线性爬升
 */
static fsm_state_t precharge_state_ramp_ton(fsm_t* fsm)
{
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);

    c->phase_elapsed_ms += c->elapsed_ms;

    precharge_phase_ramp(PWR_PRECHARGE_FREQ_LOW_HZ, PWR_PRECHARGE_FREQ_LOW_HZ,
        PWR_PRECHARGE_DUTY_P2_START, PWR_PRECHARGE_DUTY_P2_END,
        PWR_PRECHARGE_PHASE2_MS);

    if (c->phase_elapsed_ms >= PWR_PRECHARGE_PHASE2_MS) {
        return PRECHARGE_STATE_RAMP_FREQ;
    }
    return PRECHARGE_STATE_RAMP_TON;
}

/**
 * @brief 阶段三：脉宽恒（Ton≈167ns），频率 50k→600k 线性爬升
 */
static fsm_state_t precharge_state_ramp_freq(fsm_t* fsm)
{
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);

    c->phase_elapsed_ms += c->elapsed_ms;

    precharge_phase_ramp(PWR_PRECHARGE_FREQ_LOW_HZ, PWR_PRECHARGE_FREQ_HIGH_HZ,
        PWR_PRECHARGE_DUTY_P2_END, PWR_PRECHARGE_DUTY_P3_END,
        PWR_PRECHARGE_PHASE3_MS);

    if (c->phase_elapsed_ms >= PWR_PRECHARGE_PHASE3_MS) {
        return PRECHARGE_STATE_RAMP_DUTY;
    }
    return PRECHARGE_STATE_RAMP_FREQ;
}

/**
 * @brief 阶段四：600kHz 恒频，占空比 100→900‰ 线性爬升 + 电压保护
 */
static fsm_state_t precharge_state_ramp_duty(fsm_t* fsm)
{
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);

    c->phase_elapsed_ms += c->elapsed_ms;

    precharge_phase_ramp(PWR_PRECHARGE_FREQ_HIGH_HZ, PWR_PRECHARGE_FREQ_HIGH_HZ,
        PWR_PRECHARGE_DUTY_P3_END, PWR_PRECHARGE_DUTY_P4_END,
        PWR_PRECHARGE_PHASE4_MS);

    /* NO_LOAD 检测：阶段四内部 50ms（全局 100ms）后 bus_mv 仍 <1000mV → 未接负载/上电故障 */
    if (c->phase_elapsed_ms >= PWR_PRECHARGE_NO_LOAD_CHECK_MS
        && c->last_motor_bus_mv < PWR_PRECHARGE_NO_LOAD_MV) {
        c->fault = PRECHARGE_FAULT_NO_LOAD;
        SRV_PWR_CTRL_LOG_E("NO_LOAD 故障: bus_mv=%dmV < %dmV", c->last_motor_bus_mv, PWR_PRECHARGE_NO_LOAD_MV);
        return PRECHARGE_STATE_FAULT;
    }

#if PWR_PRECHARGE_EARLY_STEADY_EN
    /* 提前转稳态：输出 bus ≥ 输入母线 vin×95% 即转稳态（vin 无效时跳过，固定跑满 500ms） */
    if (c->last_vin_mv > 0
        && c->last_motor_bus_mv >= c->last_vin_mv * PWR_PRECHARGE_EARLY_STEADY_PCT / 100U) {
        SRV_PWR_CTRL_LOG_I("电压已达输入母线 95%% (bus=%umV, vin=%umV)，提前转稳态",
            (unsigned)c->last_motor_bus_mv, (unsigned)c->last_vin_mv);
        return PRECHARGE_STATE_STEADY;
    }
#endif

    if (c->phase_elapsed_ms >= PWR_PRECHARGE_PHASE4_MS) {
        return PRECHARGE_STATE_STEADY;
    }
    return PRECHARGE_STATE_RAMP_DUTY;
}

/**
 * @brief 稳态：600kHz 90% 保持输出
 */
static fsm_state_t precharge_state_steady(fsm_t* fsm)
{
    (void)fsm;
    return PRECHARGE_STATE_STEADY;
}

/**
 * @brief OCP 恢复：关输出延时 10ms 后按重试计数决定重来或锁存短路
 */
static fsm_state_t precharge_state_ocp_recover(fsm_t* fsm)
{
    precharge_ctx_t* c = (precharge_ctx_t*)fsm_user_data(fsm);

    c->phase_elapsed_ms += c->elapsed_ms;

    if (c->phase_elapsed_ms >= PWR_PRECHARGE_OCP_RECOVER_MS) {
        if (c->ocp_retry < PWR_PRECHARGE_OCP_RETRY_MAX) {
            c->ocp_retry++;
            SRV_PWR_CTRL_LOG_W("OCP 恢复: 第 %u/%u 次重试",
                (unsigned)c->ocp_retry, (unsigned)PWR_PRECHARGE_OCP_RETRY_MAX);
            return PRECHARGE_STATE_EN_CLEAR;
        }
        c->fault = PRECHARGE_FAULT_SHORT_CIRCUIT;
        SRV_PWR_CTRL_LOG_E("OCP 重试超限: 判定后级短路 (SHORT_CIRCUIT)");
        return PRECHARGE_STATE_FAULT;
    }
    return PRECHARGE_STATE_OCP_RECOVER;
}

/**
 * @brief 故障态：锁存，输出保持关闭，需 reset 退出
 */
static fsm_state_t precharge_state_fault(fsm_t* fsm)
{
    (void)fsm;
    return PRECHARGE_STATE_FAULT;
}

/**
 * @brief 线性斜坡辅助：按阶段进度输出 freq/duty（freq 起点=终点时只更新 duty）
 */
static void precharge_phase_ramp(uint32_t freq_start, uint32_t freq_end,
    uint16_t duty_start, uint16_t duty_end, uint32_t duration_ms)
{
    uint32_t t = s_precharge.phase_elapsed_ms;
    if (t > duration_ms) {
        t = duration_ms;
    }
    const uint32_t progress = (duration_ms > 0) ? (t * 1000U / duration_ms) : 1000U;

    if (freq_end > freq_start) {
        const uint32_t freq = freq_start
            + ((freq_end - freq_start) * progress) / 1000U;
        drv_pwm_set_frequency(DRV_PWM_CH_MOTOR_CHG_IN, freq);
    }

    uint16_t duty = duty_start;
    if (duty_end > duty_start) {
        duty = duty_start
            + (uint16_t)(((uint32_t)(duty_end - duty_start) * progress) / 1000U);
    }
    drv_pwm_set_duty(DRV_PWM_CH_MOTOR_CHG_IN, duty);
}
