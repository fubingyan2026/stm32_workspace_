/**
 * @file    srv_pwr_ctrl.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-05
 * @brief   电源输出监督服务实现（期望输出 + 门控使能 + 故障锁存,
 *          4 路输出各持独立 fsm_t 实例，E1_SLAVER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_ctrl.h"

#include "drv_power.h"
#include "drv_status.h"
#include "fsm.h"
#include "log.h"

#include <string.h>

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

/* Private types -------------------------------------------------------------*/

/** @brief 单路输出状态（fsm 状态，序号也作 handler 表下标） */
typedef enum {
    SRV_PWR_RAIL_STATE_OFF = 0, /**< 关闭（可随时开始使能） */
    SRV_PWR_RAIL_STATE_ENABLING, /**< 使能中：等待好状态就绪/超时 */
    SRV_PWR_RAIL_STATE_ON, /**< 运行中：监控好状态丢失 */
    SRV_PWR_RAIL_STATE_LATCHED, /**< 故障锁存关断（需 clear_latch 复位） */

    SRV_PWR_RAIL_STATE_COUNT,
} srv_pwr_rail_state_t;

/** @brief 单路输出配置（只读表，顺序 = drv_power_rail_t = 协议位顺序） */
typedef struct {
    drv_power_rail_t drv_rail; /**< 实际驱动轨 */
    uint16_t enable_timeout_ms; /**< 使能超时 (ms) */
    uint32_t mask_bit; /**< 掩码位（协议 ctrl/status） */
    const char* name; /**< 轨名称（日志） */
} srv_pwr_rail_cfg_t;

/** @brief 单路输出运行上下文（config-in-context：内嵌 fsm 实例） */
typedef struct {
    uint8_t idx; /**< 配置表下标（对应 drv rail / 协议位） */
    fsm_t fsm; /**< 本路独立 fsm 实例 */

    uint32_t settle_elapsed_ms; /**< ENABLING 内 settle 窗口累计 (ms) */
    uint32_t enable_elapsed_ms; /**< settle 结束后起累计超时 (ms) */
    uint16_t stable_ms; /**< 好状态连续稳定计数 (ms) */
    uint16_t loss_ms; /**< ON 时好状态连续丢失计数 (ms) */
} srv_pwr_rail_ctx_t;

/** @brief 每拍步进快照（所有轨共享同一拍采样，保持判定一致性） */
typedef struct {
    srv_pwr_voltage_t volt; /**< 母线电压快照（回调注入） */
    bool dc24v_pgood; /**< 24V PGOOD 原始电平 */
    bool iso12v_pgood; /**< 12V_ISO PGOOD 原始电平 */
    bool dc24v_on; /**< 本拍开始时 DC24V 是否处于 ON（依赖链门控） */
    uint16_t elapsed_ms; /**< 本拍经过时间 (ms) */
} srv_pwr_step_t;

/* Private constants ---------------------------------------------------------*/

#define SRV_PWR_RAIL_NUM (4U)

/* Private variables ---------------------------------------------------------*/

/** @brief 输出轨配置表 */
static const srv_pwr_rail_cfg_t s_rail_cfgs[SRV_PWR_RAIL_NUM] = {
    { DRV_POWER_RAIL_DC24V, SRV_PWR_EN_TIMEOUT_DC24_MS, SRV_PWR_OUT_MASK_DC24V, "DC24V" },
    { DRV_POWER_RAIL_ISO12V, SRV_PWR_EN_TIMEOUT_ISO_MS, SRV_PWR_OUT_MASK_ISO12V, "ISO12V" },
    { DRV_POWER_RAIL_LSD1, SRV_PWR_EN_TIMEOUT_LSD_MS, SRV_PWR_OUT_MASK_LSD1, "LSD1" },
    { DRV_POWER_RAIL_LSD2, SRV_PWR_EN_TIMEOUT_LSD_MS, SRV_PWR_OUT_MASK_LSD2, "LSD2" },
};

/** @brief 输出轨运行上下文（每路一个独立 fsm 实例） */
static srv_pwr_rail_ctx_t s_rails[SRV_PWR_RAIL_NUM];

static fsm_handler_t s_rail_handlers[SRV_PWR_RAIL_STATE_COUNT];
static fsm_guard_t s_rail_transitions[SRV_PWR_RAIL_STATE_COUNT * SRV_PWR_RAIL_STATE_COUNT];

static const char* s_rail_state_names[SRV_PWR_RAIL_STATE_COUNT] = {
    "OFF",
    "ENABLING",
    "ON",
    "LATCHED",
};

static uint8_t s_desired_mask; /**< 期望输出掩码（来自 0x04 控制帧） */
static uint8_t s_latch_mask; /**< 故障锁存掩码 */
static bool s_initialized;
static srv_pwr_voltage_cb_t s_voltage_cb;
static srv_pwr_step_t s_step; /**< 每拍步进快照 */

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t rail_state_off(fsm_t* ctx);
static fsm_state_t rail_state_enabling(fsm_t* ctx);
static fsm_state_t rail_state_on(fsm_t* ctx);
static fsm_state_t rail_state_latched(fsm_t* ctx);

static bool rail_precondition_ok(uint32_t idx);
static bool rail_ready_ok(uint32_t idx);
static void rail_reset_counters(srv_pwr_rail_ctx_t* rail);
static void rail_turn_off(srv_pwr_rail_ctx_t* rail);
static void rail_do_latch(srv_pwr_rail_ctx_t* rail);
static uint8_t get_on_mask(void);

/* Exported functions --------------------------------------------------------*/

void srv_pwr_ctrl_init(void)
{
    /* drv_power 内部自包含：输出全部关闭；drv_status 由 srv_pwr_det 初始化 */
    drv_power_init();

    s_desired_mask = 0;
    s_latch_mask = 0;
    s_voltage_cb = NULL;
    memset(&s_step, 0, sizeof(s_step));

    /* fsm handler 表 */
    s_rail_handlers[SRV_PWR_RAIL_STATE_OFF] = rail_state_off;
    s_rail_handlers[SRV_PWR_RAIL_STATE_ENABLING] = rail_state_enabling;
    s_rail_handlers[SRV_PWR_RAIL_STATE_ON] = rail_state_on;
    s_rail_handlers[SRV_PWR_RAIL_STATE_LATCHED] = rail_state_latched;

    /* 全连通转换矩阵（实际边由 handler 返回值触发，fsm 负责切换/进入回调） */
    fsm_config_t fsm_tpl = {
        .handlers = s_rail_handlers,
        .transitions = s_rail_transitions,
        .state_count = SRV_PWR_RAIL_STATE_COUNT,
        .state_names = s_rail_state_names,
        .user_data = NULL,
    };
    fsm_fill(&fsm_tpl, fsm_always_true);

    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        srv_pwr_rail_ctx_t* rail = &s_rails[i];
        memset(rail, 0, sizeof(*rail));
        rail->idx = (uint8_t)i;

        fsm_config_t fsm_cfg = fsm_tpl;
        fsm_cfg.user_data = rail;
        (void)fsm_init(&rail->fsm, SRV_PWR_RAIL_STATE_OFF, &fsm_cfg);
    }

    s_initialized = true;
    SRV_PWR_CTRL_LOG_I("电源输出监督服务初始化完成 (%u 路, 期望全关)",
        (unsigned)SRV_PWR_RAIL_NUM);

    srv_pwr_ctrl_request_outputs(0xFF);
}

void srv_pwr_ctrl_set_voltage_cb(srv_pwr_voltage_cb_t cb)
{
    s_voltage_cb = cb;
}

void srv_pwr_ctrl_step(uint16_t elapsed_ms)
{
    if (!s_initialized) {
        return;
    }

    /* 1. 母线电压快照（task 层从 srv_adc 读入） */
    memset(&s_step.volt, 0, sizeof(s_step.volt));
    if (s_voltage_cb != NULL) {
        s_voltage_cb(&s_step.volt);
    }

    /* 2. PGOOD 原始电平（每拍直读 drv_status） */
    s_step.dc24v_pgood = drv_status_read(DRV_STATUS_24V_PGD);
    s_step.iso12v_pgood = drv_status_read(DRV_STATUS_ISO12V_PGD);

    /* 3. DC24V ON 快照：拍首判定，保证本拍内依赖链门控一致 */
    s_step.dc24v_on = (get_on_mask() & SRV_PWR_OUT_MASK_DC24V) != 0;
    s_step.elapsed_ms = elapsed_ms;

    /* 4. 逐路推进 fsm */
    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        (void)fsm_step(&s_rails[i].fsm);
    }
}

void srv_pwr_ctrl_request_outputs(uint8_t mask)
{
    if (!s_initialized) {
        return;
    }

    s_desired_mask = (uint8_t)(mask & SRV_PWR_OUT_MASK_ALL);

    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        const srv_pwr_rail_cfg_t* cfg = &s_rail_cfgs[i];
        srv_pwr_rail_ctx_t* rail = &s_rails[i];
        const uint32_t bit = cfg->mask_bit;
        const bool want_off = (s_desired_mask & bit) == 0;

        if (want_off) {
            /* 人工关断确认：顺带清该路锁存（避免主机不依赖 0x05） */
            if ((s_latch_mask & bit) != 0) {
                s_latch_mask &= (uint8_t)~bit;
                SRV_PWR_CTRL_LOG_I("输出 %s 人工关断，清除故障锁存", cfg->name);
            }

            const fsm_state_t state = fsm_current_state(&rail->fsm);
            if (state == SRV_PWR_RAIL_STATE_ENABLING
                || state == SRV_PWR_RAIL_STATE_ON) {
                rail_turn_off(rail);
                (void)fsm_goto(&rail->fsm, SRV_PWR_RAIL_STATE_OFF);
            } else if (state == SRV_PWR_RAIL_STATE_LATCHED) {
                (void)fsm_goto(&rail->fsm, SRV_PWR_RAIL_STATE_OFF);
            }
        }
    }
}

uint8_t srv_pwr_ctrl_get_desired(void)
{
    return s_initialized ? s_desired_mask : 0;
}

uint8_t srv_pwr_ctrl_get_on_outputs(void)
{
    return s_initialized ? get_on_mask() : 0;
}

void srv_pwr_ctrl_latch_outputs(uint8_t mask)
{
    if (!s_initialized) {
        return;
    }

    mask &= (uint8_t)SRV_PWR_OUT_MASK_ALL;
    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        const srv_pwr_rail_cfg_t* cfg = &s_rail_cfgs[i];
        srv_pwr_rail_ctx_t* rail = &s_rails[i];
        if ((mask & cfg->mask_bit) != 0) {
            SRV_PWR_CTRL_LOG_E("外部策略锁存输出 %s", cfg->name);
            rail_do_latch(rail);
            (void)fsm_goto(&rail->fsm, SRV_PWR_RAIL_STATE_LATCHED);
        }
    }
}

uint8_t srv_pwr_ctrl_get_latch_mask(void)
{
    return s_initialized ? s_latch_mask : 0;
}

bool srv_pwr_ctrl_is_any_latched(void)
{
    return s_initialized && (s_latch_mask != 0);
}

void srv_pwr_ctrl_clear_latch(void)
{
    if (!s_initialized) {
        return;
    }

    s_latch_mask = 0;
    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        srv_pwr_rail_ctx_t* rail = &s_rails[i];
        if (fsm_current_state(&rail->fsm) == SRV_PWR_RAIL_STATE_LATCHED) {
            (void)fsm_goto(&rail->fsm, SRV_PWR_RAIL_STATE_OFF);
        }
    }
    SRV_PWR_CTRL_LOG_I("故障锁存已清除（期望保持，将按需自动重试）");
}

void srv_pwr_ctrl_emergency_off(void)
{
    if (!s_initialized) {
        return;
    }

    s_desired_mask = 0;
    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        rail_turn_off(&s_rails[i]);
        (void)fsm_goto(&s_rails[i].fsm, SRV_PWR_RAIL_STATE_OFF);
    }
    SRV_PWR_CTRL_LOG_E("紧急关断: 全部输出关闭（锁存保留，需 clear_latch 恢复）");
}

/* Private functions ---------------------------------------------------------*/

/* ---- fsm 状态 handler ---- */

/** @brief OFF：期望+前提满足即使能进入 ENABLING */
static fsm_state_t rail_state_off(fsm_t* ctx)
{
    srv_pwr_rail_ctx_t* rail = (srv_pwr_rail_ctx_t*)fsm_user_data(ctx);
    const srv_pwr_rail_cfg_t* cfg = &s_rail_cfgs[rail->idx];
    const uint32_t bit = cfg->mask_bit;

    if ((s_desired_mask & bit) != 0
        && (s_latch_mask & bit) == 0
        && rail_precondition_ok(rail->idx)) {
        drv_power_set(cfg->drv_rail, true);
        rail_reset_counters(rail);
        SRV_PWR_CTRL_LOG_I("输出 %s 开始使能", cfg->name);
        return SRV_PWR_RAIL_STATE_ENABLING;
    }
    return SRV_PWR_RAIL_STATE_OFF;
}

/** @brief ENABLING：settle 窗口 → 好状态去抖 → ON；前提瞬失保持等待；超时 → 故障锁存 */
static fsm_state_t rail_state_enabling(fsm_t* ctx)
{
    srv_pwr_rail_ctx_t* rail = (srv_pwr_rail_ctx_t*)fsm_user_data(ctx);
    const srv_pwr_rail_cfg_t* cfg = &s_rail_cfgs[rail->idx];
    const uint32_t bit = cfg->mask_bit;

    if ((s_desired_mask & bit) == 0 || (s_latch_mask & bit) != 0) {
        rail_turn_off(rail);
        return SRV_PWR_RAIL_STATE_OFF;
    }

    /* 使能稳定窗口：使能后头 N ms 忽略 PGOOD 电平（启动建立暂态），
     * 不计稳定、不计超时，窗口结束才进入正常判定 */
    if (rail->settle_elapsed_ms < SRV_PWR_EN_SETTLE_MS) {
        rail->settle_elapsed_ms += s_step.elapsed_ms;
        rail->stable_ms = 0;
        rail->enable_elapsed_ms = 0;
        return SRV_PWR_RAIL_STATE_ENABLING;
    }

    if (!rail_precondition_ok(rail->idx)) {
        /* 前提瞬时不满足（如 24V 刚掉），保持等待但不累计超时 */
        rail->stable_ms = 0;
        return SRV_PWR_RAIL_STATE_ENABLING;
    }

    if (rail_ready_ok(rail->idx)) {
        rail->stable_ms += s_step.elapsed_ms;
        if (rail->stable_ms >= SRV_PWR_READY_DEBOUNCE_MS) {
            rail_reset_counters(rail);
            SRV_PWR_CTRL_LOG_I("输出 %s 已就绪 (ON)", cfg->name);
            return SRV_PWR_RAIL_STATE_ON;
        }
    } else {
        rail->stable_ms = 0;
        rail->enable_elapsed_ms += s_step.elapsed_ms;
        if (rail->enable_elapsed_ms >= cfg->enable_timeout_ms) {
            SRV_PWR_CTRL_LOG_E("输出 %s 使能超时 (%ums) 未就绪 → 故障锁存关断",
                cfg->name, (unsigned)cfg->enable_timeout_ms);
            rail_do_latch(rail);
            return SRV_PWR_RAIL_STATE_LATCHED;
        }
    }
    return SRV_PWR_RAIL_STATE_ENABLING;
}

/** @brief ON：运行期好状态丢失去抖（防误关断）→ 故障锁存 */
static fsm_state_t rail_state_on(fsm_t* ctx)
{
    srv_pwr_rail_ctx_t* rail = (srv_pwr_rail_ctx_t*)fsm_user_data(ctx);
    const srv_pwr_rail_cfg_t* cfg = &s_rail_cfgs[rail->idx];
    const uint32_t bit = cfg->mask_bit;

    if ((s_desired_mask & bit) == 0 || (s_latch_mask & bit) != 0) {
        rail_turn_off(rail);
        return SRV_PWR_RAIL_STATE_OFF;
    }

    if (rail_ready_ok(rail->idx)) {
        rail->loss_ms = 0;
    } else {
        rail->loss_ms += s_step.elapsed_ms;
        if (rail->loss_ms >= SRV_PWR_LOSS_DEBOUNCE_MS) {
            SRV_PWR_CTRL_LOG_E("输出 %s 好状态持续丢失 (%ums) → 故障锁存关断",
                cfg->name, (unsigned)SRV_PWR_LOSS_DEBOUNCE_MS);
            rail_do_latch(rail);
            return SRV_PWR_RAIL_STATE_LATCHED;
        }
    }
    return SRV_PWR_RAIL_STATE_ON;
}

/** @brief LATCHED：等待 clear_latch 或人工关断确认；仅保持 */
static fsm_state_t rail_state_latched(fsm_t* ctx)
{
    (void)ctx;
    return SRV_PWR_RAIL_STATE_LATCHED;
}

/* ---- 内部工具 ---- */

/**
 * @brief 输出 i 的使能前提（依赖链）：DC24V 需 AUX 母线存在；ISO12V/LSD 需 DC24V 已 ON
 */
static bool rail_precondition_ok(uint32_t idx)
{
    switch (idx) {
    case 0: /* DC24V */
        return s_step.volt.valid && s_step.volt.aux_mv >= SRV_PWR_AUX_PRESENT_MV;
    case 1: /* ISO12V */
        return s_step.dc24v_on;
    case 2: /* LSD1 */
    case 3: /* LSD2 */
        return s_step.dc24v_on && s_step.volt.valid;
    default:
        return false;
    }
}

/**
 * @brief 输出 i 的好状态：DC24V → 24V PGOOD；ISO12V → 12V_ISO PGOOD；LSD → 节点电压低
 */
static bool rail_ready_ok(uint32_t idx)
{
    switch (idx) {
    case 0: /* DC24V */
        return s_step.dc24v_pgood;
    case 1: /* ISO12V */
        return s_step.iso12v_pgood;
    case 2: /* LSD1 */
        return s_step.volt.valid && s_step.volt.lsd1_mv < SRV_PWR_LSD_NODE_ON_MAX_MV;
    case 3: /* LSD2 */
        return s_step.volt.valid && s_step.volt.lsd2_mv < SRV_PWR_LSD_NODE_ON_MAX_MV;
    default:
        return false;
    }
}

static void rail_reset_counters(srv_pwr_rail_ctx_t* rail)
{
    rail->settle_elapsed_ms = 0;
    rail->enable_elapsed_ms = 0;
    rail->stable_ms = 0;
    rail->loss_ms = 0;
}

/** @brief 关闭单路输出（drv 关断 + 计数清零，不改变 fsm 状态） */
static void rail_turn_off(srv_pwr_rail_ctx_t* rail)
{
    drv_power_set(s_rail_cfgs[rail->idx].drv_rail, false);
    rail_reset_counters(rail);
}

/** @brief 故障锁存单路输出（drv 关断 + 置锁存位 + 计数清零，不改变 fsm 状态） */
static void rail_do_latch(srv_pwr_rail_ctx_t* rail)
{
    drv_power_set(s_rail_cfgs[rail->idx].drv_rail, false);
    s_latch_mask |= (uint8_t)s_rail_cfgs[rail->idx].mask_bit;
    rail_reset_counters(rail);
}

static uint8_t get_on_mask(void)
{
    uint8_t mask = 0;
    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        if (fsm_current_state(&s_rails[i].fsm) == SRV_PWR_RAIL_STATE_ON) {
            mask |= (uint8_t)s_rail_cfgs[i].mask_bit;
        }
    }
    return mask;
}
