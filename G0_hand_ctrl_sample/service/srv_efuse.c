/**
 * @file    srv_efuse.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-08-31
 * @brief   24V eFuse 故障保护服务实现（fsm 状态机）
 * @attention
 *
 * 使用 public_layer fsm 库实现状态机：
 *   OFF(0) — SHDN=0 关断
 *   STARTUP(1) — SHDN=1 等待 PGOOD（500ms 超时判故障）
 *   RUN(2) — PGOOD=1 且 FLT_N=1 正常输出
 *   FAULT(3) — 故障锁存（SHDN=0，不自动重试）
 *
 * 状态副作用集中在 entry 回调：进 OFF/FAULT 关断、进 STARTUP 使能并计时、
 * 进 FAULT 计数并区分上电/运行故障。
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_efuse.h"

#include "drv_efuse.h"
#include "drv_systick.h"
#include "fsm.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_EFUSE_LOG_ENABLE 1

#if SRV_EFUSE_LOG_ENABLE
#define SRV_EFUSE_LOG_E(...) LOG_E("srv_efuse", __VA_ARGS__)
#define SRV_EFUSE_LOG_W(...) LOG_W("srv_efuse", __VA_ARGS__)
#define SRV_EFUSE_LOG_I(...) LOG_I("srv_efuse", __VA_ARGS__)
#define SRV_EFUSE_LOG_D(...) LOG_D("srv_efuse", __VA_ARGS__)
#else
#define SRV_EFUSE_LOG_E(...) ((void)0)
#define SRV_EFUSE_LOG_W(...) ((void)0)
#define SRV_EFUSE_LOG_I(...) ((void)0)
#define SRV_EFUSE_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 上电等待 PGOOD 超时 (ms)，超时判故障 */
#define SRV_EFUSE_STARTUP_TIMEOUT_MS (500U)

/** @brief 使能后到开始检测 PGOOD/FLT 的延时 (ms)，避开上电瞬间 */
#define SRV_EFUSE_DETECT_DELAY_MS (100U)

/** @brief RUN 中 PGOOD 跌落去抖时间 (ms)：持续跌落超过此值才关断 */
#define SRV_EFUSE_PGOOD_DROP_SHUTDOWN_MS (100U)

/* Private variables ---------------------------------------------------------*/

static fsm_t s_fsm;
static fsm_handler_t s_handlers[SRV_EFUSE_STATE_MAX];
static fsm_guard_t s_transitions[SRV_EFUSE_STATE_MAX * SRV_EFUSE_STATE_MAX];
static const char* s_state_names[SRV_EFUSE_STATE_MAX] = {
    "OFF",
    "STARTUP",
    "RUN",
    "FAULT",
};

static bool s_enable_request;
static uint32_t s_startup_t0;
static uint32_t s_pgood_drop_t0; /**< RUN 中 PGOOD 跌落起始时间戳 (ms)，0=未跌落 */
static uint32_t s_fault_count;
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t efuse_off_handler(fsm_t* ctx);

static fsm_state_t efuse_startup_handler(fsm_t* ctx);

static fsm_state_t efuse_run_handler(fsm_t* ctx);

static fsm_state_t efuse_fault_handler(fsm_t* ctx);

static void efuse_on_entry(fsm_t* ctx, fsm_state_t state);

static void efuse_set_output(bool on);

/* Exported functions --------------------------------------------------------*/

void srv_efuse_init(void)
{
    memset(s_handlers, 0, sizeof(s_handlers));
    memset(s_transitions, 0, sizeof(s_transitions));

    s_handlers[SRV_EFUSE_STATE_OFF] = efuse_off_handler;
    s_handlers[SRV_EFUSE_STATE_STARTUP] = efuse_startup_handler;
    s_handlers[SRV_EFUSE_STATE_RUN] = efuse_run_handler;
    s_handlers[SRV_EFUSE_STATE_FAULT] = efuse_fault_handler;

    fsm_config_t fsm_cfg = {
        .handlers = s_handlers,
        .transitions = s_transitions,
        .state_count = SRV_EFUSE_STATE_MAX,
        .entry_cb = efuse_on_entry,
        .exit_cb = NULL,
        .state_names = s_state_names,
        .user_data = NULL,
    };
    fsm_fill(&fsm_cfg, fsm_always_true); /* 全连通：转换合法性由 handler 控制 */
    (void)fsm_init(&s_fsm, SRV_EFUSE_STATE_OFF, &fsm_cfg);

    s_enable_request = false;
    s_fault_count = 0;
    s_initialized = true;

    /* 安全侧：上电默认打开 24V 输出 */
    efuse_set_output(false);

    srv_efuse_enable();
    SRV_EFUSE_LOG_I("eFuse 故障保护初始化完成 (初始 OFF, 启动超时=%ums)",
        (unsigned)SRV_EFUSE_STARTUP_TIMEOUT_MS);
}

void srv_efuse_step(uint16_t elapsed_ms)
{
    (void)elapsed_ms;

    if (!s_initialized) {
        return;
    }

    (void)fsm_step(&s_fsm);
}

void srv_efuse_enable(void)
{
    if (!s_initialized) {
        return;
    }

    if (fsm_current_state(&s_fsm) == SRV_EFUSE_STATE_FAULT) {
        /* 故障锁存下不允许直接使能，需先 disable 复位 */
        SRV_EFUSE_LOG_W("故障锁存中，禁止使能（请先 disable 复位）");
        return;
    }

    s_enable_request = true;
    SRV_EFUSE_LOG_I("请求开启 24V 输出");
}

void srv_efuse_disable(void)
{
    if (!s_initialized) {
        return;
    }

    s_enable_request = false;
    s_fault_count = 0;
    efuse_set_output(true);
    (void)fsm_goto(&s_fsm, SRV_EFUSE_STATE_OFF);
    SRV_EFUSE_LOG_I("请求关断 24V 输出（故障锁存已清除）");
}

srv_efuse_state_t srv_efuse_get_state(void)
{
    if (!s_initialized) {
        return SRV_EFUSE_STATE_OFF;
    }
    return (srv_efuse_state_t)fsm_current_state(&s_fsm);
}

bool srv_efuse_is_power_ok(void)
{
    return srv_efuse_get_state() == SRV_EFUSE_STATE_RUN;
}

bool srv_efuse_is_fault(void)
{
    return srv_efuse_get_state() == SRV_EFUSE_STATE_FAULT;
}

uint32_t srv_efuse_get_fault_count(void)
{
    return s_fault_count;
}

/* Private functions ---------------------------------------------------------*/

/* ===== FSM handlers ===== */

/**
 * @brief OFF 状态处理：有待使能请求 → STARTUP
 */
static fsm_state_t efuse_off_handler(fsm_t* ctx)
{
    if (s_enable_request) {
        s_enable_request = false;
        return SRV_EFUSE_STATE_STARTUP;
    }
    return SRV_EFUSE_STATE_OFF;
}

/**
 * @brief STARTUP 状态处理：先等待检测延时，再判故障 → FAULT；PGOOD → RUN；超时 → FAULT
 */
static fsm_state_t efuse_startup_handler(fsm_t* ctx)
{
    const uint32_t elapsed = millis() - s_startup_t0;

    /* 使能后先等待 SRV_EFUSE_DETECT_DELAY_MS，期间不判 PGOOD/FLT，避开上电瞬间 */
    if (elapsed < SRV_EFUSE_DETECT_DELAY_MS) {
        return SRV_EFUSE_STATE_STARTUP;
    }

    if (drv_efuse_read(DRV_EFUSE_FLT_N)) {
        return SRV_EFUSE_STATE_FAULT;
    }
    if (drv_efuse_read(DRV_EFUSE_PGOOD)) {
        return SRV_EFUSE_STATE_RUN;
    }
    if (elapsed >= SRV_EFUSE_STARTUP_TIMEOUT_MS) {
        return SRV_EFUSE_STATE_FAULT;
    }
    return SRV_EFUSE_STATE_STARTUP;
}

/**
 * @brief RUN 状态处理：
 *        - FLT_N=0 → 立即 FAULT
 *        - PGOOD 跌落去抖：持续 > SRV_EFUSE_PGOOD_DROP_SHUTDOWN_MS 才判故障
 */
static fsm_state_t efuse_run_handler(fsm_t* ctx)
{
    if (drv_efuse_read(DRV_EFUSE_FLT_N)) {
        return SRV_EFUSE_STATE_FAULT;
    }

    if (drv_efuse_read(DRV_EFUSE_PGOOD)) {
        s_pgood_drop_t0 = 0; /* PGOOD 恢复，清除跌落计时 */
        return SRV_EFUSE_STATE_RUN;
    }

    /* PGOOD 跌落但 FLT_N=1：启动/累计跌落计时，超过阈值才关断 */
    if (s_pgood_drop_t0 == 0U) {
        s_pgood_drop_t0 = millis();
    } else if ((uint32_t)(millis() - s_pgood_drop_t0)
        >= SRV_EFUSE_PGOOD_DROP_SHUTDOWN_MS) {
        return SRV_EFUSE_STATE_FAULT;
    }

    return SRV_EFUSE_STATE_RUN;
}

/**
 * @brief FAULT 状态处理：锁存，不自动重试
 */
static fsm_state_t efuse_fault_handler(fsm_t* ctx)
{
    return SRV_EFUSE_STATE_FAULT;
}

/* ===== 状态副作用 ===== */

/**
 * @brief 状态进入回调：执行输出/计时/计数等副作用
 */
static void efuse_on_entry(fsm_t* ctx, fsm_state_t state)
{
    switch ((srv_efuse_state_t)state) {
    case SRV_EFUSE_STATE_OFF:
        efuse_set_output(false);
        s_pgood_drop_t0 = 0;
        break;

    case SRV_EFUSE_STATE_STARTUP:
        efuse_set_output(true);
        s_startup_t0 = millis();
        s_pgood_drop_t0 = 0;
        break;

    case SRV_EFUSE_STATE_RUN:
        /* 输出保持使能，无需额外动作 */
        s_pgood_drop_t0 = 0;
        break;

    case SRV_EFUSE_STATE_FAULT:
        efuse_set_output(false);
        s_fault_count++;
        /* 区分上电期间故障 / 运行中故障（entry 时 ctx->last_state 为前一个状态） */
        if (ctx->last_state == SRV_EFUSE_STATE_STARTUP) {
            SRV_EFUSE_LOG_E("上电期间故障 (FLT_N=0)，关断 24V (故障数=%lu)",
                (unsigned long)s_fault_count);
        } else if (drv_efuse_read(DRV_EFUSE_FLT_N)) {
            SRV_EFUSE_LOG_E("运行中故障 (FLT_N=0)，关断 24V (故障数=%lu)",
                (unsigned long)s_fault_count);
        } else {
            SRV_EFUSE_LOG_E("运行中 PGOOD 持续跌落 (>%ums)，关断 24V (故障数=%lu)",
                (unsigned)SRV_EFUSE_PGOOD_DROP_SHUTDOWN_MS,
                (unsigned long)s_fault_count);
        }
        s_pgood_drop_t0 = 0;
        break;

    default:
        break;
    }

    SRV_EFUSE_LOG_I("状态切换 → %s (SHDN=%u PGOOD=%u FLT_N=%u)",
        fsm_name(ctx, state),
        (unsigned)drv_efuse_read(DRV_EFUSE_SHDN),
        (unsigned)drv_efuse_read(DRV_EFUSE_PGOOD),
        (unsigned)drv_efuse_read(DRV_EFUSE_FLT_N));
}

static void efuse_set_output(bool on)
{
    drv_efuse_set_shdn(on);
}
