/**
 * @file    srv_pwr_ctrl.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   电源输出监督服务实现（期望输出 + 门控使能 + 故障锁存 FSM,
 *          E1_SLAVER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_ctrl.h"

#include "drv_power.h"
#include "drv_status.h"
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

/** @brief 单路输出状态 */
typedef enum {
    SRV_PWR_RAIL_OFF,       /**< 关闭（可随时开始使能） */
    SRV_PWR_RAIL_ENABLING,  /**< 使能中：等待好状态就绪/超时 */
    SRV_PWR_RAIL_ON,        /**< 运行中：监控好状态丢失 */
    SRV_PWR_RAIL_LATCHED,   /**< 故障锁存关断（需 clear_latch 复位） */
} srv_pwr_rail_state_t;

typedef struct {
    drv_power_rail_t drv_rail;
    uint16_t enable_timeout_ms;
    uint32_t mask_bit;
    const char* name;

    srv_pwr_rail_state_t state;
    uint32_t enable_elapsed_ms; /**< ENABLING 起累计 (ms) */
    uint16_t stable_ms;         /**< 好状态连续稳定计数 (ms) */
    uint16_t loss_ms;           /**< ON 时好状态连续丢失计数 (ms) */
} srv_pwr_rail_ctx_t;

/* Private variables ---------------------------------------------------------*/

static srv_pwr_rail_ctx_t s_rails[4] = {
    { DRV_POWER_RAIL_DC24V, SRV_PWR_EN_TIMEOUT_DC24_MS, SRV_PWR_OUT_MASK_DC24V, "DC24V", SRV_PWR_RAIL_OFF, 0, 0, 0 },
    { DRV_POWER_RAIL_ISO12V, SRV_PWR_EN_TIMEOUT_ISO_MS, SRV_PWR_OUT_MASK_ISO12V, "ISO12V", SRV_PWR_RAIL_OFF, 0, 0, 0 },
    { DRV_POWER_RAIL_LSD1, SRV_PWR_EN_TIMEOUT_LSD_MS, SRV_PWR_OUT_MASK_LSD1, "LSD1", SRV_PWR_RAIL_OFF, 0, 0, 0 },
    { DRV_POWER_RAIL_LSD2, SRV_PWR_EN_TIMEOUT_LSD_MS, SRV_PWR_OUT_MASK_LSD2, "LSD2", SRV_PWR_RAIL_OFF, 0, 0, 0 },
};
#define SRV_PWR_RAIL_NUM (uint32_t)(sizeof(s_rails) / sizeof(s_rails[0]))

static uint8_t s_desired_mask;   /**< 期望输出掩码（来自 0x10 控制帧） */
static uint8_t s_latch_mask;     /**< 故障锁存掩码 */
static bool s_initialized;
static srv_pwr_voltage_cb_t s_voltage_cb;

/* Private function prototypes -----------------------------------------------*/

static bool rail_precondition_ok(uint32_t idx, const srv_pwr_voltage_t* volt, bool dc24v_on);
static bool rail_ready_ok(uint32_t idx, const srv_pwr_voltage_t* volt, bool dc24v_pgood,
    bool iso12v_pgood);
static void rail_set_off(srv_pwr_rail_ctx_t* rail);
static void rail_latch(srv_pwr_rail_ctx_t* rail);
static uint8_t get_on_mask(void);

/* Exported functions --------------------------------------------------------*/

void srv_pwr_ctrl_init(void)
{
    /* drv_power 内部自包含：输出全部关闭；drv_status 由 srv_pwr_det 初始化 */
    drv_power_init();

    s_desired_mask = 0;
    s_latch_mask = 0;
    s_voltage_cb = NULL;

    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        s_rails[i].state = SRV_PWR_RAIL_OFF;
        s_rails[i].enable_elapsed_ms = 0;
        s_rails[i].stable_ms = 0;
        s_rails[i].loss_ms = 0;
    }

    s_initialized = true;
    SRV_PWR_CTRL_LOG_I("电源输出监督服务初始化完成 (%u 路, 期望全关)", (unsigned)SRV_PWR_RAIL_NUM);
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
    srv_pwr_voltage_t volt;
    memset(&volt, 0, sizeof(volt));
    if (s_voltage_cb != NULL) {
        s_voltage_cb(&volt);
    }

    /* 2. PGOOD 原始电平（每拍直读 drv_status） */
    const bool dc24v_pgood = drv_status_read(DRV_STATUS_24V_PGD);
    const bool iso12v_pgood = drv_status_read(DRV_STATUS_ISO12V_PGD);

    const bool dc24v_on = get_on_mask() & SRV_PWR_OUT_MASK_DC24V;

    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        srv_pwr_rail_ctx_t* rail = &s_rails[i];
        const uint32_t bit = rail->mask_bit;
        const bool desired = (s_desired_mask & bit) != 0;
        const bool latched = (s_latch_mask & bit) != 0;

        switch (rail->state) {
        case SRV_PWR_RAIL_OFF: {
            if (desired && !latched
                && rail_precondition_ok(i, &volt, dc24v_on)) {
                drv_power_set(rail->drv_rail, true);
                rail->state = SRV_PWR_RAIL_ENABLING;
                rail->enable_elapsed_ms = 0;
                rail->stable_ms = 0;
                SRV_PWR_CTRL_LOG_I("输出 %s 开始使能", rail->name);
            }
            break;
        }

        case SRV_PWR_RAIL_ENABLING: {
            if (!desired || latched) {
                rail_set_off(rail);
                break;
            }
            if (!rail_precondition_ok(i, &volt, dc24v_on)) {
                /* 前提瞬时不满足（如 24V 刚掉），保持等待但不累计超时 */
                rail->stable_ms = 0;
                break;
            }

            if (rail_ready_ok(i, &volt, dc24v_pgood, iso12v_pgood)) {
                rail->stable_ms += elapsed_ms;
                if (rail->stable_ms >= SRV_PWR_READY_DEBOUNCE_MS) {
                    rail->state = SRV_PWR_RAIL_ON;
                    rail->enable_elapsed_ms = 0;
                    rail->stable_ms = 0;
                    rail->loss_ms = 0;
                    SRV_PWR_CTRL_LOG_I("输出 %s 已就绪 (ON)", rail->name);
                }
            } else {
                rail->stable_ms = 0;
                rail->enable_elapsed_ms += elapsed_ms;
                if (rail->enable_elapsed_ms >= rail->enable_timeout_ms) {
                    SRV_PWR_CTRL_LOG_E("输出 %s 使能超时 (%ums) 未就绪 → 故障锁存关断",
                        rail->name, (unsigned)rail->enable_timeout_ms);
                    rail_latch(rail);
                }
            }
            break;
        }

        case SRV_PWR_RAIL_ON: {
            if (!desired || latched) {
                rail_set_off(rail);
                break;
            }

            /* 运行期好状态丢失监控（防误关断去抖） */
            if (rail_ready_ok(i, &volt, dc24v_pgood, iso12v_pgood)) {
                rail->loss_ms = 0;
            } else {
                rail->loss_ms += elapsed_ms;
                if (rail->loss_ms >= SRV_PWR_LOSS_DEBOUNCE_MS) {
                    SRV_PWR_CTRL_LOG_E("输出 %s 好状态持续丢失 (%ums) → 故障锁存关断",
                        rail->name, (unsigned)SRV_PWR_LOSS_DEBOUNCE_MS);
                    rail_latch(rail);
                }
            }
            break;
        }

        case SRV_PWR_RAIL_LATCHED: {
            /* 等待 clear_latch 或人工关断确认；仅保持 */
            break;
        }

        default:
            break;
        }
    }
}

void srv_pwr_ctrl_request_outputs(uint8_t mask)
{
    if (!s_initialized) {
        return;
    }

    s_desired_mask = (uint8_t)(mask & SRV_PWR_OUT_MASK_ALL);

    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        srv_pwr_rail_ctx_t* rail = &s_rails[i];
        const uint32_t bit = rail->mask_bit;
        const bool want_off = (s_desired_mask & bit) == 0;

        if (want_off) {
            /* 人工关断确认：顺带清该路锁存（避免主机不依赖 0x11） */
            if ((s_latch_mask & bit) != 0) {
                s_latch_mask &= (uint8_t)~bit;
                SRV_PWR_CTRL_LOG_I("输出 %s 人工关断，清除故障锁存", rail->name);
            }
            if (rail->state == SRV_PWR_RAIL_ENABLING || rail->state == SRV_PWR_RAIL_ON) {
                rail_set_off(rail);
            } else if (rail->state == SRV_PWR_RAIL_LATCHED) {
                rail->state = SRV_PWR_RAIL_OFF;
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
        srv_pwr_rail_ctx_t* rail = &s_rails[i];
        if ((mask & rail->mask_bit) != 0) {
            SRV_PWR_CTRL_LOG_E("外部策略锁存输出 %s", rail->name);
            rail_latch(rail);
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
        if (s_rails[i].state == SRV_PWR_RAIL_LATCHED) {
            s_rails[i].state = SRV_PWR_RAIL_OFF;
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
        rail_set_off(&s_rails[i]);
    }
    SRV_PWR_CTRL_LOG_E("紧急关断: 全部输出关闭（锁存保留，需 clear_latch 恢复）");
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 输出 i 的使能前提（依赖链）：DC24V 需 AUX 母线存在；ISO12V/LSD 需 DC24V 已 ON
 */
static bool rail_precondition_ok(uint32_t idx, const srv_pwr_voltage_t* volt, bool dc24v_on)
{
    switch (idx) {
    case 0: /* DC24V */
        return volt->valid && volt->aux_mv >= SRV_PWR_AUX_PRESENT_MV;
    case 1: /* ISO12V */
        return dc24v_on;
    case 2: /* LSD1 */
    case 3: /* LSD2 */
        return dc24v_on && volt->valid;
    default:
        return false;
    }
}

/**
 * @brief 输出 i 的好状态：DC24V → 24V PGOOD；ISO12V → 12V_ISO PGOOD；LSD → 节点电压低
 */
static bool rail_ready_ok(uint32_t idx, const srv_pwr_voltage_t* volt, bool dc24v_pgood,
    bool iso12v_pgood)
{
    switch (idx) {
    case 0: /* DC24V */
        return dc24v_pgood;
    case 1: /* ISO12V */
        return iso12v_pgood;
    case 2: /* LSD1 */
        return volt->valid && volt->lsd1_mv < SRV_PWR_LSD_NODE_ON_MAX_MV;
    case 3: /* LSD2 */
        return volt->valid && volt->lsd2_mv < SRV_PWR_LSD_NODE_ON_MAX_MV;
    default:
        return false;
    }
}

static void rail_set_off(srv_pwr_rail_ctx_t* rail)
{
    drv_power_set(rail->drv_rail, false);
    rail->state = SRV_PWR_RAIL_OFF;
    rail->enable_elapsed_ms = 0;
    rail->stable_ms = 0;
    rail->loss_ms = 0;
}

static void rail_latch(srv_pwr_rail_ctx_t* rail)
{
    drv_power_set(rail->drv_rail, false);
    s_latch_mask |= (uint8_t)rail->mask_bit;
    rail->state = SRV_PWR_RAIL_LATCHED;
    rail->enable_elapsed_ms = 0;
    rail->stable_ms = 0;
    rail->loss_ms = 0;
}

static uint8_t get_on_mask(void)
{
    uint8_t mask = 0;
    for (uint32_t i = 0; i < SRV_PWR_RAIL_NUM; i++) {
        if (s_rails[i].state == SRV_PWR_RAIL_ON) {
            mask |= (uint8_t)s_rails[i].mask_bit;
        }
    }
    return mask;
}
