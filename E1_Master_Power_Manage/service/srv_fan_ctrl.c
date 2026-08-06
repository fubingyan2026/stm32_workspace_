/**
 * @file    srv_fan_ctrl.c
 * @author  maximillian
 * @version V1.2.0
 * @date    2026-08-05
 * @brief   风扇控制服务实现（每台风扇一个 FSM：自检/运行/故障）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_fan_ctrl.h"

#include <string.h>

#include "drv_fan.h"
#include "drv_systick.h"
#include "filter.h"
#include "fsm.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_FAN_CTRL_LOG_ENABLE 0

#if SRV_FAN_CTRL_LOG_ENABLE
#define SRV_FAN_CTRL_LOG_E(...) LOG_E("srv_fan_ctrl", __VA_ARGS__)
#define SRV_FAN_CTRL_LOG_W(...) LOG_W("srv_fan_ctrl", __VA_ARGS__)
#define SRV_FAN_CTRL_LOG_I(...) LOG_I("srv_fan_ctrl", __VA_ARGS__)
#define SRV_FAN_CTRL_LOG_D(...) LOG_D("srv_fan_ctrl", __VA_ARGS__)
#else
#define SRV_FAN_CTRL_LOG_E(...) ((void)0)
#define SRV_FAN_CTRL_LOG_W(...) ((void)0)
#define SRV_FAN_CTRL_LOG_I(...) ((void)0)
#define SRV_FAN_CTRL_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define FAN_MIN_RPM (1000U) /**< 低于此 RPM 视为堵转 */
#define FAULT_DELAY_MS (4000U) /**< 持续低于 MIN_RPM(风扇堵转的话3S会自恢复) 超时触发故障 */
#define SRV_FAN_CTRL_RPM_LOG_PERIOD_MS (1000U) /**< 转速遥测日志限频窗口 (ms) */
#define FAN_SELFTEST_DUTY (25U) /**< 开机自检占空比 (100%) */
#define FAN_SELFTEST_MS (3000U) /**< 开机自检时长 (ms)：覆盖风扇从静止到可测速的启动时间 */
#define FAN_SELFTEST_MIN_RPM (FAN_MIN_RPM) /**< 自检通过的最低转速 */
#define TEMP_START_C (4000) /**< 起转温度 (0.01°C) = 40°C */
#define TEMP_STOP_C (3800) /**< 停转温度 (0.01°C) = 38°C (迟滞) */
#define TEMP_FULL_C (6500) /**< 满速温度 (0.01°C) = 65°C */
#define DUTY_MIN (20U) /**< 最低占空比 (防止低速无法启动) */
#define RPM_FILTER_CUTOFF_HZ (5U) /**< RPM 低通截止频率 (Hz), < Nyquist=5Hz */
#define RPM_SAMPLE_RATE_HZ (10U) /**< RPM 采样率 (Hz), 100ms 周期 */

/* FSM 状态 -----------------------------------------------------------------*/

/**
 * @brief 每台风扇的离散工作状态
 *
 * SELFTEST --(自检通过)--> RUN --(堵转 4s)--> FAULT
 * SELFTEST --(自检失败)--> FAULT --(测到转速)--> RUN
 */
typedef enum {
    FAN_STATE_SELFTEST = 0, /**< 开机自检：固定占空比运转并验证测速 */
    FAN_STATE_RUN, /**< 正常运行：按温度/手动控制 */
    FAN_STATE_FAULT, /**< 堵转/自检失败故障：锁存，测到转速后恢复 */
    FAN_STATE_COUNT,
} fan_ctrl_state_t;

/* Private types -------------------------------------------------------------*/

typedef struct {
    fsm_t fsm; /**< 每风扇状态机（故障标志由 FAULT 态派生，不单独存储） */
    pt1Filter_t rpm_filter; /**< RPM 低通滤波器 */
    uint8_t id; /**< 风扇编号（日志用） */
    uint8_t duty; /**< 目标占空比 */
    uint32_t rpm; /**< 滤波后 RPM */
    uint16_t low_rpm_ms; /**< 低速累计时间（RUN 态去抖） */
    uint16_t elapsed_ms; /**< 本次 step 周期 (ms)，FSM handler 内使用 */
    bool auto_mode; /**< 温控自动模式 */
    bool selftest_pass; /**< 自检中是否测到转速 */
} fan_ctrl_t;

/* Private variables ---------------------------------------------------------*/

static fan_ctrl_t s_fans[DRV_FAN_MAX];
static uint32_t s_fan_count;
static srv_fan_ctrl_temp_read_cb_t s_temp_read;
static bool s_initialized;
static uint32_t s_rpm_log_ts; /**< 转速遥测日志时间戳 (ms) */
static uint32_t s_boot_ms; /**< 启动时间戳 (ms)，自检窗口计时用 */

/* FSM 静态配置表：所有风扇同构，仅 user_data 不同（各风扇复用同一张表） */
static fsm_handler_t s_fan_handlers[FAN_STATE_COUNT];
static fsm_guard_t s_fan_transitions[FAN_STATE_COUNT * FAN_STATE_COUNT];

/* Private function prototypes -----------------------------------------------*/

static fsm_state_t fan_state_selftest(fsm_t* ctx);
static fsm_state_t fan_state_run(fsm_t* ctx);
static fsm_state_t fan_state_fault(fsm_t* ctx);
static void fan_apply_temp_duty(fan_ctrl_t* f);
static uint8_t temp_to_duty(int16_t temp_centi);

/* Exported functions --------------------------------------------------------*/

void srv_fan_ctrl_init(srv_fan_ctrl_temp_read_cb_t temp_read)
{
    drv_fan_init();
    s_fan_count = drv_fan_get_count();
    s_temp_read = temp_read;

    memset(s_fans, 0, sizeof(s_fans));

    const float rpm_k = pt1FilterGain(RPM_FILTER_CUTOFF_HZ, 1.0f / RPM_SAMPLE_RATE_HZ);

    /* 填充共享 FSM 配置表（handler 表 + 全连通转换矩阵） */
    s_fan_handlers[FAN_STATE_SELFTEST] = fan_state_selftest;
    s_fan_handlers[FAN_STATE_RUN] = fan_state_run;
    s_fan_handlers[FAN_STATE_FAULT] = fan_state_fault;

    fsm_config_t fsm_cfg = {
        .handlers = s_fan_handlers,
        .transitions = s_fan_transitions,
        .state_count = FAN_STATE_COUNT,
        .user_data = NULL, /* 循环内按风扇覆盖 */
    };
    fsm_fill(&fsm_cfg, fsm_always_true);

    for (uint32_t i = 0; i < s_fan_count; i++) {
        s_fans[i].id = (uint8_t)i;
        s_fans[i].auto_mode = (temp_read != NULL);
        pt1FilterInit(&s_fans[i].rpm_filter, rpm_k);

        fsm_cfg.user_data = &s_fans[i];
        fsm_init(&s_fans[i].fsm, FAN_STATE_SELFTEST, &fsm_cfg);
    }

    s_boot_ms = millis();
    s_initialized = true;

    SRV_FAN_CTRL_LOG_I("风扇控制服务初始化完成 (%u 台风扇, 自动温控=%d)",
        (unsigned)s_fan_count, (int)(temp_read != NULL));
}

void srv_fan_ctrl_step(uint16_t elapsed_ms)
{
    if (!s_initialized)
        return;

    const uint32_t now_ms = millis();

    /* 转速遥测限频标记：每 1s 打印一次全部风扇（循环内逐风扇打印） */
    const bool log_now = (uint32_t)(now_ms - s_rpm_log_ts) >= SRV_FAN_CTRL_RPM_LOG_PERIOD_MS;
    if (log_now) {
        s_rpm_log_ts = now_ms;
    }

    for (uint32_t i = 0; i < s_fan_count; i++) {
        fan_ctrl_t* f = &s_fans[i];

        /* 1. 读取 EXTI 脉冲计数 → 换算 RPM → 低通滤波
         *    无脉冲时喂 0，使滤波值衰减归零：停转/无 FG 信号时 RPM 不会卡在旧值。
         *    （连续信号管道，不属于 FSM） */
        uint32_t pulses = drv_fan_get_tach_delta(i);
        uint8_t ppr = drv_fan_get_pulse_per_rev(i);
        if (elapsed_ms > 0 && ppr > 0) {
            float rpm_raw = (pulses > 0)
                ? (float)pulses * 60000.0f / ((float)elapsed_ms * (float)ppr)
                : 0.0f;
            f->rpm = (uint32_t)pt1FilterApply(&f->rpm_filter, rpm_raw);
        }

        /* 2. FSM 步进：自检/运行/故障 离散状态决定目标占空比与故障状态 */
        f->elapsed_ms = elapsed_ms;
        fsm_step(&f->fsm);

        /* 3. 写入硬件 PWM */
        drv_fan_set_duty(i, f->duty);

        /* 4. 转速遥测日志：1s 限频，每台风扇一行 */
        if (log_now) {
            SRV_FAN_CTRL_LOG_D("风扇%u转速(RPM):%u", (unsigned)i, (unsigned)f->rpm);
        }
    }
}

void srv_fan_ctrl_set_duty(uint8_t id, uint8_t duty)
{
    if (!s_initialized || id >= s_fan_count)
        return;
    if (duty > 100)
        duty = 100;
    s_fans[id].duty = duty;

    SRV_FAN_CTRL_LOG_D("风扇%u 手动占空比 -> %u%%", (unsigned)id, (unsigned)duty);
}

const srv_fan_ctrl_status_t* srv_fan_ctrl_get_status(uint8_t id)
{
    static srv_fan_ctrl_status_t st;
    if (!s_initialized || id >= s_fan_count) {
        memset(&st, 0, sizeof(st));
        return &st;
    }

    st.rpm = s_fans[id].rpm;
    st.duty = s_fans[id].duty;
    st.fault = fsm_current_state(&s_fans[id].fsm) == FAN_STATE_FAULT;
    return &st;
}

bool srv_fan_ctrl_any_fault(void)
{
    for (uint32_t i = 0; i < s_fan_count; i++) {
        if (fsm_current_state(&s_fans[i].fsm) == FAN_STATE_FAULT)
            return true;
    }
    return false;
}

bool srv_fan_ctrl_is_fault(uint8_t id)
{
    if (!s_initialized || id >= s_fan_count) {
        return false;
    }
    return fsm_current_state(&s_fans[id].fsm) == FAN_STATE_FAULT;
}

void srv_fan_ctrl_set_auto(bool enable)
{
    for (uint32_t i = 0; i < s_fan_count; i++) {
        s_fans[i].auto_mode = enable;
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief FSM 自检态：固定测试占空比，验证测速
 *
 * 自检窗口结束（距上电 >= FAN_SELFTEST_MS）后按测速结果离开：
 * 测到转速 → RUN；未测到 → FAULT（故障由状态派生）。
 * 低速去抖计时在本态不累计——自检窗口(3s)短于故障阈值(4s)，不会误触发堵转。
 */
static fsm_state_t fan_state_selftest(fsm_t* ctx)
{
    fan_ctrl_t* f = (fan_ctrl_t*)fsm_user_data(ctx);

    f->duty = FAN_SELFTEST_DUTY;
    if (f->rpm >= FAN_SELFTEST_MIN_RPM) {
        f->selftest_pass = true;
    }

    if ((uint32_t)(millis() - s_boot_ms) >= FAN_SELFTEST_MS) {
        if (f->selftest_pass) {
            SRV_FAN_CTRL_LOG_I("风扇%u 自检通过", (unsigned)f->id);
            return FAN_STATE_RUN;
        }
        SRV_FAN_CTRL_LOG_E("风扇%u 自检失败: 未测到转速", (unsigned)f->id);
        return FAN_STATE_FAULT;
    }
    return FAN_STATE_SELFTEST;
}

/**
 * @brief FSM 运行态：温控/手动调速 + 堵转去抖检测
 */
static fsm_state_t fan_state_run(fsm_t* ctx)
{
    fan_ctrl_t* f = (fan_ctrl_t*)fsm_user_data(ctx);

    fan_apply_temp_duty(f);

    /* 堵转 / 低速故障检测（仅命令转动 duty>0 时判定，风扇主动关闭不误报；
     * duty==0 清零去抖计时） */
    if (f->duty > 0) {
        if (f->rpm < FAN_MIN_RPM) {
            f->low_rpm_ms += f->elapsed_ms;
        } else {
            f->low_rpm_ms = 0;
        }

        if (f->low_rpm_ms >= FAULT_DELAY_MS) {
            SRV_FAN_CTRL_LOG_E("风扇%u 堵转/低速故障: rpm=%u (<%u), 低速累计%ums (超时%ums)",
                (unsigned)f->id, (unsigned)f->rpm, (unsigned)FAN_MIN_RPM,
                (unsigned)f->low_rpm_ms, (unsigned)FAULT_DELAY_MS);
            return FAN_STATE_FAULT;
        }
    } else {
        f->low_rpm_ms = 0;
    }
    return FAN_STATE_RUN;
}

/**
 * @brief FSM 故障态：故障锁存（由状态派生），保持温控输出，测到转速后恢复
 */
static fsm_state_t fan_state_fault(fsm_t* ctx)
{
    fan_ctrl_t* f = (fan_ctrl_t*)fsm_user_data(ctx);

    /* 故障期间仍按温度驱动占空比（与重构前一致），便于恢复判定 */
    fan_apply_temp_duty(f);

    /* 风扇被命令转动且再次测到转速 → 恢复运行态（清零去抖计时防止立刻再触发） */
    if (f->duty > 0 && f->rpm >= FAN_MIN_RPM) {
        f->low_rpm_ms = 0;
        SRV_FAN_CTRL_LOG_I("风扇%u 故障恢复: rpm=%u", (unsigned)f->id, (unsigned)f->rpm);
        return FAN_STATE_RUN;
    }
    return FAN_STATE_FAULT;
}

/**
 * @brief 温控自动调速：按当前温度更新目标占空比（连续映射，保持在 FSM 之外）
 */
static void fan_apply_temp_duty(fan_ctrl_t* f)
{
    if (f->auto_mode && s_temp_read) {
        f->duty = temp_to_duty(s_temp_read(f->id));
    }
}

/**
 * @brief 温度 → 占空比线性映射（带迟滞）
 *
 *   duty
 *   100% │                  ●━━━
 *        │                ╱
 *    10% │          ●━━━━
 *        │        ╱
 *     0% │━━━━━━●
 *        └─────┬─────┬─────┬─── temp
 *            STOP  START  FULL
 *           (38°C)(40°C) (65°C)
 */
static uint8_t temp_to_duty(int16_t temp_centi)
{
    if (temp_centi >= TEMP_FULL_C) {
        return 100;
    }

    if (temp_centi <= TEMP_STOP_C) {
        return 0;
    }

    if (temp_centi < TEMP_START_C) {
        return DUTY_MIN;
    }

    int32_t range = TEMP_FULL_C - TEMP_START_C;
    int32_t pos = temp_centi - TEMP_START_C;
    return DUTY_MIN + (uint8_t)((uint32_t)pos * (100 - DUTY_MIN) / (uint32_t)range);
}
