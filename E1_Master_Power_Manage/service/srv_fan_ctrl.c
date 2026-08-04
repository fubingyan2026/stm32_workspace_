/**
 * @file    srv_fan_ctrl.c
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-07-8
 * @brief   风扇控制服务实现
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_fan_ctrl.h"

#include <string.h>

#include "drv_fan.h"
#include "filter.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_FAN_CTRL_LOG_ENABLE 1

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

#define FAN_MIN_RPM (300U) /**< 低于此 RPM 视为堵转 */
#define FAULT_DELAY_MS (2000U) /**< 持续低于 MIN_RPM 超时触发故障 */
#define TEMP_START_C (4000) /**< 起转温度 (0.01°C) = 40°C */
#define TEMP_STOP_C (3800) /**< 停转温度 (0.01°C) = 38°C (迟滞) */
#define TEMP_FULL_C (6500) /**< 满速温度 (0.01°C) = 65°C */
#define DUTY_MIN (10U) /**< 最低占空比 (防止低速无法启动) */
#define RPM_FILTER_CUTOFF_HZ (1U) /**< RPM 低通截止频率 (Hz), < Nyquist=5Hz */
#define RPM_SAMPLE_RATE_HZ (10U) /**< RPM 采样率 (Hz), 100ms 周期 */

/* Private types -------------------------------------------------------------*/

typedef struct {
    pt1Filter_t rpm_filter; /**< RPM 低通滤波器 */
    uint8_t duty; /**< 目标占空比 */
    uint32_t rpm; /**< 滤波后 RPM */
    uint16_t low_rpm_ms; /**< 低速累计时间 */
    bool fault; /**< 故障标志 */
    bool auto_mode; /**< 温控自动模式 */
} fan_ctrl_t;

/* Private variables ---------------------------------------------------------*/

static fan_ctrl_t s_fans[DRV_FAN_MAX];
static uint32_t s_fan_count;
static srv_fan_ctrl_temp_read_cb_t s_temp_read;
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static uint8_t temp_to_duty(int16_t temp_centi);

/* Exported functions --------------------------------------------------------*/

void srv_fan_ctrl_init(srv_fan_ctrl_temp_read_cb_t temp_read)
{
    drv_fan_init();
    s_fan_count = drv_fan_get_count();
    s_temp_read = temp_read;

    memset(s_fans, 0, sizeof(s_fans));

    const float rpm_k = pt1FilterGain(RPM_FILTER_CUTOFF_HZ, 1.0f / RPM_SAMPLE_RATE_HZ);

    for (uint32_t i = 0; i < s_fan_count; i++) {
        s_fans[i].auto_mode = (temp_read != NULL);
        pt1FilterInit(&s_fans[i].rpm_filter, rpm_k);
    }

    s_initialized = true;

    SRV_FAN_CTRL_LOG_I("风扇控制服务初始化完成 (%u 台风扇, 自动温控=%d)",
        (unsigned)s_fan_count, (int)(temp_read != NULL));
}

void srv_fan_ctrl_step(uint16_t elapsed_ms)
{
    if (!s_initialized)
        return;

    for (uint32_t i = 0; i < s_fan_count; i++) {
        fan_ctrl_t* f = &s_fans[i];

        /* 1. 读取 EXTI 脉冲计数 → 换算 RPM → 低通滤波 */
        uint32_t pulses = drv_fan_get_tach_delta(i);
        uint8_t ppr = drv_fan_get_pulse_per_rev(i);
        if (pulses > 0 && elapsed_ms > 0 && ppr > 0) {
            float rpm_raw = (float)pulses * 60000.0f
                / ((float)elapsed_ms * (float)ppr);
            f->rpm = (uint32_t)pt1FilterApply(&f->rpm_filter, rpm_raw);
        }

        /* 2. 堵转 / 低速故障检测（边沿触发打印：仅断言/恢复时各一次） */
        if (f->rpm < FAN_MIN_RPM) {
            f->low_rpm_ms += elapsed_ms;
        } else {
            f->low_rpm_ms = 0;
            if (f->fault) {
                f->fault = false;
                SRV_FAN_CTRL_LOG_I("风扇%u 故障恢复: rpm=%u", (unsigned)i, (unsigned)f->rpm);
            }
        }

        if (f->low_rpm_ms >= FAULT_DELAY_MS && !f->fault) {
            f->fault = true;
            SRV_FAN_CTRL_LOG_E("风扇%u 堵转/低速故障: rpm=%u (<%u), 低速累计%ums (超时%ums)",
                (unsigned)i, (unsigned)f->rpm, (unsigned)FAN_MIN_RPM,
                (unsigned)f->low_rpm_ms, (unsigned)FAULT_DELAY_MS);
        }

        /* 3. 温控自动调速 */
        if (f->auto_mode && s_temp_read) {
            int16_t temp = s_temp_read(i);
            f->duty = temp_to_duty(temp);
        }

        /* 4. 写入硬件 PWM */
        drv_fan_set_duty(i, 25);
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
    st.fault = s_fans[id].fault;
    return &st;
}

bool srv_fan_ctrl_any_fault(void)
{
    for (uint32_t i = 0; i < s_fan_count; i++) {
        if (s_fans[i].fault)
            return true;
    }
    return false;
}

bool srv_fan_ctrl_is_fault(uint8_t id)
{
    if (!s_initialized || id >= s_fan_count) {
        return false;
    }
    return s_fans[id].fault;
}

void srv_fan_ctrl_set_auto(bool enable)
{
    for (uint32_t i = 0; i < s_fan_count; i++) {
        s_fans[i].auto_mode = enable;
    }
}

/* Private functions ---------------------------------------------------------*/

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
