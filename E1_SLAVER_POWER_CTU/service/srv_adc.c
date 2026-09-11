/**
 * @file    srv_adc.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   ADC 采样服务实现（VREFINT 校准 + 物理量换算, E1_SLAVER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_adc.h"

#include "drv_adc.h"
#include "drv_systick.h"
#include "filter.h"
#include "log.h"
#include "msg_fifo.h"

#include <math.h>
#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_ADC_LOG_ENABLE 0

#if SRV_ADC_LOG_ENABLE
#define SRV_ADC_LOG_E(...) LOG_E("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_W(...) LOG_W("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_I(...) LOG_I("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_D(...) LOG_D("srv_adc", __VA_ARGS__)
#else
#define SRV_ADC_LOG_E(...) ((void)0)
#define SRV_ADC_LOG_W(...) ((void)0)
#define SRV_ADC_LOG_I(...) ((void)0)
#define SRV_ADC_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define ADC_FIFO_BUF_SIZE (512U)

/** @brief 原始快照 FIFO 字节数（512B / sizeof(srv_adc_raw_t) ≈ 14 帧） */
#define ADC_RAW_FIFO_BUF_SIZE (512U)

/** @brief 遥测日志限频窗口 (ms) */
#define SRV_ADC_TELE_LOG_PERIOD_MS (1000U)

/** @brief 告警日志限频窗口 (ms) */
#define SRV_ADC_WARN_LOG_PERIOD_MS (1000U)

#define ADC_MAX (4095U)
#define ADC_SAMPLE_RATE_HZ (100U) /**< 100Hz (10ms period) */

/** @brief 12-bit ADC 满量程原始值 */
#define SRV_ADC_RAW_MAX (4095U)

/* 外部电压分压比：AUX/MOTOR 220k/10k → ×23（满量程约 75.9V）；
 * LSD1/LSD2 100k/10k → ×11（满量程约 36.3V） */
#define ADC_SCALE_AUX (23.0f)
#define ADC_SCALE_MOTOR (23.0f)
#define ADC_SCALE_LSD (11.0f)

/* VREFINT 校准：F103 标称 1.20V（ST 出厂校准值地址 0x1FFFF7BA 可选） */
#define VREFINT_CAL_MV (1200U) /**< 内部参考电压标称值 (mV) */

/* 内部温度传感器校准（F103 高密度 @30°C/110°C） */
#define TS_CAL1_ADDR ((uint16_t*)0x1FFFF7B8) /**< 30°C 校准值 */
#define TS_CAL2_ADDR ((uint16_t*)0x1FFFF7C2) /**< 110°C 校准值 */
#define TS_CAL1_TEMP (30)
#define TS_CAL2_TEMP (110)

/** @brief 典型参数（无出厂校准时的兜底，取自 F103 数据手册） */
#define TS_TYP_V25_MV (1430.0f) /**< 25°C 时传感器典型电压 (mV) */
#define TS_TYP_SLOPE_MV_PER_DEG (4.3f) /**< 电压-温度斜率 (mV/℃) */
#define TS_TYP_T25 (25.0f)

/* Private types -------------------------------------------------------------*/

/** @brief 原始采样快照 — DMA 中断回调只填此结构，不做任何换算 */
typedef struct {
    uint32_t timestamp_ms; /**< 时间戳 (ms) */
    uint16_t raw[DRV_ADC_CH_MAX]; /**< 各逻辑通道 12-bit 原始值 */
} srv_adc_raw_t;

/* Private variables ---------------------------------------------------------*/

static uint8_t s_fifo_buf[ADC_FIFO_BUF_SIZE];
static msg_fifo_t s_fifo;

/** @brief 原始快照 FIFO：DMA 中断回调(生产) → srv_adc_step(消费)，锁自由 SPSC */
static uint8_t s_raw_fifo_buf[ADC_RAW_FIFO_BUF_SIZE];
static msg_fifo_t s_raw_fifo;

/** @brief PT1 低通滤波器（每通道一个，10Hz 截止，抑制 ADC 噪声） */
#define ADC_FILTER_CUTOFF_HZ (10U)

static pt1Filter_t s_filters[DRV_ADC_CH_MAX];

/** @brief 遥测日志时间戳 (ms) */
static uint32_t s_tele_log_ts;

/** @brief 告警日志时间戳 (ms) */
static uint32_t s_warn_log_ts;

/** @brief 内部温度传感器出厂校准值（芯片固定，初始化时读取一次） */
static uint16_t s_ts_cal1;
static uint16_t s_ts_cal2;

/** @brief 服务初始化完成标志 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void adc_sample_cb(drv_adc_inst_t inst);
static float adc_filtered(drv_adc_channel_t ch, uint16_t raw_value);
static bool warn_rate_limited(void);
static srv_adc_calc_status_t calc_vdda_mv(float vrefint_filtered, uint32_t* vdda_mv);
static srv_adc_calc_status_t calc_mcu_temp(float ts_filtered, float vdda_mv,
    int16_t* temp_x100);
static srv_adc_calc_status_t calc_external_mv(float filtered_raw, float scale,
    float raw_to_mv, uint32_t* out_mv);

/* Exported functions --------------------------------------------------------*/

void srv_adc_init(void)
{
    drv_adc_init();
    drv_adc_register_callback(adc_sample_cb);

    /* 初始化 PT1 低通滤波器（100Hz 采样率，10Hz 截止频率，全部通道） */
    const float k = pt1FilterGain(ADC_FILTER_CUTOFF_HZ, 1.0f / ADC_SAMPLE_RATE_HZ);
    for (uint32_t i = 0; i < DRV_ADC_CH_MAX; i++) {
        pt1FilterInit(&s_filters[i], k);
    }

    msg_fifo_init(&s_fifo, s_fifo_buf, ADC_FIFO_BUF_SIZE, sizeof(srv_adc_data_t));
    msg_fifo_init(&s_raw_fifo, s_raw_fifo_buf, ADC_RAW_FIFO_BUF_SIZE, sizeof(srv_adc_raw_t));

    /* 内部温度传感器出厂校准值：芯片固定，仅初始化读取一次（无效值由 calc_mcu_temp 兜底） */
    s_ts_cal1 = *TS_CAL1_ADDR;
    s_ts_cal2 = *TS_CAL2_ADDR;

    s_initialized = true;

    SRV_ADC_LOG_I("ADC 采样服务初始化完成 (采样率=%uHz, 滤波截止=%uHz, FIFO=%uB)",
        (unsigned)ADC_SAMPLE_RATE_HZ, (unsigned)ADC_FILTER_CUTOFF_HZ, (unsigned)ADC_FIFO_BUF_SIZE);
    SRV_ADC_LOG_I("内部温度校准值: cal30=0x%04X cal110=0x%04X",
        (unsigned)s_ts_cal1, (unsigned)s_ts_cal2);
}

void srv_adc_trigger(void)
{
    drv_adc_trigger_all();
}

void srv_adc_step(void)
{
    /* 取最新一帧原始快照（丢弃积压旧帧只留最新） */
    srv_adc_raw_t raw;
    bool got = false;
    while (msg_fifo_pop(&s_raw_fifo, &raw)) {
        got = true;
    }
    if (!got) {
        return; /* 尚无新快照（首拍），等待下一周期 */
    }

    srv_adc_data_t s;
    memset(&s, 0, sizeof(s));
    s.timestamp_ms = raw.timestamp_ms;

    /* ── VREFINT 校准：反推实际 VDDA ── */
    float vref = adc_filtered(DRV_ADC_CH_VREFINT, raw.raw[DRV_ADC_CH_VREFINT]);
    uint32_t vdda_mv = 3300U;
    s.vdda_status = calc_vdda_mv(vref, &vdda_mv);
    if (s.vdda_status != SRV_ADC_CALC_OK && warn_rate_limited()) {
        SRV_ADC_LOG_W("VREFINT 采样值过小 (%u)，VDDA 回退默认 3300mV", (unsigned)vref);
    }
    s.vdda_mv = vdda_mv;

    /* ── 外部电压 (mV)：raw × vdda/4095 × 分压比 ── */
    const float raw_to_mv = vdda_mv / (float)ADC_MAX;
    s.aux_status = calc_external_mv(
        adc_filtered(DRV_ADC_CH_AUX, raw.raw[DRV_ADC_CH_AUX]), ADC_SCALE_AUX, raw_to_mv, &s.aux_mv);
    s.motor_status = calc_external_mv(
        adc_filtered(DRV_ADC_CH_MOTOR, raw.raw[DRV_ADC_CH_MOTOR]), ADC_SCALE_MOTOR, raw_to_mv, &s.motor_mv);
    s.lsd1_status = calc_external_mv(
        adc_filtered(DRV_ADC_CH_LSD1, raw.raw[DRV_ADC_CH_LSD1]), ADC_SCALE_LSD, raw_to_mv, &s.lsd1_mv);
    s.lsd2_status = calc_external_mv(
        adc_filtered(DRV_ADC_CH_LSD2, raw.raw[DRV_ADC_CH_LSD2]), ADC_SCALE_LSD, raw_to_mv, &s.lsd2_mv);

    /* ── MCU 内部温度 ── */
    s.mcu_temp_status = calc_mcu_temp(
        adc_filtered(DRV_ADC_CH_TEMPSENSOR, raw.raw[DRV_ADC_CH_TEMPSENSOR]),
        (float)vdda_mv, &s.mcu_temp_x100);

    /* 遥测日志（任务上下文，限频 1s；温度字段为 ×100 整数，单位 0.01°C） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_tele_log_ts) >= SRV_ADC_TELE_LOG_PERIOD_MS) {
        s_tele_log_ts = now_ms;

        SRV_ADC_LOG_D("vdda=%umV aux=%umV motor=%umV lsd1=%umV lsd2=%umV mcuT=%d(×100)",
            (unsigned)s.vdda_mv, (unsigned)s.aux_mv, (unsigned)s.motor_mv,
            (unsigned)s.lsd1_mv, (unsigned)s.lsd2_mv, (int)s.mcu_temp_x100);
    }

    msg_fifo_push(&s_fifo, &s);
}

bool srv_adc_get_latest(srv_adc_data_t* sample)
{
    if (!sample || !s_initialized) {
        return false;
    }

    static srv_adc_data_t last_sample;
    msg_fifo_pop(&s_fifo, &last_sample);
    *sample = last_sample;
    return true;
}

/* Private functions ---------------------------------------------------------*/

static void adc_sample_cb(drv_adc_inst_t inst)
{
    (void)inst;

    /* DMA 中断上下文：只做原始值快照，不做任何换算/打印。 */
    srv_adc_raw_t raw;
    raw.timestamp_ms = millis();

    for (uint32_t i = 0; i < DRV_ADC_CH_MAX; i++) {
        raw.raw[i] = (uint16_t)drv_adc_read_raw((drv_adc_channel_t)i);
    }

    msg_fifo_push(&s_raw_fifo, &raw);
}

static float adc_filtered(drv_adc_channel_t ch, uint16_t raw_value)
{
    return pt1FilterApply(&s_filters[ch], (float)raw_value);
}

/** @brief 告警日志限频（1s 窗口），返回本次是否允许打印 */
static bool warn_rate_limited(void)
{
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_warn_log_ts) < SRV_ADC_WARN_LOG_PERIOD_MS) {
        return false;
    }
    s_warn_log_ts = now_ms;
    return true;
}

/**
 * @brief VREFINT → VDDA (mV)
 *
 * VREFINT 内部参考电压标称 1.20V，通过 ADC 采样值反推实际 VDDA：
 * VDDA = 1.20V × 4095 / VREFINT_RAW
 */
static srv_adc_calc_status_t calc_vdda_mv(float vrefint_filtered, uint32_t* vdda_mv)
{
    if (vdda_mv == NULL) {
        return SRV_ADC_CALC_ERR_PARAM;
    }

    if (vrefint_filtered < 100.0f) {
        *vdda_mv = 3300U; /* 无效值 → 回退默认 3.3V */
        return SRV_ADC_CALC_ERR_LOW_RAW;
    }

    *vdda_mv = (uint32_t)((float)VREFINT_CAL_MV * (float)ADC_MAX / vrefint_filtered);
    return SRV_ADC_CALC_OK;
}

/**
 * @brief 内部温度传感器 → 温度 (°C × 100)
 *
 * 优先使用 ST 出厂校准值 (30°C/110°C) 线性插值；
 * 无出厂校准（F103 部分器件地址读回 0xFFFF）时回退数据手册典型参数：
 *   V_sense = raw × VDDA/4095
 *   T = (V25 − V_sense)/斜率 + 25     （F1 传感器电压随温度升高而下降）
 */
static srv_adc_calc_status_t calc_mcu_temp(float ts_filtered, float vdda_mv,
    int16_t* temp_x100)
{
    if (temp_x100 == NULL) {
        return SRV_ADC_CALC_ERR_PARAM;
    }

    float t;

    if (s_ts_cal1 != 0xFFFF && s_ts_cal2 != 0xFFFF && s_ts_cal2 != s_ts_cal1) {
        /* 出厂校准两值含斜率符号，raw 低=温度高，直接线性插值即可 */
        t = (float)TS_CAL1_TEMP
            + (float)(TS_CAL2_TEMP - TS_CAL1_TEMP) * (ts_filtered - (float)s_ts_cal1)
                / (float)(s_ts_cal2 - s_ts_cal1);
    } else {
        /* 出厂校准缺失 → 典型参数法（注意 F1 为负斜率：V_sense 随温度升高而下降） */
        if (vdda_mv < 2000.0f) {
            return SRV_ADC_CALC_ERR_LOW_RAW;
        }
        const float v_sense_mv = ts_filtered * vdda_mv / 4095.0f;
        t = (TS_TYP_V25_MV - v_sense_mv) / TS_TYP_SLOPE_MV_PER_DEG + TS_TYP_T25;
    }

    if (t < -50.0f) {
        t = -50.0f;
    }
    if (t > 150.0f) {
        t = 150.0f;
    }

    *temp_x100 = (int16_t)(t * 100.0f);
    return SRV_ADC_CALC_OK;
}

/**
 * @brief 外部电压通道换算（V = raw × vdda/4095 × 分压比）
 */
static srv_adc_calc_status_t calc_external_mv(float filtered_raw, float scale,
    float raw_to_mv, uint32_t* out_mv)
{
    if (out_mv == NULL) {
        return SRV_ADC_CALC_ERR_PARAM;
    }

    if (filtered_raw < 5.0f) {
        *out_mv = 0U; /* 采样值过低：输入缺失/未上电 */
        return SRV_ADC_CALC_ERR_LOW_RAW;
    }

    float mv = filtered_raw * raw_to_mv * scale;
    if (mv > 100000.0f) {
        mv = 100000.0f; /* 上限保护 */
    }
    *out_mv = (uint32_t)mv;
    return SRV_ADC_CALC_OK;
}
