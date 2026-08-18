/**
 * @file    adc_sample.c
 * @author  maximillian
 * @version V2.1.0
 * @date    2026-07-8
 * @brief   ADC 采样服务实现（VREFINT 校准 + 物理量换算）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_adc.h"

#include "drv_adc.h"
#include "drv_cd4051b.h"
#include "drv_systick.h"
#include "filter.h"
#include "log.h"
#include "msg_fifo.h"

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

#define ADC_FIFO_BUF_SIZE (1024U)

/** @brief 原始快照 FIFO 字节数（1024B / sizeof(srv_adc_raw_t) ≈ 28 帧） */
#define ADC_RAW_FIFO_BUF_SIZE (1024U)

/** @brief 遥测日志限频窗口 (ms)：采样回调约 100Hz，需限频防刷屏 */
#define SRV_ADC_TELE_LOG_PERIOD_MS (1000U)

/** @brief 告警日志限频窗口 (ms)：VREFINT/校准异常在 100Hz 回调中连续触发时防刷屏 */
#define SRV_ADC_WARN_LOG_PERIOD_MS (1000U)

#define ADC_MAX (4095U)
#define ADC_SAMPLE_RATE_HZ (100U) /**< 100Hz (10ms period) */

/* CD4051B 多路选择 A_INx_IO */
#define SRV_ADC_AIN_NUM (3U) /**< 模拟输入路数 */
#define SRV_ADC_AIN_CH_MIN (1U) /**< A_IN1_IO 对应 CD4051B 起始通道 (Y1) */
#define SRV_ADC_AIN_CH_MAX (SRV_ADC_AIN_CH_MIN + SRV_ADC_AIN_NUM - 1U) /**< A_IN3_IO 对应 CD4051B 结束通道 (Y3) */
#define SRV_ADC_AIN_HIGH_RAW (2048U) /**< A_INx_IO 逻辑高判定阈值（12-bit，≈50% VDDA ≈ 1.65V），按分压实际调整 */

/* 外部电压分压比 */
#define ADC_SCALE_VIN (31.0f)
#define ADC_SCALE_MOTOR_POWER (31.0f)
#define ADC_SCALE_AUX_POWER (11.0f)

/* VREFINT 校准 (ST 出厂校准值 @3.3V/30°C，地址 0x1FFF7A2A) */
#ifndef VREFINT_CAL_ADDR
#define VREFINT_CAL_ADDR ((uint16_t*)0x1FFF7A2A)
#endif
#define VREFINT_CAL_MV (1210U) /**< 内部参考电压标称值 (mV) */

/* 内部温度传感器校准 (地址 0x1FFF7A2C / 0x1FFF7A2E) */
#define TS_CAL1_ADDR ((uint16_t*)0x1FFF7A2C) /**< 25°C 校准值 */
#define TS_CAL2_ADDR ((uint16_t*)0x1FFF7A2E) /**< 110°C 校准值 */
#define TS_CAL1_TEMP (25)
#define TS_CAL2_TEMP (110)

/* NTC 参数 */
#define NTC_PULLUP_R (10000.0f) /**< 上拉电阻 10kΩ */

/** @brief NTC 电阻-温度表 (NCP18XH103, 10kΩ B=3380) */
static const struct {
    float r;
    float t;
} s_ntc_table[] = {
    { 345275.0f, -40.0f },
    { 98185.0f, -20.0f },
    { 32184.0f, 0.0f },
    { 29636.0f, 10.0f },
    { 12504.0f, 20.0f },
    { 8049.0f, 30.0f },
    { 5762.0f, 38.0f },
    { 5532.0f, 39.0f },
    { 5313.0f, 40.0f },
    { 5103.0f, 41.0f },
    { 4903.0f, 42.0f },
    { 3588.0f, 50.0f },
    { 2476.0f, 60.0f },
    { 2072.0f, 65.0f },
    { 1065.0f, 85.0f },
    { 585.0f, 105.0f },
    { 341.0f, 125.0f },
};
#define NTC_TABLE_SIZE (sizeof(s_ntc_table) / sizeof(s_ntc_table[0]))

/* Private types -------------------------------------------------------------*/

/** @brief 原始采样快照 — DMA 中断回调只填此结构，不做任何换算 */
typedef struct {
    uint32_t timestamp_ms; /**< 时间戳 (ms) */
    uint8_t ain_mux_idx; /**< 本次快照对应的 CD4051B 通道 (Y1~Y3)，用于区分 A_IN1_IO/2_IO/3_IO */
    uint8_t reserved; /**< 对齐填充 */
    uint16_t raw[DRV_ADC_CH_MAX]; /**< 各逻辑通道 12-bit 原始值 */
} srv_adc_raw_t;

/* Private variables ---------------------------------------------------------*/

static uint8_t s_fifo_buf[ADC_FIFO_BUF_SIZE];
static msg_fifo_t s_fifo;

/** @brief 原始快照 FIFO：DMA 中断回调(生产) → srv_adc_step(消费)，锁自由 SPSC */
static uint8_t s_raw_fifo_buf[ADC_RAW_FIFO_BUF_SIZE];
static msg_fifo_t s_raw_fifo;

/** @brief PT1 低通滤波器（每通道一个，4Hz 截止，抑制 ADC 噪声） */
#define ADC_FILTER_CUTOFF_HZ (10U)

static pt1Filter_t s_filters[DRV_ADC_CH_MAX];

/** @brief 遥测日志时间戳 (ms) */
static uint32_t s_tele_log_ts;

/** @brief 告警日志时间戳 (ms) */
static uint32_t s_warn_log_ts;

/** @brief 内部温度传感器出厂校准值（芯片固定，初始化时读取一次） */
static uint16_t s_ts_cal1;
static uint16_t s_ts_cal2;

/** @brief 本次 DMA 正在采样的 CD4051B 通道 (Y1~Y3)（srv_adc_trigger 写入，ISR 快照） */
static uint8_t s_ain_mux_sel;
/** @brief 下一次 trigger 使用的 CD4051B 通道（1~3 轮转，不回绕到 0） */
static uint8_t s_ain_cycle;
/** @brief 三路 A_INx_IO 最近一次采样值（轮转更新，跨 step 保留旧值） */
static uint16_t s_ain_raw[SRV_ADC_AIN_NUM];

/** @brief 服务初始化完成标志（未初始化前禁止出队，防越权读取） */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void adc_sample_cb(drv_adc_inst_t inst);
static float adc_filtered(drv_adc_channel_t ch, uint16_t raw_value);
static bool warn_rate_limited(void);
static const char* calc_status_str(srv_adc_calc_status_t st);
static srv_adc_calc_status_t calc_vdda_mv(float vrefint_filtered, uint32_t* vdda_mv);
static srv_adc_calc_status_t calc_mcu_temp(float ts_filtered, int16_t* temp_x100);
static srv_adc_calc_status_t ntc_raw_to_temp(uint16_t raw, float vdda_v, int16_t* temp_x100);

/* Exported functions --------------------------------------------------------*/

void srv_adc_init(void)
{
    drv_adc_init();
    drv_adc_register_callback(adc_sample_cb);

    /* 初始化 PT1 低通滤波器（100Hz 采样率，4Hz 截止频率，全部通道） */
    const float k = pt1FilterGain(ADC_FILTER_CUTOFF_HZ, 1.0f / ADC_SAMPLE_RATE_HZ);
    for (uint32_t i = 0; i < DRV_ADC_CH_MAX; i++) {
        pt1FilterInit(&s_filters[i], k);
    }

    msg_fifo_init(&s_fifo, s_fifo_buf, ADC_FIFO_BUF_SIZE, sizeof(srv_adc_data_t));
    msg_fifo_init(&s_raw_fifo, s_raw_fifo_buf, ADC_RAW_FIFO_BUF_SIZE, sizeof(srv_adc_raw_t));

    /* CD4051B 多路选择器：A_IN1_IO/2_IO/3_IO 经 Y1/Y2/Y3 → PC5 采样 */
    drv_cd4051b_init();
    s_ain_mux_sel = 0; /* 尚无快照，置无效通道 */
    s_ain_cycle = SRV_ADC_AIN_CH_MIN; /* 从 Y1 起轮转 */
    memset(s_ain_raw, 0, sizeof(s_ain_raw));

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
    /* CD4051B 轮转：本次 DMA 采样通道 s_ain_cycle (Y1~Y3)，ISR 据 s_ain_mux_sel 快照归位 */
    s_ain_mux_sel = s_ain_cycle;
    (void)drv_cd4051b_select(s_ain_mux_sel);
    s_ain_cycle = (s_ain_cycle >= SRV_ADC_AIN_CH_MAX) ? SRV_ADC_AIN_CH_MIN : (s_ain_cycle + 1U);

    drv_adc_trigger_all();
}

void srv_adc_step(void)
{
    /* 取最新一帧原始快照（每周期最多 3 次 DMA 回调入队，丢弃积压旧帧只留最新） */
    srv_adc_raw_t raw;
    bool got = false;
    while (msg_fifo_pop(&s_raw_fifo, &raw)) {
        got = true;
    }
    if (!got) {
        return; /* 尚无新快照（首拍），等待下一周期 */
    }

    srv_adc_data_t s;
    s.timestamp_ms = raw.timestamp_ms;

    /* ── VREFINT 校准：反推实际 VDDA ── */
    float vref = adc_filtered(DRV_ADC_CH_VREFINT, raw.raw[DRV_ADC_CH_VREFINT]);
    uint32_t vdda_mv = 3300U;
    s.vdda_status = calc_vdda_mv(vref, &vdda_mv);
    if (s.vdda_status != SRV_ADC_CALC_OK && warn_rate_limited()) {
        SRV_ADC_LOG_W("VREFINT 采样值过小 (%u)，VDDA 回退默认 3300mV", (unsigned)vref);
    }
    s.vdda_mv = vdda_mv;
    float vdda_v = (float)vdda_mv * 0.001f;

    /* ── 外部电压 (mV) ── */
    const float raw_to_mv = vdda_mv / (float)ADC_MAX;

    s.vin_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_VIN, raw.raw[DRV_ADC_CH_VIN]) * raw_to_mv * ADC_SCALE_VIN);
    s.motor_power_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_MOTOR_POWER, raw.raw[DRV_ADC_CH_MOTOR_POWER]) * raw_to_mv * ADC_SCALE_MOTOR_POWER);
    s.aux_power_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_AUX_POWER, raw.raw[DRV_ADC_CH_AUX_POWER]) * raw_to_mv * ADC_SCALE_AUX_POWER);

    /* ── E-STOP 双通道冗余 (12-bit 原始值) ── */
    s.e_stop1_adc1 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP1_ADC1, raw.raw[DRV_ADC_CH_E_STOP1_ADC1]);
    s.e_stop1_adc2 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP1_ADC2, raw.raw[DRV_ADC_CH_E_STOP1_ADC2]);
    s.e_stop2_adc1 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP2_ADC1, raw.raw[DRV_ADC_CH_E_STOP2_ADC1]);
    s.e_stop2_adc2 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP2_ADC2, raw.raw[DRV_ADC_CH_E_STOP2_ADC2]);
    s.e_stop3_adc1 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP3_ADC1, raw.raw[DRV_ADC_CH_E_STOP3_ADC1]);
    s.e_stop3_adc2 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP3_ADC2, raw.raw[DRV_ADC_CH_E_STOP3_ADC2]);
    s.e_stop4_adc1 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP4_ADC1, raw.raw[DRV_ADC_CH_E_STOP4_ADC1]);
    s.e_stop4_adc2 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP4_ADC2, raw.raw[DRV_ADC_CH_E_STOP4_ADC2]);

    /* ── VBAT 备份电池电压 ── */
    s.vbat_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_VBAT, raw.raw[DRV_ADC_CH_VBAT]) * raw_to_mv);

    /* ── CD4051B 多路选择 A_INx_IO：仅更新本次 DMA 对应的一路 ──
     * 使用未滤波原始值（CD4051B 通道每周期轮转切换输入，PT1 滤波会把三路混叠）。
     * ain_mux_idx 为通道号 (Y1~Y3)，换算为 0 基下标写入 s_ain_raw。 */
    if (raw.ain_mux_idx >= SRV_ADC_AIN_CH_MIN && raw.ain_mux_idx <= SRV_ADC_AIN_CH_MAX) {
        const uint8_t idx = (uint8_t)(raw.ain_mux_idx - SRV_ADC_AIN_CH_MIN);
        s_ain_raw[idx] = (uint16_t)raw.raw[DRV_ADC_CH_CD4051B];
    }
    s.a_in1_io_raw = s_ain_raw[0];
    s.a_in2_io_raw = s_ain_raw[1];
    s.a_in3_io_raw = s_ain_raw[2];

    /* ── NTC 温度 (使用校准后的 VDDA) ── */
    s.ntc1_status = ntc_raw_to_temp(
        (uint16_t)adc_filtered(DRV_NTC1_ADC, raw.raw[DRV_NTC1_ADC]), vdda_v, &s.ntc1_temp_x100);
    s.ntc2_status = ntc_raw_to_temp(
        (uint16_t)adc_filtered(DRV_NTC2_ADC, raw.raw[DRV_NTC2_ADC]), vdda_v, &s.ntc2_temp_x100);

    /* ── MCU 内部温度 ── */
    s.mcu_temp_status = calc_mcu_temp(
        adc_filtered(DRV_ADC_CH_TEMPSENSOR, raw.raw[DRV_ADC_CH_TEMPSENSOR]), &s.mcu_temp_x100);

    /* 遥测日志（任务上下文，限频 1s；温度字段为 ×100 整数，单位 0.01°C） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_tele_log_ts) >= SRV_ADC_TELE_LOG_PERIOD_MS) {
        s_tele_log_ts = now_ms;

        // if (s.ntc1_status != SRV_ADC_CALC_OK) {
        //     SRV_ADC_LOG_W("NTC1 温度异常: %s (raw=%u)", calc_status_str(s.ntc1_status),
        //         (unsigned)raw.raw[DRV_NTC1_ADC]);
        // }
        // if (s.ntc2_status != SRV_ADC_CALC_OK) {
        //     SRV_ADC_LOG_W("NTC2 温度异常: %s (raw=%u)", calc_status_str(s.ntc2_status),
        //         (unsigned)raw.raw[DRV_NTC2_ADC]);
        // }
        // if (s.mcu_temp_status != SRV_ADC_CALC_OK) {
        //     SRV_ADC_LOG_W("MCU 温度校准值无效 (cal1=0x%04X cal2=0x%04X)，温度输出 0",
        //         (unsigned)s_ts_cal1, (unsigned)s_ts_cal2);
        // }

        SRV_ADC_LOG_D("ADC测试:vdda(inter)=%umV,vin=%umV,motor=%umV,aux=%umV,vbat(inter)=%umV,mcuT(inter)=%d,ntc1=%d,ntc2=%d(温度*100),ain1=%u,ain2=%u,ain3=%u",
            (unsigned)s.vdda_mv, (unsigned)s.vin_mv, (unsigned)s.motor_power_mv,
            (unsigned)s.aux_power_mv, (unsigned)s.vbat_mv,
            (int)s.mcu_temp_x100, (int)s.ntc1_temp_x100, (int)s.ntc2_temp_x100,
            (unsigned)s.a_in1_io_raw, (unsigned)s.a_in2_io_raw, (unsigned)s.a_in3_io_raw);

        /* E-STOP 双通道冗余状态：4 个急停开关 × (ADC1/ADC2 两路原始值)。
         * 偏差 = ADC1 - ADC2；两路偏差过大提示冗余通道失效/线缆异常。 */
        // SRV_ADC_LOG_D("急停冗余采样: S1=%u/%u S2=%u/%u S3=%u/%u S4=%u/%u (ADC1/ADC2) 偏差=%+d/%+d/%+d/%+d",
        //     (unsigned)s.e_stop1_adc1, (unsigned)s.e_stop1_adc2,
        //     (unsigned)s.e_stop2_adc1, (unsigned)s.e_stop2_adc2,
        //     (unsigned)s.e_stop3_adc1, (unsigned)s.e_stop3_adc2,
        //     (unsigned)s.e_stop4_adc1, (unsigned)s.e_stop4_adc2,
        //     (int)s.e_stop1_adc1 - (int)s.e_stop1_adc2,
        //     (int)s.e_stop2_adc1 - (int)s.e_stop2_adc2,
        //     (int)s.e_stop3_adc1 - (int)s.e_stop3_adc2,
        //     (int)s.e_stop4_adc1 - (int)s.e_stop4_adc2);

        /* 换算状态观测：VDDA/NTC1/NTC2/MCU 温度计算是否异常 */
        // SRV_ADC_LOG_D("换算状态: vdda=%s ntc1=%s ntc2=%s mcuT=%s",
        //     calc_status_str(s.vdda_status), calc_status_str(s.ntc1_status),
        //     calc_status_str(s.ntc2_status), calc_status_str(s.mcu_temp_status));
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

uint8_t srv_adc_read_ain(void)
{
    srv_adc_data_t s;
    if (!srv_adc_get_latest(&s)) {
        return 0; /* 未初始化/尚无快照 → 全部低电平 */
    }

    uint8_t mask = 0;
    if (s.a_in1_io_raw >= SRV_ADC_AIN_HIGH_RAW)
        mask |= (1U << 0);
    if (s.a_in2_io_raw >= SRV_ADC_AIN_HIGH_RAW)
        mask |= (1U << 1);
    if (s.a_in3_io_raw >= SRV_ADC_AIN_HIGH_RAW)
        mask |= (1U << 2);
    return mask;
}

/* Private functions ---------------------------------------------------------*/

static void adc_sample_cb(drv_adc_inst_t inst)
{
    (void)inst;

    /* DMA 中断上下文：只做原始值快照，不做任何换算/打印。
     * 全部 float 计算与日志已移至 srv_adc_step()（主循环上下文）。 */
    srv_adc_raw_t raw;
    raw.timestamp_ms = millis();
    raw.ain_mux_idx = s_ain_mux_sel; /* 快照本次 DMA 对应的 A_INx_IO 索引（trigger 已设置） */
    raw.reserved = 0;

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

/** @brief 换算状态码 → 中文描述 */
static const char* calc_status_str(srv_adc_calc_status_t st)
{
    switch (st) {
    case SRV_ADC_CALC_OK:
        return "正常";
    case SRV_ADC_CALC_ERR_PARAM:
        return "参数非法";
    case SRV_ADC_CALC_ERR_CAL:
        return "校准值无效";
    case SRV_ADC_CALC_ERR_SHORT:
        return "传感器短路";
    case SRV_ADC_CALC_ERR_OPEN:
        return "传感器开路";
    case SRV_ADC_CALC_ERR_LOW_RAW:
        return "采样值过低";
    default:
        return "未知";
    }
}

/**
 * @brief VREFINT → VDDA (mV)
 *
 * VREFINT 内部参考电压标称 1.21V，通过 ADC 采样值反推实际 VDDA：
 * VDDA = 1.21V × 4095 / VREFINT_RAW
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
 * 使用 ST 出厂校准值 (30°C / 110°C) 线性插值。
 */
static srv_adc_calc_status_t calc_mcu_temp(float ts_filtered, int16_t* temp_x100)
{
    if (temp_x100 == NULL) {
        return SRV_ADC_CALC_ERR_PARAM;
    }

    if (s_ts_cal1 == 0xFFFF || s_ts_cal2 == 0xFFFF || s_ts_cal2 == s_ts_cal1) {
        *temp_x100 = 0; /* 校准值无效 → 温度输出 0 */
        return SRV_ADC_CALC_ERR_CAL;
    }

    float t = (float)TS_CAL1_TEMP + (float)(TS_CAL2_TEMP - TS_CAL1_TEMP) * (ts_filtered - (float)s_ts_cal1) / (float)(s_ts_cal2 - s_ts_cal1);

    *temp_x100 = (int16_t)(t * 100.0f);
    return SRV_ADC_CALC_OK;
}

/**
 * @brief NTC 原始 ADC 值 → 温度 (°C × 100)
 *
 * 电路：V_ntc = VDDA × R_ntc / (R_pullup + R_ntc)
 * 换算：R_ntc = R_pullup × V_ntc / (VDDA - V_ntc)
 * 查表：线性插值
 *
 * @param raw    ADC 原始值 (12-bit)
 * @param vdda_v 校准后的 VDDA (V)
 */
static srv_adc_calc_status_t ntc_raw_to_temp(uint16_t raw, float vdda_v, int16_t* temp_x100)
{
    if (temp_x100 == NULL) {
        return SRV_ADC_CALC_ERR_PARAM;
    }

    if (raw == 0) {
        *temp_x100 = -4000; /* 短路 → 最低温哨兵值 */
        return SRV_ADC_CALC_ERR_SHORT;
    }

    float v_ntc = (float)raw * vdda_v / (float)ADC_MAX;

    if (v_ntc >= vdda_v) {
        *temp_x100 = (int16_t)(s_ntc_table[0].t * 100.0f); /* 开路 → 最低温 */
        return SRV_ADC_CALC_ERR_OPEN;
    }

    float r_ntc = NTC_PULLUP_R * v_ntc / (vdda_v - v_ntc);

    /* 线性插值查表 */
    const uint32_t last = NTC_TABLE_SIZE - 1;
    if (r_ntc >= s_ntc_table[0].r) {
        *temp_x100 = (int16_t)(s_ntc_table[0].t * 100.0f);
        return SRV_ADC_CALC_ERR_OPEN; /* 阻值超上限 → 开路 */
    }
    if (r_ntc <= s_ntc_table[last].r) {
        *temp_x100 = (int16_t)(s_ntc_table[last].t * 100.0f);
        return SRV_ADC_CALC_ERR_SHORT; /* 阻值超下限 → 短路 */
    }

    for (uint32_t i = 0; i < last; i++) {
        if (r_ntc <= s_ntc_table[i].r && r_ntc >= s_ntc_table[i + 1].r) {
            float ratio = (r_ntc - s_ntc_table[i + 1].r)
                / (s_ntc_table[i].r - s_ntc_table[i + 1].r);
            float t = s_ntc_table[i + 1].t
                + ratio * (s_ntc_table[i].t - s_ntc_table[i + 1].t);
            *temp_x100 = (int16_t)(t * 100.0f);
            return SRV_ADC_CALC_OK;
        }
    }

    *temp_x100 = 0;
    return SRV_ADC_CALC_ERR_PARAM; /* 查表未命中（理论不可达） */
}
