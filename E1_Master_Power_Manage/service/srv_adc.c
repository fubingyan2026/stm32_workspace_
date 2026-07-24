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
#include "drv_systick.h"
#include "filter.h"
#include "msg_fifo.h"

/* Private constants ---------------------------------------------------------*/

#define ADC_FIFO_BUF_SIZE (1024U)

#define ADC_MAX (4095U)
#define ADC_SAMPLE_RATE_HZ (100U) /**< 100Hz (10ms period) */

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
#define TS_CAL1_ADDR ((uint16_t*)0x1FFF7A2C) /**< 30°C 校准值 */
#define TS_CAL2_ADDR ((uint16_t*)0x1FFF7A2E) /**< 110°C 校准值 */
#define TS_CAL1_TEMP (30)
#define TS_CAL2_TEMP (110)

/* VBAT 分压比 (内部 4x 桥式分压 → 实际电压 = raw * 4) */
#define VBAT_SCALE (4.0f)

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

/* Private variables ---------------------------------------------------------*/

static uint8_t s_fifo_buf[ADC_FIFO_BUF_SIZE];
static msg_fifo_t s_fifo;

/** @brief PT1 低通滤波器（每通道一个，4Hz 截止，抑制 ADC 噪声） */
#define ADC_FILTER_CUTOFF_HZ (4U)

static pt1Filter_t s_filters[DRV_ADC_CH_MAX];

/* Private function prototypes -----------------------------------------------*/

static void adc_sample_cb(drv_adc_inst_t inst);
static float adc_filtered(drv_adc_channel_t ch);
static uint32_t calc_vdda_mv(float vrefint_filtered);
static int16_t calc_mcu_temp(float ts_filtered);
static int16_t ntc_raw_to_temp(uint16_t raw, float vdda_v);

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
}

void srv_adc_trigger(void)
{
    drv_adc_trigger_all();
}

bool srv_adc_get_latest(srv_adc_data_t* sample)
{
    if (!sample)
        return false;
    return msg_fifo_pop(&s_fifo, sample);
}

/* Private functions ---------------------------------------------------------*/

static void adc_sample_cb(drv_adc_inst_t inst)
{
    (void)inst;

    srv_adc_data_t s;
    s.timestamp_ms = millis();

    /* ── VREFINT 校准：反推实际 VDDA ── */
    float vref = adc_filtered(DRV_ADC_CH_VREFINT);
    s.vdda_mv = calc_vdda_mv(vref);
    float vdda_v = (float)s.vdda_mv / 1000.0f;

    /* ── 外部电压 (mV) ── */
    float raw_to_v = vdda_v / (float)ADC_MAX;
    s.vin_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_VIN) * raw_to_v * ADC_SCALE_VIN);
    s.motor_power_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_MOTOR_POWER) * raw_to_v * ADC_SCALE_MOTOR_POWER);
    s.aux_power_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_AUX_POWER) * raw_to_v * ADC_SCALE_AUX_POWER);

    /* ── E-STOP 双通道冗余 (12-bit 原始值) ── */
    s.e_stop1_adc1 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP1_ADC1);
    s.e_stop1_adc2 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP1_ADC2);
    s.e_stop2_adc1 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP2_ADC1);
    s.e_stop2_adc2 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP2_ADC2);
    s.e_stop3_adc1 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP3_ADC1);
    s.e_stop3_adc2 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP3_ADC2);
    s.e_stop4_adc1 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP4_ADC1);
    s.e_stop4_adc2 = (uint16_t)adc_filtered(DRV_ADC_CH_E_STOP4_ADC2);

    /* ── NTC 温度 (使用校准后的 VDDA) ── */
    s.ntc1_temp_x100 = ntc_raw_to_temp((uint16_t)adc_filtered(DRV_NTC1_ADC), vdda_v);
    s.ntc2_temp_x100 = ntc_raw_to_temp((uint16_t)adc_filtered(DRV_NTC2_ADC), vdda_v);

    /* ── MCU 内部温度 ── */
    s.mcu_temp_x100 = calc_mcu_temp(adc_filtered(DRV_ADC_CH_TEMPSENSOR));

    /* ── VBAT 备份电池电压 ── */
    s.vbat_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_VBAT) * raw_to_v * VBAT_SCALE);

    msg_fifo_push(&s_fifo, &s);
}

static float adc_filtered(drv_adc_channel_t ch)
{
    uint32_t raw = drv_adc_read_raw(ch);
    return pt1FilterApply(&s_filters[ch], (float)raw);
}

/**
 * @brief VREFINT → VDDA (mV)
 *
 * VREFINT 内部参考电压标称 1.21V，通过 ADC 采样值反推实际 VDDA：
 * VDDA = 1.21V × 4095 / VREFINT_RAW
 */
static uint32_t calc_vdda_mv(float vrefint_filtered)
{
    if (vrefint_filtered < 100.0f) {
        return 3300; /* 无效值 → 回退默认 3.3V */
    }
    return (uint32_t)((float)VREFINT_CAL_MV * (float)ADC_MAX / vrefint_filtered);
}

/**
 * @brief 内部温度传感器 → 温度 (°C × 100)
 *
 * 使用 ST 出厂校准值 (30°C / 110°C) 线性插值。
 */
static int16_t calc_mcu_temp(float ts_filtered)
{
    uint16_t ts_cal1 = *TS_CAL1_ADDR;
    uint16_t ts_cal2 = *TS_CAL2_ADDR;

    if (ts_cal1 == 0xFFFF || ts_cal2 == 0xFFFF || ts_cal2 == ts_cal1) {
        return 0; /* 校准值无效 */
    }

    float t = (float)TS_CAL1_TEMP
        + (float)(TS_CAL2_TEMP - TS_CAL1_TEMP)
            * (ts_filtered - (float)ts_cal1)
            / (float)(ts_cal2 - ts_cal1);

    return (int16_t)(t * 100.0f);
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
static int16_t ntc_raw_to_temp(uint16_t raw, float vdda_v)
{
    if (raw == 0)
        return -4000; /* 短路 → 最低温 */

    float v_ntc = (float)raw * vdda_v / (float)ADC_MAX;

    if (v_ntc >= vdda_v)
        return (int16_t)(s_ntc_table[0].t * 100.0f); /* 开路 → 最低温 */

    float r_ntc = NTC_PULLUP_R * v_ntc / (vdda_v - v_ntc);

    /* 线性插值查表 */
    const uint32_t last = NTC_TABLE_SIZE - 1;
    if (r_ntc >= s_ntc_table[0].r)
        return (int16_t)(s_ntc_table[0].t * 100.0f);
    if (r_ntc <= s_ntc_table[last].r)
        return (int16_t)(s_ntc_table[last].t * 100.0f);

    for (uint32_t i = 0; i < last; i++) {
        if (r_ntc <= s_ntc_table[i].r && r_ntc >= s_ntc_table[i + 1].r) {
            float ratio = (r_ntc - s_ntc_table[i + 1].r)
                / (s_ntc_table[i].r - s_ntc_table[i + 1].r);
            float t = s_ntc_table[i + 1].t
                + ratio * (s_ntc_table[i].t - s_ntc_table[i + 1].t);
            return (int16_t)(t * 100.0f);
        }
    }
    return 0;
}
