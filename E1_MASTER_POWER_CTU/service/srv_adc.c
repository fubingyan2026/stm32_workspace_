/**
 * @file    srv_adc.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   ADC 采样服务实现（VREFINT 校准 + 物理量换算, E1_MASTER_POWER_CTU）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_adc.h"

#include "drv_adc.h"
#include "drv_cd4051b.h"
#include "drv_status.h"
#include "drv_systick.h"
#include "filter.h"
#include "log.h"
#include "msg_fifo.h"
#include "utils_math.h"

#include <math.h>
#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_ADC_LOG_ENABLE 1

#if SRV_ADC_LOG_ENABLE
#define SRV_ADC_LOG_E(...) LOG_E("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_W(...) LOG_W("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_I(...) LOG_I("srv_adc", __VA_ARGS__)
#define SRV_ADC_LOG_D(...) ((void)0)//LOG_D("srv_adc", __VA_ARGS__)
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

/** @brief PT1 低通滤波器（每通道一个，100Hz 截止，抑制 ADC 噪声） */
#define ADC_FILTER_CUTOFF_HZ (100U)

#define ADC_MAX (4095U)
#define ADC_SAMPLE_RATE_HZ (500U) /**< 500Hz (2ms period) */

/** @brief ADC 无采样快照看门狗：连续无快照超过该时长即复位 DMA 链路 (ms) */
#define SRV_ADC_NO_SNAPSHOT_RESET_MS (100U)

/** @brief 12-bit ADC 满量程原始值 */
#define SRV_ADC_RAW_MAX (4095U)

/* 外部电压分压比（VIN 与 VIN_DC-DC 均为 220k/10k → 1/23，满量程约 75.9V） */
#define ADC_SCALE_VIN (23.0f)
#define ADC_SCALE_VIN_DCDC (23.0f)

/* VREFINT 校准：F103 标称 1.20V（ST 出厂校准值地址 0x1FFFF7BA 可选） */
#define VREFINT_CAL_MV (1200U) /**< 内部参考电压标称值 (mV) */

/* 内部温度传感器校准（F103 出厂校准缺失时回退典型参数） */
#define TS_CAL1_ADDR ((uint16_t*)0x1FFFF7B8) /**< 30°C 校准值（部分器件无，读 0xFFFF） */
#define TS_CAL2_ADDR ((uint16_t*)0x1FFFF7C2) /**< 110°C 校准值（部分器件无，读 0xFFFF） */
#define TS_CAL1_TEMP (30)
#define TS_CAL2_TEMP (110)

/** @brief 典型参数（无出厂校准时的兜底，取自 F103 数据手册） */
#define TS_TYP_V25_MV (1430.0f) /**< 25°C 时传感器典型电压 (mV) */
#define TS_TYP_SLOPE_MV_PER_DEG (4.3f) /**< 电压-温度斜率 (mV/℃) */
#define TS_TYP_T25 (25.0f)

/* NTC 参数（B 参数法，R25=10kΩ，B=3950；待实机标定确认） */
#define NTC_R25_OHM (10000.0f) /**< 25°C 标称阻值 */
#define NTC_BETA (3950.0f) /**< B 常数 */
#define NTC_T25_KELVIN (298.15f)
#define NTC_PULLUP_R (10000.0f) /**< 上偏置电阻 10kΩ */

/** @brief NTC 短路/开路原始值门限（接近满量程/接近 0 视为异常，查阻值兜底） */
#define NTC_RAW_OPEN_MIN (4085U) /**< raw ≥ 此值（Vntc≈VDDA）判开路 */
#define NTC_RAW_SHORT_MAX (10U) /**< raw ≤ 此值判短路 */

/* CD4051B 多路 E-STOP 双冗余采样：偶数通道 = E_STOPx_ADC1，奇数 = E_STOPx_ADC2 */
#define SRV_ADC_MUX_CH_NUM (8U) /**< CD4051B 通道数 (Y0~Y7) */
#define SRV_ADC_ESTOP_NUM (4U) /**< E-STOP 回路数 (E_STOP1~4) */

/** @brief E-STOP 双通道冗余容差（raw，12-bit）：两节点偏差 ≤ 此值判闭合（触点电平一致）。
 *        偏差 ≈ 满量程 → 冗余互补（急停按下断开）；中间区间 → 冗余/线缆异常 */
#define SRV_ADC_ESTOP_REDUND_TOL_RAW (256U)

/** @brief E-STOP 冗余故障去抖时间 (ms)：单路 mux 轮转下冗余节点不同时刷新，
 *        急停切换瞬间偏差会扫过中间区间约一个轮转周期(≈80ms)，去抖需覆盖之 */
#define SRV_ADC_ESTOP_FAULT_DEBOUNCE_MS (1000U)

/** @brief E-STOP 冗余故障去抖帧数（step 以 100Hz 周期运行，2ms/帧） */
#define SRV_ADC_ESTOP_FAULT_DEBOUNCE_FRAMES \
    ((SRV_ADC_ESTOP_FAULT_DEBOUNCE_MS * ADC_SAMPLE_RATE_HZ) / 1000U)

/** @brief 急停动作抑制窗口 (ms)：数字侧按下/释放边沿后窗口内跳过冗余异常诊断
 *         （mux 轮询扫描/通道切换延时产生中间区读数属正常，不判线缆异常） */
#define SRV_ADC_ESTOP_ANOM_SUPPRESS_MS (500U)

/* Private types -------------------------------------------------------------*/

/** @brief 原始采样快照 — DMA 中断回调只填此结构，不做任何换算 */
typedef struct {
    uint32_t timestamp_ms; /**< 时间戳 (ms) */
    uint8_t mux_ch; /**< 本次快照对应的 CD4051B 通道 (Y0~Y7) */
    uint16_t raw[DRV_ADC_CH_MAX]; /**< 各逻辑通道 12-bit 原始值 */
} srv_adc_raw_t;

/* Private variables ---------------------------------------------------------*/

static uint8_t s_fifo_buf[ADC_FIFO_BUF_SIZE];
static msg_fifo_t s_fifo;

/** @brief 原始快照 FIFO：DMA 中断回调(生产) → srv_adc_step(消费)，锁自由 SPSC */
static uint8_t s_raw_fifo_buf[ADC_RAW_FIFO_BUF_SIZE];
static msg_fifo_t s_raw_fifo;

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

/** @brief 无采样快照看门狗计数 (ms)，连续无快照超过阈值自动复位 ADC 链路 */
static uint32_t s_no_snapshot_ms;

/**
 * @brief 当前已选通的 CD4051B 通道 (Y0~Y7)：在上一周期 srv_adc_step 末预选（GPIO 置好），
 *        到本周期 trigger 启动转换之间已有整整一个周期稳定时间。
 *        本周期 step 末再预选下一通道（无阻塞，不需要 delay_us）。
 */
static uint8_t s_mux_gpio;

/**
 * @brief 本次转换对应的 CD4051B 通道（trigger 写入、ISR 快照标签）。
 *        与 s_mux_gpio 分离：step 末预选下一通道不影响本帧标签，避免 ISR 竞态。
 */
static volatile uint8_t s_mux_sampling;

/** @brief 各 CD4051B 通道最近一次采样值（轮转更新，跨 step 保留旧值） */
static uint16_t s_mux_raw[SRV_ADC_MUX_CH_NUM];
/** @brief 各通道是否已至少采样一次（bitN=1 → YN 已就绪） */
static uint8_t s_mux_ready_mask;

/** @brief E-STOP 冗余故障去抖计数（每回路一帧计数，达 DEBOUNCE_FRAMES 触发一次告警后清零） */
static uint16_t s_estop_redund_fault_cnt[SRV_ADC_ESTOP_NUM];

/** @brief 急停数字侧边沿检测 + 动作时间戳（供异常诊断抑制使用） */
static utils_edge_det_t s_estop_gpio_edge;
static uint32_t s_estop_gpio_change_ms;

/* Private function prototypes -----------------------------------------------*/

static void adc_sample_cb(drv_adc_inst_t inst);
static float adc_filtered(drv_adc_channel_t ch, uint16_t raw_value);
static bool warn_rate_limited(void);
static srv_adc_calc_status_t calc_vdda_mv(float vrefint_filtered, uint32_t* vdda_mv);
static srv_adc_calc_status_t calc_mcu_temp(float ts_filtered, float vdda_mv, int16_t* temp_x100);
static srv_adc_calc_status_t ntc_raw_to_temp(uint16_t raw, float vdda_v, int16_t* temp_x100);
static void estop_redundancy_check(void);

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

    /* 内部温度传感器出厂校准值：芯片固定，仅初始化读取一次（无效值回退典型参数） */
    s_ts_cal1 = *TS_CAL1_ADDR;
    s_ts_cal2 = *TS_CAL2_ADDR;
    if (s_ts_cal1 == 0xFFFF || s_ts_cal2 == 0xFFFF || s_ts_cal2 == s_ts_cal1) {
        SRV_ADC_LOG_W("内部温度出厂校准缺失 (cal30=0x%04X cal110=0x%04X)，使用典型参数换算",
            (unsigned)s_ts_cal1, (unsigned)s_ts_cal2);
    }

    /* CD4051B 多路选择器：默认选通 Y0；GPIO 预选在本周期 step 末推进（无阻塞） */
    drv_cd4051b_init();
    s_mux_gpio = 0;
    s_mux_sampling = 0;
    memset(s_mux_raw, 0, sizeof(s_mux_raw));
    s_mux_ready_mask = 0;
    memset(s_estop_redund_fault_cnt, 0, sizeof(s_estop_redund_fault_cnt));
    s_no_snapshot_ms = 0;

    s_initialized = true;

    SRV_ADC_LOG_I("ADC 采样服务初始化完成 (采样率=%uHz, 滤波截止=%uHz, FIFO=%uB)",
        (unsigned)ADC_SAMPLE_RATE_HZ, (unsigned)ADC_FILTER_CUTOFF_HZ, (unsigned)ADC_FIFO_BUF_SIZE);
    SRV_ADC_LOG_I("内部温度校准值: cal30=0x%04X cal110=0x%04X",
        (unsigned)s_ts_cal1, (unsigned)s_ts_cal2);
}

void srv_adc_trigger(void)
{
    /* 本周期要采样的通道由上一周期 step 末预选完成（GPIO 已置好约一个采样周期），
       此处仅记录本次转换的通道标签并启动 DMA，不做切换、不阻塞。 */
    s_mux_sampling = s_mux_gpio;
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
        /* 看门狗：连续无快照说明 ADC DMA 完成回调丢失/busy 卡死，
           超时后强制复位 ADC 链路，避免 485 高频查询后 ADC 永久停采 */
        s_no_snapshot_ms += 2U; /* 本周期约 2ms */
        if (s_no_snapshot_ms >= SRV_ADC_NO_SNAPSHOT_RESET_MS) {
            s_no_snapshot_ms = 0;
            for (uint32_t i = 0; i < DRV_ADC_INST_NUM; i++) {
                (void)drv_adc_recover((drv_adc_inst_t)i);
            }
            SRV_ADC_LOG_W("ADC 连续 %ums 无采样快照，已复位 DMA 链路",
                (unsigned)SRV_ADC_NO_SNAPSHOT_RESET_MS);
        }
        return;
    }
    s_no_snapshot_ms = 0;

    srv_adc_data_t s;
    memset(&s, 0, sizeof(s));
    s.timestamp_ms = raw.timestamp_ms;

    /* ── CD4051B 多路采样：仅更新本次快照对应的一路（用未滤波原始值，
     *    避免 PT1 滤波把各通道混叠）── */
    if (raw.mux_ch < SRV_ADC_MUX_CH_NUM) {
        s_mux_raw[raw.mux_ch] = raw.raw[DRV_ADC_CH_CD4051B];
        s_mux_ready_mask |= (uint8_t)(1U << raw.mux_ch);
    }

    /* 本次转换已结束（ISR 已入队该帧），此刻预选下一通道：
       GPIO 于此处切换后，到下周期 trigger 的 Rank5(PC4) 采样之间隔整整一个采样周期，
       无阻塞、无需 delay_us，硬件有充足稳定时间。 */
    s_mux_gpio = (uint8_t)(s_mux_gpio + 1U);
    if (s_mux_gpio >= SRV_ADC_MUX_CH_NUM) {
        s_mux_gpio = 0;
    }
    (void)drv_cd4051b_select(s_mux_gpio);

    /* E-STOP 双通道冗余状态巡检（仅闭环上每拍运行，日志受限频） */
    estop_redundancy_check();

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
    s.vin_dcdc_mv = (uint32_t)(adc_filtered(DRV_ADC_CH_VIN_DC_DC, raw.raw[DRV_ADC_CH_VIN_DC_DC]) * raw_to_mv * ADC_SCALE_VIN_DCDC);

    /* ── NTC 温度 (使用校准后的 VDDA) ── */
    s.ntc1_status = ntc_raw_to_temp(
        (uint16_t)adc_filtered(DRV_ADC_CH_NTC1, raw.raw[DRV_ADC_CH_NTC1]), vdda_v, &s.ntc1_temp_x100);
    s.ntc2_status = ntc_raw_to_temp(
        (uint16_t)adc_filtered(DRV_ADC_CH_NTC2, raw.raw[DRV_ADC_CH_NTC2]), vdda_v, &s.ntc2_temp_x100);

    /* ── MCU 内部温度 ── */
    s.mcu_temp_status = calc_mcu_temp(
        adc_filtered(DRV_ADC_CH_TEMPSENSOR, raw.raw[DRV_ADC_CH_TEMPSENSOR]),
        (float)vdda_mv, &s.mcu_temp_x100);

    /* 遥测日志（任务上下文，限频 1s；温度字段为 ×100 整数，单位 0.01°C） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_tele_log_ts) >= SRV_ADC_TELE_LOG_PERIOD_MS) {
        s_tele_log_ts = now_ms;

        SRV_ADC_LOG_D("vdda=%umV vin=%umV vin_dcdc=%umV mcuT=%d ntc1=%d ntc2=%d(×100)",
            (unsigned)s.vdda_mv, (unsigned)s.vin_mv, (unsigned)s.vin_dcdc_mv,
            (int)s.mcu_temp_x100, (int)s.ntc1_temp_x100, (int)s.ntc2_temp_x100);
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
    raw.mux_ch = s_mux_sampling; /* 本次转换对应的 CD4051B 通道（trigger 写入） */

    for (uint32_t i = 0; i < DRV_ADC_CH_MAX; i++) {
        raw.raw[i] = (uint16_t)drv_adc_read_raw((drv_adc_channel_t)i);
    }

    msg_fifo_push(&s_raw_fifo, &raw);
}

uint16_t srv_adc_mux_raw(uint8_t ch)
{
    if (ch >= SRV_ADC_MUX_CH_NUM) {
        return 0;
    }
    return s_mux_raw[ch];
}

uint8_t srv_adc_estop_closed_mask(void)
{
    uint8_t mask = 0;

    for (uint8_t i = 0; i < SRV_ADC_ESTOP_NUM; i++) {
        /* E_STOP(i+1)：偶数 mux 通道 = ADC1 节点，奇数 = ADC2 节点 */
        const int16_t adc1 = (int16_t)s_mux_raw[(uint8_t)(2U * i)];
        const int16_t adc2 = (int16_t)s_mux_raw[(uint8_t)(2U * i + 1U)];
        const int16_t diff = adc1 - adc2;
        const int16_t adiff = (diff < 0) ? (int16_t)-diff : diff;

        /* 两节点电平一致（偏差 ≈0）判闭合；互补/中间偏差均判开路 */
        if (adiff <= (int16_t)SRV_ADC_ESTOP_REDUND_TOL_RAW) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

bool srv_adc_estop_valid(void)
{
    /* 全部通道完成至少一轮采样才有效（首个完整轮转约 80ms） */
    return s_initialized && s_mux_ready_mask == 0xFFU;
}

static float adc_filtered(drv_adc_channel_t ch, uint16_t raw_value)
{
    return pt1FilterApply(&s_filters[ch], (float)raw_value);
}

/**
 * @brief E-STOP 双通道冗余巡检（主循环上下文，每 step 调用）
 *
 * 每回路两冗余节点（偶数 mux=ADC1、奇数=ADC2）。偏差处于中间区间
 * （既不一致也不互补）判冗余通道失效/线缆异常；去抖后仅报一次。
 * 仅诊断告警，不影响闭合掩码/急停判定（由 srv_pwr_det 消费）。
 */
static void estop_redundancy_check(void)
{
    if (!srv_adc_estop_valid()) {
        return; /* 首轮轮转未完成，不做诊断 */
    }

    /* 急停动作抑制：数字侧按下/释放边沿 → 记录时刻；
       按下期间或边沿后 SUPPRESS 窗口内，冗余中间态偏差属轮询切换正常现象，
       跳过诊断并清计数，避免松开/按下过程误报。 */
    const bool estop_digital = drv_status_read(DRV_STATUS_E_STOP_ON);
    const uint32_t now_ms = millis();
    if (utils_edge_detect(&s_estop_gpio_edge, estop_digital) != UTILS_EDGE_NONE) {
        s_estop_gpio_change_ms = now_ms;
    }
    if (estop_digital
        || (uint32_t)(now_ms - s_estop_gpio_change_ms) < SRV_ADC_ESTOP_ANOM_SUPPRESS_MS) {
        for (uint8_t i = 0; i < SRV_ADC_ESTOP_NUM; i++) {
            s_estop_redund_fault_cnt[i] = 0;
        }
        return;
    }

    for (uint8_t i = 0; i < SRV_ADC_ESTOP_NUM; i++) {
        const int16_t adc1 = (int16_t)s_mux_raw[(uint8_t)(2U * i)];
        const int16_t adc2 = (int16_t)s_mux_raw[(uint8_t)(2U * i + 1U)];
        const int16_t diff = adc1 - adc2;
        const int16_t adiff = (diff < 0) ? (int16_t)-diff : diff;
        const int16_t tol = (int16_t)SRV_ADC_ESTOP_REDUND_TOL_RAW;
        const int16_t full = (int16_t)SRV_ADC_RAW_MAX;

        const bool abnormal = !(adiff <= tol || adiff >= full - tol);
        if (abnormal) {
            if (s_estop_redund_fault_cnt[i] < SRV_ADC_ESTOP_FAULT_DEBOUNCE_FRAMES) {
                s_estop_redund_fault_cnt[i]++;
                if (s_estop_redund_fault_cnt[i] >= SRV_ADC_ESTOP_FAULT_DEBOUNCE_FRAMES) {
                    /* 持续异常达去抖阈值，仅报一次；恢复（计数清零）后再次异常才会重报 */
                    SRV_ADC_LOG_E("急停 E_STOP%u 冗余通道偏差异常: ADC1=%u ADC2=%u 偏差=%+d (正常应≈0或≈±4095)",
                        (unsigned)i + 1U, (unsigned)adc1, (unsigned)adc2, (int)diff);
                }
            }
        } else {
            s_estop_redund_fault_cnt[i] = 0;
        }
    }
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
static srv_adc_calc_status_t calc_mcu_temp(float ts_filtered, float vdda_mv, int16_t* temp_x100)
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
 * @brief NTC 原始 ADC 值 → 温度 (°C × 100)
 *
 * 电路：V_ntc = VDDA × R_ntc / (R_pullup + R_ntc)（NTC 为下臂）
 * 换算：R_ntc = R_pullup × V_ntc / (VDDA - V_ntc)
 * 温度：T = 1/(ln(R/R25)/B + 1/T25) − 273.15
 *
 * @param raw    ADC 原始值 (12-bit)
 * @param vdda_v 校准后的 VDDA (V)
 */
static srv_adc_calc_status_t ntc_raw_to_temp(uint16_t raw, float vdda_v, int16_t* temp_x100)
{
    if (temp_x100 == NULL) {
        return SRV_ADC_CALC_ERR_PARAM;
    }

    if (raw <= NTC_RAW_SHORT_MAX) {
        *temp_x100 = -4000; /* 短路 → 最低温哨兵值 */
        return SRV_ADC_CALC_ERR_SHORT;
    }
    if (raw >= NTC_RAW_OPEN_MIN) {
        *temp_x100 = -4000; /* 开路 → 最低温哨兵值 */
        return SRV_ADC_CALC_ERR_OPEN;
    }

    float v_ntc = (float)raw * vdda_v / (float)ADC_MAX;
    if (v_ntc >= vdda_v) {
        *temp_x100 = -4000;
        return SRV_ADC_CALC_ERR_OPEN;
    }

    float r_ntc = NTC_PULLUP_R * v_ntc / (vdda_v - v_ntc);
    if (r_ntc <= 1.0f) {
        *temp_x100 = -4000;
        return SRV_ADC_CALC_ERR_SHORT;
    }

    /* B 参数方程求解温度 */
    const float inv_t = 1.0f / NTC_T25_KELVIN
        + logf(r_ntc / NTC_R25_OHM) / NTC_BETA;
    float t_c = 1.0f / inv_t - 273.15f;

    if (t_c < -50.0f) {
        t_c = -50.0f;
    }
    if (t_c > 150.0f) {
        t_c = 150.0f;
    }

    *temp_x100 = (int16_t)(t_c * 100.0f);
    return SRV_ADC_CALC_OK;
}
