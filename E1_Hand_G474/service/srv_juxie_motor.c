/**
 * @file    srv_juxie_motor.c
 * @brief   橘虾(juxie) CAN FD MIT 电机服务实现
 *
 * 单机控制帧（0x110|ID, DLC 9, FD+BRS）每 1ms 重发，控制帧自动触发电机反馈帧
 * （0x300|ID, DLC 16/12），on_rx 在 ISR 中仅解析存储。
 * 上电默认 Kp=Kd=Tq=0、失能、抱闸吸合，安全无突动。
 */

#include "srv_juxie_motor.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/
#define SRV_JUXIE_LOG_ENABLE 1

#if SRV_JUXIE_LOG_ENABLE
#define SRV_JUXIE_LOG_I(...) LOG_I("srv_juxie_motor", __VA_ARGS__)
#define SRV_JUXIE_LOG_W(...) LOG_W("srv_juxie_motor", __VA_ARGS__)
#define SRV_JUXIE_LOG_E(...) LOG_E("srv_juxie_motor", __VA_ARGS__)
#else
#define SRV_JUXIE_LOG_I(...) ((void)0)
#define SRV_JUXIE_LOG_W(...) ((void)0)
#define SRV_JUXIE_LOG_E(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 电机 Dev_ID（可改；多控/单轴帧 ID 按此合成） */
#define SRV_JUXIE_DEV_ID 1U
/** @brief MIT 单轴控制帧 ID 基址（0x110 | Dev_ID） */
#define SRV_JUXIE_CAN_ID_CTRL_BASE 0x110U
/** @brief 反馈帧 ID 基址（0x300 | Dev_ID） */
#define SRV_JUXIE_CAN_ID_FB_BASE 0x300U
/** @brief MIT 控制帧长度（DLC 9，CAN FD） */
#define SRV_JUXIE_MIT_LEN 9U
/** @brief 带力矩传感器反馈帧长度 */
#define SRV_JUXIE_FB_LEN_FULL 16U
/** @brief 不带力矩传感器反馈帧长度 */
#define SRV_JUXIE_FB_LEN_SHORT 12U

/** @brief 电机失联判定周期 (ms)：超过该时长未收到反馈视为掉线 */
#define SRV_JUXIE_OFFLINE_MS 200U

/** @brief 默认位置量程 Pos_Max (0.1°)，即 180.0° */
#define SRV_JUXIE_DEFAULT_POS_MAX_X10DEG 1800U
/** @brief 默认速度量程 Vel_Max (rpm) */
#define SRV_JUXIE_DEFAULT_VEL_MAX_RPM 3000U
/** @brief 默认力矩量程 T_Max (0.01Nm)，即 50.00Nm */
#define SRV_JUXIE_DEFAULT_TQ_MAX_001NM 5000U

/** @brief Kp 满量程物理值（juxie §4.1：12bit 4095 ↔ 500） */
#define SRV_JUXIE_KP_FULL_SCALE 500
/** @brief Kd 满量程物理值（12bit 4095 ↔ 5） */
#define SRV_JUXIE_KD_FULL_SCALE 5

/** @brief 电机标零 SDO 写帧 CAN ID（CANopen SDO 请求，节点 1） */
#define SRV_JUXIE_SDO_ZERO_ID 0x601U
/** @brief 电机标零 SDO 载荷：写 Index 0x2531 Sub 0 = 1（当前角度置零） */
static const uint8_t s_sdo_zero_data[8] = { 0x23, 0x31, 0x25, 0x00, 0x01, 0x00, 0x00, 0x00 };

/** @brief CAN1 接收数据低频日志周期 (ms)：1s 一条，便于观察电机反馈 */
#define SRV_JUXIE_STATUS_LOG_MS 1000U

/** @brief MIT 控制频率限制：500Hz（周期 2ms） */
#define SRV_JUXIE_CTRL_PERIOD_MS 2U

/** @brief 同步帧周期发送使能：1=周期发 0x80 同步帧触发电机反馈广播（juxie §2.1 触发方式 1） */
#define SRV_JUXIE_SYNC_ENABLE 1
/** @brief 同步帧 ID */
#define SRV_JUXIE_SYNC_ID 0x80U
/** @brief 同步帧发送周期 (ms) */
#define SRV_JUXIE_SYNC_MS 10U

/* Private types -------------------------------------------------------------*/

/** @brief CAN1 最近接收帧快照（ISR 写，主循环低频打印用） */
typedef struct {
    uint32_t id;    /**< CAN ID */
    uint8_t dlc;    /**< 数据长度 */
    uint8_t data[16]; /**< 数据负载 */
} srv_juxie_last_rx_t;

/* Private variables ---------------------------------------------------------*/

/** @brief 本服务绑定的 CAN 总线（接线处注入，通道无关） */
static const srv_can_bus_t* s_bus;

/** @brief 电机配置与 MIT 目标（主循环写，step 读） */
static srv_juxie_cfg_t s_cfg;

/** @brief 最新电机反馈（ISR 写，主循环读） */
static srv_juxie_fb_t s_fb;

/** @brief 是否收到过反馈帧（区分「从未应答」与「掉线」） */
static bool s_fb_seen;

/** @brief 上次在线状态（边沿检测，避免重复日志） */
static bool s_online_last;

/** @brief CAN1 最近接收帧快照（ISR 写，主循环低频打印） */
static srv_juxie_last_rx_t s_last_rx;

/** @brief CAN1 总线收到任意帧计数（ISR 写，用于确认电机是否回帧） */
static uint32_t s_last_rx_cnt;

/** @brief 上次低频 RX 日志时间 (millis) */
static uint32_t s_last_status_ms;

/** @brief 上次同步帧发送时间 (millis) */
static uint32_t s_sync_last_ms;

/** @brief 上次 MIT 控制帧发送时间 (millis)，用于 500Hz 限频 */
static uint32_t s_mit_last_ms;

/* Private function prototypes -----------------------------------------------*/

static void srv_juxie_pack_mit(uint8_t* buf, uint16_t pos, uint16_t vel,
    uint16_t kp, uint16_t kd, uint16_t tq);
static void srv_juxie_update_online(uint32_t now);
static void srv_juxie_build_hex(char* out, const uint8_t* data, uint8_t len);
static void srv_juxie_log_rx_status(uint32_t now);

/* Exported functions --------------------------------------------------------*/

void srv_juxie_motor_init(const srv_can_bus_t* bus)
{
    if (!bus) {
        return;
    }
    s_bus = bus;

    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.pos_max_x10deg = SRV_JUXIE_DEFAULT_POS_MAX_X10DEG;
    s_cfg.vel_max_rpm = SRV_JUXIE_DEFAULT_VEL_MAX_RPM;
    s_cfg.tq_max_001nm = SRV_JUXIE_DEFAULT_TQ_MAX_001NM;

    memset(&s_fb, 0, sizeof(s_fb));
    memset(&s_last_rx, 0, sizeof(s_last_rx));
    s_fb_seen = false;
    s_online_last = false;
    s_last_rx_cnt = 0;
    s_last_status_ms = millis();
    s_sync_last_ms = millis();
    s_mit_last_ms = millis();

    SRV_JUXIE_LOG_I("init: Dev_ID=%u (ctrl 0x%03X, fb 0x%03X), Pos_Max=%d (0.1deg), Vel_Max=%u rpm, T_Max=%d (0.01Nm)",
        SRV_JUXIE_DEV_ID,
        SRV_JUXIE_CAN_ID_CTRL_BASE | SRV_JUXIE_DEV_ID,
        SRV_JUXIE_CAN_ID_FB_BASE | SRV_JUXIE_DEV_ID,
        (unsigned)s_cfg.pos_max_x10deg,
        (unsigned)s_cfg.vel_max_rpm,
        (unsigned)s_cfg.tq_max_001nm);
}

void srv_juxie_motor_step(void)
{
    if (!s_bus) {
        return;
    }

    uint32_t now = millis();

    /* 在线/掉线状态更新与边沿日志（主循环上下文） */
    srv_juxie_update_online(now);

    /* 低频打印 CAN1 接收数据（调试用，1s 一条） */
    srv_juxie_log_rx_status(now);

#if SRV_JUXIE_SYNC_ENABLE
    /* 周期发送同步帧触发反馈广播（反馈触发方式：同步帧/多控帧/单轴控帧） */
    if ((now - s_sync_last_ms) >= SRV_JUXIE_SYNC_MS) {
        s_sync_last_ms = now;
        if (srv_can_bus_tx_ready(s_bus)) {
            static const drv_can_msg_t sync = {
                .id = SRV_JUXIE_SYNC_ID,
                .is_extended = false,
                .is_fd = false,
                .dlc = 0,
            };
            (void)srv_can_bus_send(s_bus, &sync);
        }
    }
#endif

    /* 500Hz 限频：MIT 控制帧每 2ms 下发一帧 */
    if ((now - s_mit_last_ms) < SRV_JUXIE_CTRL_PERIOD_MS) {
        return;
    }
    s_mit_last_ms = now;

    if (!srv_can_bus_tx_ready(s_bus)) {
        return;
    }

    drv_can_msg_t tx = {
        .id = SRV_JUXIE_CAN_ID_CTRL_BASE | SRV_JUXIE_DEV_ID,
        .is_extended = false,
        .is_fd = true,
        .brs = true, /* 巨蟹电机为 FD+BRS；数据段=仲裁段=1M，位速率无差别 */
        .dlc = SRV_JUXIE_MIT_LEN,
    };

    tx.data[0] = (uint8_t)(((s_cfg.enable & 1U) << 7)
        | ((s_cfg.brake & 1U) << 6)
        | ((s_cfg.clear_err & 1U) << 5)
        | 0x0CU); /* bit[4:1]=0x06 MIT，bit0 预留 */
    srv_juxie_pack_mit(tx.data, s_cfg.pos_raw, s_cfg.vel_raw,
        s_cfg.kp_raw, s_cfg.kd_raw, s_cfg.tq_raw);

    if (srv_can_bus_send(s_bus, &tx) == DRV_CAN_OK) {
        /* 清错误为脉冲：发完一帧自动复位，避免持续复位电机错误状态 */
        s_cfg.clear_err = 0;
    }
}

void srv_juxie_motor_on_rx(const drv_can_msg_t* msg, void* user_data)
{
    (void)user_data;

    if (!msg || !s_bus) {
        return;
    }

    /* 先快照总线上的所有接收帧（含非电机反馈 ID），供主循环低频打印定位 */
    s_last_rx.id = msg->id;
    s_last_rx.dlc = (msg->dlc <= 16U) ? msg->dlc : 16U;
    memcpy(s_last_rx.data, msg->data, s_last_rx.dlc);
    s_last_rx_cnt++;

    if (msg->is_extended) {
        return;
    }
    if (msg->id != (SRV_JUXIE_CAN_ID_FB_BASE | SRV_JUXIE_DEV_ID)) {
        return;
    }
    if ((msg->dlc != SRV_JUXIE_FB_LEN_FULL) && (msg->dlc != SRV_JUXIE_FB_LEN_SHORT)) {
        return;
    }

    /* 负载端位置 ±32768 ↔ ±180°，换算 0.01° */
    const int16_t pos_raw = (int16_t)(((uint16_t)msg->data[0] << 8) | msg->data[1]);
    s_fb.pos_deg_x100 = ((int32_t)pos_raw * 18000) / 32768;

    s_fb.speed_rpm = (int16_t)(((uint16_t)msg->data[2] << 8) | msg->data[3]);
    s_fb.iq_ma = (int16_t)(((uint16_t)msg->data[4] << 8) | msg->data[5]);
    s_fb.err = (uint16_t)(((uint16_t)msg->data[6] << 8) | msg->data[7]);
    s_fb.temp_x10 = (int16_t)(((uint16_t)msg->data[8] << 8) | msg->data[9]);

    if (msg->dlc == SRV_JUXIE_FB_LEN_FULL) {
        /* 力矩反馈 0.05Nm/LSB → 0.01Nm（饱和到 int16） */
        const int32_t tq_x20 = (int32_t)(int16_t)(((uint16_t)msg->data[10] << 8) | msg->data[11]);
        int32_t tq_x100 = tq_x20 * 5;
        if (tq_x100 > INT16_MAX) {
            tq_x100 = INT16_MAX;
        } else if (tq_x100 < INT16_MIN) {
            tq_x100 = INT16_MIN;
        }
        s_fb.tq_001nm = (int16_t)tq_x100;
        s_fb.mode = msg->data[14];
        s_fb.status = msg->data[15];
    } else {
        s_fb.tq_001nm = 0;
        s_fb.mode = msg->data[10];
        s_fb.status = msg->data[11];
    }

    s_fb.seq++;
    s_fb.last_seen_ms = millis();
    s_fb_seen = true;
}

void srv_juxie_motor_set_mit_raw(uint16_t pos_raw, uint16_t vel_raw,
    uint16_t kp_raw, uint16_t kd_raw, uint16_t tq_raw)
{
    s_cfg.pos_raw = pos_raw;
    s_cfg.vel_raw = vel_raw & 0x0FFFU;
    s_cfg.kp_raw = kp_raw & 0x0FFFU;
    s_cfg.kd_raw = kd_raw & 0x0FFFU;
    s_cfg.tq_raw = tq_raw & 0x0FFFU;
}

bool srv_juxie_motor_config(uint8_t param, int16_t value)
{
    switch (param) {
    case SRV_JUXIE_PARAM_ENABLE:
        s_cfg.enable = (value != 0) ? 1U : 0U;
        SRV_JUXIE_LOG_I("enable=%u", (unsigned)s_cfg.enable);
        return true;

    case SRV_JUXIE_PARAM_CLEAR_ERR:
        s_cfg.clear_err = (value != 0) ? 1U : 0U;
        SRV_JUXIE_LOG_I("clear_err request queued");
        return true;

    case SRV_JUXIE_PARAM_BRAKE:
        s_cfg.brake = (value != 0) ? 1U : 0U;
        SRV_JUXIE_LOG_I("brake=%u (1=release)", (unsigned)s_cfg.brake);
        return true;

    case SRV_JUXIE_PARAM_POS_MAX:
        if (value <= 0) {
            return false;
        }
        s_cfg.pos_max_x10deg = (uint16_t)value;
        SRV_JUXIE_LOG_I("Pos_Max=%d (0.1deg)", (int)value);
        return true;

    case SRV_JUXIE_PARAM_VEL_MAX:
        if (value <= 0) {
            return false;
        }
        s_cfg.vel_max_rpm = (uint16_t)value;
        SRV_JUXIE_LOG_I("Vel_Max=%d rpm", (int)value);
        return true;

    case SRV_JUXIE_PARAM_TQ_MAX:
        if (value <= 0) {
            return false;
        }
        s_cfg.tq_max_001nm = (uint16_t)value;
        SRV_JUXIE_LOG_I("T_Max=%d (0.01Nm)", (int)value);
        return true;

    default:
        return false;
    }
}

const srv_juxie_fb_t* srv_juxie_motor_get_fb(void)
{
    return &s_fb;
}

const srv_juxie_cfg_t* srv_juxie_motor_get_cfg(void)
{
    return &s_cfg;
}

bool srv_juxie_motor_zero(void)
{
    if (!s_bus || !srv_can_bus_tx_ready(s_bus)) {
        return false;
    }

    drv_can_msg_t tx = {
        .id = SRV_JUXIE_SDO_ZERO_ID,
        .is_extended = false,
        .is_fd = true, /* 巨蟹电机为 CAN FD，标零帧同样按 FD 发送 */
        .brs = true,   /* FD+BRS；数据段=仲裁段=1M */
        .dlc = 8,
    };
    memcpy(tx.data, s_sdo_zero_data, sizeof(s_sdo_zero_data));

    if (srv_can_bus_send(s_bus, &tx) != DRV_CAN_OK) {
        return false;
    }

    SRV_JUXIE_LOG_I("motor zero sent (CAN 0x%03X: SDO 0x2531=1)", SRV_JUXIE_SDO_ZERO_ID);
    return true;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief MIT 单轴控制帧载荷打包（docs/juxie_canfd_cmd.md §4.1）
 * @param buf 目标缓冲区（≥9 字节；buf[0] 由调用方填控制指令头）
 * @param pos 目标位置 16bit 原始值（大端 Byte[1~2]）
 * @param vel 目标速度 12bit 原始值
 * @param kp  Kp 12bit 原始值
 * @param kd  Kd 12bit 原始值
 * @param tq  目标力矩 12bit 原始值
 */
static void srv_juxie_pack_mit(uint8_t* buf, uint16_t pos, uint16_t vel,
    uint16_t kp, uint16_t kd, uint16_t tq)
{
    buf[1] = (uint8_t)(pos >> 8);
    buf[2] = (uint8_t)(pos & 0xFFU);
    buf[3] = (uint8_t)(vel >> 4);
    buf[4] = (uint8_t)(((vel & 0x0FU) << 4) | ((kp >> 8) & 0x0FU));
    buf[5] = (uint8_t)(kp & 0xFFU);
    buf[6] = (uint8_t)(kd >> 4);
    buf[7] = (uint8_t)(((kd & 0x0FU) << 4) | ((tq >> 8) & 0x0FU));
    buf[8] = (uint8_t)(tq & 0xFFU);
}

/**
 * @brief 在线/掉线状态边沿检测与日志（step 主循环上下文调用）
 * @param now 当前时间 (millis)
 */
static void srv_juxie_update_online(uint32_t now)
{
    const bool online = s_fb_seen && ((now - s_fb.last_seen_ms) < SRV_JUXIE_OFFLINE_MS);
    if (online != s_online_last) {
        s_online_last = online;
        if (online) {
            SRV_JUXIE_LOG_I("motor Dev_ID=%u online", SRV_JUXIE_DEV_ID);
        } else {
            SRV_JUXIE_LOG_W("motor Dev_ID=%u offline/no response", SRV_JUXIE_DEV_ID);
        }
    }
}

/**
 * @brief 字节数组转大写 HEX 字符串（空格分隔）
 * @param out  输出缓冲（长度 ≥ 3×len，含结尾 '\0'）
 * @param data 数据指针
 * @param len  数据长度
 */
static void srv_juxie_build_hex(char* out, const uint8_t* data, uint8_t len)
{
    static const char hexdig[] = "0123456789ABCDEF";
    uint8_t pos = 0;
    for (uint8_t i = 0; i < len; i++) {
        out[pos++] = hexdig[(data[i] >> 4) & 0x0FU];
        out[pos++] = hexdig[data[i] & 0x0FU];
        if ((i + 1) < len) {
            out[pos++] = ' ';
        }
    }
    out[pos] = '\0';
}

/**
 * @brief 低频打印 CAN1 接收的数据（原始帧 HEX + 解析反馈，1s 一条）
 * @param now 当前时间 (millis)
 */
static void srv_juxie_log_rx_status(uint32_t now)
{
    if ((now - s_last_status_ms) < SRV_JUXIE_STATUS_LOG_MS) {
        return;
    }
    s_last_status_ms = now;

    char hex[3 * 16 + 1];
    srv_juxie_build_hex(hex, s_last_rx.data, s_last_rx.dlc);

    SRV_JUXIE_LOG_I("CAN1 rx: id=0x%03X dlc=%u data=%s rx_cnt=%lu | motor: pos=%ld(0.01deg) spd=%d rpm iq=%d mA tq=%d(0.01Nm) temp=%d(0.1C) err=0x%04X mode=%u st=0x%02X seq=%lu age=%lu ms",
        (unsigned)s_last_rx.id,
        (unsigned)s_last_rx.dlc,
        hex,
        (unsigned long)s_last_rx_cnt,
        (long)s_fb.pos_deg_x100,
        (int)s_fb.speed_rpm,
        (int)s_fb.iq_ma,
        (int)s_fb.tq_001nm,
        (int)s_fb.temp_x10,
        (unsigned)s_fb.err,
        (unsigned)s_fb.mode,
        (unsigned)s_fb.status,
        (unsigned long)s_fb.seq,
        (unsigned long)(now - s_fb.last_seen_ms));
}
