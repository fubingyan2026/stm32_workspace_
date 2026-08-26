/**
 * @file    srv_mz_sensor.c
 * @brief   Mz 扭矩传感器服务实现
 *
 * 轮询状态机（1ms 步进）：
 *   查询模式：无在途查询 → 发查询帧；有在途但超时 → 丢弃等待重发。
 *   标零模式：挂起请求 → 发标零帧，收到应答/超时后回到查询模式。
 * 应答由 on_rx（ISR）解析并清除在途标志，主循环 step 下一拍立即补发新查询，
 * 实现响应门控的最高查询频率。
 */

#include "srv_mz_sensor.h"

#include "drv_systick.h"
#include "log.h"

#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/
#define SRV_MZ_SENSOR_LOG_ENABLE 1

#if SRV_MZ_SENSOR_LOG_ENABLE
#define SRV_MZ_SENSOR_LOG_I(...) LOG_I("srv_mz_sensor", __VA_ARGS__)
#define SRV_MZ_SENSOR_LOG_W(...) LOG_W("srv_mz_sensor", __VA_ARGS__)
#define SRV_MZ_SENSOR_LOG_E(...) LOG_E("srv_mz_sensor", __VA_ARGS__)
#else
#define SRV_MZ_SENSOR_LOG_I(...) ((void)0)
#define SRV_MZ_SENSOR_LOG_W(...) ((void)0)
#define SRV_MZ_SENSOR_LOG_E(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define SRV_MZ_SENSOR_ID_QUERY 0x510U /**< 查询/标零帧 ID */
#define SRV_MZ_SENSOR_ID_RESP 0x410U /**< 应答帧 ID */
#define SRV_MZ_SENSOR_DLC 7U /**< 定长 DLC（读查询与标零均为 7 字节） */

/** @brief 应答超时 (ms)：在途查询超过该时长未应答视为丢帧，允许重发 */
#define SRV_MZ_SENSOR_TIMEOUT_MS 3U
/** @brief 传感器轮询频率限制：500Hz（周期 2ms） */
#define SRV_MZ_SENSOR_PERIOD_MS 2U
/** @brief 传感器离线判定周期 (ms)：超过该时长未收到应答视为离线 */
#define SRV_MZ_SENSOR_OFFLINE_MS 50U
/** @brief 查询率统计窗口 (ms) */
#define SRV_MZ_SENSOR_RATE_WINDOW_MS 1000U

/** @brief 读查询帧数据（读功能码 04 + 数据地址 0x0000 + 4B 占位） */
static const uint8_t s_query_data[SRV_MZ_SENSOR_DLC] = { 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/** @brief 标零帧数据（写功能码 10 + 地址 0x4604 + float32 1.0 大端） */
static const uint8_t s_zero_data[SRV_MZ_SENSOR_DLC] = { 0x10, 0x46, 0x04, 0x3F, 0x80, 0x00, 0x00 };

/* Private types -------------------------------------------------------------*/

/** @brief 轮询模式 */
typedef enum {
    SRV_MZ_SENSOR_MODE_QUERY = 0, /**< 周期读 Mz */
    SRV_MZ_SENSOR_MODE_ZERO,      /**< 标零（发标零帧等待应答） */
} srv_mz_sensor_mode_t;

/* Private variables ---------------------------------------------------------*/

/** @brief 本服务绑定的 CAN 总线（接线处注入，通道无关） */
static const srv_can_bus_t* s_bus;

/** @brief 传感器最新数据（ISR 写，主循环读） */
static srv_mz_sensor_fb_t s_fb;

/** @brief 轮询模式 */
static srv_mz_sensor_mode_t s_mode;

/** @brief 在途标志：已发帧等待应答 */
static bool s_inflight;

/** @brief 在途帧发出时间 (millis) */
static uint32_t s_send_ms;

/** @brief 挂起的标零请求（主循环置位，step 消费） */
static bool s_zero_pending;

/** @brief 上次离线状态（边沿检测） */
static bool s_online_last;

/** @brief 查询率计数（step 每发一帧 +1） */
static uint32_t s_rate_cnt;

/** @brief 查询率统计窗口起点 (millis) */
static uint32_t s_rate_window_ms;

/** @brief 标零超时锁存（避免重复告警） */
static bool s_zero_timeout_latch;

/** @brief 是否收到过应答（区分「从未应答」与「掉线」，避免上电误报在线） */
static bool s_seen;

/** @brief 上次查询发送时间 (millis)，用于 500Hz 限频 */
static uint32_t s_poll_last_ms;

/* Private function prototypes -----------------------------------------------*/

static void srv_mz_sensor_send_frame(uint8_t fc);
static void srv_mz_sensor_update_online(uint32_t now);
static void srv_mz_sensor_update_rate(uint32_t now);

/* Exported functions --------------------------------------------------------*/

void srv_mz_sensor_init(const srv_can_bus_t* bus)
{
    if (!bus) {
        return;
    }
    s_bus = bus;

    memset(&s_fb, 0, sizeof(s_fb));
    s_mode = SRV_MZ_SENSOR_MODE_QUERY;
    s_inflight = false;
    s_send_ms = 0;
    s_zero_pending = false;
    s_online_last = false;
    s_rate_cnt = 0;
    s_rate_window_ms = millis();
    s_zero_timeout_latch = false;
    s_seen = false;
    s_poll_last_ms = millis();

    SRV_MZ_SENSOR_LOG_I("init: query 0x%03X resp 0x%03X, DLC=%u, query rate 500Hz",
        SRV_MZ_SENSOR_ID_QUERY, SRV_MZ_SENSOR_ID_RESP, SRV_MZ_SENSOR_DLC);
}

void srv_mz_sensor_step(void)
{
    if (!s_bus) {
        return;
    }

    const uint32_t now = millis();

    srv_mz_sensor_update_online(now);
    srv_mz_sensor_update_rate(now);

    /* 标零挂起：抢占总线发标零帧 */
    if (s_zero_pending && !s_inflight) {
        s_zero_pending = false;
        s_mode = SRV_MZ_SENSOR_MODE_ZERO;
        srv_mz_sensor_send_frame(0x10U);
        return;
    }

    if (s_mode == SRV_MZ_SENSOR_MODE_QUERY) {
        if (s_inflight) {
            /* 在途超时：丢帧，下一拍重发 */
            if ((now - s_send_ms) >= SRV_MZ_SENSOR_TIMEOUT_MS) {
                s_inflight = false;
            }
        } else if ((now - s_poll_last_ms) >= SRV_MZ_SENSOR_PERIOD_MS) {
            /* 500Hz 限频：每 2ms 发一次查询 */
            s_poll_last_ms = now;
            srv_mz_sensor_send_frame(0x04U);
        }
        return;
    }

    /* SRV_MZ_SENSOR_MODE_ZERO：应答已在 on_rx 处理并切回查询；仅处理超时 */
    if (s_inflight && ((now - s_send_ms) >= SRV_MZ_SENSOR_TIMEOUT_MS)) {
        s_inflight = false;
        s_mode = SRV_MZ_SENSOR_MODE_QUERY;
        if (!s_zero_timeout_latch) {
            s_zero_timeout_latch = true;
            SRV_MZ_SENSOR_LOG_W("zero ACK timeout, resume polling");
        }
    }
}

void srv_mz_sensor_on_rx(const drv_can_msg_t* msg, void* user_data)
{
    (void)user_data;

    if (!msg || !s_bus) {
        return;
    }
    if (msg->is_extended || (msg->id != SRV_MZ_SENSOR_ID_RESP)) {
        return;
    }
    if (msg->dlc < SRV_MZ_SENSOR_DLC) {
        return;
    }

    s_fb.last_seen_ms = millis();
    s_inflight = false;
    s_seen = true;

    if (msg->data[0] == 0x04U) {
        /* Mz 读取应答：Byte1~2=地址 0x0000，Byte3~6=float32 大端 */
        uint32_t u = ((uint32_t)msg->data[3] << 24)
            | ((uint32_t)msg->data[4] << 16)
            | ((uint32_t)msg->data[5] << 8)
            | (uint32_t)msg->data[6];
        memcpy(&s_fb.mz_nm, &u, sizeof(u));
        s_fb.seq++;
    } else if (msg->data[0] == 0x10U) {
        /* 标零应答（回显） */
        s_fb.zero_ok = true;
        s_zero_timeout_latch = false;
        s_mode = SRV_MZ_SENSOR_MODE_QUERY;
        SRV_MZ_SENSOR_LOG_I("zero complete");
    }
}

bool srv_mz_sensor_zero(void)
{
    if (!s_bus || s_zero_pending || (s_mode == SRV_MZ_SENSOR_MODE_ZERO)) {
        return false;
    }
    s_zero_pending = true;
    s_fb.zero_ok = false;
    return true;
}

const srv_mz_sensor_fb_t* srv_mz_sensor_get(void)
{
    return &s_fb;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 发送传感器帧（查询/标零）
 * @param fc 功能码：0x04=读 Mz，0x10=标零
 * @note  发送成功后置在途标志并刷新查询率计数（ISR 应答后由 step 下一拍补发）
 */
static void srv_mz_sensor_send_frame(uint8_t fc)
{
    if (!srv_can_bus_tx_ready(s_bus)) {
        return;
    }

    drv_can_msg_t tx = {
        .id = SRV_MZ_SENSOR_ID_QUERY,
        .is_extended = false,
        .is_fd = false, /* 经典 CAN */
        .dlc = SRV_MZ_SENSOR_DLC,
    };
    memcpy(tx.data, (fc == 0x04U) ? s_query_data : s_zero_data, SRV_MZ_SENSOR_DLC);

    if (srv_can_bus_send(s_bus, &tx) == DRV_CAN_OK) {
        s_inflight = true;
        s_send_ms = millis();
        s_rate_cnt++;
    }
}

/**
 * @brief 在线/离线状态边沿检测与日志
 * @param now 当前时间 (millis)
 */
static void srv_mz_sensor_update_online(uint32_t now)
{
    const bool online = s_seen && ((now - s_fb.last_seen_ms) < SRV_MZ_SENSOR_OFFLINE_MS);
    if (online != s_online_last) {
        s_online_last = online;
        if (online) {
            SRV_MZ_SENSOR_LOG_I("sensor online, Mz=%ld (0.001Nm)",
                (long)(int32_t)(s_fb.mz_nm * 1000.0f));
        } else {
            SRV_MZ_SENSOR_LOG_W("sensor offline/no response");
        }
    }
}

/**
 * @brief 查询率滚动统计：每秒刷新 rate_qps
 * @param now 当前时间 (millis)
 */
static void srv_mz_sensor_update_rate(uint32_t now)
{
    if ((now - s_rate_window_ms) >= SRV_MZ_SENSOR_RATE_WINDOW_MS) {
        s_fb.rate_qps = s_rate_cnt;
        s_rate_cnt = 0;
        s_rate_window_ms = now;
    }
}
