/**
 * @file    srv_can.c
 * @brief   CAN FD 电机控制协议服务实现
 *
 * 控制帧解析 → srv_motor_behavior API。
 * 反馈帧打包 → drv_can_send (CAN FD, 64B max)。
 */

#include "srv_can.h"

#include "drv_systick.h"
#include "srv_motor_behavior.h"

#include <string.h>

/* 模块测试开关 ----------------------------------------------------------------*/

/** @brief 电机测试模式：1=上电自动启动（使能 + 正转/停留/反转/停留 各 30s 循环）；0=需手动调用 srv_can_test_start() */
#define SRV_CAN_TEST_AUTO_START 1

/* Private constants ---------------------------------------------------------*/

/** @brief 测试帧参数（CAN FD, 64B） */
#define SRV_CAN_TEST_ID_EN 0x10U /**< 测试：电机使能/失能帧 CAN ID */
#define SRV_CAN_TEST_ID_SPD 0x20U /**< 测试：速度旋转帧 CAN ID */
#define SRV_CAN_TEST_FRAME_LEN 64U /**< 测试帧长度（CAN FD 满帧） */
#define SRV_CAN_TEST_GROUP_LEN 8U /**< 每组字节数 */
#define SRV_CAN_TEST_GROUP_NUM 8U /**< 组数（8×8=64） */
#define SRV_CAN_TEST_SPD_PERIOD_MS 1000U /**< 旋转阶段速度帧发送周期 (ms) */
#define SRV_CAN_TEST_PHASE_MS 30000U /**< 每个阶段时长 (ms)：正转/停留/反转/停留 */
#define SRV_CAN_TEST_DURATION_MS 86400000U /**< 测试总时长 (ms)：24h 后自动停止 */
#define SRV_CAN_TEST_EN_FILL 0xFCU /**< 使能帧组内末字节 */
#define SRV_CAN_TEST_DIS_FILL 0xFDU /**< 失能帧组内末字节 */

/** @brief 测试速度帧 8B 模式（正转/反转/速度 0） */
static const uint8_t pattern[SRV_CAN_TEST_GROUP_LEN] =        { 0x7F, 0xff, 0xbf, 0xf0, 0x00, 0x33, 0x37, 0xff };
static const uint8_t pattern_invert[SRV_CAN_TEST_GROUP_LEN] = { 0x7F, 0xff, 0x3f, 0xf0, 0x00, 0x33, 0x37, 0xFF };
static const uint8_t pattern_zero[SRV_CAN_TEST_GROUP_LEN] =   { 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x05, 0x17, 0xFF };

/* Private types -------------------------------------------------------------*/

/** @brief 测试循环阶段：正转 → 停留 → 反转 → 停留，来回一个循环 */
typedef enum {
    SRV_CAN_TEST_PHASE_FORWARD = 0, /**< 正转 */
    SRV_CAN_TEST_PHASE_DWELL_F, /**< 正转后停留 */
    SRV_CAN_TEST_PHASE_REVERSE, /**< 反转 */
    SRV_CAN_TEST_PHASE_DWELL_R, /**< 反转后停留 */
    SRV_CAN_TEST_PHASE_NUM, /**< 阶段总数 */
} srv_can_test_phase_t;

/* Private variables ---------------------------------------------------------*/

/** @brief ISR → 主循环控制数据暂存 */
static struct {
    bool pending;
    uint8_t ctrl;
    int16_t pos[9];
    int16_t spd[9];
    int16_t cur[9];
} s_rx;

/** @brief 测试模式运行标志 */
static bool s_test_running;

/** @brief 测试当前阶段 */
static srv_can_test_phase_t s_test_phase;

/** @brief 当前阶段起始时间 (millis) */
static uint32_t s_test_phase_start_ms;

/** @brief 上次速度帧发送时间 (millis) */
static uint32_t s_test_last_spd_ms;

/** @brief 测试起始时间 (millis)，用于 24h 自动停止 */
static uint32_t s_test_start_ms;

/* Private function prototypes -----------------------------------------------*/

static void srv_can_test_send_cmd(uint8_t fill);
static void srv_can_test_send_speed(const uint8_t* pat);

/* Exported functions --------------------------------------------------------*/

void srv_can_init(void)
{
    memset(&s_rx, 0, sizeof(s_rx));

#if SRV_CAN_TEST_AUTO_START
    srv_can_test_start(); /* 测试模式：上电立即使能，之后由 step() 每秒发送速度帧 */
#endif
}

/**
 * @brief ISR 回调：快速复制数据，置标志位
 */
void srv_can_on_rx(const drv_can_msg_t* msg)
{
    if (!msg || msg->id != SRV_CAN_ID_CTRL || msg->dlc < SRV_CAN_CTRL_LEN)
        return;

    s_rx.ctrl = msg->data[0];
    memcpy(s_rx.pos, &msg->data[1], 18);
    memcpy(s_rx.spd, &msg->data[19], 18);
    memcpy(s_rx.cur, &msg->data[37], 18);
    s_rx.pending = true;
}

/**
 * @brief 主循环处理：将缓存的控制数据下发到电机行为层
 * @note  ctrl 字节（bit0/bit1 使能位）已废弃：电机使能由 srv_motor 上电自治，
 *        FAULT 由 err_code 清零自动恢复；目标值仅在 RUNNING 态被接受。
 */
void srv_can_process(void)
{
    if (!s_rx.pending)
        return;
    s_rx.pending = false;

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++)
        srv_motor_behavior_set_setpoint(i, s_rx.pos[i], s_rx.spd[i], s_rx.cur[i]);
}

/* --- 测试模式 ----------------------------------------------------------------*/

/**
 * @brief 启动电机测试模式
 * @note  立即发送一次使能帧 (ID 0x10)，随后进入正转循环；
 *        由 srv_can_test_step() 驱动正转/停留/反转/停留 每阶段 30s，无限循环
 */
void srv_can_test_start(void)
{
    s_test_running = true;
    s_test_phase = SRV_CAN_TEST_PHASE_FORWARD;
    s_test_phase_start_ms = millis();
    s_test_last_spd_ms = s_test_phase_start_ms;
    s_test_start_ms = s_test_phase_start_ms; /* 24h 自动停止计时起点 */
    srv_can_test_send_cmd(SRV_CAN_TEST_EN_FILL);
}

/**
 * @brief 停止电机测试模式并发送速度 0 帧 (ID 0x20)
 * @note  停止不发送失能帧，只发速度 0 让电机停下
 */
void srv_can_test_stop(void)
{
    s_test_running = false;
    srv_can_test_send_speed(pattern_zero);
}

/**
 * @brief 测试模式周期步进（由 can_task 每 10ms 调用）
 * @note  阶段循环：正转 30s → 停留(速度 0) 30s → 反转 30s → 停留(速度 0) 30s → 循环（可连续运行 24h）
 *        正转/反转发方向速度帧，停留发速度 0 帧 (pattern_zero)，不失能
 */
void srv_can_test_step(void)
{
    if (!s_test_running)
        return;

    uint32_t now = millis();

    /* 24h 到时自动停止：发送失能并退出循环 */
    if ((now - s_test_start_ms) >= SRV_CAN_TEST_DURATION_MS) {
        srv_can_test_stop();
        return;
    }

    /* 阶段切换 */
    if ((now - s_test_phase_start_ms) >= SRV_CAN_TEST_PHASE_MS) {
        s_test_phase = (srv_can_test_phase_t)(((uint32_t)s_test_phase + 1U) % (uint32_t)SRV_CAN_TEST_PHASE_NUM);
        s_test_phase_start_ms = now;
        s_test_last_spd_ms = now;

        /* 进入旋转阶段时补发一次使能（停留期间可能已失能） */
        if ((s_test_phase == SRV_CAN_TEST_PHASE_FORWARD) ||
            (s_test_phase == SRV_CAN_TEST_PHASE_REVERSE)) {
            srv_can_test_send_cmd(SRV_CAN_TEST_EN_FILL);
        }
    }

    /* 正转/反转发方向速度帧；停留发速度 0 帧 (pattern_zero)，不失能 */
    const uint8_t* pat = pattern_zero;
    if (s_test_phase == SRV_CAN_TEST_PHASE_FORWARD) {
        pat = pattern;
    } else if (s_test_phase == SRV_CAN_TEST_PHASE_REVERSE) {
        pat = pattern_invert;
    }

    if ((now - s_test_last_spd_ms) >= SRV_CAN_TEST_SPD_PERIOD_MS) {
        s_test_last_spd_ms = now;
        srv_can_test_send_speed(pat);
    }

    /* 上电 1s 内重发使能帧（首帧可能因 TX FIFO 未就绪被丢弃） */
    if (now < 1000) {
        static uint8_t cnt = 0;
        if (cnt++ > 40) {
            cnt = 0;
            srv_can_test_start();
        }
    }
}

/**
 * @brief 打包并发送 9 电机反馈数据 (CAN FD)
 */
void srv_can_send_feedback(void)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    uint8_t buf[SRV_CAN_FB_LEN];
    memset(buf, 0, sizeof(buf));

    buf[0] = (uint8_t)srv_motor_behavior_get_state();

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);

        buf[1 + i] = fb ? (uint8_t)fb->fsm_state : 0;

        if (fb) {
            buf[10 + i * 2] = (uint8_t)fb->angle_fb;
            buf[10 + i * 2 + 1] = (uint8_t)(fb->angle_fb >> 8);
            buf[28 + i * 2] = (uint8_t)fb->speed_fb;
            buf[28 + i * 2 + 1] = (uint8_t)(fb->speed_fb >> 8);
            buf[46 + i * 2] = (uint8_t)fb->q_cur;
            buf[46 + i * 2 + 1] = (uint8_t)(fb->q_cur >> 8);
        }
    }

    drv_can_msg_t tx = {
        .id = SRV_CAN_ID_FEEDBACK,
        .is_extended = false,
        .is_fd = true,
        .dlc = SRV_CAN_FB_LEN,
    };
    memcpy(tx.data, buf, SRV_CAN_FB_LEN);
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 打包并发送 9 电机低频状态数据 (CAN FD, 500ms)
 */
void srv_can_send_status(void)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    uint8_t buf[SRV_CAN_STATUS_LEN];
    memset(buf, 0, sizeof(buf));

    buf[0] = (uint8_t)srv_motor_behavior_get_state();

    for (uint32_t i = 0; i < SRV_MOTOR_TOTAL; i++) {
        const srv_motor_feedback_t* fb = srv_motor_behavior_get_fb(i);

        if (fb) {
            buf[1 + i] = (uint8_t)fb->fsm_state;
            buf[10 + i] = (uint8_t)fb->err_code;
            buf[19 + i] = (uint8_t)fb->temp;
            buf[28 + i * 2] = (uint8_t)fb->vbus;
            buf[28 + i * 2 + 1] = (uint8_t)(fb->vbus >> 8);
        }
    }

    drv_can_msg_t tx = {
        .id = SRV_CAN_ID_STATUS,
        .is_extended = false,
        .is_fd = true,
        .dlc = SRV_CAN_STATUS_LEN,
    };
    memcpy(tx.data, buf, SRV_CAN_STATUS_LEN);
    drv_can_send(DRV_CAN_CH_1, &tx);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 发送测试使能/失能帧 (CAN ID 0x10, 64B)
 * @param fill 组内末字节：0xFC=使能，0xFD=失能
 * @note  8 组 × 8B 模式：FF FF FF FF FF FF FF {fill}
 */
static void srv_can_test_send_cmd(uint8_t fill)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = SRV_CAN_TEST_ID_EN,
        .is_extended = false,
        .is_fd = true,
        .dlc = SRV_CAN_TEST_FRAME_LEN,
    };
    memset(tx.data, 0xFF, sizeof(tx.data));

    for (uint32_t i = 0; i < SRV_CAN_TEST_GROUP_NUM; i++)
        tx.data[i * SRV_CAN_TEST_GROUP_LEN + (SRV_CAN_TEST_GROUP_LEN - 1U)] = fill;

    drv_can_send(DRV_CAN_CH_1, &tx);
}

/**
 * @brief 发送测试速度旋转帧 (CAN ID 0x20, 64B)
 * @param pat 要发送的 8B 模式
 * @note  8 组 × 8B 固定模式：
 *        正转 A6 66 0C D0 00 02 90 AC
 *        反转 59 99 0C D0 00 02 90 AC（前两字节为按位取反）
 *        速度 0 00 00 0C D0 00 02 90 AC（停留用，不失能）
 */
static void srv_can_test_send_speed(const uint8_t* pat)
{
    if (!drv_can_tx_ready(DRV_CAN_CH_1))
        return;

    drv_can_msg_t tx = {
        .id = SRV_CAN_TEST_ID_SPD,
        .is_extended = false,
        .is_fd = true,
        .dlc = SRV_CAN_TEST_FRAME_LEN,
    };
    for (uint32_t i = 0; i < SRV_CAN_TEST_GROUP_NUM; i++) {
        memcpy(&tx.data[i * SRV_CAN_TEST_GROUP_LEN], pat, SRV_CAN_TEST_GROUP_LEN);
    }
    drv_can_send(DRV_CAN_CH_1, &tx);
}
