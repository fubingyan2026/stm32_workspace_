/**
 * @file    srv_can_mst.h
 * @author  maximillian
 * @version V1.2.0
 * @date    2026-07-2
 * @brief   P_CAN 主机上报服务 — 帧打包与发送队列
 * @attention
 *
 * 遵循 service 层回调注入模式，不直接调用硬件驱动。
 *
 * ## 职责
 * - 接收 task 层通过 read_data 回调注入的状态数据
 * - 打包为 CAN 帧（0x001 系统状态 + 0x011-0x0B1 电池数据）
 * - 通过 msg_fifo 队列 + pending 重试机制发送，不丢帧
 *
 * ## 用法
 * @code
 *   // 1. 实现数据读取回调和发送回调
 *   static void my_read_data(srv_can_mst_data_t* d) { ... }
 *   static bool my_send_frame(uint16_t id, const uint8_t* data, uint8_t len) { ... }
 *
 *   // 2. 初始化
 *   srv_can_mst_config_t cfg = { .read_data = my_read_data, .send_frame = my_send_frame };
 *   srv_can_mst_init(&cfg);
 *
 *   // 3. 定周期触发上报
 *   srv_can_mst_request(0xFF);       // 请求全部数据
 *   srv_can_mst_task();              // 发送队列中的帧（每周期调一次）
 * @endcode
 */

#ifndef __SRV_CAN_MST_H
#define __SRV_CAN_MST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "srv_can_dual.h"

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief P_CAN 帧 ID 枚举（0x001 ~ 0x0B1）
 *
 * 电源板向主机上报数据的 CAN ID 定义。
 * 0x001 为系统状态帧（必发），0x011-0x0B1 为电池数据帧（按 feedback_select 选发）。
 */
typedef enum {
    SRV_CAN_MST_ID_STATUS = 0x001, /**< 系统状态 + 错误标志（始终发送） */
    SRV_CAN_MST_ID_BAT_BASE = 0x011, /**< 电池基础参数：SOC/SOH/循环次数/型号 */
    SRV_CAN_MST_ID_BAT_VOLT = 0x021, /**< 电池电压：电池组电压 + 充电电压 */
    SRV_CAN_MST_ID_BAT_CURR = 0x031, /**< 电池电流：充电电流 + 放电电流 */
    SRV_CAN_MST_ID_BAT_TEMP14 = 0x041, /**< 双电池电芯温度 */
    SRV_CAN_MST_ID_BAT_STATUS = 0x051, /**< 电池版本 + 循环次数 */
    SRV_CAN_MST_ID_BAT_ERROR = 0x061, /**< 双电池故障码 */
    SRV_CAN_MST_ID_BAT_CNT = 0x071, /**< 故障等级 + 预警 */
    SRV_CAN_MST_ID_BAT_SER0 = 0x081, /**< 序列号 [0:7] */
    SRV_CAN_MST_ID_BAT_SER1 = 0x091, /**< 序列号 [8:15] */
    SRV_CAN_MST_ID_BAT_SER2 = 0x0A1, /**< 序列号 [16:23] */
    SRV_CAN_MST_ID_BAT_SER3 = 0x0B1, /**< 序列号 [24:31] */
} srv_can_mst_can_id_t;

/**
 * @brief feedback_select 位掩码枚举
 *
 * 主机通过 0x001 控制帧的 feedback_select 字节选择需要哪些电池数据帧。
 * srv_can_mst_request(fb) 时传入此掩码的组合。
 */
typedef enum {
    SRV_CAN_MST_FEEDBACK_BAT_BASE = (1U << 0), /**< 请求电池基础参数 (0x011) */
    SRV_CAN_MST_FEEDBACK_BAT_VOLTAGE = (1U << 1), /**< 请求电池电压 (0x021) */
    SRV_CAN_MST_FEEDBACK_BAT_CURRENT = (1U << 2), /**< 请求电池电流 (0x031) */
    SRV_CAN_MST_FEEDBACK_BAT_TEMPERATURE = (1U << 3), /**< 请求电池温度 (0x041) */
    SRV_CAN_MST_FEEDBACK_BAT_STATUS = (1U << 4), /**< 请求 T5+状态码 (0x051) */
    SRV_CAN_MST_FEEDBACK_BAT_ERROR = (1U << 5), /**< 请求电池错误码 (0x061) */
    SRV_CAN_MST_FEEDBACK_BAT_COUNTER = (1U << 6), /**< 请求有效标志+计数 (0x071) */
    SRV_CAN_MST_FEEDBACK_BAT_SERIAL = (1U << 7), /**< 请求序列号 (0x081-0x0B1) */
} srv_can_mst_feedback_t;

/**
 * @brief 上报数据体 — 对应 CAN 帧 0x001 及电池帧
 *
 * task 层的 read_data 回调填充此结构。
 * 结构字段顺序按 CAN 协议字节排列，方便对照协议文档。
 */
typedef struct {
    /* === 0x001 Byte0: 系统运行状态 === */
    bool stop_key_state; /**< [bit0] 急停状态：0=释放, 1=按下 */
    bool battery_key_state; /**< [bit1] 电池开关：0=关闭, 1=打开 */
    bool battery_charging; /**< [bit2] 充电状态：0=放电, 1=充电 */
    bool battery_temp_error; /**< [bit3] 电池温度异常标志 */
    bool bat1_online; /**< [bit4] 电池1 在线 */
    bool bat_has_error; /**< [bit5] 任一电池有错误 */
    bool bat2_online; /**< [bit6] 电池2 在线 */

    /* === 0x001 Byte1: 内部/外部电源轨错误 === */
    bool err_vin; /**< [bit0] 主输入电压异常（48V 母线） */
    bool err_vin_dcdc; /**< [bit1] DCDC 输出异常 */
    bool err_12v_int; /**< [bit2] 内部 12V 电源轨异常 */
    bool err_5v_int; /**< [bit3] 内部 5V 电源轨异常 */
    bool err_12v_ext; /**< [bit4] 外部 12V 输出异常 */
    bool err_24v_ext; /**< [bit5] 外部 24V 输出异常 */
    bool err_12v_user; /**< [bit6] 用户 12V 输出异常 */
    bool err_24v_user; /**< [bit7] 用户 24V 输出异常 */

    /* === 0x001 Byte2: 输出电源错误 === */
    bool err_24v_comp; /**< [bit0] 工控机 24V 输出异常 */
    bool err_power; /**< [bit1] 从板电源异常（SLAVE_POWER） */
    bool err_motor; /**< [bit2] 电机电源异常 */
    bool err_chg_out; /**< [bit3] 预充电异常（Pre-charge fault） */
    bool err_hsd1_12v; /**< [bit4] HSD1 12V 通道异常 */
    bool err_hsd2_12v; /**< [bit5] HSD2 12V 通道异常 */
    bool err_hsd3_12v; /**< [bit6] HSD3 12V 通道异常 */
    bool err_dbr; /**< [bit7] 制动电阻异常（DBR overcurrent 等） */

    /* === 0x001 Byte3: HSD-24V / LSD / 风扇错误 === */
    bool err_hsd1_24v; /**< [bit0] HSD1 24V 通道异常 */
    bool err_hsd2_24v; /**< [bit1] HSD2 24V 通道异常 */
    bool err_hsd3_24v; /**< [bit2] HSD3 24V 通道异常 */
    bool err_lsd1_24v; /**< [bit3] LSD1 24V 通道异常 */
    bool err_lsd2_24v; /**< [bit4] LSD2 24V 通道异常 */
    bool err_fan[3]; /**< [bit5~7] 风扇 0-2 异常 */

    /* === 0x001 Byte4: NTC 温度异常（8 路） === */
    bool err_ntc[8]; /**< [bit0~7] NTC 热敏电阻 0-7 温度超限标志 */

    /* === 0x001 Byte5: 数字输入 + 上电时序故障（bit6~7 保留） === */
    bool din1; /**< [bit0] 数字输入 1 */
    bool din2; /**< [bit1] 数字输入 2 */
    bool din3; /**< [bit2] 数字输入 3 */
    bool seq_vin_fault; /**< [bit3] VIN_DCDC 上电时序故障 */
    bool seq_chg_fault; /**< [bit4] 预充电时序故障 */
    bool seq_motor_fault; /**< [bit5] 电机上电时序故障 */

    /* === 0x001 Byte6-7: 电池简略信息 === */
    uint8_t bat1_soc; /**< 电池1 SOC（0-100%） */
    uint8_t bat2_soc; /**< 电池2 SOC（0-100%），0=无电池2 */

    /* === 电池帧数据 (0x011-0x0B1) — 双电池核心参数 === */
    /* 电池1 */
    uint16_t bat1_voltage_dv; /**< 电池1 电压 (0.1V), e.g. 480=48.0V */
    int16_t bat1_current_da; /**< 电池1 电流 (0.1A, 正=放电) */
    int8_t bat1_temp_c; /**< 电池1 电芯温度 (°C) */
    bool bat1_charging; /**< 电池1 充电中 */
    /* 电池2 */
    uint16_t bat2_voltage_dv;
    int16_t bat2_current_da;
    int8_t bat2_temp_c;
    bool bat2_charging;
    /* 电池1 版本信息 */
    uint32_t bat1_capacity_mah; /**< 电池1 设计容量 (mAh) */
    uint16_t bat1_cycle_count; /**< 电池1 循环次数 */
    uint16_t bat1_hw_version; /**< 电池1 硬件版本 */
    uint16_t bat1_sw_version; /**< 电池1 软件版本 */
    /* 电池2 版本信息 */
    uint32_t bat2_capacity_mah;
    uint16_t bat2_cycle_count;
    uint16_t bat2_hw_version;
    uint16_t bat2_sw_version;
    /* 电池故障码 */
    srv_can_dual_fault_bat1_t bat1_fault; /**< 电池1 故障详情 */
    srv_can_dual_fault_bat2_t bat2_fault; /**< 电池2 故障详情 */
} srv_can_mst_data_t;

/**
 * @brief 数据读取回调函数类型
 *
 * task 层实现此回调，填充 srv_can_mst_data_t 结构体。
 * 回调在 srv_can_mst_request() 中同步调用，不应长时间阻塞。
 *
 * @param data 待填充的数据体指针
 */
typedef void (*srv_can_mst_read_cb_t)(srv_can_mst_data_t* data);

/**
 * @brief CAN 帧发送回调函数类型
 *
 * task 层实现此回调，负责将打包好的帧发送到 CAN 总线。
 * service 层在发送失败时会保留帧等待下次重试，因此回调应尽快返回。
 *
 * @param can_id CAN ID（标准 11-bit）
 * @param data   帧数据指针（8 字节）
 * @param len    数据长度（字节）
 * @return true=发送成功, false=发送忙（帧留待重试）
 */
typedef bool (*srv_can_mst_send_cb_t)(uint16_t can_id,
    const uint8_t* data, uint8_t len);

/**
 * @brief 服务配置结构体
 *
 * 两个回调均在 srv_can_mst_init() 时注入，不可为 NULL。
 */
typedef struct {
    srv_can_mst_read_cb_t read_data; /**< 数据读取回调（必填） */
    srv_can_mst_send_cb_t send_frame; /**< CAN 发送回调（必填） */
} srv_can_mst_config_t;

/**
 * @brief 主机下发的控制指令（从 0x001 RX 帧解析）
 */
typedef struct {
    uint8_t feedback_select; /**< 反馈请求位掩码 */
    uint8_t rgb_mode; /**< RGB 灯效模式 */
    uint32_t rgb_color; /**< RGB 颜色 (0xRRGGBB) */
    uint8_t buzzer_duty; /**< 蜂鸣器占空比 0-100 */
    bool hsd1_12v_on; /**< HSD1 12V 输出 */
    bool hsd2_12v_on; /**< HSD2 12V 输出 */
    bool lsd1_24v_on; /**< LSD1 24V 输出 */
    bool lsd2_24v_on; /**< LSD2 24V 输出 */
} srv_can_mst_cmd_t;

/**
 * @brief 服务错误码
 */
typedef enum {
    SRV_CAN_MST_OK = 0, /**< 操作成功 */
    SRV_CAN_MST_ERROR_NULL_PTR, /**< 空指针或回调未注册 */
    SRV_CAN_MST_ERROR_UNINITIALIZED, /**< 服务未初始化 */
    SRV_CAN_MST_ERROR_QUEUE_FULL, /**< 发送队列已满（未使用，保留） */
} srv_can_mst_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 CAN 上报服务
 *
 * 验证回调非 NULL，初始化 msg_fifo 发送队列和 pending 帧状态。
 *
 * @param config 服务配置（read_data + send_frame 两个回调必须提供）
 * @return SRV_CAN_MST_OK 成功，SRV_CAN_MST_ERROR_NULL_PTR 参数错误
 */
srv_can_mst_error_t srv_can_mst_init(const srv_can_mst_config_t* config);

/**
 * @brief 反初始化 CAN 上报服务
 *
 * 清空发送队列，清除 pending 帧，重置回调指针。
 * 调用后所有帧被丢弃。
 */
void srv_can_mst_deinit(void);

/**
 * @brief 检查服务是否已初始化
 * @return true=已初始化, false=未初始化
 */
bool srv_can_mst_is_initialized(void);

/**
 * @brief 请求发送一轮上报帧
 *
 * 同步调用 read_data 回调读取当前状态，打包 0x001 系统状态帧
 * 和 0x011-0x0B1 电池数据帧（按 feedback_select 选择），入队等待发送。
 *
 * 如果队列中还有上一轮的帧未发完，则放弃本轮请求
 * （避免堆积过时数据）。
 *
 * @param feedback_select 位掩码（0xFF=发送全部电池帧，0x00=仅发 0x001）
 */
void srv_can_mst_request(uint8_t feedback_select);

/**
 * @brief 定时任务：从队列取一帧并发送
 *
 * 应在 sw_timer 回调或 main loop 中每 10ms 调用一次。
 * - 有 pending 帧时：重试发送，成功则清除 pending
 * - 无 pending 帧时：从 FIFO 取下一帧，立即尝试发送
 * - 成功/失败均在本周期完成，每周期最多处理一帧
 */
void srv_can_mst_task(void);

/**
 * @brief 解析主机下发的 0x001 控制帧
 * @param data 帧数据（7 字节）
 * @param len  帧长度
 * @note  内部缓存解析结果和 feedback_select，srv_can_mst_task 中自动按需回复。
 *        可通过 srv_can_mst_get_cmd() 读取解析后的指令。
 */
void srv_can_mst_process_rx(const uint8_t* data, uint8_t len);

/** @brief 获取最近一次主机指令 */
const srv_can_mst_cmd_t* srv_can_mst_get_cmd(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_CAN_MST_H */
