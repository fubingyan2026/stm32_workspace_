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
 * - 打包为 CAN 帧（0x001 系统状态 + 0x011-0x015 电池数据）
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
 * @brief P_CAN 帧 ID 枚举（0x001 ~ 0x015）
 *
 * 电源板向主机上报数据的 CAN ID 定义。
 * 0x001 为系统状态帧（必发），0x011-0x015 为电池数据帧（按 feedback_select 选发，与位一一对应）。
 */
typedef enum {
    SRV_CAN_MST_ID_STATUS = 0x001, /**< 系统状态 + 错误标志（始终发送） */
    SRV_CAN_MST_ID_BAT_BASE = 0x011, /**< 电池基础参数：容量 + 循环 + 充电标志 */
    SRV_CAN_MST_ID_BAT_VOLT = 0x012, /**< 电池电压 + 电流 */
    SRV_CAN_MST_ID_BAT_STATUS = 0x013, /**< 电池版本：HW/SW 版本 */
    SRV_CAN_MST_ID_BAT_ERROR = 0x014, /**< 双电池故障码 */
    SRV_CAN_MST_ID_BAT_CNT = 0x015, /**< 故障等级 + 预警 + 温度 */
} srv_can_mst_can_id_t;

/**
 * @brief feedback_select 位掩码枚举
 *
 * 主机通过 0x001 控制帧的 feedback_select 字节选择需要哪些电池数据帧。
 * srv_can_mst_request(fb) 时传入此掩码的组合。
 */
typedef enum {
    SRV_CAN_MST_FEEDBACK_BAT_BASE = (1U << 0), /**< 请求电池基础参数 (0x011): 容量/循环/充电 */
    SRV_CAN_MST_FEEDBACK_BAT_VOLTAGE = (1U << 1), /**< 请求电池电压+电流 (0x012) */
    SRV_CAN_MST_FEEDBACK_BAT_STATUS = (1U << 2), /**< 请求电池版本 (0x013) */
    SRV_CAN_MST_FEEDBACK_BAT_ERROR = (1U << 3), /**< 请求电池错误码 (0x014) */
    SRV_CAN_MST_FEEDBACK_BAT_COUNTER = (1U << 4), /**< 请求故障等级+预警+温度 (0x015) */
    /* bit5-7 保留 */
} srv_can_mst_feedback_t;

/**
 * @brief 0x001 状态帧（Byte0-7）视图（字段声明顺序 = bit 顺序，bit0 在前）
 *
 * 位域由编译器自动对齐，替代手写移位打包；union 的 bytes 视图便于整帧观察/校验。
 * Byte0-5 为状态位域，Byte6-7 为双电池 SOC 简略信息。
 *
 * @attention 位域分配由编译器/ABI 决定（C 标准未规定）：
 *   - GCC ARM (AAPCS, little-endian) 下从 LSB 分配，实测与协议一致；
 *   - 更换工具链/大小端会静默改变线上字节位，srv_can_mst_init 内有布局自检兜底。
 */
typedef union {
    struct __attribute__((packed)) {
        /* === byte0: 系统运行状态 === */
        uint8_t stop_key_state : 1; /**< [bit0] 急停状态：0=释放, 1=按下 */
        uint8_t battery_key_state : 1; /**< [bit1] 电池开关：0=关闭, 1=打开 */
        uint8_t battery_charging : 1; /**< [bit2] 充电状态：0=放电, 1=充电 */
        uint8_t battery_temp_error : 1; /**< [bit3] 电池温度异常标志 */

        uint8_t device_online_slaver : 1; // 副电源管理控制板在线
        uint8_t device_online_dual : 1; // 双电池控制板在线
        uint8_t device_online_bat1 : 1; // 电池1 在线
        uint8_t device_online_bat2 : 1; // 电池2 在线

        /* === byte1: 内部/外部电源轨错误 === */
        uint8_t err_vin : 1; /**< [bit0] 主输入电压异常（48V 母线） */
        uint8_t err_vin_dcdc : 1; /**< [bit1] DCDC 输出异常 */
        uint8_t err_12v_int : 1; /**< [bit2] 内部 12V 电源轨异常 */
        uint8_t err_5v_int : 1; /**< [bit3] 内部 5V 电源轨异常 */
        uint8_t err_12v_ext : 1; /**< [bit4] 外部 12V 输出异常 */
        uint8_t err_24v_ext : 1; /**< [bit5] 外部 24V 输出异常 */
        uint8_t err_12v_user : 1; /**< [bit6] 用户 12V 输出异常 */
        uint8_t err_24v_user : 1; /**< [bit7] 用户 24V 输出异常 */
        /* === byte2: 输出电源错误 === */
        uint8_t err_24v_comp : 1; /**< [bit0] 工控机 24V 输出异常 */
        uint8_t err_power : 1; /**< [bit1] 从板电源异常（SLAVE_POWER） */
        uint8_t err_motor : 1; /**< [bit2] 电机电源异常 */
        uint8_t err_chg_out : 1; /**< [bit3] 预充电异常（Pre-charge fault） */
        uint8_t err_hsd1_12v : 1; /**< [bit4] HSD1 12V 通道异常 */
        uint8_t err_hsd2_12v : 1; /**< [bit5] HSD2 12V 通道异常 */
        uint8_t err_hsd3_12v : 1; /**< [bit6] HSD3 12V 通道异常 */
        uint8_t err_dbr : 1; /**< [bit7] 制动电阻异常（DBR overcurrent 等） */
        /* === byte3: HSD-24V / LSD / 风扇错误 === */
        uint8_t err_hsd1_24v : 1; /**< [bit0] HSD1 24V 通道异常 */
        uint8_t err_hsd2_24v : 1; /**< [bit1] HSD2 24V 通道异常 */
        uint8_t err_hsd3_24v : 1; /**< [bit2] HSD3 24V 通道异常 */
        uint8_t err_lsd1_24v : 1; /**< [bit3] LSD1 24V 通道异常 */
        uint8_t err_lsd2_24v : 1; /**< [bit4] LSD2 24V 通道异常 */
        uint8_t err_fan0 : 1; /**< [bit5] 风扇0 异常 */
        uint8_t err_fan1 : 1; /**< [bit6] 风扇1 异常 */
        uint8_t byte3_fixed1 : 1; /**< [bit7] 协议固定为 1 */
        /* === byte4: NTC 温度异常（8 路） === */
        uint8_t err_ntc0 : 1; /**< [bit0] NTC0 温度超限 */
        uint8_t err_ntc1 : 1; /**< [bit1] NTC1 温度超限 */
        uint8_t err_ntc2 : 1; /**< [bit2] NTC2 温度超限 */
        uint8_t err_ntc3 : 1; /**< [bit3] NTC3 温度超限 */
        uint8_t err_ntc4 : 1; /**< [bit4] NTC4 温度超限 */
        uint8_t err_ntc5 : 1; /**< [bit5] NTC5 温度超限 */
        uint8_t err_ntc6 : 1; /**< [bit6] NTC6 温度超限 */
        uint8_t err_ntc7 : 1; /**< [bit7] NTC7 温度超限 */
        /* === byte5: 模拟输入 + 上电时序故障 === */
        uint8_t a_in1_io : 1; /**< [bit0] A_IN1_IO 模拟输入 */
        uint8_t a_in2_io : 1; /**< [bit1] A_IN2_IO 模拟输入 */
        uint8_t a_in3_io : 1; /**< [bit2] A_IN3_IO 模拟输入 */
        uint8_t seq_vin_fault : 1; /**< [bit3] VIN_DCDC 上电时序故障 */
        uint8_t seq_chg_fault : 1; /**< [bit4] 预充电时序故障 */
        uint8_t seq_motor_fault : 1; /**< [bit5] 电机上电时序故障 */
        uint8_t byte5_reserved : 2; /**< [bit6~7] 保留 */
        /* === byte6-7: 电池简略信息 === */
        uint8_t bat1_soc; /**< [Byte6] 电池1 SOC（0-100%） */
        uint8_t bat2_soc; /**< [Byte7] 电池2 SOC（0-100%），0=无电池2 */
    } bits;
    uint8_t bytes[8]; /**< 原始字节视图（0x001 帧 Byte0-7） */
} srv_can_mst_status_frame_t;

/**
 * @brief 单电池上报参数（双电池各自持有实例）
 *
 * 仅承载电池帧数据（0x011-0x015），与 0x001 状态帧的 SOC 位域分开。
 */
typedef struct {
    /* 核心参数（2×uint16 先占 4B，使紧随的 uint32 对齐到 4 无需填充） */
    uint16_t voltage_dv; /**< 电池电压 (0.1V), e.g. 480=48.0V */
    int16_t current_da; /**< 电池电流 (0.1A, 正=放电) */
    /* 版本信息 */
    uint32_t capacity_mah; /**< 设计容量 (mAh) */
    uint16_t cycle_count; /**< 循环次数 */
    uint16_t hw_version; /**< 硬件版本 */
    uint16_t sw_version; /**< 软件版本 */
    /* 单字节字段收尾 */
    int8_t temp_c; /**< 电芯温度 (°C) */
    bool charging; /**< 充电中 */
} srv_can_mst_bat_t; /* sizeof = 16B（重排消除 4B 填充，无需 packed） */

/**
 * @brief 上报数据体 — 对应 CAN 帧 0x001 及电池帧
 *
 * task 层通过 read_data 回调（app_status_report_fill）填充此结构。
 * 结构体首成员 status 即为完整的 0x001 帧（Byte0-7），打包时整段 memcpy 即可
 * （偏移由 srv_can_mst.c 内 offsetof 静态断言保障）。
 */
typedef struct {
    /* === 0x001 帧 Byte0-7: 系统运行/电源轨/输出/风扇/NTC/模拟输入 + 双电池 SOC === */
    srv_can_mst_status_frame_t status; /**< 0x001 状态帧（写入 .bits 成员） */

    /* === 电池帧数据 (0x011-0x015) — 双电池核心参数 === */
    srv_can_mst_bat_t bat1; /**< 电池1 上报参数 */
    srv_can_mst_bat_t bat2; /**< 电池2 上报参数 */
    /* 电池故障码（两电池故障结构不同，保持独立类型） */
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
 * @brief 主机下发的控制指令（从 0x001 RX 控制帧解析，3 字节，不含 LED）
 */
typedef struct {
    uint8_t feedback_select; /**< 反馈请求位掩码 (Byte0) */
    uint8_t buzzer_duty; /**< 蜂鸣器占空比 0-50 (Byte1) */
    bool hsd1_12v_on; /**< HSD1 12V 输出：1=开, 0=关 (Byte2 bit4，bit5 有效) */
    bool hsd1_24v_on; /**< HSD1 24V 输出：1=开, 0=关 (Byte2 bit2，bit3 有效) */
    bool hsd2_24v_on; /**< HSD2 24V 输出：1=开, 0=关 (Byte2 bit0，bit1 有效) */
} srv_can_mst_cmd_t;

/**
 * @brief 服务错误码
 */
typedef enum {
    SRV_CAN_MST_OK = 0, /**< 操作成功 */
    SRV_CAN_MST_ERROR_NULL_PTR, /**< 空指针或回调未注册 */
    SRV_CAN_MST_ERROR_UNINITIALIZED, /**< 服务未初始化 */
    SRV_CAN_MST_ERROR_QUEUE_FULL, /**< 发送队列已满（未使用，保留） */
    SRV_CAN_MST_ERROR_LAYOUT, /**< 位域帧布局与协议不符（init 布局自检失败） */
} srv_can_mst_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 CAN 上报服务
 *
 * 验证回调非 NULL，初始化 msg_fifo 发送队列和 pending 帧状态。
 *
 * @param config 服务配置（read_data + send_frame 两个回调必须提供）
 * @return SRV_CAN_MST_OK 成功，SRV_CAN_MST_ERROR_NULL_PTR 参数错误，
 *         SRV_CAN_MST_ERROR_LAYOUT 位域布局自检失败（协议字节位不匹配）
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
 * 和 0x011-0x015 电池数据帧（按 feedback_select 选择），入队等待发送。
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
