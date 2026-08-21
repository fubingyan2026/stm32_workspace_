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

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief P_CAN 上报帧 ID 枚举（0x010 ~ 0x012，连续）
 *
 * 电源板向主机上报数据的 CAN ID 定义，全部随 0x001 主机控制帧触发/周期发送。
 */
typedef enum {
    SRV_CAN_MST_ID_STATUS = 0x010, /**< 系统状态 + 错误标志（始终发送） */
    SRV_CAN_MST_ID_VOLT_TEMP = 0x011, /**< NTC/MCU 温度 */
    SRV_CAN_MST_ID_POWER_FAULT = 0x012, /**< 电源电压 + 电机预充故障 */
} srv_can_mst_can_id_t;

/**
 * @brief 0x010 状态帧（Byte0-1）视图（字段声明顺序 = bit 顺序，bit0 在前）
 *
 * 位域由编译器自动对齐，替代手写移位打包；union 的 bytes 视图便于整帧观察/校验。
 * 按当前工程精简：0x001 状态帧压缩为 2 字节（状态位 + NTC 连接状态）。
 *
 * @attention 位域分配由编译器/ABI 决定（C 标准未规定）：
 *   - GCC ARM (AAPCS, little-endian) 下从 LSB 分配，实测与协议一致；
 *   - 更换工具链/大小端会静默改变线上字节位，srv_can_mst_init 内有布局自检兜底。
 */
typedef union {
    struct __attribute__((packed)) {
        /* === byte0: 急停 + 电源/输出异常 === */
        uint8_t stop_key_state : 1; /**< [bit0] 急停：0=释放, 1=按下 */
        uint8_t err_12v_ext : 1; /**< [bit1] 外部 12V 输出异常 */
        uint8_t err_24v_ext : 1; /**< [bit2] 外部 24V 输出异常 */
        uint8_t err_24v_computer : 1; /**< [bit3] 工控机 24V 输出异常 */
        uint8_t err_aux_power : 1; /**< [bit4] 辅电电源异常（AUX PGD） */
        uint8_t err_motor_power : 1; /**< [bit5] 电机电源异常（MOTOR PGD） */
        uint8_t err_chg_out : 1; /**< [bit6] 预充电异常（CHG OCP） */
        uint8_t err_hsd_fault : 1; /**< [bit7] HSD公用通道异常 */
        /* === byte1: 制动/模拟输入/风扇 === */
        uint8_t err_dbr : 1; /**< [bit0] 制动电阻过流（DBR OCP） */
        uint8_t a_in1_io : 1; /**< [bit1] A_IN1_IO 模拟输入 */
        uint8_t a_in2_io : 1; /**< [bit2] A_IN2_IO 模拟输入 */
        uint8_t a_in3_io : 1; /**< [bit3] A_IN3_IO 模拟输入 */
        uint8_t err_fan0 : 1; /**< [bit4] 风扇0 异常 */
        uint8_t err_fan1 : 1; /**< [bit5] 风扇1 异常 */
        uint8_t err_ntc1 : 1; /**< [bit6] ntc1 未连接 */
        uint8_t err_ntc2 : 1; /**< [bit7] ntc2 未连接 */
    } bits;

    uint8_t bytes[2]; /**< 原始字节视图（0x001 帧 Byte0-7） */
} srv_can_mst_status_frame_t;

/**
 * @brief 0x011 温度帧视图（NTC/MCU 温度，8 字节）
 */
typedef union {
    struct __attribute__((packed)) {
        int16_t ntc1_temp_x100; /**< [Byte0-1] NTC1 温度 (°C×100, int16 LE) */
        int16_t ntc2_temp_x100; /**< [Byte2-3] NTC2 温度 (°C×100, int16 LE) */
        int16_t mcu_temp_x100; /**< [Byte4-5] MCU 温度 (°C×100, int16 LE) */
        uint8_t reserved[2]; /**< [Byte6-7] 保留 */
    } data;
    uint8_t bytes[8]; /**< 原始字节视图 */
} srv_can_mst_volt_temp_frame_t;

/**
 * @brief 0x012 电源电压+预充故障帧视图（8 字节）
 */
typedef union {
    struct __attribute__((packed)) {
        uint16_t vin_mv; /**< [Byte0-1] 主输入电压 (mV, uint16 LE) */
        uint16_t motor_power_mv; /**< [Byte2-3] 电机电源电压 (mV, uint16 LE) */
        uint16_t aux_power_mv; /**< [Byte4-5] 辅助电源电压 (mV, uint16 LE) */
        uint8_t precharge_fault; /**< [Byte6] 电机预充故障码：0=无,1=短路,2=未接负载 */
        uint8_t byte7_reserved; /**< [Byte7] 保留 */
    } data;
    uint8_t bytes[8]; /**< 原始字节视图 */
} srv_can_mst_power_fault_frame_t;

/**
 * @brief 上报数据体 — 对应 CAN 帧 0x001 及电池帧
 *
 * task 层通过 read_data 回调（app_status_report_fill）填充此结构。
 * 结构体首成员 status 即为完整的 0x001 帧（Byte0-7），打包时整段 memcpy 即可
 * （偏移由 srv_can_mst.c 内 offsetof 静态断言保障）。
 */
typedef struct {
    /* === 0x010 帧 Byte0-1: 急停/电源/输出/制动/模拟输入/风扇/NTC 连接状态 === */
    srv_can_mst_status_frame_t status; /**< 0x010 状态帧（写入 .bits 成员） */
    /** add 电池状态数据和故障码 */
    /* === 0x011 帧: NTC/MCU 温度 === */
    srv_can_mst_volt_temp_frame_t volt_temp; /**< 0x011 温度帧 */
    /* === 0x012 帧: 电源电压 + 电机预充故障 === */
    srv_can_mst_power_fault_frame_t power_fault; /**< 0x012 电压+预充故障帧 */
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
 * @brief 主机下发的控制指令（从 0x001 RX 控制帧解析，6 字节，含 LED RGB）
 */
typedef struct {
    uint8_t buzzer_duty; /**< 蜂鸣器占空比 0-50 (Byte0) */
    bool hsd1_12v_on; /**< HSD1 12V 输出：1=开, 0=关 (Byte1 bit4，bit5 有效) */
    bool hsd1_24v_on; /**< HSD1 24V 输出：1=开, 0=关 (Byte1 bit2，bit3 有效) */
    bool hsd2_24v_on; /**< HSD2 24V 输出：1=开, 0=关 (Byte1 bit0，bit1 有效) */
    uint8_t led_index; /**< LED 索引 (Byte2)：0-31=通道1(RGB1), 32-63=通道2(RGB2) */
    uint8_t led_r; /**< LED 红亮度 (Byte3) */
    uint8_t led_g; /**< LED 绿亮度 (Byte4) */
    uint8_t led_b; /**< LED 蓝亮度 (Byte5) */
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
