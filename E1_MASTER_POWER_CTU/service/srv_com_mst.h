/**
 * @file    srv_com_mst.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-03
 * @brief   RS485 主机协议服务（E1_MASTER_POWER_CTU）— 帧解析/打包 + 命令处理
 * @attention
 *
 * 485 为主机查询应答式：本板作为从站，不自主上报，仅应答主机。
 * 帧格式与 G0 上位机协议一致（参考 srv_uart_rx_cmd/tx_cmd）：
 *   [ 'z' ][cmd][data_len][payload...][CRC8][ '\n' ]，总长 = data_len + 5
 *   - 帧头 'z'(0x7A) 1B | cmd 1B | data_len 1B | payload data_len B |
 *     CRC8 1B（get_CRC8_check_sum 初值 0xFF）| 帧尾 '\n'(0x0A) 1B
 *
 * 解析使用 m_middlewares protocol_parser 库、打包使用 protocol_packer 库。
 * 命令码与负载布局见 docs/protocol_master_485.md。
 *
 * ## 用法
 * @code
 *   srv_com_mst_config_t cfg = {
 *       .read_data = app_status_report_fill,   // 数据读取回调
 *       .ctrl      = my_ctrl_cb,               // 控制命令应用回调
 *       .send_frame= my_send_cb,               // 原始帧发送回调（task 层 → dev_rs485）
 *   };
 *   srv_com_mst_init(&cfg);
 *
 *   // task 层周期调用：
 *   n = dev_rs485_rx_read(buf, sizeof(buf));
 *   srv_com_mst_rx_feed(buf, n);
 *   srv_com_mst_rx_tick();
 * @endcode
 */

#ifndef __SRV_COM_MST_H
#define __SRV_COM_MST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

#define SRV_COM_MST_CMD_REPLY_FLAG (0x80U) /**< 应答帧命令标志位 */
#define SRV_COM_MST_CMD_ERR (0x7FU) /**< 错误应答命令 */

/** @brief 最大负载长度（状态 2 / 温度 6 / 电压 4 / ACK 1） */
#define SRV_COM_MST_MAX_PAYLOAD_LEN (16U)

/** @brief 最大帧长 = MAX_PAYLOAD + 5（z/cmd/len + payload + crc + \n） */
#define SRV_COM_MST_MAX_FRAME_LEN (SRV_COM_MST_MAX_PAYLOAD_LEN + 5U)

/** @brief 错误码（应答负载第 0 字节） */
typedef enum {
    SRV_COM_MST_ERR_NONE = 0x00, /**< 无错误 */
    SRV_COM_MST_ERR_UNKNOWN_CMD = 0x01, /**< 未知命令 */
    SRV_COM_MST_ERR_NOT_SUPPORTED = 0x02, /**< 功能暂不支持（如升级请求） */
    SRV_COM_MST_ERR_BAD_LEN = 0x03, /**< 帧长度与命令不匹配 */
} srv_com_mst_err_code_t;

/** @brief 主机查询/控制命令码（下行） */
typedef enum {
    SRV_COM_MST_CMD_READ_STATUS = 0x01, /**< 读系统状态（2B 位域） */
    SRV_COM_MST_CMD_READ_TEMP = 0x02, /**< 读温度（6B：NTC1/NTC2/MCU ×100℃） */
    SRV_COM_MST_CMD_READ_VOLT = 0x03, /**< 读电压（4B：VIN/VIN_DC-DC mV） */
    SRV_COM_MST_CMD_CTRL = 0x10, /**< 控制（蜂鸣器 + 预留） */
    SRV_COM_MST_CMD_UPGRADE = 0x11, /**< 升级请求（预留，本阶段应答不支持） */
} srv_com_mst_cmd_t;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 系统状态帧（2 字节，字段声明顺序 = bit 顺序）
 */
typedef union {
    struct __attribute__((packed)) {
        /* === byte0 === */
        uint8_t stop_key_state : 1; /**< [bit0] 急停：0=释放, 1=按下 */
        uint8_t err_12v : 1; /**< [bit1] 12V 电源异常（PGOOD_12V） */
        uint8_t err_24v : 1; /**< [bit2] 24V 降压电源异常（DC_DC_24V_PGOOD） */
        uint8_t err_vin_dcdc : 1; /**< [bit3] VIN_DC-DC 前端异常（LM5060_PGOOD） */
        uint8_t err_aux_power : 1; /**< [bit4] 辅助电源异常（AUX PGD） */
        uint8_t err_motor_power : 1; /**< [bit5] 电机电源异常（MOTOR PGD） */
        uint8_t reserved6 : 1; /**< [bit6] 预留 */
        uint8_t reserved7 : 1; /**< [bit7] 预留 */
        /* === byte1 === */
        uint8_t err_fan0 : 1; /**< [bit0] 风扇0 异常 */
        uint8_t err_fan1 : 1; /**< [bit1] 风扇1 异常 */
        uint8_t err_ntc1 : 1; /**< [bit2] NTC1 未连接 */
        uint8_t err_ntc2 : 1; /**< [bit3] NTC2 未连接 */
        uint8_t reserved_b1_4 : 1; /**< [bit4] 预留 */
        uint8_t reserved_b1_5 : 1; /**< [bit5] 预留 */
        uint8_t reserved_b1_6 : 1; /**< [bit6] 预留 */
        uint8_t reserved_b1_7 : 1; /**< [bit7] 预留 */
    } bits;
    uint8_t bytes[2];
} srv_com_mst_status_frame_t;

/**
 * @brief 主机控制指令（0x10 控制帧，长度 1）
 */
typedef struct {
    uint8_t buzzer_duty; /**< 蜂鸣器导通占空比 0-50% (0=静音, 50=最响；驱动会截断超限) */
} srv_com_mst_ctrl_t;

/**
 * @brief 上报数据体（read_data 回调填充，服务按需打包）
 */
typedef struct {
    srv_com_mst_status_frame_t status; /**< 系统状态（写 .bits） */
    int16_t ntc1_temp_x100; /**< NTC1 温度 (°C×100) */
    int16_t ntc2_temp_x100; /**< NTC2 温度 (°C×100) */
    int16_t mcu_temp_x100; /**< MCU 温度 (°C×100) */
    uint16_t vin_mv; /**< 主输入电压 (mV) */
    uint16_t vin_dcdc_mv; /**< VIN_DC-DC 输入电压 (mV) */
} srv_com_mst_report_t;

/** @brief 数据读取回调（task 层实现，通常为 app_status_report_fill） */
typedef void (*srv_com_mst_read_cb_t)(srv_com_mst_report_t* report);

/** @brief 控制命令应用回调（task 层实现，如蜂鸣器占空比 → drv_buzzer） */
typedef void (*srv_com_mst_ctrl_cb_t)(const srv_com_mst_ctrl_t* ctrl);

/** @brief 应答帧发送回调（task 层实现 → dev_rs485_send） */
typedef void (*srv_com_mst_send_cb_t)(const uint8_t* data, uint32_t len);

/** @brief 服务配置结构体 */
typedef struct {
    srv_com_mst_read_cb_t read_data; /**< 数据读取回调（必填） */
    srv_com_mst_ctrl_cb_t ctrl; /**< 控制命令回调（必填） */
    srv_com_mst_send_cb_t send_frame; /**< 原始帧发送回调（必填） */
} srv_com_mst_config_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化主机协议服务（内部初始化 protocol_parser + protocol_packer） */
void srv_com_mst_init(const srv_com_mst_config_t* config);

/** @brief 反初始化（清回调） */
void srv_com_mst_deinit(void);

/** @brief 查询服务是否已初始化 */
bool srv_com_mst_is_initialized(void);

/**
 * @brief 喂入原始字节流（task 从 dev_rs485 读取后调用），内部解析完整帧并应答
 * @note  main 循环上下文调用
 */
void srv_com_mst_rx_feed(const uint8_t* data, uint32_t len);

/** @brief 空闲超时 tick：转发 protocol_parser_tick（与 feed 同周期调用即可） */
void srv_com_mst_rx_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_COM_MST_H */
