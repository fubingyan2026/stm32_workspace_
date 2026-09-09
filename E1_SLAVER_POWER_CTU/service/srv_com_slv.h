/**
 * @file    srv_com_slv.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   RS485 从机协议服务（E1_SLAVER_POWER_CTU）— 帧解析/打包 + 命令处理
 * @attention
 *
 * 副电源模块作为 485 从站，主机查询应答式：本板不自主上报，仅应答主机。
 * 两兄弟板（E1_MASTER/E1_SLAVER）**共用同一帧头/信封**（避免帧头差异导致解析失败），
 * 设备区分放在 **payload 首字节的设备 ID** 上：
 *   [ 'z' ][cmd][data_len][payload...][CRC8][ '\n' ]，总长 = data_len + 5
 *   - 帧头 'z'(0x7A) 1B | cmd 1B | data_len 1B | payload data_len B |
 *     CRC8 1B（get_CRC8_check_sum 初值 0xFF）| 帧尾 '\n'(0x0A) 1B
 *   - 下行请求 payload[0] = 目标设备 ID（0x01=MASTER / 0x02=SLAVER）；
 *     ID 不匹配本板的帧直接忽略不响应（定向，避免两板同时应答撞线）
 *   - 上行应答 payload[0] = 源设备 ID（本板 ID），其后为数据/错误码
 *
 * 解析使用 m_middlewares protocol_parser 库、打包使用 protocol_packer 库。
 * 命令码与负载布局见 docs/protocol_slaver_485.md。
 *
 * ## 用法
 * @code
 *   srv_com_slv_config_t cfg = {
 *       .read_data   = app_status_report_fill, // 数据读取回调
 *       .ctrl        = my_ctrl_cb,             // 0x04 输出控制应用回调
 *       .reset_latch = my_reset_cb,            // 0x05 清除锁存回调
 *       .upgrade     = my_upgrade_cb,          // 0x06 升级请求回调（可选）
 *       .send_frame  = my_send_cb,             // 原始帧发送回调（task 层 → dev_rs485）
 *   };
 *   srv_com_slv_init(&cfg);
 *
 *   // task 层周期调用：
 *   n = dev_rs485_rx_read(buf, sizeof(buf));
 *   srv_com_slv_rx_feed(buf, n);
 *   srv_com_slv_rx_tick();
 * @endcode
 */

#ifndef __SRV_COM_SLV_H
#define __SRV_COM_SLV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

#define SRV_COM_SLV_CMD_REPLY_FLAG (0x80U) /**< 应答帧命令标志位 */
#define SRV_COM_SLV_CMD_ERR (0x7FU) /**< 错误应答命令 */

/** @brief 本板 485 设备 ID（E1_SLAVER，payload 首字节标识/定向，见 docs） */
#define SRV_COM_SLV_DEV_ID (0x02U)

/** @brief 最大负载长度（含首字节设备 ID：状态 3 / 电压 9 / 温度 5 / 控制 5 / ACK 2） */
#define SRV_COM_SLV_MAX_PAYLOAD_LEN (16U)

/** @brief 最大帧长 = MAX_PAYLOAD + 5（z/cmd/len + payload + crc + \n） */
#define SRV_COM_SLV_MAX_FRAME_LEN (SRV_COM_SLV_MAX_PAYLOAD_LEN + 5U)

/** @brief 控制帧负载长度（0x04，不含首字节 ID 后的 4B） */
#define SRV_COM_SLV_CTRL_LEN (4U)

/** @brief 错误码（应答负载第 0 字节） */
typedef enum {
    SRV_COM_SLV_ERR_NONE = 0x00, /**< 无错误 */
    SRV_COM_SLV_ERR_UNKNOWN_CMD = 0x01, /**< 未知命令 */
    SRV_COM_SLV_ERR_NOT_SUPPORTED = 0x02, /**< 功能暂不支持 */
    SRV_COM_SLV_ERR_BAD_LEN = 0x03, /**< 帧长度与命令不匹配 */
} srv_com_slv_err_code_t;

/** @brief 主机查询/控制命令码（下行，连续编号） */
typedef enum {
    SRV_COM_SLV_CMD_READ_STATUS = 0x01, /**< 读系统状态（2B：故障位 + 输出/锁存位） */
    SRV_COM_SLV_CMD_READ_VOLT = 0x02, /**< 读电压（8B：aux/motor/lsd1/lsd2 mV） */
    SRV_COM_SLV_CMD_READ_TEMP = 0x03, /**< 读温度/VDDA（4B：mcu_temp×100 + vdda_mv） */
    SRV_COM_SLV_CMD_CTRL = 0x04, /**< 输出控制（4B：输出掩码 + 补光亮度） */
    SRV_COM_SLV_CMD_RESET_LATCH = 0x05, /**< 清除故障锁存（1B magic=0x01） */
    SRV_COM_SLV_CMD_UPGRADE = 0x06, /**< 升级请求（1B magic=0x01 → 跳转 Boot） */
} srv_com_slv_cmd_t;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 系统状态帧（2 字节，字段声明顺序 = bit 顺序）
 */
typedef union {
    struct __attribute__((packed)) {
        /* === byte0：故障位（1=异常） === */
        uint8_t err_24v : 1; /**< [bit0] 24V 降压电源故障/锁存 */
        uint8_t err_12v : 1; /**< [bit1] 12V_ISO 电源故障/锁存 */
        uint8_t err_aux : 1; /**< [bit2] AUX 输入缺失/低于阈值 */
        uint8_t err_motor : 1; /**< [bit3] MOTOR 输入缺失/低于阈值 */
        uint8_t err_lsd1 : 1; /**< [bit4] LSD1 输出故障/锁存 */
        uint8_t err_lsd2 : 1; /**< [bit5] LSD2 输出故障/锁存 */
        uint8_t reserved_b0_6 : 1; /**< [bit6] 预留 */
        uint8_t reserved_b0_7 : 1; /**< [bit7] 预留 */
        /* === byte1：输出/锁存状态 === */
        uint8_t out_24v : 1; /**< [bit0] 24V 实际输出 */
        uint8_t out_12v : 1; /**< [bit1] 12V_ISO 实际输出 */
        uint8_t out_lsd1 : 1; /**< [bit2] LSD1 实际输出 */
        uint8_t out_lsd2 : 1; /**< [bit3] LSD2 实际输出 */
        uint8_t latch_active : 1; /**< [bit4] 存在故障锁存 */
        uint8_t reserved_b1_5 : 1; /**< [bit5] 预留 */
        uint8_t reserved_b1_6 : 1; /**< [bit6] 预留 */
        uint8_t reserved_b1_7 : 1; /**< [bit7] 预留 */
    } bits;
    uint8_t bytes[2];
} srv_com_slv_status_frame_t;

/**
 * @brief 主机控制指令（0x04 控制帧，长度 4）
 */
typedef struct {
    uint8_t output_mask; /**< 输出掩码：bit0=24V、bit1=12V_ISO、bit2=LSD1、bit3=LSD2 */
    uint8_t reserved; /**< 预留，恒 0 */
    uint16_t fill_duty; /**< 补光灯亮度 0..1000 (0.1%)，uint16 LE */
} srv_com_slv_ctrl_t;

/**
 * @brief 上报数据体（read_data 回调填充，服务按需打包）
 */
typedef struct {
    srv_com_slv_status_frame_t status; /**< 系统状态（写 .bits） */
    uint32_t aux_mv; /**< AUX 输入电压 (mV) */
    uint32_t motor_mv; /**< MOTOR 输入电压 (mV) */
    uint32_t lsd1_mv; /**< LSD1 节点电压 (mV) */
    uint32_t lsd2_mv; /**< LSD2 节点电压 (mV) */
    int16_t mcu_temp_x100; /**< MCU 温度 (°C×100) */
    uint32_t vdda_mv; /**< VDDA (mV) */
} srv_com_slv_report_t;

/** @brief 数据读取回调（task 层实现，通常为 app_status_report_fill） */
typedef void (*srv_com_slv_read_cb_t)(srv_com_slv_report_t* report);

/** @brief 控制命令应用回调（task 层实现：输出掩码 → srv_pwr_ctrl、补光 → drv_pwm） */
typedef void (*srv_com_slv_ctrl_cb_t)(const srv_com_slv_ctrl_t* ctrl);

/** @brief 清除故障锁存回调（task 层实现 → srv_pwr_ctrl_clear_latch；可空） */
typedef void (*srv_com_slv_reset_cb_t)(void);

/** @brief 升级请求回调（task 层实现 → srv_boot_ctrl 写升级标志并复位；可空） */
typedef void (*srv_com_slv_upgrade_cb_t)(void);

/** @brief 应答帧发送回调（task 层实现 → dev_rs485_send） */
typedef void (*srv_com_slv_send_cb_t)(const uint8_t* data, uint32_t len);

/** @brief 服务配置结构体 */
typedef struct {
    srv_com_slv_read_cb_t read_data; /**< 数据读取回调（必填） */
    srv_com_slv_ctrl_cb_t ctrl; /**< 控制命令回调（必填） */
    srv_com_slv_reset_cb_t reset_latch; /**< 清除锁存回调（可选） */
    srv_com_slv_upgrade_cb_t upgrade; /**< 升级请求回调（可选） */
    srv_com_slv_send_cb_t send_frame; /**< 原始帧发送回调（必填） */
} srv_com_slv_config_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化从机协议服务（内部初始化 protocol_parser + protocol_packer） */
void srv_com_slv_init(const srv_com_slv_config_t* config);

/** @brief 反初始化（清回调） */
void srv_com_slv_deinit(void);

/** @brief 查询服务是否已初始化 */
bool srv_com_slv_is_initialized(void);

/**
 * @brief 喂入原始字节流（task 从 dev_rs485 读取后调用），内部解析完整帧并应答
 * @note  main 循环上下文调用
 */
void srv_com_slv_rx_feed(const uint8_t* data, uint32_t len);

/** @brief 空闲超时 tick：转发 protocol_parser_tick（与 feed 同周期调用即可） */
void srv_com_slv_rx_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_COM_SLV_H */
