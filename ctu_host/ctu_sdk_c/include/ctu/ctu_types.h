//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_types.h
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU SDK 公共类型：设备、命令、数据模型与回调。
 */

#ifndef __CTU_TYPES_H
#define __CTU_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "ctu_config.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 目标设备（枚举值即应用协议地址）
 */
typedef enum {
    CTU_DEVICE_MASTER = CTU_MASTER_ADDR, /**< E1_MASTER 主控电源板 (0x01) */
    CTU_DEVICE_SLAVER = CTU_SLAVER_ADDR, /**< E1_SLAVER 副电源模块 (0x02) */
} ctu_device_t;

/**
 * @brief 命令码（两板统一）
 */
typedef enum {
    CTU_CMD_READ_STATUS = 0x01,  /**< 读系统状态 */
    CTU_CMD_READ_VOLT = 0x02,    /**< 读电压 */
    CTU_CMD_READ_TEMP = 0x03,    /**< 读温度 */
    CTU_CMD_CTRL = 0x04,         /**< 控制 */
    CTU_CMD_RESET_LATCH = 0x05,  /**< 清除故障锁存 */
    CTU_CMD_UPGRADE = 0x06,      /**< 升级请求 / Boot SELECT */
    CTU_CMD_READ_INFO = 0x07,    /**< 读固件信息 */
} ctu_cmd_t;

/**
 * @brief 设备 0x7F 错误应答码
 */
typedef enum {
    CTU_DEV_ERR_NONE = 0x00,         /**< 无错误 */
    CTU_DEV_ERR_UNKNOWN_CMD = 0x01,  /**< 未知命令 */
    CTU_DEV_ERR_UNSUPPORTED = 0x02,  /**< 功能暂不支持 */
    CTU_DEV_ERR_BAD_LENGTH = 0x03,   /**< 帧长度与命令不匹配 */
} ctu_dev_err_t;

/**
 * @brief Boot 升级错误码
 */
typedef enum {
    CTU_BOOT_ERR_NONE = 0x00,        /**< 无错误 */
    CTU_BOOT_ERR_BAD_LENGTH = 0x01,  /**< 帧长非法 */
    CTU_BOOT_ERR_BAD_STATE = 0x02,   /**< 状态错 / 未选中 */
    CTU_BOOT_ERR_BAD_BLOCK = 0x03,   /**< 块号错 */
    CTU_BOOT_ERR_BAD_CRC16 = 0x04,   /**< 数据 CRC16 错 */
    CTU_BOOT_ERR_FLASH = 0x05,       /**< Flash 写失败 */
    CTU_BOOT_ERR_BAD_SIZE = 0x06,    /**< 长度 / 容量错 */
} ctu_boot_err_t;

/**
 * @brief 固件信息标志位
 */
typedef enum {
    CTU_FW_FLAG_META_VALID = 0x01,     /**< metadata 有效 */
    CTU_FW_FLAG_UPGRADE_REQ = 0x02,    /**< 升级待处理 */
    CTU_FW_FLAG_UPGRADE_DONE = 0x04,   /**< 升级完成 */
} ctu_fw_flag_t;

/* ---- 数据模型（与各命令数据段一一对应）---- */

/**
 * @brief MASTER 系统状态（0x01）
 * @note  这些位为 true 表示“异常/有效”，不是“正常”
 */
typedef struct {
    bool estop;               /**< 急停触发 */
    bool rail_12v_fault;      /**< 12V 电源异常 */
    bool rail_24v_fault;      /**< 24V 电源异常 */
    bool vin_dcdc_fault;      /**< VIN_DC-DC 异常 */
    bool aux_fault;           /**< AUX 电源异常 */
    bool motor_fault;         /**< MOTOR 电源异常 */
    bool fan0_fault;          /**< 风扇 0 异常 */
    bool fan1_fault;          /**< 风扇 1 异常 */
    bool ntc1_disconnected;   /**< NTC1 断开 */
    bool ntc2_disconnected;   /**< NTC2 断开 */
} ctu_master_status_t;

/**
 * @brief SLAVER 系统状态（0x01）
 */
typedef struct {
    bool out_24v;       /**< 24V 输出已开启 */
    bool out_12v;       /**< 12V_ISO 输出已开启 */
    bool out_lsd1;      /**< LSD1 输出已开启 */
    bool out_lsd2;      /**< LSD2 输出已开启 */
    bool fault_24v;     /**< 24V 故障 */
    bool fault_12v;     /**< 12V_ISO 故障 */
    bool fault_lsd1;    /**< LSD1 故障 */
    bool fault_lsd2;    /**< LSD2 故障 */
    bool fault_aux;     /**< AUX 输入故障 */
    bool fault_motor;   /**< MOTOR 输入故障 */
    bool latch_active;  /**< 故障锁存有效 */
} ctu_slaver_status_t;

/**
 * @brief MASTER 电压（0x02，单位 mV）
 */
typedef struct {
    uint16_t vin_mv;       /**< VIN 电压 */
    uint16_t vin_dcdc_mv;  /**< VIN_DC-DC 电压 */
} ctu_master_voltage_t;

/**
 * @brief SLAVER 电压（0x02，单位 mV）
 */
typedef struct {
    uint16_t aux_mv;    /**< AUX 电压 */
    uint16_t motor_mv;  /**< MOTOR 电压 */
    uint16_t lsd1_mv;   /**< LSD1 电压 */
    uint16_t lsd2_mv;   /**< LSD2 电压 */
} ctu_slaver_voltage_t;

/**
 * @brief MASTER 温度（0x03，单位 °C）
 */
typedef struct {
    float ntc1_c;  /**< NTC1 温度 */
    float ntc2_c;  /**< NTC2 温度 */
    float mcu_c;   /**< MCU 温度 */
} ctu_master_temp_t;

/**
 * @brief SLAVER 温度 / VDDA（0x03）
 */
typedef struct {
    float mcu_c;      /**< MCU 温度（°C） */
    uint16_t vdda_mv; /**< VDDA（mV） */
} ctu_slaver_temp_t;

/**
 * @brief 固件 / Boot 信息（0x07）
 */
typedef struct {
    uint16_t app_version;   /**< App 版本号 */
    uint16_t meta_version;  /**< metadata（Boot）版本号 */
    uint32_t fw_size;       /**< 固件字节数 */
    uint32_t fw_checksum;   /**< 累加和（sum & 0xFFFFFFFF） */
    uint16_t reboot_counts; /**< 上电次数 */
    uint8_t flags;          /**< 标志位（见 ctu_fw_flag_t） */
} ctu_fw_info_t;

/**
 * @brief 无数据段命令的应答（0x04/0x05/0x06）
 */
typedef struct {
    ctu_device_t device;  /**< 应答设备 */
    uint8_t command;      /**< 命令码 */
    uint32_t elapsed_ms;  /**< 往返耗时（ms） */
} ctu_ack_t;

/**
 * @brief 轮询统计快照
 */
typedef struct {
    uint32_t tx;       /**< 发送请求数 */
    uint32_t rx;       /**< 成功应答数 */
    uint32_t loss;     /**< 超时/失败数 */
    float loss_rate;   /**< 丢包率（%） */
    float fps;         /**< 近 1s 应答速率 */
} ctu_poll_stats_t;

/* ---- 回调 ---- */

/**
 * @brief 日志回调
 * @param level 级别字符串：info / warn / error / tx / rx
 * @param text  日志文本
 * @param user  用户上下文
 */
typedef void (*ctu_log_cb_t)(const char* level, const char* text, void* user);

/**
 * @brief 升级进度回调
 * @param fraction 进度 0.0 ~ 1.0
 * @param user     用户上下文
 */
typedef void (*ctu_progress_cb_t)(float fraction, void* user);

/**
 * @brief 升级阶段回调
 * @param phase 阶段描述
 * @param user  用户上下文
 */
typedef void (*ctu_phase_cb_t)(const char* phase, void* user);

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 设备地址转名称
 * @param device 设备
 * @return 静态字符串（"master" / "slaver" / "unknown"）
 */
const char* ctu_device_name(ctu_device_t device);

/**
 * @brief 校验设备是否合法
 * @param device 设备
 * @return true 表示合法
 */
bool ctu_device_is_valid(ctu_device_t device);

#ifdef __cplusplus
}
#endif

#endif /* __CTU_TYPES_H */
