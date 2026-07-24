``` c
#include <stdint.h>

// 强制编译器按1字节对齐，防止内存中出现空洞填充，保证完全匹配 CAN 的 8 字节载荷
#pragma pack(push, 1)

// =========================================================
// [ID: 0x200] 单电池核心动态帧 (高频MUX 100ms) 
// =========================================================
typedef struct {
    uint8_t  bat_id;            // Byte 0:   电池编号 (0x01=Bat1, 0x02=Bat2)
    uint16_t voltage;           // Byte 1-2: 单包总电压 (0.01V/bit)
    int16_t  current;           // Byte 3-4: 单包总电流 (0.01A/bit)
    uint8_t  soc;               // Byte 5:   单包 SOC (0~100%)
    int8_t   cell_temp;         // Byte 6:   电芯温度 (℃)
    uint8_t  is_charging        // Byte 7: 是否正在充电
} Uplink_BatCore_t;

// =========================================================
// [ID: 0x201] 固化/低频信息帧 (低频MUX, 可做请求响应)
// =========================================================
typedef struct {
    uint8_t  info_key;          // Byte 0:   信息键值 (0x01=Bat1容量, 0x03=Bat1版本等)
    
    union {
        // Key 映射 1: 容量数据
        struct {
            uint32_t design_cap; // Byte 1-4: 设计容量 (mAh)
            uint16_t full_cap;   // Byte 5-6: 满充容量 (mAh)
            uint8_t  reserved1;  // Byte 7:   预留
        } capacity_info;
        
        // Key 映射 2: 版本与循环数据
        struct {
            uint16_t hw_version; // Byte 1-2: 硬件版本 
            uint16_t sw_version; // Byte 3-4: 软件版本
            uint16_t cycle_count;// Byte 5-6: 循环次数
            uint8_t  reserved2;  // Byte 7:   预留
        } version_info;
        
        uint8_t raw_payload[7];  // 通用载荷
    };
} Uplink_StaticInfo_t;

// =========================================================
// [ID: 0x202-A] 电池 1 (Can 1.1) 原生专属故障包 (长 7 Byte)
// =========================================================
typedef struct {
    uint8_t cell_ov          : 1; // Bit0: 电芯过压保护
    uint8_t total_ov         : 1; // Bit1: 总压过压保护
    uint8_t fully_charged    : 1; // Bit2: 充满保护
    uint8_t cell_uv          : 1; // Bit3: 电芯欠压保护
    uint8_t total_uv         : 1; // Bit4: 总压欠压保护
    uint8_t volt_rsv         : 3; 
    
    uint8_t short_circuit    : 1; // Bit0: 放电短路保护
    uint8_t dischg_oc        : 1; // Bit1: 放电过流保护
    uint8_t chg_oc           : 1; // Bit2: 充电过流保护
    uint8_t curr_rsv         : 5; 

    uint8_t chg_ov_temp      : 1; // Bit0: 充电高温保护
    uint8_t dischg_ov_temp   : 1; // Bit1: 放电高温保护
    uint8_t mos_ov_temp      : 1; // Bit2: MOS 过温保护
    uint8_t amb_ov_temp      : 1; // Bit3: 环境高温保护
    uint8_t amb_low_temp     : 1; // Bit4: 环境低温保护
    uint8_t temp_rsv         : 3; 

    uint8_t temp_sensor_fail : 1; // Bit0: 温度采集失效
    uint8_t volt_sensor_fail : 1; // Bit1: 电压采集失效
    uint8_t dischg_mos_fail  : 1; // Bit2: 放电 MOS 失效
    uint8_t chg_mos_fail     : 1; // Bit3: 充电 MOS 失效
    uint8_t cell_imbalance   : 1; // Bit4: 电芯不均衡告警
    uint8_t hw_rsv           : 3; 

    uint16_t warnings;            // Byte 5-6: 其他轻微预警状态集合
    uint8_t  fault_level;         // Byte 7: 严重等级 (0=正常, 1=轻微, 2=严重, 3=致命)
} Bat_can_Detailed_Fault_t;

// =========================================================
// [ID: 0x202-B] 电池 2 (RYDER) 原生专属故障包 (长 7 Byte)
// =========================================================
typedef struct {
    uint8_t chg_temp_prot    : 2; // Bit0-1: 充电高/低温保护
    uint8_t dischg_temp_prot : 2; // Bit2-3: 放电高/低温保护
    uint8_t chg_temp_warn    : 2; // Bit4-5: 充电高/低温告警
    uint8_t dischg_temp_warn : 2; // Bit6-7: 放电高/低温告警

    uint8_t chg_ov_prot      : 1; // Bit0: 充电过压保护
    uint8_t chg_ov_hw        : 1; // Bit1: 充电过压保护(硬件)
    uint8_t chg_ov_second    : 1; // Bit2: 充电过压二次保护(硬件)
    uint8_t dischg_uv_prot   : 1; // Bit3: 放电欠压保护
    uint8_t dischg_uv_hw     : 1; // Bit4: 放电欠压保护(硬件)
    uint8_t volt_rsv         : 3; 

    uint8_t chg_oc_prot      : 1; // Bit0: 充电过流保护
    uint8_t short_circuit    : 1; // Bit1: 短路保护
    uint8_t dischg_oc_prot   : 1; // Bit2: 放电过流保护
    uint8_t dischg_oc_second : 1; // Bit3: 放电过流二次保护
    uint8_t dischg_oc_hw     : 1; // Bit4: 放电过流硬件保护
    uint8_t hw_defected      : 1; // Bit5: 电池硬件损坏
    uint8_t curr_rsv         : 2; 

    uint8_t chg_ov_warn      : 1; // Bit0: 充电过压告警
    uint8_t dischg_uv_warn   : 1; // Bit1: 放电欠压告警
    uint8_t chg_oc_warn      : 1; // Bit2: 充电过流告警
    uint8_t dischg_oc_warn   : 1; // Bit3: 放电过流告警
    uint8_t chg_fet_fail     : 1; // Bit4: 充电 FET(MOS) 失效
    uint8_t dischg_fet_fail  : 1; // Bit5: 放电 FET(MOS) 失效
    uint8_t fuse_blown       : 1; // Bit6: ★三端保险丝熔断
    uint8_t fuse_rsv         : 1; 

    uint16_t extra_warnings;      // Byte 5-6: 其他告警
    uint8_t  fault_level;         // Byte 7: 严重等级 (0=正常, 1=轻微, 2=严重, 3=致命)
} Bat_RYDER_Detailed_Fault_t;

// =========================================================
// [ID: 0x202] 独立详细故障数据包 (事件触发+心跳MUX)
// =========================================================
typedef struct {
    uint8_t bat_id;               // Byte 0: 电池编号 (0x01=电池1, 0x02=电池2)
    union {
        Bat_can_Detailed_Fault_t bat_can; 
        Bat_RYDER_Detailed_Fault_t bat_RYDER;
        uint8_t raw_payload[7]; 
    } faults;
} Uplink_Detailed_Fault_t;

// =========================================================
// 统一 CAN 发送载荷联合体 (永远占 8 字节)
// =========================================================
typedef union {
    uint8_t data[8];                        // 丢给硬件发送寄存器用的纯数组
    Uplink_BatCore_t        bat_core;       // 映射 ID: 0x200
    Uplink_StaticInfo_t     static_info;    // 映射 ID: 0x201
    Uplink_Detailed_Fault_t detailed_fault; // 映射 ID: 0x202
} Uplink_CAN_Payload_u;

#pragma pack(pop) // 恢复默认内存对齐方式

#endif // __CAN_DUAL_POWER_PRO_H__