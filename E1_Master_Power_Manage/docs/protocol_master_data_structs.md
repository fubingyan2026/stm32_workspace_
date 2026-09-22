# 主电源板(CTU) 协议数据结构说明

- **作者**：maximillian
- **日期**：2026-09-17
- **版本**：V3.0.0
- **摘要**：主电源板（STM32F407）CAN 上报与控制协议对应的固件内部数据结构，仅描述**帧内数据内容**，不含发送/队列等传输机制。

> 位域声明顺序即位序（bit0 在前）；位域分配由编译器/ABI 决定，`srv_can_mst_init()` 内建布局自检兜底。

---

## 上报帧

```c
/**
 * @brief 上报数据体 — 一帧对应一个成员
 */
typedef struct {
    srv_can_mst_status_frame_t      status;      /* 系统状态与故障位 */
    srv_can_mst_volt_temp_frame_t   volt_temp;   /* NTC1/NTC2/MCU 温度 */
    srv_can_mst_power_fault_frame_t power_fault; /* VIN/MOTOR/AUX 电压 + 预充故障码 */
} srv_can_mst_data_t;
```

### 系统状态帧（2 字节）

```c
typedef union {
    struct __attribute__((packed)) {
        /* ── byte0：急停 + 电源/输出异常（1=故障，0=正常） ── */
        uint8_t stop_key_state   : 1; /* [bit0] 有效急停：0=释放, 1=按下（数字+冗余ADC双确认） */
        uint8_t err_12v_ext      : 1; /* [bit1] 外部 12V 输出异常 */
        uint8_t err_24v_ext      : 1; /* [bit2] 外部 24V 输出异常 */
        uint8_t err_24v_computer : 1; /* [bit3] 工控机 24V 输出异常 */
        uint8_t err_aux_power    : 1; /* [bit4] 辅助电源异常（AUX PGD） */
        uint8_t err_motor_power  : 1; /* [bit5] 电机电源异常（MOTOR PGD） */
        uint8_t err_chg_out      : 1; /* [bit6] 电机预充电过流（CHG OCP） */
        uint8_t err_hsd_fault    : 1; /* [bit7] HSD 高边驱动公共通道故障 */

        /* ── byte1：制动 / 模拟输入 / 风扇 / NTC 连接 ── */
        uint8_t err_dbr          : 1; /* [bit0] 制动电阻过流（DBR OCP） */
        uint8_t a_in1_io         : 1; /* [bit1] A_IN1_IO 逻辑状态：0=低, 1=高（状态位） */
        uint8_t a_in2_io         : 1; /* [bit2] A_IN2_IO 逻辑状态：0=低, 1=高（状态位） */
        uint8_t a_in3_io         : 1; /* [bit3] A_IN3_IO 逻辑状态：0=低, 1=高（状态位） */
        uint8_t err_fan0         : 1; /* [bit4] 风扇 0 异常（堵转/转速过低） */
        uint8_t err_fan1         : 1; /* [bit5] 风扇 1 异常（堵转/转速过低） */
        uint8_t err_ntc1         : 1; /* [bit6] NTC1 未连接：0=已连接, 1=未连接 */
        uint8_t err_ntc2         : 1; /* [bit7] NTC2 未连接：0=已连接, 1=未连接 */
    } bits;

    uint8_t bytes[2]; /* 原始字节视图 */
} srv_can_mst_status_frame_t;
```

### 温度帧（8 字节）

```c
typedef union {
    struct __attribute__((packed)) {
        int16_t ntc1_temp_x100; /* [Byte0-1] NTC1 外部温度，单位 0.01°C（4500 = 45.00°C） */
        int16_t ntc2_temp_x100; /* [Byte2-3] NTC2 外部温度，单位 0.01°C */
        int16_t mcu_temp_x100;  /* [Byte4-5] MCU 内部温度，单位 0.01°C */
        uint8_t reserved[2];    /* [Byte6-7] 保留，恒 0 */
    } data;

    uint8_t bytes[8]; /* 原始字节视图 */
} srv_can_mst_volt_temp_frame_t;
```

### 电源电压 + 预充故障帧（8 字节）

```c
typedef union {
    struct __attribute__((packed)) {
        uint16_t vin_mv;          /* [Byte0-1] 主输入电压 VIN，单位 mV */
        uint16_t motor_power_mv;  /* [Byte2-3] 电机电源电压，单位 mV */
        uint16_t aux_power_mv;    /* [Byte4-5] 辅助电源电压，单位 mV */
        uint8_t  precharge_fault; /* [Byte6] 电机预充故障码：0=无, 1=后级短路, 2=未接负载/二极管断路 */
        uint8_t  byte7_reserved;  /* [Byte7] 保留，恒 0 */
    } data;

    uint8_t bytes[8]; /* 原始字节视图 */
} srv_can_mst_power_fault_frame_t;
```

---

## 控制帧

```c
/**
 * @brief 主机下发控制指令（解析后缓存）
 *
 * HSD 三路开关以位域合并在 Byte1：仅对应 valid=1 时更新，
 * valid=0 时忽略；本结构只保留解析后的开关结果。
 */
typedef struct {
    uint8_t buzzer_duty; /* 蜂鸣器占空比 0-50 */

    /**
     * @brief HSD 输出控制位 (Byte1)：bits 为位域视图，byte 为原始字节视图
     */
    union {
        struct __attribute__((packed)) {
            uint8_t hsd1_12v_on : 1; /* [bit0] HSD1 12V 输出：0=关, 1=开 */
            uint8_t hsd1_24v_on : 1; /* [bit1] HSD1 24V 输出：0=关, 1=开 */
            uint8_t hsd2_24v_on : 1; /* [bit2] HSD2 24V 输出：0=关, 1=开 */
            uint8_t reserved : 5;    /* [bit3-7] 保留，恒 0 */
        } bits;
        uint8_t byte; /* 原始字节视图 */
    } ctrl;

    uint8_t led_index; /* LED 索引：0-31=通道1(RGB1)，32-63=通道2(RGB2)；每帧控制一个 LED */
    uint8_t led_mode;  /* LED 模式：取值由主机约定 */
    uint8_t led_r;     /* LED 红亮度 0-255 */
    uint8_t led_g;     /* LED 绿亮度 0-255 */
    uint8_t led_b;     /* LED 蓝亮度 0-255 */
} srv_can_mst_cmd_t;
```

```c
/**
 * @brief 主机可控输出通道枚举
 */
typedef enum {
    SRV_CAN_MST_OUTPUT_HSD1_12V, /* HSD1 12V 输出 */
    SRV_CAN_MST_OUTPUT_HSD1_24V, /* HSD1 24V 输出 */
    SRV_CAN_MST_OUTPUT_HSD2_24V, /* HSD2 24V 输出 */
} srv_can_mst_output_t;
```
