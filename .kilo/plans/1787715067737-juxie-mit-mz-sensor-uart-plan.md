# 橘虾(juxie) MIT 电机(CAN1 FD) + Mz 扭矩传感器(CAN2) + USART1 主机协议 实施计划

## 目标

- CAN1（FDCAN1）：橘虾伺服执行器，**只保留 MIT 模式控制**（单轴帧 `0x110|Dev_ID`），支持动态设置 MIT 参数（Kp/Kd/Pos_Max/Vel_Max/T_Max/使能/抱闸/清错）。
- CAN2（FDCAN2）：Mz 扭矩传感器（经典 CAN 2.0A，1 Mbps），**响应门控高频轮询**（目标 ~1 kHz）。
- USART1 控制台串口（drv_log_uart，115200）：二进制定长帧（沿用 uart_protocol.md 的 `0x1400AA55` 20 字节格式）实现电机控制、MIT 参数设置、传感器与电机反馈读取、流式输出。
- 新建 service 层：`srv_can_bus`（CAN 收发回调注入抽象，接线时选择通道）、`srv_juxie_motor`、`srv_mz_sensor`、`srv_uart_host`。

## 硬件/协议事实（已确认）

- 电机 1 台，Dev_ID=1。反馈由控制帧自动触发（juxie_canfd_cmd.md §2.1），无需单独查询。
- juxie 协议为 CAN FD：仲裁 1 Mbps / **数据段 5 Mbps（BRS）**。当前 `drv_can.c` 硬编码 `BitRateSwitch=FDCAN_BRS_OFF`，需支持 per-frame BRS。
- Mz 传感器：请求 `0x510`（DLC 7，`04 00 00 00 00 00 00`），应答 `0x410`（`04 00 00` + float32 大端 N·m）；标零写 `0x510`（DLC 8，`10 46 04 3F 80 00 00`），应答回显。
- CRC：`get_CRC16_CCITT_FALSE`（`../public_layer/m_middlewares/algorithm/crc.h`，srv_motor.c 已有用法）。

## 新建文件

### 1. `service/srv_can_bus.h/.c` — CAN 总线抽象（回调注入，通道可配）

```c
typedef void (*srv_can_bus_rx_handler_t)(const drv_can_msg_t* msg, void* user_data);

typedef struct {
    drv_can_channel_t channel;            /* 接线时选择的通道 */
    srv_can_bus_rx_handler_t rx_handler;  /* 服务注册的 RX 处理回调 */
    void* rx_user_data;
    uint32_t tx_cnt;
    uint32_t rx_cnt;
} srv_can_bus_t;

void srv_can_bus_init(void);              /* 内部对 CH1/CH2 注册唯一通道级分发回调 */
drv_can_error_t srv_can_bus_bind(srv_can_bus_t* bus, drv_can_channel_t ch,
    srv_can_bus_rx_handler_t rx, void* user_data);
drv_can_error_t srv_can_bus_send(const srv_can_bus_t* bus, const drv_can_msg_t* msg); /* 封装 drv_can_send(bus->channel,...) */
bool srv_can_bus_tx_ready(const srv_can_bus_t* bus);
void srv_can_bus_poll_status(const srv_can_bus_t* bus);
```

- `srv_can_bus_init()` 调 `drv_can_register_rx_callback(CH_1, bus_dispatch)` 与 `(CH_2, bus_dispatch)`；`bus_dispatch` 按 channel 查表调用绑定的 `rx_handler(msg, user_data)`（替换 can_task 现有 `can_rx_callback` 手工分发）。
- 服务层**不出现通道号**，一律 `srv_can_bus_send(&bus, ...)`；换通道只需改 can_task 中的 `srv_can_bus_bind`。

### 2. `service/srv_juxie_motor.h/.c` — 橘虾 CAN FD MIT 电机服务

接口：
```c
void srv_juxie_motor_init(const srv_can_bus_t* bus);       /* 绑定总线 + 注册 rx_handler */
void srv_juxie_motor_step(void);                           /* 1ms 定时器：重发 MIT 控制帧 */
void srv_juxie_motor_on_rx(const drv_can_msg_t* msg, void* ctx); /* ISR，仅解析存储 */
/* 主机协议调用的接口（主循环上下文） */
void srv_juxie_motor_set_mit(int16_t pos_deg_q7, int16_t vel_rpm, uint16_t kp_x100, uint16_t kd_x100, int16_t tq_001nm);
bool srv_juxie_motor_config(uint8_t param, int16_t value); /* 1使能/2清错/3抱闸/4Pos_Max(0.1°)/5Vel_Max(rpm)/6T_Max(0.01Nm) */
const srv_juxie_fb_t* srv_juxie_motor_get_fb(void);
const srv_juxie_cfg_t* srv_juxie_motor_get_cfg(void);
```

- MIT 单轴帧（juxie §4.1）：CAN ID `0x110|1 = 0x111`，`is_extended=false, is_fd=true, brs=true, dlc=9`。
  - Byte[0]：`(enable<<7)|(brake<<6)|(clear_err<<5)|(0x06<<1)`，enable/brake/clear 来自 config。
  - Byte[1~2]：pos 16bit 大端，`[0..65535]↔±Pos_Max`，`raw=(pos/Pos_Max+1)/2*65535`（饱和）。
  - Byte[3]+Byte[4][7:4]：vel 12bit；Byte[4][3:0]+Byte[5]：kp 12bit `[0..4095]↔[0..500]`；Byte[6]+Byte[7][7:4]：kd 12bit `[0..4095]↔[0..5]`；Byte[7][3:0]+Byte[8]：tq 12bit。打包写法仿 `srv_pa430_torque_test_pack_mit`。
- 默认值：`kp=kd=tq=0, pos=0, enable=0, brake=0(吸合), Pos_Max=180.0°, Vel_Max=3000rpm, T_Max=50Nm`。上电即安全（零刚度零力矩），主机设置目标前电机不动作。
- `step()` 每 1ms 重发一帧（含使能位），控制帧自动触发电机反馈。
- 反馈 `0x300|1=0x301`，同时兼容 DLC 16 与 DLC 12：
  - `pos=(b0<<8)|b1`(±32768↔±180°)、`speed=(b2<<8)|b3`(rpm)、`iq=(b4<<8)|b5`(mA)、`err=(b6<<8)|b7`、`temp=(b8<<8)|b9`(0.1℃)、`tq=(b10<<8)|b11`(0.05Nm，仅 DLC16)、`b14=模式`、`b15=状态`。
- 在线判定：`last_seen_ms` 超过 200ms 无反馈 → 离线标志+限频日志，持续发控制帧等待恢复。ISR 只写数据/时间戳，不打日志。
- 反馈存储结构（1 电机静态）：`srv_juxie_fb_t { pos_deg_x100, speed_rpm, iq_ma, err, temp_x10, tq_001nm, mode, status, last_seen_ms, seq }`。

### 3. `service/srv_mz_sensor.h/.c` — Mz 扭矩传感器服务

接口：
```c
void srv_mz_sensor_init(const srv_can_bus_t* bus);
void srv_mz_sensor_step(void);                       /* 1ms：响应门控发下一查询/标零 */
void srv_mz_sensor_on_rx(const drv_can_msg_t* msg, void* ctx); /* ISR 解析 0x410 */
bool srv_mz_sensor_zero(void);                       /* 标零（通道1，挂起待发） */
const srv_mz_sensor_fb_t* srv_mz_sensor_get(void);   /* Mz(Nm) + seq + last_seen + rate */
```

- 查询帧：经典 CAN，`0x510`，DLC 7，`data={04,00,00,00,00,00,00}`。
- 轮询（“频率尽量高”）：响应门控——上次查询收到应答（或 3ms 超时）后立即发下一查询；1ms 节拍下实际 ~1kHz（经典 CAN 1Mbps 往返约 250µs，充足）。测得的查询率 `rate_qps`（每秒计数）随 GET_MZ 上报。
- 标零：置 `pending_zero`，暂停轮询，发 `0x510` DLC8 `{10,46,04,3F,80,00,00}`，收到应答或超时后恢复轮询。
- 应答解析：`id==0x410 && data[0]==0x04 && data[1..2]==0x0000` → `mz = float32(big-endian data[3..6])`；`seq++`、`last_seen_ms=millis()`、清 in-flight。
- 传感器离线：>50ms 无应答 → 标志+限频日志。

### 4. `service/srv_uart_host.h/.c` — USART1 主机二进制定长帧协议服务

接口：`void srv_uart_host_init(void); void srv_uart_host_step(void);`

- 从 `drv_log_uart_rx_read()` 取字节，状态机同步帧头 `55 AA 00 14`（`0x1400AA55` 小端），收满 20 字节后校验 CRC16_CCITT_FALSE（覆盖 can_id+data 共 12 字节，低字节在前，同 srv_motor.c:742）。错帧/超时即复位重同步。
- 响应经 `drv_log_uart_send()`（TX 忙则本帧丢弃/下拍重试）。
- 流式输出：`STREAM on` 后按 interval 周期发流帧；TX 忙跳过。

#### UART can_id 定义（can_id 4B 小端）

**TX（主机→设备）**

| can_id | 命令 | data[8] |
| :--- | :--- | :--- |
| `0x000001D0` | MIT 控制 | juxie 载荷 Byte[1..8]（pos 16bit 大端 + vel/kp/kd/tq 12bit 打包，与 juxie §4.1 完全一致） |
| `0x000001D1` | 电机配置 | `data[0]=param, data[1..2]=int16 value`（见下） |
| `0x000001E0` | 读电机反馈 | `data[0]=1` |
| `0x000001E1` | 读电机状态 | `data[0]=1` |
| `0x000000E2` | 读 Mz | 忽略 |
| `0x000000E3` | 流式控制 | `data[0]=0/1, data[1]=interval_ms(1~1000, 默认100)` |

param（0x1D1）：`1=使能(1/0)` `2=清错误(1)` `3=抱闸(1释放/0吸合)` `4=Pos_Max(0.1°)` `5=Vel_Max(rpm)` `6=T_Max(0.01Nm)`

**RX（设备→主机）**

| can_id | 内容 | data[8]（多字节小端） |
| :--- | :--- | :--- |
| `0x00E00100` | 电机反馈 | `pos(2B 0.01°) + speed(2B rpm) + iq(2B mA) + tq(2B 0.01Nm)` |
| `0x00E10100` | 电机状态 | `err(2B) + temp(2B 0.1℃) + mode(1B) + status(1B) + 预留2B` |
| `0x00E20000` | Mz | `mz(4B float32 LE) + seq(2B) + flags(1B: bit0在线 bit1标零完成) + 预留1B` |
| `0x00E30000` | 流帧 | `mz(4B float32 LE) + pos(2B 0.01°) + tq(2B 0.01Nm)` |
| `0x00FF0000` | ACK/NAK | `data[0]=1/0, data[1]=命令字节回显` |

- Mz float32：固件用 float32 变量（G474 单精度 FPU 可用），`memcpy` 转字节发送，与 printf 浮点禁用无关。
- 所有命令均回 `0x00FF0000` ACK（电机反馈/Mz 读取等成功时先回数据帧，再回 ACK 或免 ACK——实施时以数据帧本身为准，仅配置类命令必须 ACK/NAK）。

## 修改文件

### 5. `device_drivers/drv_can.h/.c` — 支持 per-frame BRS

- `drv_can_msg_t` 新增 `bool brs;`（默认 0，保持旧行为不变）。
- `drv_can_send`（drv_can.c:245）改为 `.BitRateSwitch = msg->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF`。
- 数据段位时序（DBTP Seg1=11/Seg2=4 → 5Mbps）与 TDC 已配置，juxie 服务置 `brs=true` 即可；PA430 等旧 FD 帧不置 brs，行为不变。

### 6. `service/srv_motor_test_select.h` — 新增选择项

- CAN1 枚举加 `SRV_MOTOR_TEST_JUXIE = 3`，宏 `#define SRV_MOTOR_TEST_JUXIE 3`，判定宏 `SRV_MOTOR_TEST_IS_JUXIE`；默认值改 `SRV_MOTOR_TEST_JUXIE`。
- CAN2 枚举加 `SRV_MOTOR_TEST_MZ = 2`，宏与判定宏同套路；默认值改 `SRV_MOTOR_TEST_MZ`。

### 7. `tasks/can_task.c` — 接线

- `TASK_PERIOD_MS` 5 → **1**（MIT 控制 1kHz；旧测试模块内部自带周期门控，不受影响）。
- `can_task_init`：`drv_can_init()` → `srv_can_bus_init()` → 定义 `static srv_can_bus_t s_motor_bus, s_sensor_bus;` → `srv_can_bus_bind(&s_motor_bus, DRV_CAN_CH_1, srv_juxie_motor_on_rx, NULL)`、`(&s_sensor_bus, DRV_CAN_CH_2, srv_mz_sensor_on_rx, NULL)` → `srv_juxie_motor_init(&s_motor_bus)`、`srv_mz_sensor_init(&s_sensor_bus)`、`srv_uart_host_init()`。
- 定时器回调：`srv_can_bus_poll_status`（双通道，或保留 `drv_can_poll_status`）→ `srv_juxie_motor_step()` → `srv_mz_sensor_step()` → `srv_uart_host_step()`。
- 旧的 can_rx_callback 手工分发保留给 HT/TONGZHI/PA430 模式（JUXIE/MZ 模式走 bus 分发）；`#if SRV_MOTOR_TEST_IS_JUXIE` 等分支接线新增模块，旧分支原样保留，保证各选择项可回切。

### 8. `docs/juxie_mz_uart_protocol.md`（可选但推荐）

记录第 4 节的 can_id 表、MIT 载荷打包示例、Mz 查询/标零示例、CRC 计算说明。

## 数据流

```
主机 ──USART1 20B帧──▶ srv_uart_host ──▶ srv_juxie_motor(MIT目标/配置) ──CAN1 0x111 FD──▶ 电机
电机 ──CAN1 0x301 反馈──▶ srv_juxie_motor_on_rx(ISR) ──存储──▶ srv_uart_host(GET/流帧)
srv_mz_sensor_step(1ms) ──CAN2 0x510──▶ Mz传感器 ──0x410──▶ srv_mz_sensor_on_rx(ISR) ──存储──▶ UART(GET/流帧)
```

## 风险与注意

- **CAN FD 5M 数据段**：drv_can 的 DBTP 是按 Motorevo 文档调的；若 juxie 电机数据段位时序（Seg2）要求不同，可能出现位错误，需按 docs/can_fd_tdc_troubleshooting.md 微调 DBTP/TDCO。先以 brs=true 实测。
- **USART1 日志与协议帧同 TX**：log_task 与协议响应共用 drv_log_uart_send，字节不会交错进单次 DMA，但多帧之间会插入日志行。主机解析必须按帧头 `55 AA 00 14` 重同步；如需纯净通道，可加编译开关在流式期间静音日志。
- **1ms 定时器负载**：1ms 下构建并发送 1 个 FD 帧（CAN1）+ 1 个经典查询（CAN2）+ UART 解析/流帧，开销很小，G474@170MHz 无压力；回调保持精简、ISR 禁日志。
- **上电安全**：MIT 帧默认 kp=kd=tq=0、使能=0、抱闸吸合，电机不会突动；主机须先读反馈确认当前位置再下发目标。
- 旧测试模块（HT/TONGZHI/PA430）代码仍编译，但默认选择已切到 JUXIE+MZ；CAN2 不再跑 PA430 FD 帧，BRS 改动对其无影响。

## 验证

1. `build.bat`（Debug）编译通过，无 warning 新增。
2. 用 CAN 分析仪/上位机观察：CAN1 周期收到 `0x111` FD 帧（数据段 5M），CAN2 周期 `0x510/0x410` 请求应答对（~1kHz）。
3. 主机发 `0x1D1` param1=1 使能 → 发 `0x1D0` 带小 kp/kd 与目标位置 → 电机动作，`0xE00100`/`0xE30000` 反馈位置/力矩随动。
4. 主机发 `0xE2` 读 Mz，数值与 0x410 解析一致；`0xE3` 开流式，验证帧率。
5. 回切 `SRV_MOTOR_TEST_SELECT` 旧值，确认旧测试模块仍可编译运行（回归）。

## 范围外（本期不做）

- 多电机（>1 台）与多控帧 `0x210` 的封装（本期 1 台走单轴帧；预留 `srv_juxie_motor` 数组结构便于扩展）。
- MIT 轨迹规划（梯形/S 形）、位置/力矩闭环逻辑（由主机下发目标，固件只做透明转发）。
- 传感器标定（零点已有标零命令）、CAN 与 UART 的协议文档自动生成。
