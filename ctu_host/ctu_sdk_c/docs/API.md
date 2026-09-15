# ctu_sdk_c 接口文档

E1 CTU 电源板（E1_MASTER_POWER_CTU `0x01` + E1_SLAVER_POWER_CTU `0x02`，同一 RS485 总线）
RS485 C 语言 SDK。C11，仅依赖 Linux/POSIX（termios + poll），**无动态内存、无线程依赖、无第三方库**。

覆盖上位机全部功能：

| 能力 | 命令 | API |
|------|------|-----|
| 读系统状态 | `0x01` | `ctu_client_read_master_status()` / `ctu_client_read_slaver_status()` |
| 读电压 | `0x02` | `ctu_client_read_master_voltage()` / `ctu_client_read_slaver_voltage()` |
| 读温度 | `0x03` | `ctu_client_read_master_temp()` / `ctu_client_read_slaver_temp()` |
| 读固件信息 | `0x07` | `ctu_client_read_info()` |
| 主控蜂鸣器 | `0x04` | `ctu_client_set_buzzer_duty()` |
| 副板输出控制 | `0x04` | `ctu_client_set_outputs()` |
| 清除故障锁存 | `0x05` | `ctu_client_clear_fault_latch()` |
| 升级请求 | `0x06` | `ctu_client_request_upgrade()` |
| **固件升级** | `0x06`+`0x08/09/0A/0B` | `ctu_client_upgrade()` / `ctu_boot_transfer()` |
| 在线探测 | `0x01` | `ctu_client_probe()` |
| 周期轮询 + 丢包统计 | `0x01/02/03` | `ctu_poller_poll_once()` / `ctu_poller_get_stats()` |
| 命令行工具 | 全部 | `ctu_cli` |

---

## 1. 构建与集成

### 1.1 依赖

- Linux（或其它 POSIX 系统）
- CMake ≥ 3.16、支持 C11 的编译器（gcc/clang）
- 测试需要 `pthread`

**无外部依赖**：协议解析所需的 `protocol_parser`（+ `kfifo`）已作为字节级副本随包提供于
`middleware/`，无需仓库其它部分即可构建部署（来源与同步方式见 `middleware/README.md`）。

### 1.2 构建

```bash
cd ctu_sdk_c
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

> 协议解析复用仓库共享中间件 `protocol_parser`（与固件 `service/srv_com_*.c` 同一实现，
> 副本字节一致），因此**帧解析行为与板端完全一致**，不存在两套解析逻辑。

产物：

| 目标 | 说明 |
|------|------|
| `libctu_sdk.a` | 静态库（`ctu::sdk`） |
| `ctu_cli` | 命令行工具 |
| `ctu_example_monitor` / `ctu_example_upgrade` | 示例程序 |

CMake 选项：

| 选项 | 默认 | 说明 |
|------|------|------|
| `CTU_BUILD_CLI` | ON | 构建命令行工具 |
| `CTU_BUILD_EXAMPLES` | ON | 构建示例 |
| `CTU_WARNINGS_AS_ERRORS` | OFF | 把 `-Wall -Wextra -Wpedantic -Wshadow` 告警视为错误 |
| `CTU_SANITIZE` | OFF | 启用 ASan/UBSan |

### 1.3 集成到其它工程

方式 A：`add_subdirectory`

```cmake
add_subdirectory(path/to/ctu_sdk_c)
target_link_libraries(my_app PRIVATE ctu::sdk)
```

方式 B：安装后使用

```bash
cmake --install build --prefix /usr/local
# 头文件 <prefix>/include/ctu/*.h，静态库 <prefix>/lib/libctu_sdk.a
```

### 1.4 编译要求

库内部使用 `clock_gettime`/`nanosleep`/`termios`/`cfmakeraw`/`B460800`/`glob`，
需要 `_GNU_SOURCE`（CMake 已为目标设置）。**公共头文件不需要任何特殊宏**，
只依赖 `stdbool.h` / `stddef.h` / `stdint.h`。

---

## 2. 设计约定

- **复用共享中间件**：协议解析使用仓库共享的 `protocol_parser`（配 `kfifo`），与固件端同一实现。
  为保证 SDK 可**独立部署**，这两个中间件以**字节级副本**随包提供于 `middleware/`
  （来源、同步方式与已知告警见 `middleware/README.md`）；本 SDK 只提供 `ctu_` 风格的适配
  接口与静态存储。
- **组帧不引入 `protocol_packer`**：`protocol_packer` 是"帧头+数据+校验+帧尾"的有状态打包器，
  需要自带输出缓冲与两个回调；而本 SDK 的组帧是**无状态纯函数**（约 15 行 memcpy + CRC，
  返回值可被任意调用点复用），改用打包器会为每种帧引入上下文与额外拷贝而无功能收益。
  帧格式仍只有一处定义（`ctu_protocol_build_frame` 与解析回调共用同一组长度/CRC 规则），
  且已与 Python SDK 逐字节交叉验证。如后续要求与固件打包完全同构，可再切换到 `protocol_packer`。
- **无动态内存**：所有上下文与缓冲区由调用者提供（可静态分配），SDK 内部不 `malloc`。
- **Config-in-context**：每个模块有 `xxx_config_t`（含回调）与 `xxx_context_t`（内嵌 config）。
- **先校验后使用**：所有公共 API 首先检查 `NULL` / 未初始化 / 参数越界。
- **中文 Doxygen 注释**：公共 API 与复杂逻辑均有中文说明。
- **风格**：WebKit（4 空格缩进、函数 Allman、控制语句 K&R），命名遵循
  `ctu_模块_` 前缀 + 蛇形命名，类型以 `_t` 结尾，详见仓库 `MODULE_CODING_GUIDE.md`。
- **线程模型**：SDK 本身不创建线程；`ctu_client_*` 为同步阻塞调用，`ctu_client_request_cancel()`
  可从其它线程或信号处理函数调用以中止升级。

---

## 3. 快速开始

```c
#include <stdio.h>
#include "ctu/ctu.h"

int main(void)
{
    ctu_client_t client;
    ctu_client_config_t config = { 0 };
    ctu_master_status_t status;
    ctu_master_voltage_t voltage;

    config.timeout_ms = 200;
    if (ctu_client_init(&client, &config) != CTU_OK) {
        return 1;
    }
    if (ctu_client_open(&client, "/dev/ttyUSB0", 115200) != CTU_OK) {
        ctu_client_deinit(&client);
        return 1;
    }

    if (ctu_client_read_master_status(&client, &status) == CTU_OK) {
        printf("急停=%d 12V异常=%d\n", status.estop, status.rail_12v_fault);
    }
    if (ctu_client_read_master_voltage(&client, &voltage) == CTU_OK) {
        printf("VIN=%u mV\n", voltage.vin_mv);
    }

    /* 副板：24V + 12V_ISO 开，补光 50% */
    (void)ctu_client_set_outputs(&client, 0x03U, 500U, NULL);

    /* 固件升级 */
    if (ctu_client_upgrade(&client, CTU_DEVICE_MASTER, "fw.bin") != CTU_OK) {
        printf("升级失败\n");
    }

    ctu_client_deinit(&client);
    return 0;
}
```

编译：

```bash
gcc main.c -Ictu_sdk_c/include ctu_sdk_c/build/libctu_sdk.a -o app
```

---

## 4. 错误码 `ctu_error_t`

| 错误码 | 含义 |
|--------|------|
| `CTU_OK` | 成功 |
| `CTU_ERROR_NULL_PTR` | 空指针 |
| `CTU_ERROR_INVALID_PARAM` | 无效参数（含不支持的波特率、非法设备） |
| `CTU_ERROR_UNINITIALIZED` | 对象未初始化 |
| `CTU_ERROR_BUFFER_TOO_SMALL` | 输出缓冲不足 |
| `CTU_ERROR_NOT_OPEN` | 串口未打开 |
| `CTU_ERROR_IO` | 串口读写失败 |
| `CTU_ERROR_PORT` | 串口打开/配置失败 |
| `CTU_ERROR_TIMEOUT` | 等待应答超时 |
| `CTU_ERROR_UNEXPECTED_REPLY` | 应答命令码与请求不符 |
| `CTU_ERROR_DEVICE` | 设备返回 `0x7F` 错误应答（码见 `ctu_client_last_device_error()`） |
| `CTU_ERROR_PROTOCOL` | 帧格式/长度非法 |
| `CTU_ERROR_STATE` | 状态错误（升级流程顺序不对 / Boot err != 0） |
| `CTU_ERROR_IMAGE` | 固件镜像预检失败 |
| `CTU_ERROR_FILE` | 文件读写失败 |
| `CTU_ERROR_UPGRADE` | 固件升级失败 |
| `CTU_ERROR_CANCELED` | 操作被取消 |
| `CTU_ERROR_NOT_SUPPORTED` | 当前平台不支持 |
| `CTU_ERROR_GENERIC` | 未分类错误 |

`const char* ctu_error_str(ctu_error_t error)` 返回中文描述。

设备/ Boot 错误码文案：`ctu_protocol_dev_err_str()`、`ctu_protocol_boot_err_str()`。

---

## 5. 设备与数据模型

### 5.1 设备 `ctu_device_t`

| 枚举 | 值 | 名称 |
|------|----|------|
| `CTU_DEVICE_MASTER` | `0x01` | `ctu_device_name()` → `"master"` |
| `CTU_DEVICE_SLAVER` | `0x02` | `"slaver"` |

`ctu_device_is_valid()` 校验合法性。

### 5.2 `ctu_master_status_t`（0x01）

| 字段 | 类型 | 语义（`true` 表示异常/有效） |
|------|------|------------------------------|
| `estop` | `bool` | 急停触发 |
| `rail_12v_fault` / `rail_24v_fault` | `bool` | 12V / 24V 电源异常 |
| `vin_dcdc_fault` | `bool` | VIN_DC-DC 异常 |
| `aux_fault` / `motor_fault` | `bool` | AUX / MOTOR 异常 |
| `fan0_fault` / `fan1_fault` | `bool` | 风扇异常 |
| `ntc1_disconnected` / `ntc2_disconnected` | `bool` | NTC 断开 |

辅助：`ctu_master_status_has_fault(const ctu_master_status_t*)`。

### 5.3 `ctu_slaver_status_t`（0x01）

| 字段 | 类型 | 语义 |
|------|------|------|
| `out_24v` / `out_12v` / `out_lsd1` / `out_lsd2` | `bool` | `true` = 输出已开启 |
| `fault_24v` / `fault_12v` / `fault_lsd1` / `fault_lsd2` | `bool` | `true` = 故障 |
| `fault_aux` / `fault_motor` | `bool` | `true` = 输入故障 |
| `latch_active` | `bool` | `true` = 故障锁存有效 |

辅助：`ctu_slaver_status_has_fault()`、`ctu_slaver_status_output_mask()`（返回输出位域）。

### 5.4 电压 / 温度 / 固件信息

| 类型 | 字段（单位） |
|------|--------------|
| `ctu_master_voltage_t` | `vin_mv`, `vin_dcdc_mv`（mV） |
| `ctu_slaver_voltage_t` | `aux_mv`, `motor_mv`, `lsd1_mv`, `lsd2_mv`（mV） |
| `ctu_master_temp_t` | `ntc1_c`, `ntc2_c`, `mcu_c`（°C，float） |
| `ctu_slaver_temp_t` | `mcu_c`（°C）, `vdda_mv`（mV） |
| `ctu_fw_info_t` | `app_version`, `meta_version`, `fw_size`, `fw_checksum`, `reboot_counts`, `flags`（标志见 `ctu_fw_flag_t`） |

### 5.5 回调 `ctu_boot_config_t` / `ctu_client_config_t` / `ctu_poller_config_t`

| 回调类型 | 签名 | 说明 |
|----------|------|------|
| `ctu_log_cb_t` | `(const char* level, const char* text, void* user)` | `level ∈ {info, warn, error, tx, rx}` |
| `ctu_progress_cb_t` | `(float fraction, void* user)` | 升级进度 0.0~1.0 |
| `ctu_phase_cb_t` | `(const char* phase, void* user)` | 升级阶段描述 |
| `ctu_poll_sample_cb_t` | `(const ctu_poll_sample_t* sample, void* user)` | 一轮采样结果 |
| `ctu_poll_error_cb_t` | `(ctu_device_t, uint8_t cmd, ctu_error_t, void* user)` | 单条查询失败 |
| `ctu_port_cb_t` | `(const char* name, void* user)` | 串口枚举 |

---

## 6. `ctu_protocol` — 协议编解码（纯计算，可独立单元测试）

| 函数 | 说明 |
|------|------|
| `uint8_t ctu_protocol_crc8(const uint8_t*, size_t)` | CRC8（多项式 `0x31` 反射 `0x8C`，初值 `0xFF`） |
| `uint16_t ctu_protocol_crc16_xmodem(const uint8_t*, size_t)` | CRC16-XMODEM |
| `uint16_t/uint32_t/int16_t ctu_protocol_u16_le/u32_le/i16_le()` | 小端解包 |
| `size_t ctu_protocol_build_frame(cmd, payload, payload_len, out, cap)` | 组包（返回帧长，0=失败） |
| `ctu_protocol_build_read_status/volt/temp/info(addr, out, cap)` | 构建读取帧（0x01/02/03/07） |
| `ctu_protocol_build_master_ctrl(addr, duty, out, cap)` | 蜂鸣器帧（0x04，duty 0-50 截断） |
| `ctu_protocol_build_slaver_ctrl(addr, mask, fill_duty, out, cap)` | 输出控制帧（0x04，duty 0-1000 截断） |
| `ctu_protocol_build_reset_latch(addr, out, cap)` | 清除锁存帧（0x05） |
| `ctu_protocol_build_upgrade(addr, out, cap)` | 升级请求 / SELECT（0x06） |
| `ctu_protocol_build_boot_select/start/data/end/abort(...)` | Boot 帧（0x06/08/09/0A/0B） |
| `ctu_protocol_parser_init/feed/pop(...)` | 流式帧解析（**适配 public_layer `protocol_parser`**；自动重同步、帧头/帧尾/CRC8 校验） |
| `ctu_protocol_frame_addr/cmd/payload/payload_len/is_valid/is_error(...)` | 帧字段访问 |
| `ctu_protocol_frame_to_hex(frame, len, out, cap)` | 帧 → 十六进制文本 |
| `ctu_protocol_cmd_name/dev_err_str/boot_err_str()` | 名称查询 |
| `ctu_protocol_decode_master_status/slaver_status/master_voltage/slaver_voltage/master_temp/slaver_temp/fw_info(...)` | 数据段解码（入参为**去掉设备 ID 后的内容段**） |

解析器用法：

```c
ctu_protocol_parser_t parser;
uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
size_t length = 0;

ctu_protocol_parser_init(&parser);
ctu_protocol_parser_feed(&parser, data, data_len);        /* 可任意分片 */
while (ctu_protocol_parser_pop(&parser, frame, sizeof(frame), &length)) {
    /* 处理一个合法帧 */
}
```

`ctu_protocol_parser_t` 内含中间件上下文与两块静态缓冲（输入 FIFO 512B、组帧输出 260B），
可整体静态分配；`feed` 可分片调用，`pop` 内部循环取帧直到数据不足。

中间件的帧头/帧尾/长度/校验由本 SDK 注册的三个回调描述：

| 回调 | 实现 |
|------|------|
| `header` / `footer` | `{0x7A}` / `{0x0A}`，`header_len=1`、`footer_len=1` |
| `get_len_cb` | `buffer[2] + 5`（`buffer[2]` 为 `data_len` 字段） |
| `check_cb` | CRC8(`buffer[0 .. len-3]`) 与 `buffer[len-2]` 比较 |

> 本 SDK **不调用** `protocol_parser_tick()`，即中间件的"空闲超时丢弃"不生效；
> 接收超时完全由 `ctu_transport_read_frame()` 的 `timeout_ms` 控制。

---

## 7. `ctu_transport` — 串口传输（Linux termios + poll）

```c
typedef struct { uint32_t write_timeout_ms; uint32_t poll_slice_ms; } ctu_transport_config_t;
```

| 函数 | 说明 |
|------|------|
| `ctu_transport_init(transport, config)` | 初始化（config 可为 `NULL`，使用默认值） |
| `ctu_transport_deinit(transport)` | 反初始化（自动关闭串口） |
| `ctu_transport_open(transport, port, baud)` | 打开串口：8N1、原始模式、非阻塞 |
| `ctu_transport_close(transport)` / `ctu_transport_is_open()` | 关闭 / 查询 |
| `ctu_transport_write(transport, data, len, timeout_ms)` | 写（0 使用配置超时），内部 `poll(POLLOUT)` |
| `ctu_transport_read_frame(transport, out, cap, &len, timeout_ms)` | 等待一个合法帧；超时返回 `CTU_ERROR_TIMEOUT` |
| `ctu_transport_flush_input(transport)` | 清空接收缓冲与解析缓存（半双工总线发命令前建议调用） |
| `ctu_transport_baud_is_supported(baud)` | 支持的波特率：9600/19200/38400/57600/115200/230400/460800/921600 |
| `ctu_transport_foreach_port(cb, user)` | 枚举 `/dev/ttyUSB*`、`/dev/ttyACM*`、`/dev/ttyS*` |
| `ctu_transport_port_name(transport)` | 当前设备名 |

---

## 8. `ctu_client` — 高层客户端

### 8.1 配置与生命周期

```c
typedef struct {
    uint32_t timeout_ms;                 /* 单次请求超时，0 → 200ms */
    ctu_transport_config_t transport;    /* 传输层配置 */
    ctu_boot_config_t boot;              /* 升级配置 */
    ctu_log_cb_t log_cb;                 /* 日志回调（可 NULL） */
    void* user;                          /* 回调上下文 */
} ctu_client_config_t;
```

| 函数 | 说明 |
|------|------|
| `ctu_client_init(client, config)` | 初始化（不打开串口） |
| `ctu_client_deinit(client)` | 反初始化并关闭串口 |
| `ctu_client_open(client, port, baud)` / `ctu_client_close()` | 打开 / 关闭串口 |
| `ctu_client_is_initialized()` / `ctu_client_is_open()` | 状态查询 |
| `ctu_client_port_name(client)` | 当前串口名 |
| `ctu_client_request_cancel(client)` | 请求中止升级（可从其它线程/信号处理调用） |
| `ctu_client_clear_cancel(client)` | 清除取消标志 |
| `ctu_client_last_device_error(client)` | 最近一次 `0x7F` 错误码 |

### 8.2 通用事务

```c
ctu_error_t ctu_client_request(ctu_client_t* client, const uint8_t* frame,
                               size_t frame_len, ctu_device_t device,
                               uint8_t expected_cmd, uint8_t* content,
                               size_t content_capacity, size_t* content_len,
                               uint32_t* elapsed_ms);
```

发送并等待指定设备对指定命令的应答；返回的 `content` 为**去掉设备 ID 后的内容段**。
总线上的其它设备帧会被自动忽略。`elapsed_ms` 输出往返耗时。

### 8.3 查询与控制

| 函数 | 说明 |
|------|------|
| `ctu_client_read_master_status(client, &status)` | 0x01 |
| `ctu_client_read_slaver_status(client, &status)` | 0x01 |
| `ctu_client_read_master_voltage(client, &voltage)` | 0x02 |
| `ctu_client_read_slaver_voltage(client, &voltage)` | 0x02 |
| `ctu_client_read_master_temp(client, &temp)` | 0x03 |
| `ctu_client_read_slaver_temp(client, &temp)` | 0x03 |
| `ctu_client_read_info(client, device, &info)` | 0x07 |
| `ctu_client_set_buzzer_duty(client, duty, &ack)` | 0x04（MASTER，0-50） |
| `ctu_client_set_outputs(client, mask, fill_duty, &ack)` | 0x04（SLAVER，mask 位域 / duty 0-1000） |
| `ctu_client_clear_fault_latch(client, device, &ack)` | 0x05 |
| `ctu_client_request_upgrade(client, device, &ack)` | 0x06（ACK 后板端复位进 Bootloader） |
| `ctu_client_probe(client, device, &online)` | 探测在线（总返回 `CTU_OK`，在线与否看 `online`） |

`ack` 可为 `NULL`。输出位域：`bit0=24V`、`bit1=12V_ISO`、`bit2=LSD1`、`bit3=LSD2`。

### 8.4 固件升级

```c
ctu_error_t ctu_client_upgrade(ctu_client_t* client, ctu_device_t device,
                               const char* filepath);
```

流程：

1. 预检固件（栈顶落在 RAM `0x20000000~0x2000C000`、复位向量落在 AppA
   `[0x08008000, 0x08020000)` 且为奇地址、长度 ≤ 96KB）；不通过返回 `CTU_ERROR_IMAGE` 且**不占用总线**。
2. `0x06` 邀请：运行中的 App 复位进 Boot；已在 Boot 则等同 SELECT（幂等）。
3. `SELECT → START → DATA×N → END`，每帧等待应答并按 `max_retries` 重试。
4. 失败或取消时发送 `ABORT`，返回对应错误码。

配置（`ctu_client_config_t.boot`，`ctu_boot_config_t`）：

| 字段 | 默认 | 说明 |
|------|------|------|
| `invite_delay_ms` | 500 | 0x06 邀请后等待进入 Boot |
| `select_timeout_ms` | 2000 | SELECT/START 应答超时 |
| `block_timeout_ms` | 2000 | 单个 DATA 块应答超时 |
| `end_timeout_ms` | 30000 | END 应答超时（板端需校验与提交） |
| `max_retries` | 8 | 单步骤最大重试次数 |
| `progress_cb` / `phase_cb` / `log_cb` / `user` | — | 回调与上下文 |

中止升级：

```c
/* 其它线程或 SIGINT 处理函数中 */
ctu_client_request_cancel(&client);   /* → ctu_client_upgrade 返回 CTU_ERROR_CANCELED */
```

### 8.5 独立使用 Boot 模块

```c
ctu_boot_context_t boot;
ctu_boot_config_t boot_config = { 0 };
ctu_boot_init(&boot, &boot_config);
boot.addr = CTU_MASTER_ADDR;
boot.cancel = &cancel_flag;

ctu_boot_invite(&boot, &transport);                     /* 0x06 邀请 */
ctu_boot_transfer(&boot, &transport, "fw.bin");         /* 完整升级 */
ctu_boot_check_file("fw.bin", &size, &sum, reason, sizeof(reason));
```

---

## 9. `ctu_poller` — 周期轮询与统计

```c
typedef struct {
    ctu_device_t devices[2];       /* 参与轮询的设备 */
    size_t device_count;           /* 0 → 默认 master + slaver */
    ctu_poll_sample_cb_t sample_cb;
    ctu_poll_error_cb_t error_cb;
    void* user;
} ctu_poller_config_t;
```

| 函数 | 说明 |
|------|------|
| `ctu_poller_init(poller, client, config)` | 初始化 |
| `ctu_poller_poll_once(poller)` | 对所有设备执行一轮（状态 → 电压 → 温度） |
| `ctu_poller_poll_device(poller, device)` | 单设备一轮 |
| `ctu_poller_get_stats(poller, &stats)` | 取统计快照（刷新近 1s 接收速率） |
| `ctu_poller_reset_stats(poller)` | 清零统计 |

`ctu_poll_stats_t`：`tx`（发送数）、`rx`（成功应答数）、`loss`（超时/失败数）、
`loss_rate`（%）、`fps`（近 1s 接收速率）。

C 版本不创建线程，调用方按自己的节奏循环：

```c
ctu_poller_t poller;
ctu_poller_config_t pc = { 0 };
ctu_poller_init(&poller, &client, &pc);
for (;;) {
    ctu_poller_poll_once(&poller);
    ctu_poll_stats_t stats;
    ctu_poller_get_stats(&poller, &stats);
    sleep_ms(500);
}
```

`ctu_poll_sample_t` 一次给出该设备本轮的状态/电压/温度及各自的 `*_valid` 标志。

---

## 10. 协议速查

### 10.1 帧格式

```
[0x7A 'z'][cmd 1B][data_len 1B][payload][CRC8 1B][0x0A]
```

- `payload[0]` = 设备 ID（下行=目标地址，上行=源地址）
- CRC8：多项式 `0x31`（反射 `0x8C`），初值 `0xFF`，覆盖 `z` 到 payload 末字节
- 帧总长 = `data_len + 5`；多字节小端
- 应答帧 `cmd = 0x80 | 下行命令码`；错误应答 `cmd = 0x7F`，payload `[addr, err]`

### 10.2 命令与数据段

| cmd | 含义 | 下行 payload（含 ID 后） | 应答内容段 |
|-----|------|--------------------------|------------|
| `0x01` | 读系统状态 | 无 | MASTER 2B / SLAVER 2B 位域 |
| `0x02` | 读电压 | 无 | MASTER 4B / SLAVER 8B（u16 LE ×n） |
| `0x03` | 读温度 | 无 | MASTER 6B / SLAVER 4B |
| `0x04` | 控制 | MASTER `[duty]`；SLAVER `[mask, 0x00, duty u16 LE]` | 回显 |
| `0x05` | 清除故障锁存 | `[0x01]` | 空 |
| `0x06` | 升级请求 / SELECT | `[0x01]` | `[err]` |
| `0x07` | 读固件信息 | 无 | 15B |

位域表：

| | bit0 | bit1 | bit2 | bit3 | bit4 | bit5 |
|---|---|---|---|---|---|---|
| MASTER status byte0 | estop | 12V | 24V | VIN_DC-DC | AUX | MOTOR |
| MASTER status byte1 | fan0 | fan1 | ntc1 | ntc2 | — | — |
| SLAVER status byte0 | err_24v | err_12v | err_aux | err_motor | err_lsd1 | err_lsd2 |
| SLAVER status byte1 | out_24v | out_12v | out_lsd1 | out_lsd2 | latch | — |

温度：`i16`，单位 `0.01 °C`。MASTER 顺序 `ntc1, ntc2, mcu`；SLAVER `mcu(i16), vdda(u16 mV)`。
固件信息 15B：`u16 app_ver, u16 meta_ver, u32 size, u32 checksum, u16 reboot, u8 flags`。

### 10.3 Boot 升级协议

| cmd | 名称 | payload（含 ID 后） |
|-----|------|---------------------|
| `0x06` | SELECT | `[0x01]` |
| `0x08` | START | `[size u32 LE][checksum u32 LE]` |
| `0x09` | DATA | `[blk u16 LE][crc16 u16 LE][chunk ≤248B]` |
| `0x0A` | END | 无 |
| `0x0B` | ABORT | 无 |

- 每帧由设备单独应答 `cmd|0x80`，payload `[addr, err]`；`err != 0` 即失败
- `crc16` = XMODEM（poly `0x1021`，初值 0）覆盖该块数据
- `checksum` = `sum(firmware) & 0xFFFFFFFF`
- Boot 错误码：`0x01` 帧长非法、`0x02` 状态错、`0x03` 块号错、`0x04` CRC16 错、
  `0x05` Flash 写失败、`0x06` 长度/容量错

---

## 11. 命令行工具 `ctu_cli`

```
ctu_cli [-p PORT] [-b BAUD] [-v] <命令> [选项]
```

| 命令 | 说明 |
|------|------|
| `ports` | 列出可用串口 |
| `scan` | 探测两板在线状态 |
| `status --device M` | 读系统状态 |
| `volt --device M` | 读电压 |
| `temp --device M` | 读温度 |
| `info --device M` | 读固件 / Boot 信息 |
| `buzzer --duty N` | 主控蜂鸣器（0-50） |
| `output [--mask 0xNN] [--on a,b] [--off a,b] [--duty N] [--preserve]` | 副板输出控制 |
| `clear-latch --device M` | 清除故障锁存 |
| `request-upgrade --device M` | 升级请求 |
| `upgrade --device M --file FW.bin` | 固件升级（stderr 打印进度/阶段） |
| `monitor [--device M] [--interval S] [--duration S]` | 周期轮询（Ctrl+C 停止） |

设备写法：`master` / `slaver` / `0x01` / `0x02` / `1` / `2` / `主` / `副`。
`-v` 时打印收发帧十六进制。退出码：`0` 成功、`1` 运行错误、`2` 参数错误。

---

## 12. 自检与上线检查

构建建议 **Debug 与 Release 都开一次** `-DCTU_WARNINGS_AS_ERRORS=ON`：部分告警
（如 `-Wformat-truncation`、`warn_unused_result`）只在 `-O2` 下出现。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTU_WARNINGS_AS_ERRORS=ON
cmake --build build -j
```

接上真实硬件后的读写自检（推荐按顺序执行）：

```bash
./build/ctu_cli ports                          # 确认设备名（也可简写 ttyUSB0，自动补 /dev/）
./build/ctu_cli -p /dev/ttyUSB0 scan           # 两板在线探测
./build/ctu_cli -p /dev/ttyUSB0 -v status --device master   # -v 查看收发帧十六进制
./build/ctu_cli -p /dev/ttyUSB0 info   --device master
./build/ctu_cli -p /dev/ttyUSB0 monitor --interval 0.5      # 连续轮询与丢包统计
```

排查提示：

- 打印「无法打开串口 xxx: Permission denied（权限不足…）」→ 将用户加入 `dialout` 组或用 `sudo` 运行；
- 能发帧但一直超时 → 先 `scan` 分辨是单板还是整条总线无应答，再核对设备地址、波特率（默认 115200）与 A/B 接线；
- `-v` 下若连一个字节都收不到，说明对端未驱动总线（未上电 / 收发器 / 接线），属硬件侧问题。

---

## 13. 版本

`ctu_version_string()` → `"1.0.0"`，与上位机 GUI 及 Python SDK 同源（协议行为一致，
帧格式经逐字节交叉验证）。

约定：新增接口向后兼容；破坏性变更提升主版本号。
