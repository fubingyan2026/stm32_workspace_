# ctu_sdk 接口文档

E1 CTU 电源板（E1_MASTER_POWER_CTU `0x01` + E1_SLAVER_POWER_CTU `0x02`，同一 RS485 总线）
RS485 上位机 SDK。纯 Python，**不依赖 Qt**，可在 Linux / Windows / WSL 上运行。

SDK 覆盖上位机（`ctu_host` GUI）的全部功能：

| 能力 | 对应命令 | SDK 入口 |
|------|----------|----------|
| 读系统状态 | `0x01` | `CtuClient.read_status()` |
| 读电压 | `0x02` | `CtuClient.read_voltage()` |
| 读温度 | `0x03` | `CtuClient.read_temperature()` |
| 读固件信息 | `0x07` | `CtuClient.read_info()` |
| 主控蜂鸣器控制 | `0x04` | `CtuClient.set_buzzer_duty()` |
| 副板输出控制 | `0x04` | `CtuClient.set_outputs()` / `set_output()` |
| 清除故障锁存 | `0x05` | `CtuClient.clear_fault_latch()` |
| 升级请求 | `0x06` | `CtuClient.request_upgrade()` |
| **固件升级** | `0x06`+`0x08/09/0A/0B` | `CtuClient.upgrade()` / `upgrade_image()` |
| 周期轮询 + 丢包统计 | `0x01/02/03` | `CtuPoller` |
| 在线探测 | `0x01` | `CtuClient.probe()` / `scan()` |
| 命令行工具 | 全部 | `python -m ctu_sdk` |

---

## 1. 安装与部署

### 1.1 依赖

- Python ≥ 3.9
- `pyserial` ≥ 3.5

### 1.2 在仓库内直接使用（推荐先这样验证）

```bash
cd ctu_host
python3 -m ctu_sdk ports                 # 若能列出串口即表示可用
```

### 1.3 安装为包

```bash
cd ctu_host
pip install ./ctu_sdk                    # 或 pip install -e ./ctu_sdk
```

### 1.4 Boot 升级协议层的定位

固件升级复用 `E1_CTU_BOOT/host/boot_protocol.py`（单一来源）。SDK 按以下顺序查找：

1. 环境变量 `CTU_BOOT_PROTOCOL_DIR` 指向的目录；
2. 仓库内相对路径 `<repo>/E1_CTU_BOOT/host`。

独立部署到 Linux 时，二选一：

```bash
# 方式 A：把 boot_protocol.py 放到任意目录并指定
export CTU_BOOT_PROTOCOL_DIR=/opt/ctu/boot_host
# 方式 B：保持仓库结构（ctu_host/ 与 E1_CTU_BOOT/ 同级）
```

可用 `ctu_sdk.boot_host_dir()` 查看实际生效的目录（未找到返回 `None`）。

---

## 2. 快速开始

```python
from ctu_sdk import CtuClient, MASTER, SLAVER

with CtuClient("/dev/ttyUSB0", baud=115200) as ctu:
    # 在线探测
    online = ctu.scan()                     # {'master': True, 'slaver': False}

    # 查询
    status = ctu.read_status(MASTER)        # MasterStatus
    print(status.healthy, status.faults)
    print(ctu.read_voltage(MASTER).vin_mv, "mV")
    print(ctu.read_temperature(MASTER).mcu_c, "°C")
    print(ctu.read_info(SLAVER).app_version)

    # 控制
    ctu.set_buzzer_duty(10)                 # MASTER 蜂鸣器 10%
    ctu.set_outputs(0x03, fill_duty=500)    # SLAVER: 24V + 12V_ISO 开，补光 50%
    ctu.clear_fault_latch(SLAVER)

    # 固件升级（0x06 邀请 + SELECT/START/DATA/END）
    ctu.upgrade(SLAVER, "E1_SLAVER_POWER_CTU.bin",
                progress=lambda f: print(f"{f * 100:5.1f}%"))
```

---

## 3. 设备标识

```python
from ctu_sdk import MASTER, SLAVER, normalize_device, device_addr, boot_device_id
```

| 名称 | 应用协议地址 | Boot 设备 ID |
|------|--------------|--------------|
| `MASTER`（`"master"`） | `0x01` | `"master"` |
| `SLAVER`（`"slaver"`） | `0x02` | `"slaver"` |

所有接受 `device` 的接口都经过 `normalize_device()`，以下写法等价：
`"master"` / `"MASTER"` / `1` / `"0x01"` / `"主"`；`"slaver"` / `2` / `"0x02"` / `"副"`。

`ALL_DEVICES = ("master", "slaver")`。

---

## 4. 异常体系

所有可预期失败都派生自 `CtuError`：

| 异常 | 触发条件 | 关键属性 |
|------|----------|----------|
| `CtuNotConnected` | 串口未打开就发起请求 | — |
| `CtuTimeout` | 超时未收到符合预期的应答 | — |
| `CtuFramingError` | 数据无法构成合法帧（帧头/帧尾/CRC 失败） | — |
| `CtuDeviceError` | 设备返回 `0x7F` 错误应答 | `.device` `.code` `.text` |
| `CtuUnexpectedReply` | 应答命令码与请求不匹配 | `.expected_cmd` `.actual_cmd` |
| `FirmwareImageError` | 固件未通过预检（非 AppA 镜像 / 超容量 / 读取失败） | — |
| `UpgradeError` | 升级过程失败（SELECT/START/DATA/END 未确认）或被中止 | — |

设备错误码（`CtuDeviceError.code`，见 `ctu_sdk.protocol.ERR_TEXT`）：

| code | 含义 |
|------|------|
| `0x00` | 无错误 |
| `0x01` | 未知命令 |
| `0x02` | 功能暂不支持 |
| `0x03` | 帧长度与命令不匹配 |

---

## 5. `CtuClient`

### 5.1 构造与生命周期

```python
CtuClient(port: str | None = None, baud: int = 115200,
          timeout: float = 0.2, logger: Callable[[str, str], None] | None = None)
```

| 参数 | 说明 |
|------|------|
| `port` | 串口设备，如 `/dev/ttyUSB0`、`COM5`。为 `None` 时不立即打开 |
| `baud` | 波特率，默认 `115200`（板端默认 115200-8N1） |
| `timeout` | 单次请求超时秒数，默认 `0.2`；GUI 的严格判定为 10 ms，脚本场景建议 ≥ 50 ms |
| `logger` | 可选回调 `(text, level)`，`level ∈ {info, warn, error, tx, rx}`，用于观察收发帧 |

| 成员 | 说明 |
|------|------|
| `open(port=None, baud=None)` | 打开串口（可改端口/波特率，改波特率会重开） |
| `close()` | 关闭串口（可重复调用） |
| `with CtuClient(...) as ctu:` | 上下文管理，退出时自动 `close()` |
| `is_open` / `port` / `baud` | 当前状态 |
| `transport` | 底层 `SerialTransport`（高级用法，如自行收发） |

### 5.2 查询方法

| 方法 | 返回类型 | 说明 |
|------|----------|------|
| `read_status(device, timeout=None)` | `MasterStatus` / `SlaverStatus` | `0x01` 系统状态 |
| `read_voltage(device, timeout=None)` | `MasterVoltage` / `SlaverVoltage` | `0x02` 电压（mV） |
| `read_temperature(device, timeout=None)` | `MasterTemperature` / `SlaverTemperature` | `0x03` 温度（°C） |
| `read_info(device, timeout=None)` | `FirmwareInfo` | `0x07` 固件 / Boot 信息 |

设备决定返回的具体类型；两板数据段不同，详见第 6 节。

### 5.3 控制方法

| 方法 | 说明 |
|------|------|
| `set_buzzer_duty(duty, timeout=None) -> Ack` | MASTER 蜂鸣器占空比 `0-50`（%），超出自动截断 |
| `set_outputs(output_mask, fill_duty=0, timeout=None) -> Ack` | SLAVER 输出位域 + 补光亮度 `0-1000` |
| `set_output(channel, on, *, mask=None, timeout=None) -> Ack` | 按通道开关：`channel ∈ {24v, 12v, lsd1, lsd2}`；`mask` 为其它通道当前位域 |
| `clear_fault_latch(device, timeout=None) -> Ack` | `0x05` 清除故障锁存 |
| `request_upgrade(device, timeout=None) -> Ack` | `0x06` 升级请求（板端 ACK 后复位进 Bootloader） |

输出位域：`bit0=24V`、`bit1=12V_ISO`、`bit2=LSD1`、`bit3=LSD2`。

### 5.4 固件升级

```python
upgrade(device, filepath, *, progress=None, phase=None, log=None,
        cancel=None, timeout=None) -> None
upgrade_image(device, data: bytes, name="image.bin", *, ...) -> None
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `device` | 设备标识 | 目标板 |
| `filepath` | `str` | 固件 `.bin` 路径 |
| `data` | `bytes` | （`upgrade_image`）内存中的固件内容 |
| `progress` | `Callable[[float], None]` | 进度回调，参数 `0.0 ~ 1.0` |
| `phase` | `Callable[[str], None]` | 阶段描述（如“选中设备并进入升级会话”“传输固件数据”） |
| `log` | `Callable[[str, str], None]` | 详细日志 `(文本, 级别)` |
| `cancel` | `threading.Event` | 置位后中止：SDK 会调用底层 `cancel()` 并发送 `ABORT` |

行为：

1. 预检固件（向量表 sanity：栈顶落在 RAM、复位向量落在 AppA `[0x08008000, 0x08020000)`，容量 ≤ 96 KB）；
   不通过直接抛 `FirmwareImageError`，**不碰总线**。
2. 发送 `0x06` 邀请：运行中的 App 复位进 Boot；已在 Boot 则等同 SELECT（幂等）。
3. `BootProtoSender.transfer()`：SELECT → START → DATA×N → END，逐块应答，失败重试。
4. 失败或中止抛 `UpgradeError`。

> 升级期间 SDK 独占串口锁，其它请求会被阻塞至升级结束；不会与轮询交错。

### 5.5 便利方法

| 方法 | 说明 |
|------|------|
| `probe(device, timeout=None) -> bool` | 读一次状态判断在线 |
| `scan(devices=ALL_DEVICES, timeout=None) -> dict[str, bool]` | 批量在线探测 |
| `send(frame)` / `read_frame(timeout=None)` | 低层收发（自行组帧时使用） |
| `request(frame, device, command, timeout=None) -> bytes` | 发送并等待指定应答，返回内容段（已去掉设备 ID 字节） |

---

## 6. 数据模型

所有模型均为不可变 `dataclass`，具备 `to_dict()`，可直接 JSON 序列化。

### 6.1 `MasterStatus`（`0x01`，MASTER）

| 字段 | 类型 | 语义 |
|------|------|------|
| `estop` | `bool` | `True` = 急停触发 |
| `rail_12v_fault` / `rail_24v_fault` | `bool` | `True` = 该电源轨异常 |
| `vin_dcdc_fault` | `bool` | `True` = VIN_DC-DC 异常 |
| `aux_fault` / `motor_fault` | `bool` | `True` = 异常 |
| `fan0_fault` / `fan1_fault` | `bool` | `True` = 异常 |
| `ntc1_disconnected` / `ntc2_disconnected` | `bool` | `True` = 断开 |
| `raw` | `dict[str, bool]` | 原始位域键值 |
| `faults`（属性） | `list[str]` | 当前异常项名称列表 |
| `healthy`（属性） | `bool` | 无任何异常 |

> 位域语义：这些字段 `True` 表示**异常/有效**，不是“正常”。

### 6.2 `SlaverStatus`（`0x01`，SLAVER）

| 字段 | 类型 | 语义 |
|------|------|------|
| `out_24v` / `out_12v` / `out_lsd1` / `out_lsd2` | `bool` | `True` = 输出已开启 |
| `fault_24v` / `fault_12v` | `bool` | `True` = 故障 |
| `fault_lsd1` / `fault_lsd2` | `bool` | `True` = 故障 |
| `fault_aux` / `fault_motor` | `bool` | `True` = 输入故障 |
| `latch_active` | `bool` | `True` = 故障锁存有效 |
| `output_mask`（属性） | `int` | 由 `out_*` 组合的输出位域 |
| `faults` / `healthy`（属性） | `list[str]` / `bool` | 异常项 / 是否健康 |

### 6.3 电压与温度

| 模型 | 字段（单位） |
|------|--------------|
| `MasterVoltage` | `vin_mv`, `vin_dcdc_mv`（mV） |
| `SlaverVoltage` | `aux_mv`, `motor_mv`, `lsd1_mv`, `lsd2_mv`（mV） |
| `MasterTemperature` | `ntc1_c`, `ntc2_c`, `mcu_c`（°C） |
| `SlaverTemperature` | `mcu_c`（°C）, `vdda_mv`（mV） |

### 6.4 `FirmwareInfo`（`0x07`）

| 字段 | 类型 | 说明 |
|------|------|------|
| `app_version` / `meta_version` | `int` | App / metadata（Boot）版本号 |
| `fw_size` | `int` | 固件字节数 |
| `fw_checksum` | `int` | 累加和（`sum(data) & 0xFFFFFFFF`） |
| `reboot_counts` | `int` | 上电次数 |
| `flags` | `int` | 标志位 |
| `meta_valid` / `upgrade_pending` / `upgrade_done`（属性） | `bool` | `flags` 位 `0x01/0x02/0x04` |
| `flags_text`（属性） | `str` | 人类可读标志描述 |

### 6.5 `Ack` / `PollStats`

| 模型 | 字段 |
|------|------|
| `Ack` | `device`, `command`, `elapsed_ms` |
| `PollStats` | `tx`, `rx`, `loss`, `loss_rate`（%）, `fps` |

---

## 7. `CtuPoller`（周期轮询）

```python
CtuPoller(client, devices=ALL_DEVICES, interval=0.1, *,
          on_status=None, on_voltage=None, on_temperature=None,
          on_error=None, on_stats=None)
```

| 回调 | 签名 | 触发时机 |
|------|------|----------|
| `on_status` | `(device, MasterStatus/SlaverStatus)` | 状态应答成功 |
| `on_voltage` | `(device, MasterVoltage/SlaverVoltage)` | 电压应答成功 |
| `on_temperature` | `(device, MasterTemperature/SlaverTemperature)` | 温度应答成功 |
| `on_error` | `(device, command, Exception)` | 某次查询失败（不中断轮询） |
| `on_stats` | `(PollStats)` | 约每 200 ms 推送一次统计 |

| 成员 | 说明 |
|------|------|
| `start()` / `stop()` / `join(timeout)` | 后台线程控制（`daemon=True`） |
| `with CtuPoller(...) as p:` | 进入即 `start()`，退出即 `stop()` |
| `poll_once()` | 立即执行一轮（主线程手动轮询时用） |
| `running` | 是否在运行 |
| `stats` | `PollStats` 快照 |
| `reset_stats()` | 清零统计 |

一轮 = 对每个设备依次查询 状态 → 电压 → 温度。超时或异常计入 `loss`。

```python
poller = CtuPoller(ctu, [MASTER], interval=0.5,
                   on_status=lambda d, s: print(d, s.faults),
                   on_stats=lambda s: print(s.tx, s.loss_rate))
poller.start()
...
poller.stop(); poller.join(2.0)
```

---

## 8. `SerialTransport`（低层）

| 成员 | 说明 |
|------|------|
| `SerialTransport(port, baud=115200, write_timeout=0.5, logger=None)` | 构造（不打开） |
| `open()` / `close()` / `with` | 生命周期 |
| `write(frame)` | 发送完整帧 |
| `read_frame(timeout) -> bytes \| None` | 等待一个合法帧 |
| `drain(timeout=0.0) -> list[bytes]` | 取出所有已到达帧 |
| `flush_input()` / `reset_parser()` | 清理接收缓冲（半双工总线建议发命令前 flush） |
| `raw` | 底层 `serial.Serial`（供 Boot 升级复用） |
| `lock` | 可重入锁，保证一次事务不被交错 |

---

## 9. 协议层与固件工具

```python
from ctu_sdk.protocol import *      # 帧编解码 / 数据段解码 / 下行帧构建
from ctu_sdk.firmware import *      # 固件预检 / Boot 协议导出
```

| 分类 | 成员 |
|------|------|
| 帧 | `FrameParser`, `build_frame(cmd, payload)`, `crc8(data)` |
| 解析 | `parse_addr`, `parse_cmd`, `parse_payload`, `frame_hex`, `describe_frame` |
| 解包 | `unpack_u16_le`, `unpack_i16_le`, `unpack_u32_le` |
| 构建下行帧 | `build_read_status/volt/temp/info`, `build_mst_ctrl`, `build_slv_ctrl`, `build_reset_latch`, `build_upgrade` |
| 解码 | `decode_mst_status/volt/temp`, `decode_slv_status/volt/temp`, `decode_info` |
| 常量 | `CMD_*`, `CMD_REPLY_FLAG`(0x80), `CMD_ERR`(0x7F), `DEV_ADDR_*`, `ERR_TEXT`, `INFO_FLAG_*` |
| 固件 | `inspect_firmware(path) -> (ok, size, reason)`, `MAX_FW_SIZE`(96 KB), `check_app_image`, `BootProtoSender`, `request_boot` |
| Boot | `CMD_SELECT/START/DATA/END/ABORT`, `DATA_MAX`(248), `crc16_xmodem`, `BOOT_ERR_NONE`, `BOOT_ERR_TEXT` |

---

## 10. 协议速查

### 10.1 帧格式

```
[0x7A 'z'][cmd 1B][data_len 1B][payload][CRC8 1B][0x0A]
```

- `payload[0]` = 设备 ID（下行=目标地址，上行=源地址）
- `CRC8`：多项式 `0x31`（反射 `0x8C`），初值 `0xFF`，覆盖 `z` 到 payload 末字节
- 帧总长 = `data_len + 5`；多字节小端
- 应答帧 `cmd = 0x80 | 下行命令码`；错误应答 `cmd = 0x7F`

### 10.2 命令与数据段

| cmd | 含义 | payload（含设备 ID 后） | 应答内容段 |
|-----|------|------------------------|------------|
| `0x01` | 读系统状态 | 无 | MASTER 2B / SLAVER 2B 位域 |
| `0x02` | 读电压 | 无 | MASTER 4B / SLAVER 8B（u16 LE ×n） |
| `0x03` | 读温度 | 无 | MASTER 6B / SLAVER 4B |
| `0x04` | 控制 | MASTER: `[占空比]`；SLAVER: `[位域, 0x00, duty u16 LE]` | 回显 |
| `0x05` | 清除故障锁存 | `[0x01]` | `[err]` |
| `0x06` | 升级请求 / SELECT | `[0x01]` | `[err]` |
| `0x07` | 读固件信息 | 无 | 15B（见 6.4） |

位域表：

| | bit0 | bit1 | bit2 | bit3 | bit4 | bit5 |
|---|---|---|---|---|---|---|
| MASTER status byte0 | estop | 12V | 24V | VIN_DC-DC | AUX | MOTOR |
| MASTER status byte1 | fan0 | fan1 | ntc1 | ntc2 | — | — |
| SLAVER status byte0 | err_24v | err_12v | err_aux | err_motor | err_lsd1 | err_lsd2 |
| SLAVER status byte1 | out_24v | out_12v | out_lsd1 | out_lsd2 | latch | — |

温度：`i16`，单位 `0.01 °C`。MASTER 顺序 `ntc1, ntc2, mcu`；SLAVER `mcu(i16), vdda(u16 mV)`。

### 10.3 Boot 升级协议

| cmd | 名称 | payload（含设备 ID 后） |
|-----|------|------------------------|
| `0x06` | SELECT | `[0x01]`（magic） |
| `0x08` | START | `[size u32 LE][checksum u32 LE]` |
| `0x09` | DATA | `[blk u16 LE][crc16 u16 LE][chunk ≤248B]` |
| `0x0A` | END | 无 |
| `0x0B` | ABORT | 无 |

- 每帧由设备单独应答 `cmd|0x80`，payload `[addr][err]`；非 0 即失败
- `crc16` = XMODEM（poly `0x1021`，初值 0）覆盖该块数据
- `checksum` = `sum(firmware) & 0xFFFFFFFF`
- Boot 错误码：`0x01` 帧长非法、`0x02` 状态错、`0x03` 块号错、`0x04` 数据 CRC16 错、`0x05` Flash 写失败、`0x06` 长度/容量错

---

## 11. 命令行（CLI）

```
python -m ctu_sdk [-p PORT] [-b BAUD] [--timeout S] [--json] [-v] <command>
```

公共参数可写在子命令前或后（`-p`、`-b`、`--timeout`、`--json`、`-v`）。

| 子命令 | 说明 |
|--------|------|
| `ports` | 列出可用串口 |
| `scan` | 探测两板在线状态 |
| `status --device M` | 读系统状态 |
| `volt --device M` | 读电压 |
| `temp --device M` | 读温度 |
| `info --device M` | 读固件 / Boot 信息 |
| `buzzer --duty N` | 主控蜂鸣器（0-50） |
| `output [--mask 0x03] [--on 24v,12v] [--off lsd1] [--duty N] [--preserve]` | 副板输出控制 |
| `clear-latch --device M` | 清除故障锁存 |
| `request-upgrade --device M` | 升级请求 |
| `upgrade --device M --file PATH` | 固件升级 |
| `monitor [--device M]... [--interval S] [--duration S]` | 周期轮询监测 |

`--json` 输出机器可读结果；`-v` 把收发帧打到 stderr。

```bash
python -m ctu_sdk ports
python -m ctu_sdk -p /dev/ttyUSB0 scan --json
python -m ctu_sdk -p /dev/ttyUSB0 status --device master
python -m ctu_sdk -p /dev/ttyUSB0 output --on 24v,12v --duty 500 --preserve
python -m ctu_sdk -p /dev/ttyUSB0 upgrade --device slaver --file fw.bin
python -m ctu_sdk -p /dev/ttyUSB0 monitor --interval 0.5
```

CLI 退出码：`0` 成功，`1` `CtuError`（时间戳/设备错误/升级失败），`130` Ctrl+C。

---

## 12. 线程模型与超时

- 单条请求/应答事务在 `transport.lock` 保护下完成，**不会被其它线程交错**。
- `CtuPoller` 使用独立 daemon 线程，但每次查询仍经过同一把锁，因此可与业务线程共存。
- 升级全程持锁，期间其它请求阻塞；升级由内部看门线程响应 `cancel` 事件。
- `timeout` 是**每次请求**的超时（默认 0.2 s）。Linux 非实时内核下不建议低于 20 ms；
  上位机 GUI 的 10 ms 严格判定不适用于通用脚本场景。

---

## 13. 验证与测试

SDK 自带**无需真实硬件**的端到端测试：用 `pty` 建立虚拟串口，用
`ctu_sdk.tests.fake_board.FakeCtuBoard` 模拟电源板（同时实现应用协议与 Boot 升级协议）。

在 WSL / Linux 上运行：

```bash
cd ctu_host
python3 -m unittest discover -s ctu_sdk/tests -t . -v
```

覆盖范围：状态/电压/温度/固件信息解码、蜂鸣器与输出控制、清锁存、升级请求、
超时与错误应答、在线探测、轮询统计与丢包、**完整固件升级（含中止）**、CLI 各子命令。

---

## 14. 版本

`ctu_sdk.__version__ = "1.0.0"`，对应上位机协议实现（`ctu_host` GUI）同源。

约定：新增接口向后兼容；破坏性变更提升主版本号。
