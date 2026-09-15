# ctu_sdk — E1 CTU 电源板 RS485 SDK

E1_MASTER_POWER_CTU（`0x01`）与 E1_SLAVER_POWER_CTU（`0x02`）共用一条 RS485 总线时的
主机侧 SDK。**纯 Python，不依赖 Qt**，适用于 Linux（含 WSL）/ Windows 上的脚本、
测试台、产线与无头监测服务。

覆盖上位机 `ctu_host` GUI 的全部功能：状态/电压/温度/固件信息读取、蜂鸣器与输出控制、
清除故障锁存、升级请求、**完整固件升级（Boot 分块传输，支持中止）**、周期轮询与丢包统计。

> 接口细节见 **[docs/SDK.md](docs/SDK.md)**。

## 安装

```bash
pip install pyserial
pip install ./            # 在本目录执行；或直接把本目录加入 PYTHONPATH
```

固件升级需要 Boot 协议层（`boot_protocol.py`）。SDK 会先看环境变量
`CTU_BOOT_PROTOCOL_DIR`，其次找仓库内的 `<repo>/E1_CTU_BOOT/host`：

```bash
export CTU_BOOT_PROTOCOL_DIR=/opt/ctu/boot_host     # 独立部署时指定
```

## 快速开始

```python
from ctu_sdk import CtuClient, MASTER, SLAVER

with CtuClient("/dev/ttyUSB0", baud=115200) as ctu:
    print(ctu.scan())                                   # {'master': True, 'slaver': True}
    if ctu.read_status(MASTER).healthy:
        print("MASTER 正常")
    print(ctu.read_voltage(SLAVER).to_dict())           # {'aux_mv': 12000, ...}

    ctu.set_outputs(0x03, fill_duty=500)                # 24V + 12V_ISO 开，补光 50%
    ctu.upgrade(SLAVER, "E1_SLAVER_POWER_CTU.bin",
                progress=lambda f: print(f"{f * 100:5.1f}%"))
```

循环监测：

```python
from ctu_sdk import CtuClient, CtuPoller, MASTER, SLAVER

with CtuClient("/dev/ttyUSB0") as ctu, CtuPoller(
        ctu, [MASTER, SLAVER], interval=0.5,
        on_status=lambda dev, s: print(dev, s.faults or "OK"),
        on_stats=lambda s: print(s.to_dict())) as poller:
    input("回车停止\n")
```

## 命令行

```bash
python -m ctu_sdk --help
python -m ctu_sdk ports
python -m ctu_sdk -p /dev/ttyUSB0 scan
python -m ctu_sdk -p /dev/ttyUSB0 status --device master
python -m ctu_sdk -p /dev/ttyUSB0 output --on 24v,12v --duty 500 --preserve
python -m ctu_sdk -p /dev/ttyUSB0 upgrade --device slaver --file fw.bin
python -m ctu_sdk -p /dev/ttyUSB0 monitor --interval 0.5
```

## 示例

- [examples/monitor.py](examples/monitor.py) — 周期轮询 + 异常打印
- [examples/upgrade_firmware.py](examples/upgrade_firmware.py) — 带进度与中止的固件升级

## 测试（无需真实硬件）

用 `pty` 虚拟串口 + 虚拟电源板（同时实现应用协议与 Boot 协议）做端到端验证：

```bash
cd ..                                  # ctu_host/
python3 -m unittest discover -s ctu_sdk/tests -t . -v
```

覆盖：读取、控制、清锁存、升级请求、超时/错误应答、在线探测、轮询统计与丢包、
完整固件升级（含中止）、CLI 全部子命令。

## 目录

```
ctu_sdk/
  protocol.py      z 帧编解码、数据段解码、下行帧构建（唯一协议来源）
  firmware.py      Boot 协议层适配 + 固件预检
  devices.py       设备标识（master/slaver ↔ 地址 / Boot ID）
  errors.py        异常体系
  models.py        返回数据模型（不可变 dataclass）
  transport.py     pyserial 收发 + 帧解析（线程安全）
  client.py        CtuClient：全部业务接口
  poller.py        CtuPoller：周期轮询与统计
  cli.py           命令行工具
  docs/SDK.md      接口文档
  examples/        示例
  tests/           虚拟板 + 端到端测试
```
