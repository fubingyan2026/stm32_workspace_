# ctu_host — E1 CTU 电源板 RS485 双板调试上位机

E1_MASTER_POWER_CTU（主控板，addr 0x01）与 E1_SLAVER_POWER_CTU（副电源模块，addr 0x02）
共用一条 RS485 总线时的调试工具。基于 **PyQt6 + pyserial**，多页切换、界面自适应缩放；
同一仓库内的 `ctu_sdk` 提供等价的 **Linux/跨平台纯 Python SDK**（无 GUI，含固件升级）。

## 运行

```bash
pip install -r requirements.txt
python ctu_host.py
```

串口为 USB-RS485（115200-8N1 默认，可切 460800/921600）。

## 打包为 exe

双击 `build_exe.bat`，或手动执行：

```bash
pip install pyinstaller
python -m PyInstaller --noconfirm --clean ctu_host.spec
```

产物为单文件 `dist\ctu_host.exe`（约 36 MB，GUI 无控制台窗口），已内置 PyQt6、pyserial
与 Boot 协议层（`boot_protocol`），目标机无需安装 Python。构建中间文件在 `build\`。

## 界面结构

顶栏常驻连接与轮询控制，左侧导航切换五个页面，底部状态栏显示实时通信统计。

| 页面 | 内容 |
|------|------|
| 系统概览 | 两板关键健康指标一屏速览 / 在线状态 |
| E1_MASTER 主控板 | 查询、蜂鸣器控制、实时数据、固件信息 |
| E1_SLAVER 副电源板 | 查询、输出控制、实时数据、固件信息 |
| 固件升级 | 目标选择、固件预检、分块传输进度（App 内直接下载，Boot 提交） |
| 通信日志 | 全部 TX / RX / 告警 |

说明：

- 「自动轮询」按间隔依次查询**勾选了“参与自动轮询”**的板（默认两板），轮询期间不记录原始
  收发帧以防刷屏，错误应答始终记录。
- 手动查询/控制显示 TX 帧，应答显示 `RX {板名}: hex [命令]`，0x7F 为错误应答。
- 「固件升级」开始时会**自动暂停轮询并独占串口**，完成后自动重连并恢复原轮询状态。
  升级协议直接复用 `E1_CTU_BOOT/host/boot_protocol.py`（单一来源）。
  运行中的 App 收到 `0x06` 后进入 **App 内升级会话**：不跳转、电源保持输出，
  直接在 App 内把新固件写入 B 暂存槽；END 通过后写 `flag=2` 并**继续运行当前固件
  （不复位、不断电）**，**重新上电**时由 Boot 提升 B→A 生效。
  无有效 App 时由 Boot 兜底完成整个下载并立即提交复位。
- 高分辨率屏幕：启动时使用 `PassThrough` 缩放策略并按点设置字体，页面均置于滚动区，
  在 125% / 150% 等非整数缩放下不会裁切。
- 固件文件下拉框**按目标设备分别保存最近使用的 10 个固件路径**（QSettings 持久化，
  重启/重装 exe 后仍在）；切换目标设备会自动切到该设备自己的历史与最近一次固件，
  master / slaver 互不影响；「清除历史」只清当前设备的旧记录、保留其当前已选固件。

## 代码结构（分层，仅向下依赖）

协议实现集中在 `ctu_sdk`，GUI（`app`）通过 `app/protocol.py`、`app/firmware.py`
两个再导出模块复用同一份代码，避免重复实现。

```
ctu_host.py            启动入口（High-DPI 策略 + QApplication）
ctu_sdk/               Linux/跨平台 SDK（纯 Python，无 Qt 依赖）
  protocol.py          z 帧编解码、命令/错误码、各板数据段解码（协议唯一来源）
  firmware.py          Boot 协议层适配 + 固件预检（复用 E1_CTU_BOOT）
  devices.py           设备标识（master/slaver ↔ 地址 / Boot ID）
  errors.py            异常体系
  models.py            返回数据模型（不可变 dataclass）
  transport.py         pyserial 收发 + 帧解析（线程安全）
  client.py            CtuClient：查询 / 控制 / 清锁存 / 升级
  poller.py            CtuPoller：周期轮询与丢包统计
  cli.py               python -m ctu_sdk 命令行
  docs/SDK.md          接口文档
  tests/               虚拟板 + 端到端测试（pty，无需硬件）
ctu_sdk_c/             Linux C11 SDK（静态库 + CLI，无第三方依赖）
  include/ctu/         公共头文件（ctu.h 为总入口）
  src/                 protocol / transport / boot / client / poller
  middleware/          随包发布的共享中间件副本（protocol_parser + kfifo）
  tools/ctu_cli.c      命令行工具
  examples/            monitor / upgrade 示例
  docs/API.md          接口文档
app/
  protocol.py          再导出 ctu_sdk.protocol（保持原导入路径）
  firmware.py          再导出 ctu_sdk.firmware
  transport.py         SerialWorker / UpgradeWorker（Qt 工作线程）
  session.py           连接生命周期、等间隔轮询、10ms 丢包统计（无界面依赖）
  settings.py          QSettings 持久化（按设备分别保存固件路径历史）
  theme.py             马卡龙调色板与全局 QSS
  ui/
    main_window.py     顶栏 + 侧栏导航 + 多页堆叠（唯一编排者）
    widgets.py         卡片 / 键值网格 / 状态胶囊 / 连接栏 / 日志视图
    board_page.py      板卡页基类（查询、固件信息、应答路由）
    master_page.py     E1_MASTER 页
    slaver_page.py     E1_SLAVER 页
    overview_page.py   系统概览页
    upgrade_page.py    固件升级页
    log_page.py        通信日志页
```

## Linux SDK

无 GUI 场景（Linux 脚本 / 测试台 / 无头监测）使用 `ctu_sdk`，功能与上位机完全一致
（含固件升级与中止）：

```python
from ctu_sdk import CtuClient, MASTER, SLAVER

with CtuClient("/dev/ttyUSB0", baud=115200) as ctu:
    print(ctu.scan())
    ctu.set_outputs(0x03, fill_duty=500)
    ctu.upgrade(SLAVER, "E1_SLAVER_POWER_CTU.bin", progress=print)
```

```bash
python -m ctu_sdk ports
python -m ctu_sdk -p /dev/ttyUSB0 monitor --interval 0.5
```

接口文档见 `ctu_sdk/docs/SDK.md`。无需硬件的端到端验证：

```bash
python3 -m unittest discover -s ctu_sdk/tests -t . -v
```

### C SDK（ctu_sdk_c）

面向 Linux 无 GUI 场景另提供 **C11 SDK**（`ctu_sdk_c/`），功能与上位机一致，
无动态内存、无第三方依赖（termios + poll），**协议解析复用共享中间件 `protocol_parser`
（与固件端同一实现，副本随包提供，可独立部署）**，含命令行工具与示例：

```bash
cd ctu_sdk_c
cmake -S . -B build -DCTU_WARNINGS_AS_ERRORS=ON && cmake --build build -j
./build/ctu_cli -p /dev/ttyUSB0 scan
./build/ctu_cli -p /dev/ttyUSB0 -v status --device master
./build/ctu_cli -p /dev/ttyUSB0 upgrade --device slaver --file fw.bin
```

接口文档见 `ctu_sdk_c/docs/API.md`。


## 统一命令表

| cmd | 含义 | MASTER 数据段 | SLAVER 数据段 |
|-----|------|--------------|--------------|
| 0x01 | 读系统状态 | 2B 急停/电源轨 + 风扇/NTC 位域 | 2B 故障位 + 输出/锁存位 |
| 0x02 | 读电压 | 4B VIN / VIN_DC-DC | 8B AUX / MOTOR / LSD1 / LSD2 |
| 0x03 | 读温度 | 6B NTC1/NTC2/MCU | 4B MCU 温度 + VDDA |
| 0x04 | 控制 | 1B 蜂鸣器占空比 0-50% | 4B 输出掩码 + 补光亮度 0-1000 |
| 0x05 | 清除故障锁存 | 1B magic=0x01 | 1B magic=0x01 |
| 0x06 | 升级请求 | 进入 App 内升级会话（不跳转） | 同左 |
| 0x07 | 读固件信息 | 15B App/Boot 版本 + 大小 + 校验和 + 重启次数 + 标志 | 同左 |

协议细节见 `E1_MASTER_POWER_CTU/docs/protocol_master_485.md` 与
`E1_SLAVER_POWER_CTU/docs/protocol_slaver_485.md`；帧格式
`[z][cmd][data_len][payload][CRC8][\n]`，设备 ID 在 payload 首字节。
