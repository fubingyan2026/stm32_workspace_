# ctu_host — E1 CTU 电源板 RS485 双板调试上位机

E1_MASTER_POWER_CTU（主控板，addr 0x01）与 E1_SLAVER_POWER_CTU（副电源模块，addr 0x02）
共用一条 RS485 总线时的调试工具。基于 **PyQt6 + pyserial**，多页切换、界面自适应缩放。

## 运行

```bash
pip install -r requirements.txt
python ctu_host.py
```

串口为 USB-RS485（115200-8N1 默认，可切 460800/921600）。

## 界面结构

顶栏常驻连接与轮询控制，左侧导航切换五个页面，底部状态栏显示实时通信统计。

| 页面 | 内容 |
|------|------|
| 系统概览 | 两板关键健康指标一屏速览 / 在线状态 |
| E1_MASTER 主控板 | 查询、蜂鸣器控制、实时数据、固件信息 |
| E1_SLAVER 副电源板 | 查询、输出控制、实时数据、固件信息 |
| 固件升级 | 目标选择、固件预检、Boot 分块传输进度 |
| 通信日志 | 全部 TX / RX / 告警 |

说明：

- 「自动轮询」按间隔依次查询**勾选了“参与自动轮询”**的板（默认两板），轮询期间不记录原始
  收发帧以防刷屏，错误应答始终记录。
- 手动查询/控制显示 TX 帧，应答显示 `RX {板名}: hex [命令]`，0x7F 为错误应答。
- 「固件升级」开始时会**自动暂停轮询并独占串口**，完成后自动重连并恢复原轮询状态。
  升级协议直接复用 `E1_CTU_BOOT/host/boot_protocol.py`（单一来源）。
- 高分辨率屏幕：启动时使用 `PassThrough` 缩放策略并按点设置字体，页面均置于滚动区，
  在 125% / 150% 等非整数缩放下不会裁切。

## 代码结构（分层，仅向下依赖）

```
ctu_host.py            启动入口（High-DPI 策略 + QApplication）
app/
  protocol.py          z 帧编解码、命令/错误码、各板数据段解码（无 Qt 依赖）
  transport.py         SerialWorker / UpgradeWorker（Qt 工作线程）
  session.py           连接生命周期、等间隔轮询、10ms 丢包统计（无界面依赖）
  firmware.py          复用 Boot 工程协议层 + 固件预检入口
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

## 统一命令表

| cmd | 含义 | MASTER 数据段 | SLAVER 数据段 |
|-----|------|--------------|--------------|
| 0x01 | 读系统状态 | 2B 急停/电源轨 + 风扇/NTC 位域 | 2B 故障位 + 输出/锁存位 |
| 0x02 | 读电压 | 4B VIN / VIN_DC-DC | 8B AUX / MOTOR / LSD1 / LSD2 |
| 0x03 | 读温度 | 6B NTC1/NTC2/MCU | 4B MCU 温度 + VDDA |
| 0x04 | 控制 | 1B 蜂鸣器占空比 0-50% | 4B 输出掩码 + 补光亮度 0-1000 |
| 0x05 | 清除故障锁存 | 1B magic=0x01 | 1B magic=0x01 |
| 0x06 | 升级请求 | ACK 后复位进 Bootloader | ACK 后复位进 Bootloader |
| 0x07 | 读固件信息 | 15B App/Boot 版本 + 大小 + 校验和 + 重启次数 + 标志 | 同左 |

协议细节见 `E1_MASTER_POWER_CTU/docs/protocol_master_485.md` 与
`E1_SLAVER_POWER_CTU/docs/protocol_slaver_485.md`；帧格式
`[z][cmd][data_len][payload][CRC8][\n]`，设备 ID 在 payload 首字节。
