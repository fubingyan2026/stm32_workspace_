# ctu_host — E1 CTU 电源板 RS485 双板调试上位机

E1_MASTER_POWER_CTU（主控板，addr 0x01）与 E1_SLAVER_POWER_CTU（副电源模块，addr 0x02）
共用一条 RS485 总线时的**单窗口同屏调试工具**（由原两工程各自的上位机合并而来）。

- `ctu_host.py` — PySide6 + pyserial 界面：左 E1_MASTER、右 E1_SLAVER，顶部连接栏 + 底部共用日志。
- `ctu_protocol.py` — 协议层：两板公用 z 帧信封 `[z][cmd][data_len][payload][CRC8][\n]`，设备区分在 payload 首字节 ID（下行=目标 ID 定向、应答=源 ID）
  与统一命令码，负载数据段按板不同。

## 运行

```bash
python ctu_host.py
```

依赖：`pyserial`、`PySide6`。串口为 USB-RS485（115200-8N1 默认，可切 460800/921600）。

## 统一命令表

| cmd | 含义 | MASTER 数据段 | SLAVER 数据段 |
|-----|------|--------------|--------------|
| 0x01 | 读系统状态 | 2B 急停/电源轨 + 风扇/NTC 位域 | 2B 故障位 + 输出/锁存位 |
| 0x02 | 读电压 | 4B VIN / VIN_DC-DC | 8B AUX / MOTOR / LSD1 / LSD2 |
| 0x03 | 读温度 | 6B NTC1/NTC2/MCU | 4B MCU 温度 + VDDA |
| 0x04 | 控制 | 1B 蜂鸣器占空比 0-50% | 4B 输出掩码 + 补光亮度 0-1000 |
| 0x05 | 清除故障锁存 | 1B magic=0x01 | 1B magic=0x01 |
| 0x06 | 升级请求（预留） | 应答不支持 | 应答后跳 E1_CTU_BOOT |

协议细节见 `E1_MASTER_POWER_CTU/docs/protocol_master_485.md` 与
`E1_SLAVER_POWER_CTU/docs/protocol_slaver_485.md`。

## 使用提示

- 「自动轮询」会按间隔依次查询**勾选了"自动轮询此板"**的面板（默认两板都查）；
  轮询期间收发帧不刷日志，避免刷屏（错误应答始终显示）。
- 手动查询/控制发送会显示 TX 帧，收到应答显示 `RX {板名}: hex [命令]`，0x7F 为错误应答。
- MASTER 侧「清除保护锁存 (0x05)」需烧录命令码统一后的新固件（V4.0.0）才有效；
  升级功能（0x06）由 E1_CTU_BOOT/host/boot_host.py 承担。
