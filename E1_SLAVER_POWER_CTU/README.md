# E1_SLAVER_POWER_CTU

副电源管理模块固件（E1 系统从板）。基于 STM32F103RCT7（Cortex-M3, 72MHz），由 STM32CubeMX 生成并扩展出分层协作式任务/服务架构（device_drivers → service → applications → tasks + 共享 public_layer）。参考兄弟工程 `E1_MASTER_POWER_CTU` 搭建。

## 主要功能

- 电源输出远程指令 + 门控保护（`srv_pwr_ctrl`，1ms 步进）：24V DC-DC(LM5146) / 12V_ISO(URB2412S) / LSD1/LSD2(ZXMS6004FF 低边开关) 四路输出，PGOOD/节点电压门控 + 使能超时 + 运行期丢失去抖（100ms 防误关断）→ 故障锁存
- 母线监控：AUX/MOTOR 48V 输入电压、LSD1/LSD2 输出节点电压（ADC1 DMA + VREFINT 校准，分压 ×23/×23/×11/×11）；母线级 AUX 缺失保护（`app_fault_policy`）
- RS485 通信（USART3，主机查询应答式）：z 帧 `[z][cmd][len][payload][CRC8][\n]`，解析/打包用 `protocol_parser`/`protocol_packer`，详见 [docs/protocol_slaver_485.md](docs/protocol_slaver_485.md)
- 补光灯 PWM（TIM4_CH3 → PT4115 DIM，20kHz，0~1000‰ 亮度）；单颗蓝色状态灯（TIM4_CH1 + `app_status_indicator` + `srv_signal` 灯效）
- 日志：USART1 TX DMA（本地 `tasks/log_task.*`）

## 引脚为准

驱动层引脚一律以 CubeMX 生成的 `Core/Inc/main.h` 宏名为准；`docs/hardware_pin.md` 仅作物理描述（分压/收发器）参考，如有出入以 main.h 为准。

## 构建

```bash
cmake --preset Debug
ninja -C build/Debug
```

产物在 `build/Debug|Release/`（`.elf/.bin/.hex`）。clangd 索引需先 Debug 构建（`.clangd` 指向 build/Debug）。

## 目录结构

```
applications/   应用策略（状态上报聚合 / 母线故障保护 / 状态灯）
tasks/          任务入口（app_main + 各 sw_timer 任务 + 本地 log_task）
service/        srv_adc / srv_com_slv / srv_pwr_ctrl / srv_pwr_det
device_drivers/ drv_adc / drv_uart / dev_rs485 / drv_pwm / drv_power / drv_status /
                drv_led / drv_log_uart / drv_systick
docs/           hardware_pin.md（硬件） / protocol_slaver_485.md（485 协议）
```

## 版本记录

### 2026.09.03
#### V0.1.0（骨架首版）
- 仿 E1_MASTER_POWER_CTU 搭建分层架构，适配 STM32F103RCT7（ADC1 6 Rank / TIM4 CH1+CH3 / 双 USART）
- RS485 主机查询应答协议与驱动、输出监督 FSM（远程指令 + 门控 + 锁存）、补光灯与状态灯、UART 日志
- 无 flash 参数存储 / bootloader（升级请求预留）
- 阈值/时序/协议命令码为草案，待实机与主机联调确认
