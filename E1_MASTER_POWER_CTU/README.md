# E1_MASTER_POWER_CTU

E1 量产主控电源板固件（E1_Master_Power_Manage 的 F103 量产版本，通信方式由 CAN 改为 **RS485**）。基于 STM32F103RC（Cortex-M3, 72MHz），由 STM32CubeMX 生成并扩展出分层协作式任务/服务架构（device_drivers → service → applications → tasks + 共享 public_layer）。

## 主要功能

- 电源管理（`srv_pwr_ctrl` fsm）：输入 VIN∈[36,58]V **有效后才允许使能任何输出**（超范围四路全不使能 + 蜂鸣器周期上报）；5 步上电：VIN 合格 → VIN_DC-DC≥90%VIN → 使能 VIN_DC-DC_EN(+100ms) → 使能 DC_DC_24V_EN → 双 PGD(DC24V+LM5060) 就绪=上电成功；AUX 在输入有效后即使能（不依赖其它轨成功）；已上电后 **低压(<36V)不关断**，**≥60V 持续 5s 关断全部输出并锁存**（回落 ≤58V 自动重启）；**急停只控制 MOTOR**（VIN/24V/AUX 不被急停关断），MOTOR 由故障策略门控（未锁存+已上电+未急停时使能，上电前请求延后生效）
- 故障保护：E-STOP 双判据（PC9 数字串链 + CD4051B 轮询 4 组 E-STOP 双冗余节点，任一冗余不一致）→ 关断 MOTOR + 风扇满速 + 锁存（`app_fault_policy`）；485 命令 0x05 可清除锁存
- RS485 通信（USART3，主机查询应答式）：两兄弟板共用统一帧头 z 帧 `[z][cmd][len][payload][CRC8][\n]`，设备区分在 **payload 首字节 ID**（下行=目标 ID 定向，应答=源 ID；E1_MASTER=0x01/E1_SLAVER=0x02 共用总线不撞线），解析/打包用 `protocol_parser`/`protocol_packer`，详见 [docs/protocol_master_485.md](docs/protocol_master_485.md)
- ADC 采样：ADC1 + DMA + VREFINT 校准，NTC1/NTC2 温度、VIN/VIN_DC-DC 电压、MCU 温度（无出厂校准自动回退典型参数）；CD4051B（PA4/5/6 + PC4）逐周期轮转采样 4 组 E-STOP 双冗余节点（Y0/1、Y2/3…=ADC1/ADC2）；内置“无快照看门狗”自动复位 DMA 链路
- 风扇 PWM 调速（TIM4 25kHz 共用组）+ FG 测速/堵转（PA0/PA1 EXTI；暂无 FG 时可用 `SRV_FAN_CTRL_TACH_BYPASS` 宏屏蔽故障）、蜂鸣器（TIM3_CH3，导通占空比 0-50%）、单颗状态指示灯（`app_status_indicator` + `srv_signal`）
- 日志：USART1 TX DMA（本地 `tasks/log_task.*`，量产默认 UART；支持运行时切 SEGGER RTT，`log_task_set_output()`）

## 引脚为准

驱动层引脚一律以 CubeMX 生成的 `Core/Inc/main.h` 宏名为准；`docs/hardware_pin.md` 仅作物理描述（分压/NTC/收发器）参考，如有出入以 main.h 为准。

## 构建

```bash
cmake --preset Debug
ninja -C build/Debug
```

Windows 一键脚本：`build.bat`（自动在 `%LOCALAPPDATA%\stm32cube\bundles` 下找工具链/CMake/Ninja，默认 RelWithDebInfo）。产物在 `build/Debug|RelWithDebInfo/`。

## 目录结构

```
applications/   应用策略（状态上报聚合 / 故障保护 / 状态灯）
tasks/          任务入口（app_main + 各 sw_timer 任务 + 本地 log_task）
service/        srv_adc / srv_com_mst / srv_fan_ctrl / srv_pwr_ctrl / srv_pwr_det
device_drivers/ drv_adc / drv_cd4051b / dev_rs485 / drv_uart / drv_pwm / drv_led / drv_buzzer /
                drv_fan / drv_power / drv_status / drv_log_uart / drv_systick
docs/           hardware_pin.md（硬件） / protocol_master_485.md（485 协议）
```

## 版本记录

### 2026.09.03
#### V0.1.0（骨架首版）
- 按 E1_Master_Power_Manage 分层架构搭建，适配 STM32F103RC 单 ADC1/TIM3/TIM4/双 USART
- RS485 查询应答协议与驱动、顺序上电 FSM、故障保护、风扇温控、状态灯、UART 日志
- 无 flash 参数存储 / bootloader（升级请求预留）
- 急停采用 PC9 数字 + CD4051B 4 组 E-STOP 双冗余节点轮询的双判据；通道映射/容差按最终原理图精调

### 2026.09.08
#### V0.2.0（联调迭代）
- 电源语义定型：输入电压有效后才使能（超范围四路不使能+蜂鸣上报）、5 步上电、低压不关断、≥60V/5s 过压关断并自动恢复；急停只控制 MOTOR，MOTOR 延后使能
- 485 协议 V4：与 E1_SLAVER 统一（addr 寻址、命令码重排、0x05 清除锁存）
- 485 协议 V5：帧头去掉 addr 统一信封，设备改由 payload 首字节 ID 定向/标识（E1_MASTER=0x01）
- MCU 温度典型参数兜底（斜率符号修正）、CD4051B 无阻塞预选、急停冗余判稳/抑制误报
- ADC 无快照看门狗自动恢复（防 485 高频查询停采）；风扇 FG 暂无时可宏屏蔽
- 上位机 host 支持主/从寻址、收发帧显示、固定槽位数据区
