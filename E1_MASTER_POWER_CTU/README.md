# E1_MASTER_POWER_CTU

E1 量产主控电源板固件（E1_Master_Power_Manage 的 F103 量产版本，通信方式由 CAN 改为 **RS485**）。基于 STM32F103RC（Cortex-M3, 72MHz），由 STM32CubeMX 生成并扩展出分层协作式任务/服务架构（device_drivers → service → applications → tasks + 共享 public_layer）。

## 主要功能

- 电源管理：AUX **初始化直接使能**（不参与等待）；5 步上电流程（`srv_pwr_ctrl` fsm）：VIN∈[36,58]V → VIN_DC-DC≥90%VIN → 使能 VIN_DC-DC_EN(+100ms) → 使能 DC_DC_24V_EN → 双 PGD(DC24V+LM5060) 就绪=上电成功；**急停只控制 MOTOR**（VIN/24V/AUX 不被急停关断），MOTOR 由故障策略门控（未锁存+已上电+未急停时使能）
- 故障保护：E-STOP 双判据（PC9 数字串链 + CD4051B 轮询 4 组 E-STOP 双冗余节点，任一冗余不一致）→ 紧急断电 + 风扇满速 + 锁存（`app_fault_policy`）
- RS485 通信（USART3，主机查询应答式）：z 帧 `[z][cmd][len][payload][CRC8][\n]`（与 G0 上位机协议同构），解析/打包用 `protocol_parser`/`protocol_packer`，详见 [docs/protocol_master_485.md](docs/protocol_master_485.md)
- ADC 采样：ADC1 + DMA + VREFINT 校准，NTC1/NTC2 温度、VIN/VIN_DC-DC 电压、MCU 温度；CD4051B（PA4/5/6 + PC4）逐周期轮转采样 4 组 E-STOP 双冗余节点（Y0/1、Y2/3…=ADC1/ADC2）
- 风扇 PWM 调速（TIM4 25kHz 共用组）与 FG 堵转检测（PA0/PA1 EXTI）、蜂鸣器（TIM3_CH3）、单颗状态指示灯（`app_status_indicator` + `srv_signal`）
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
