# E1_Master_Power_Manage

E1 主控电源板固件。基于 STM32F407（Cortex-M4, 168MHz），由 STM32CubeMX 生成并扩展出分层协作式中间件/任务架构。
主要功能：
- 电源管理：电源上下电时序控制（`srv_pwr_ctrl` 电源 FSM + 电机预充电软启动 FSM 双状态机，1ms 步进）、故障保护（E-STOP / 关键电源轨丢失 → 紧急断电 + 风扇全速，`app_fault_policy` 锁存至显式复位）
- CAN 通信：CAN1（P_CAN，1Mbps）上报 0x010 系统状态 / 0x011 温度 / 0x012 电压+预充故障三帧（固定 100ms 周期），接收 0x001 主机统一控制帧（蜂鸣器 + HSD 输出 + LED RGB）与 0x003 升级请求，详见 [docs/protocol_master.md](docs/protocol_master.md)
- 状态上报：电源轨故障 / E-STOP / HSD 公共通道 / 预充电异常 / DBR / 风扇故障 / A_IN 模拟输入 / NTC 连接打包为 0x010 位域帧
- ADC 采样：三路 ADC + DMA + VREFINT 校准，NTC 温度 / 电压轨 / 模拟输入（CD4051B 轮询）
- 风扇 PWM 调速与堵转检测、状态 LED（WS2812B 双通道，SPI DMA）、蜂鸣器（0x001 buzzer_duty 直驱）、参数存储与重启计数（Flash ring_storage）
- 构建：单 App 镜像，链接地址 0x08000000（原双镜像 E1_Boot 目标已停用）

## 构建

```bash
cmake --preset Debug
ninja -C build/Debug
```

Windows 一键脚本：`build.bat`；Linux/WSL：`./build.sh`。

## 版本记录

### 2026.09.03
#### V1.1.0
	- 1.CAN 状态上报重构：新增专用上报帧 0x010（2 字节位域：急停/电源轨/HSD 公共通道/CHG OCP/DBR OCP/风扇/A_IN/NTC 连接）、0x011（NTC1/NTC2/MCU 温度）、0x012（VIN/MOTOR/AUX 电压 + 电机预充故障码），每 100ms 固定周期发送（msg_fifo 逐帧发送 + 发送忙重试），0x001 不再承载状态上报
	- 2.0x001 统一主机控制帧接通（len=6）：蜂鸣器占空比 0-50 + 3 路 HSD 输出（valid/value，经 set_output 回调映射 `drv_power` 诊断使能，service 层与驱动同层解耦）+ LED RGB（led_index 0-31=RGB1 / 32-63=RGB2）；ISR 解析、主循环应用（蜂鸣器 PWM 与 LED SPI DMA 均不进 ISR）
	- 3.0x003 进 Boot 升级命令：置标志 → `srv_boot_ctrl_request_boot`（主循环消费，涉及 Flash 写 + 复位不放在 ISR）
	- 4.电源控制重构为 `srv_pwr_ctrl` V2：电源 FSM + 电机预充电软启动 FSM（双状态机，1ms 步进；预充四阶段：清 OCP 锁存 → 50kHz 恒频脉宽爬升 → 50k→600kHz 变频 → 600kHz 占空比爬升至稳态，含 OCP 重试与 NO_LOAD 判定）；电源轨 PGD 使能门控判定（EN=0 时 PGD 低为正常）
	- 5.故障保护：`app_fault_policy` 锁存 E-STOP / 关键电源轨丢失 → `srv_pwr_ctrl_emergency_off` + 风扇全速，需显式复位；`app_status_indicator` 故障/告警 LED 蓝+红优先级编码指示
	- 6.上报聚合 `app_status_report` 统一填充 0x010/0x011/0x012（聚合 `srv_pwr_det` / `srv_adc` / `srv_fan_ctrl` / 预充故障码）
	- 7.模块精简：移除 `srv_can_dual`（0x200~0x202 双电池解析）、`srv_can_slv`（0x002 从板）、`srv_device_monitor`；蜂鸣器去任务化（`buzzer_task`/`app_buzzer_ctrl` 移除，改由 0x001 控制帧直驱 `drv_buzzer`）
	- 8.构建调整：E1_Boot 目标注释停用，App 链接回 0x08000000（单镜像）；协议文档 protocol_master.md 同步重写为当前帧集

### 2026.08.14
#### V1.0.0
	- 1.正式版本初版
	- 2.电源上下电时序控制与故障保护（E-STOP / 关键电源轨丢失 → 紧急断电 + 风扇全速）
	- 3.CAN 通信（CAN1 复用：0x001 主机控制/状态上报、0x002 从板电源板、0x200~0x202 双电池）
	- 4.三路 ADC + DMA 采样（电压轨 / NTC 温度 / E-STOP / CD4051B 模拟输入，VREFINT 校准）
	- 5.风扇转速控制与堵转检测
	- 6.状态 LED（WS2812B 双通道）与蜂鸣器提示
	- 7.参数存储与 Boot 元数据（Flash ring_storage）
	- 8.双镜像构建（App + Boot）与 CAN 固件升级上位机
