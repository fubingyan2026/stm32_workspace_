# E1_Master_Power_Manage

E1主控电源板固件。基于 STM32F407（Cortex-M4, 168MHz），由 STM32CubeMX 生成并扩展出分层协作式中间件/任务架构。
主要功能：
- 电源管理：电源上下电时序控制、故障保护（E-STOP / 关键电源轨丢失 → 紧急断电 + 风扇全速，`app_fault_policy` 锁定）
- CAN 通信：CAN1 复用三套服务（0x001 主机控制/状态上报、0x002 从板电源板、0x200~0x202 双电池），详见 [docs/protocol_master.md](docs/protocol_master.md)
- 状态上报：电源故障 / E-STOP / 风扇故障 / 双电池快照，0x001 打包位域帧
- ADC 采样：三路 ADC + DMA + VREFINT 校准，NTC 温度 / 电压轨 / 模拟输入
- 风扇控制、状态 LED（WS2812B）、蜂鸣器、参数存储（Flash ring_storage）
- 双镜像构建：App（0x08020000）+ Boot（0x08000000），支持 CAN 固件升级

## 构建

```bash
cmake --preset Debug
ninja -C build/Debug
```

Windows 一键脚本：`build.bat`；Linux/WSL：`./build.sh`。

## 版本记录

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
