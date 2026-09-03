# E1_MASTER_POWER_CTU 分层固件骨架搭建计划

## 1. 背景与目标

`E1_MASTER_POWER_CTU/` 目前只是 STM32CubeMX 生成骨架（STM32F103RC、仅 Core/Drivers/cmake、无应用代码）。目标是参考兄弟工程 `E1_Master_Power_Manage`（STM32F407，分层架构 device_drivers→service→applications/tasks + 共享 public_layer），为量产主控板创建对应的分层文件夹与源码文件，通信方式由 CAN 改为 **RS485（USART3，主机查询应答式）**。

规划期间已确认的决策：

| # | 决策 |
|---|---|
| 1 | RS485 总线仅「本板 ↔ 上位机/主机」。**不**移植从板协议（0x002/srv_can_slv）。帧内容由 manage 的 `protocol_master.md` 语义平移并新封装为 485 字节流帧（地址/长度/CRC），新协议文档建在 `docs/protocol_master_485.md`。 |
| 2 | 引脚映射以当前 CubeMX `.ioc`/`Core/Inc/main.h` 为准（`hardware_pin.md` 仅作物理分压公式/NTC 等描述参考，与 main.h 冲突处以 main.h 为准）。 |
| 3 | CD4051B MUX **本期不启用**：急停只判 PC9 `E_STOP_ON` 数字输入；PC4 `CD4051B_ADC` 通道保留为 ADC 路由预留位，不做轮询。 |
| 4 | 电源行为：板上**自主顺序上电、无单轨指令**（VIN_DC-DC→DC-DC 24V→AUX→MOTOR，PGOOD 门控）；急停/关键轨丢失 → 紧急关断并锁存，急停释放沿自动复位重新上电。无预充 FET/HSD/DBR 分支。 |
| 5 | 485 上报时序：**主机查询应答式**，板不作自主周期上报。板作为 485 从站。 |
| 6 | 本阶段**不建任何 flash 模块**：无 `srv_param_store`/`srv_boot_ctrl`/`flash_task`/F1 flash 驱动；485「升级请求」命令仅占位并在协议文档标注预留。 |
| 7 | 允许修改**本工程自身**根 `CMakeLists.txt` 接入分层源码与 `../public_layer/m_middlewares`；不改动其他任何工程。 |
| 8 | 日志输出走 USART1 DMA（量产无 J-Link RTT）；在工程内建**本地 `tasks/log_task.*`**（遵循 G0_hand 先例），不修改共享 public_layer。 |

## 2. 目标目录结构（新建）

```
E1_MASTER_POWER_CTU/
├── applications/   app_fault_policy / app_status_indicator / app_status_report
├── service/        srv_adc / srv_com_mst / srv_fan_ctrl / srv_pwr_ctrl / srv_pwr_det
├── tasks/          app_main / com_task / fan_task / led_task / log_task(本地副本) /
│                   power_task / sample_task
├── device_drivers/ drv_adc / drv_buzzer / drv_fan / drv_led / drv_log_uart /
│                   drv_power / drv_pwm / drv_rs485 / drv_status / drv_systick
└── docs/           hardware_pin.md(已有) + protocol_master_485.md(新增) + README.md(新增，简短)
```

> 裁剪掉的 manage 模块：`drv_can/drv_ws2812b/drv_cd4051b/drv_revision/drv_hw_timer`、`srv_can_mst/srv_can_dual/srv_can_slv/srv_ws2812b/srv_param_store/srv_boot_ctrl`、`can_task/ws2812_task/flash_task/buzzer_task`。`drv_can_mst` 的宿主通信职责由新 `srv_com_mst` + `com_task` + `drv_rs485` 承担。

## 3. 硬件资源速查（来自 CubeMX Core 生成代码，代码以此为准）

- 时钟：HSE 8MHz，SYSCLK 72MHz；APB1=36MHz（TIM 时钟 72MHz）、APB2=72MHz；ADC 时钟 = APB2/6 = 12MHz。
- ADC1 + DMA1_Channel1：7 个 Rank 连续扫描、软件触发、Normal 模式。
  Rank1~7 = PC0 `NTC1_ADC`(IN10)、PC1 `NTC2_ADC`(IN11)、PC2 `VIN_DC_DC_ADC`(IN12)、PC3 `VIN_ADC`(IN13)、PC4 `CD4051B_ADC`(IN14，预留)、`TEMPSENSOR`、`VREFINT`。
- TIM3：CH3 = `BUZZ_PWM` PB0（蜂鸣器）。
- TIM4：CH1 = `LED_PWM` PB6、CH3 = `FAN1_PWM_IO` PB8、CH4 = `FAN0_PWM_IO` PB9（当前 ARR=65535/PSC=0，需在驱动里改 PSC/ARR 定频）。
- 风扇 FG：`FAN0_FG_IO` PA0（EXTI0、IT_RISING、上拉）、`FAN1_FG_IO` PA1（EXTI1）→ 上升沿脉冲计数。
- USART1 = 日志串口（PA9/PA10，115200，DMA1_Ch4=TX、Ch5=RX circular）。
- USART3 = RS485（PB10/PB11，115200，DMA1_Ch2=TX、Ch3=RX circular）+ `RS485_EN` PC5（高=发送方向）。
- 使能轨输出：`VIN_DC_DC_EN` PA11、`DC_DC_24V_EN` PB7、`AUX_POWER_EN` PD2、`MOTOR_POWER_EN` PC11（均高有效）。
- 状态/故障输入：`LM5060_PGOOD` PA8、`DC_DC_24V_PGOOD` PB5、`PGOOD_12V` PA12、`AUX_PWER_PGD` PC12、`MOTOR_POWER_PGD` PC10、`E_STOP_ON` PC9（急停，极性默认低有效按下，做成宏配置待实机确认）。
- 未启用（保留规划）：CD4051B 选择 `CD4051B_A/B/C` PA4/5/6、`CD4051B_ADC` PC4。

## 4. 文件清单与适配要点

### 4.1 device_drivers/

- `drv_systick.c/h`：F407 版移植，`delay_init` 按 72MHz 系数；`millis()`=HAL_GetTick。
- `drv_adc.c/h`：按 §3 的 7 Rank 重写路由表（枚举如 `DRV_ADC_CH_NTC1/VIN_DC_DC/VIN/CD4051B/VREFINT/TEMP`），DMA1_Channel1 Normal、软件触发；保留 `drv_adc_trigger_all()` + `HAL_ADC_ConvCpltCallback` 弱回调把原始帧交给 srv_adc（与 manage 的 trigger/step 两段式一致）。**不含** CD4051B 轮转。
- `drv_log_uart.c/h`：F407 版裁剪移植——USART1 DMA TX（DMA1_Ch4）单缓冲 + busy 标志 + `HAL_UART_TxCpltCallback`；本阶段 RX 不用（无控制台），可不接收（如需保留初始化需注意 F1 HAL 无 `UARTEx`/IDLE 辅助，勿用）。
- `drv_rs485.c/h`（新建核心驱动）：USART3 半双工。
  - TX：DMA1_Ch2 Normal 单发，发送前拉高 `RS485_EN`、置 busy、`HAL_UART_Transmit_DMA`；`HAL_UART_TxCpltCallback` 中拉低 `RS485_EN`（注意加小延时保证最后字节发完，或回调中关使能）并清 busy。对外 `drv_rs485_send(buf,len)` / `drv_rs485_is_tx_busy()`。
  - RX：USART3 RX DMA1_Ch3 circular；不做 IDLE 中断，暴露 `drv_rs485_rx_poll()`：读取 DMA 剩余计数（CNDTR）与上次水位差，把新字节拷贝进内置 kfifo（或返回给调用方）。由 com_task 每周期轮询拉取（与帧解析状态机配合，不依赖帧间空闲）。也可选择 RXNE 逐字节 ISR 方案，二选一，实现时确认寄存器/句柄写法（`hdma_usart3_rx` extern）。
  - init 完成 `HAL_UART_Receive_DMA` 循环启动。
- `drv_pwm.c/h`：管理 TIM4 三通道统一定频（默认 25kHz，PSC/ARR 在 init 一次写好）：`DRV_PWM_CH_LED=TIM4_CH1`、`DRV_PWM_CH_FAN1=TIM4_CH3`、`DRV_PWM_CH_FAN0=TIM4_CH4`；API 沿用 manage 千分比占空比 `drv_pwm_set_duty(ch, permille)`，`HAL_TIM_PWM_Start`。避免各驱动各自改 ARR 互相干扰。
- `drv_led.c/h`：单颗状态 LED 亮度封装（0..1023 → 对应 TIM4_CH1 占空比），供 srv_signal 回调使用。
- `drv_fan.c/h`：移植 F407 版；PWM 改走 `drv_pwm` 通道；FG 计数用 PA0/PA1 EXTI 上升沿（`HAL_GPIO_EXTI_Callback` 弱回调 + ISR 内计数），初始化里使能 NVIC EXTI0/EXTI1；对外 `drv_fan_get_tach_delta()` 等沿用 manage 接口。
- `drv_buzzer.c/h`：改为 TIM3_CH3，固定基频（默认 4kHz 待实机确认），`drv_buzzer_set(duty 0..100)` 语义同 manage。
- `drv_power.c/h`：4 路使能轨表（VIN_DC_DC/DC_DC_24V/AUX/MOTOR，均高有效），`drv_power_init/deinit/set/rail_name` 沿用。
- `drv_status.c/h`：状态输入表 = LM5060_PGOOD/DC_DC_24V_PGOOD/PGOOD_12V/AUX_PGD/MOTOR_PGD/E_STOP_ON；`active_low` 仅 E_STOP_ON 置 true（宏可调）；位掩码枚举/读取接口沿用 manage。

### 4.2 service/

- `srv_adc.c/h`：采样管道（trigger/step/msg_fifo/PT1）。F103 校准要点：VREFINT 标称 1.20V；温度传感器出厂校准地址 0x1FFFF7B8(30℃)/0x1FFFF7C2(110℃)（读到 0xFFFF 则回退），VREFINT 校准地址 0x1FFFF7BA。`srv_adc_data_t` 字段裁剪为：`vin_mv / vin_dcdc_mv / vdda_mv / ntc1_temp_x100 / ntc2_temp_x100 / mcu_temp_x100 / ntc1_status / ntc2_status / mcu_temp_status / timestamp_ms`（去掉 motor/aux/e_stop/adc2/ain）。分压公式按 doc：VIN 与 VIN_DC-DC 均为 1/23 分压，3.3V 满量程≈75.9V。NTC：先由分压求阻值（NTC1 上偏置 10k、NTC2 上偏置 10k ±1%），B=3950、R25=10k 默认配置宏，实机标定后调。
- `srv_com_mst.c/h`（新，替代 srv_can_mst）：485 主机协议编解码/应答服务，**回调注入**（不直连 drv_*）：`read_data`（填充上报结构）与 `send_frame`（发应答帧）。RX 命令解析在主循环；应答帧经 TX msg_fifo 排队，com_task 逐帧经 `drv_rs485_send` 发送。内部持有状态快照数据结构 `srv_com_mst_data_t`（状态/温度/电压），打包函数与 4.3 app_status_report 对接。
- `srv_pwr_ctrl.c/h`：移植并裁剪 —— 去掉预充 FSM/HSD/DBR/PWM 充电，保留「电源 FSM + 1ms step」骨架。状态机：`IDLE→VIN_DCDC→DC24V→AUX→MOTOR→POWERED`，每步使能对应轨并等待 PGD（带超时）；`FAULT` 态全关（紧急关断）；`emergency_off()`/`request_on()`/`is_powered_on()` 等 API 沿用 manage 命名。步骤细节（顺序、PGD 门控、超时）做成常量宏。
- `srv_pwr_det.c/h`：状态读取：各轨 ok 位（由 drv_status）+ `estop_on`（PC9，本阶段无 ADC 冗余/无 inconsistent 标志，字段保留置 0）。API 结构沿用 manage（供 app_status_report/app_fault_policy 读取）。
- `srv_fan_ctrl.c/h`：几乎原样移植（每扇 FSM、RPM 滤波、堵转、温度映射）；温度回调由 fan_task 注入（fan0=NTC1、fan1=NTC2）。

### 4.3 applications/

- `app_status_report.c/h`：聚合 srv_pwr_det/srv_adc/srv_fan_ctrl/srv_pwr_ctrl → 填充 `srv_com_mst_data_t`（2B 状态位域 + 温度 + 电压），供 com_task/srv_com_mst 应答。
- `app_fault_policy.c/h`：移植。触发 = `estop_on`（无条件）或（已上电且关键轨 PGD 丢失：MOTOR/AUX/24V/LM5060 在使能后掉 PGD）→ `srv_pwr_ctrl_emergency_off()` + 风扇 100% + 锁存；12V PGD 仅上报不触发。急停释放沿自动 reset + `request_on()`。具体哪些轨判关键做成可调配置。
- `app_status_indicator.c/h`：移植，但仅 1 路 LED（注入本地 srv_signal 实例），分级灯效（正常/警告/关键/急停）。

### 4.4 tasks/

- `log_task.c/h`（本地副本）：拷贝 public_layer 版裁剪——`s_output_mode=LOG_OUTPUT_UART`（USART1 DMA）、去掉控制台命令与 srv_log_flash 相关（init/step/dump），保留 10ms sw_timer 排空 log kfifo → `drv_log_uart_send`；保留 `log_task_flush()`（供复位前/将来 boot 用，可选）。
- `app_main.c/h`：初始化顺序与主循环（去掉 flash/can/ws2812）：
  1) `delay_init()` → 2) `log_task_init()`（打印版本 banner）→ 3) `com_task_init()`（drv_rs485 + srv_com_mst 注入回调 + drv_buzzer_init）→ 4) `fan_task_init()` → 5) `led_task_init()`（drv_led + srv_signal + app_status_indicator）→ 6) `sample_task_init()` → 7) `power_task_init()`（drv_power/drv_status + srv_pwr_ctrl + app_fault_policy；非急停则 request_on()）→ 8) `for(;;){ sw_timer_tick(millis()); sw_timer_task(); }`。
  `Core/Src/main.c` 在 USER CODE 区调用 `app_main()`。
- `sample_task.c/h`：10ms `srv_adc_trigger()`+`srv_adc_step()`。
- `power_task.c/h`：1ms `srv_pwr_ctrl_step(1)` + `app_fault_policy_step()`。
- `fan_task.c/h`：100ms `srv_fan_ctrl_step` + 温度回调。
- `led_task.c/h`：10ms app_status_indicator + srv_signal_task_refresh。
- `com_task.c/h`（新，替代 can_task）：
  - init：注册 `srv_com_mst` 的 read_data/send_frame 回调；启动 10ms sw_timer。
  - timer 回调：`drv_rs485_rx_poll()` → 逐字节喂帧解析状态机（SOF/ADDR/CMD/LEN/PAYLOAD/CRC）→ 完整帧校验通过后调 `srv_com_mst_process_rx()`；随后排空应答 TX msg_fifo（busy 则保留重试）；`srv_com_mst_step()`。

### 4.5 docs/ 与根文件

- 新建 `docs/protocol_master_485.md`（**本阶段关键交付物之一**，草案如下，待主机联调锁定）：
  - 总线：RS485 半双工，115200-8N1，主机=主站，本板=从站地址 0x01。
  - 信封：`[SOF 0xA5][ADDR][CMD][LEN][PAYLOAD...][CRC16-CCITT(LE, 覆盖 SOF..PAYLOAD)]`；CRC 用 public_layer `get_CRC16_CCITT_FALSE`。
  - 命令（下行/应答均 0x01 地址；应答 CMD = 0x80|CMD，bit7=应答；未知命令应答 0x7F+错误码）：
    - `0x01 读系统状态` → 应答 2B 位域（状态位映射建议：byte0 bit0 estop、bit1 err_12v(PGOOD_12V)、bit2 err_24v(DC-DC 24V PGOOD)、bit3 err_vin_dcdc(LM5060 PGOOD)、bit4 err_aux、bit5 err_motor，bit6/7 留；byte1 bit0 err_fan0、bit1 err_fan1、bit2 err_ntc1、bit3 err_ntc2，其余留）。
    - `0x02 读温度` → 6B：ntc1/ntc2/mcu 温度 int16 LE ×100℃。
    - `0x03 读电压` → 4B：`vin_mv`、`vin_dcdc_mv` uint16 LE（后续启用 mux/增加 24V 电压再扩）。
    - `0x10 控制` → payload `[buzzer_duty(0..50)][reserved 0x00 x3]`，收到即生效（蜂鸣器占空比直通），应答 ACK；LED 由板上自动状态指示管理（不做主机控制）。
    - `0x11 升级请求(预留)`：本阶段应答「不支持(0x7F, err=0x02)」，文档标注为将来 RS485 升级预留。
- 新建 `README.md`（简短）：工程定位、芯片/485/构建方式/版本记录表头。
- 根 `CMakeLists.txt`（本工程允许改）：
  - 仿 manage 增加 `aux_source_directory(tasks|device_drivers|service|applications)`；
  - `add_subdirectory(../public_layer/m_middlewares m_middlewares)` + `target_link_libraries(... m_middlewares)`；
  - 增加 include：`tasks device_drivers service applications ../public_layer/service`；
  - 源增加 `${CMAKE_SOURCE_DIR}/../public_layer/service/srv_signal.c`（log_task 用本地副本，不再引入共享 log_task）；
  - 编译宏：`PRINTF_DISABLE_SUPPORT_FLOAT`、`PRINTF_DISABLE_SUPPORT_EXPONENTIAL`；不加 flash/log_flash 相关宏。
  - 其余保持 CubeMX 骨架内容不动；链接脚本 `STM32F103XX_FLASH.ld`（0x08000000）不变。
- `Core/Src/stm32f1xx_it.c`（USER CODE 块内）：新增 `EXTI0_IRQHandler`/`EXTI1_IRQHandler`（调 `HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0/1)`）；`Core/Src/main.c` USER CODE 中调 `app_main()`。仅 USER CODE 内改动，避免被 CubeMX 再生成覆盖。
- （可选）参照 manage 生成 Windows 一键 `build.bat`（按本工程改名）。

## 5. 实现顺序（建议）

1. 先建目录 + 拷贝 manage 对应文件到新工程做基线（git 或直接拷贝均可）。
2. device_drivers 层按 §3 引脚/外设逐文件适配（drv_systick→drv_adc→drv_pwm/drv_led/drv_buzzer→drv_fan→drv_power/drv_status→drv_log_uart→drv_rs485）。
3. service 层（srv_adc→srv_pwr_det→srv_pwr_ctrl→srv_fan_ctrl→srv_com_mst）。
4. applications（app_status_report→app_fault_policy→app_status_indicator）。
5. tasks + 本地 log_task + app_main + 主循环接线。
6. `docs/protocol_master_485.md` 落盘。
7. 根 CMakeLists 接线 + Core USER CODE 最小改动（main 调 app_main、EXTI handler）。
8. 构建验证（见 §6）。

## 6. 验证

- 无测试基础设施；验证方式为编译 + 链接成功 + 无警告（目标工程相关）。
- 命令（Windows，toolchain 来自 cmake/gcc-arm-none-eabi.cmake）：
  `cmake --preset Debug` → `cmake --build --preset Debug`，产物 `build/Debug/E1_MASTER_POWER_CTU.elf/.bin/.hex/.map`；`flash map` 检 RAM 占用（F103RC 48KB）不超限。
- clangd 索引（`.clangd` 指向 build/Debug，须先 Debug 构建）。
- 无法上电联调的部分（E_STOP 极性、蜂鸣器频率、NTC B/R25、485 帧字节序与主机端约定）在代码中集中为宏/常量并留注释，文档标注待实机确认。

## 7. 风险与待确认默认值（实现时在代码集中标注）

1. E_STOP_ON 极性默认低=按下（与 manage 一致），实机需确认。
2. 蜂鸣器基频默认 4kHz、风扇 PWM 默认 25kHz、485 115200 8N1。
3. NTC1（外部探头）默认 R25=10k/B=3950（同 NTC2），实机标定。
4. 485 信封与命令码为**本工程起草**，需与主机侧联调后锁定（协议文档是唯一契约）。
5. TIM4 供 LED 与双风扇共用同一频率（25kHz），LED 调光走占空比不影响风扇。
6. 未来扩展（MUX/参数持久化/485 升级）已预留占位，不在本期实现。

## 8. 明确不做（范围外）

- 从板（E1_SLAVER_POWER_CTU）通信与协议。
- Bootloader / RS485 固件升级。
- Flash 参数持久化与日志落盘。
- WS2812B/CAN/多 ADC/预充 PWM 等 manage 中 F407 专属功能。
- 修改除本工程根 CMakeLists.txt 外的任何 CMake/共享 public_layer 代码。
