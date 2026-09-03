# E1_SLAVER_POWER_CTU 分层固件骨架搭建计划

## 1. 背景与目标

`E1_SLAVER_POWER_CTU/`（副电源管理模块，STM32F103RCT7/LQFP64）目前是纯 STM32CubeMX 骨架（Core/Drivers/cmake + `docs/hardware_pin.md`，无任何业务代码）。目标是参考兄弟工程 `E1_MASTER_POWER_CTU`（STM32F103RC，RS485 主机查询应答从站，分层 device_drivers→service→applications/tasks + 共享 public_layer），搭建本工程对应分层架构，并按本板硬件实现完整的电源输出/状态采集/485 通信固件。

模块拷贝来源：`../E1_MASTER_POWER_CTU/`（下称「master 工程」）。复用/新增的共享层：`../public_layer/m_middlewares` + `../public_layer/service/srv_signal.c`（不改共享原件）。

## 2. 规划期已确认决策

| # | 决策 |
|---|---|
| 1 | **485 拓扑**：从板为独立 485 从站，直接接上位机/主机（与 master 工程同构，z 帧无地址）；**新起草** `docs/protocol_slaver_485.md` 并完整实现。 |
| 2 | **输出控制模型**：远程指令 + 门控保护。上电默认全关；主机经 0x10 控制帧设置期望输出与补光亮度。本地 FSM 按依赖（24V→12V_ISO、24V 就绪后才允许 LSD）门控使能，PGOOD/节点监测超时 → 关断 + 故障锁存；状态帧上报，主机发清锁存命令后自动重试。不做自主顺序上电。 |
| 3 | **LSD1_IN/LSD2_IN(PA2/PA3)**：改为 `GPIO_Output`（高有效），驱动 ZXMS6004FF 低边智能开关的 IN。当前 .ioc 误配为 Input，需改 .ioc 并重新生成。 |
| 4 | **引脚以 `Core/Inc/main.h` + `.ioc` 为准**（沿用 master 工程 README 规则）；`docs/hardware_pin.md` 仅作物理描述参考。与 main.h 出入处（doc 中 RS485_EN=PC4、LED_PWM=PB7、LSD IN 为输出）实施时同步修订 doc。 |
| 5 | 允许修改**本工程自身**根 `CMakeLists.txt`；不动其他工程 CMake、不动 public_layer。 |
| 6 | 日志走 USART1 DMA，在工程内建**本地 `tasks/log_task.*`**（照抄 master 工程本地副本，不引入共享 log_task）。 |
| 7 | 无 flash 参数存储 / 无 bootloader；485「升级请求」仅占位（0x1F 应答不支持）。 |
| 8 | 本板无风扇/蜂鸣器/CD4051B/EXTI/E-STOP/多电源轨自主上电：对应 master 模块不移植。 |

## 3. 硬件资源速查（以 CubeMX 生成代码为准）

- 时钟：HSE 8MHz、SYSCLK 72MHz、APB1=36MHz（TIM×2=72MHz）、APB2=72MHz、ADC=12MHz。
- **ADC1 + DMA1_CH1**（Word、NORMAL、软触发、扫描一次 6 Rank）：
  Rank1 `ADC_CHANNEL_10`=PC0 `AUX_POWER_ADC`、Rank2 IN11=PC1 `MOTOR_POWER_ADC`、Rank3 IN12=PC2 `LSD1_ADC`、Rank4 IN13=PC3 `LSD2_ADC`、Rank5 `TEMPSENSOR`、Rank6 `VREFINT`。
- **输出（GPIO 高有效）**：`DC_DC_EN`=PA0、`ISO_EN_12V`=PA4、`LSD1_IN`=PA2、`LSD2_IN`=PA3（PA2/PA3 需 .ioc 改为 Output）。
- **状态输入**：`EXT_PCOOG_24V`=PA1（LM5146 24V PGOOD，高=好）、`ISO_PGOOD_12V`=PA5（12V_ISO 窗口比较器 11.4~12.6V，高=好）。
- **TIM4**：CH1=PB6 `LED_PWM`（状态灯）、CH3=PB8 `FILL_LED_PWM`（补光灯 PT4115 DIM）。ARR=65535/PSC=0 需驱动覆写。
- **USART1**=日志（PA9/PA10，DMA1_CH4 TX normal / CH5 RX circular）；**USART3**=RS485（PB10/PB11，DMA1_CH2 TX normal / CH3 RX circular，`RS485_EN`=PC5 高=发）。
- 分压（电压换算）：AUX 220k/10k→×23（0~75.9V）；MOTOR ×23；LSD1/LSD2 100k/10k→×11（0~36.3V）。
- 供电拓扑：48V `AUX_POWER`→LM5146→`24V_EXT`；24V_EXT→URB2412S→`12V_ISO`→PT4115 补光（CN701）；24V_EXT 为 CN601 LSD1/LSD2 外部负载供电轨。
- 芯片：ZXMS6004FFTA 为**低边智能开关（IN 逻辑输入，自恢复限流/过温保护）**。

## 4. 目标目录与文件映射

> 拷贝源均为 `../E1_MASTER_POWER_CTU/` 对应路径。状态列：`COPY`=照搬（可能仅改文件头注释/标识）、`ADAPT`=拷骨架改内容、`NEW`=新写。所有阈值/时序做成模块头文件集中宏，标注「待实机确认」。

### 4.1 device_drivers/（新建）

| 文件 | 状态 | 说明 |
|---|---|---|
| drv_systick.c/h | COPY | master 原样。 |
| drv_uart.c/h | COPY | 仅登记 USART3 通道，与 slave 完全同构。前提：hal_conf `USE_HAL_UART_REGISTER_CALLBACKS=1U`、USART3 IRQ 已注册并使能（见 §6）。 |
| dev_rs485.c/h | COPY | 引用 `RS485_EN_*`(PC5)、`huart3`，slave 同名同引脚。 |
| drv_log_uart.c/h | COPY | USART1 TX DMA。 |
| drv_pwm.c/h | ADAPT | 路由表改为 2 通道：`DRV_PWM_CH_LED`=TIM4_CH1、`DRV_PWM_CH_FILL`=TIM4_CH3；默认组频 **20kHz**（PT4115 DIM 建议）；去掉 FAN0/FAN1 通道。 |
| drv_led.c/h | COPY | 只依赖 `DRV_PWM_CH_LED` + duty 0~1023。 |
| drv_power.c/h | ADAPT | 表改为 4 路输出（均高有效）：`DRV_POWER_OUT_24V_DCDC`=PA0、`DRV_POWER_OUT_12V_ISO`=PA4、`DRV_POWER_OUT_LSD1`=PA2、`DRV_POWER_OUT_LSD2`=PA3。API 保持 master 风格（init/set/toggle/name）。 |
| drv_status.c/h | ADAPT | 信号表改为 2 路输入：`DRV_STATUS_24V_PGD`(EXT_PCOOG_24V)、`DRV_STATUS_12V_ISO_PGD`(ISO_PGOOD_12V)，均高电平=好、无 active_low。 |

不移植：drv_buzzer、drv_fan、drv_cd4051b。

### 4.2 service/（新建）

| 文件 | 状态 | 说明 |
|---|---|---|
| srv_adc.c/h | ADAPT | 骨架照 master（trigger/step 两段 + VREFINT→VDDA + 内部温度 + PT1/msg_fifo 管道），内容改写：无 NTC、无 CD4051B/E-STOP。`srv_adc_data_t` 字段 = `aux_mv/motor_mv/lsd1_mv/lsd2_mv/vdda_mv/mcu_temp_x100/timestamp_ms` + 各换算 status。分压表 {×23,×23,×11,×11}。F103 校准地址：TS 30/110℃=`0x1FFFF7B8/0x1FFFF7C2`，VREFINT=`0x1FFFF7BA`（读到 0xFFFF 则回退默认并置 status）。 |
| srv_pwr_ctrl.c/h | ADAPT/NEW | 「期望输出 + 门控使能 FSM」（语义新写，非 master 顺序上电）。详见 §5.1。依赖：drv_power、drv_status、注入母线电压读取回调（task 层接线到 srv_adc）。 |
| srv_pwr_det.c/h | ADAPT | 瘦身为「瞬时物理状态」：`srv_pwr_det_status_t` = { `dc24v_pgood_ok`, `iso12v_pgood_ok` }（读 drv_status）；供 app_status_report 取原始 PGOOD。不含 E-STOP/CD4051B。 |
| srv_com_slv.c/h | NEW | 从 master `srv_com_mst` 骨架改写并改名（`srv_com_slv_*`）。z 帧解析/打包、`config_t{read_data/ctrl/send_frame}` 三回调、`srv_com_slv_report_t`、命令/错误码枚举。命令语义见 §7。 |

### 4.3 applications/（新建）

| 文件 | 状态 | 说明 |
|---|---|---|
| app_status_report.c/h | ADAPT | 聚合 srv_adc + srv_pwr_det + srv_pwr_ctrl → 填充 `srv_com_slv_report_t`（作为 read_data 回调，签名同 `srv_com_slv_read_cb_t`）。 |
| app_fault_policy.c/h | ADAPT | 运行时监控：对 `srv_pwr_ctrl` 中处于 ON 的输出检测好状态丢失，去抖（默认 ≥100ms，防误关断）后调 `srv_pwr_ctrl_latch_outputs()` 锁存关断；AUX 缺失且 24V 期望开/实际开同样处理；汇总 `is_tripped()`。 |
| app_status_indicator.c/h | ADAPT | 单颗蓝色状态灯等级：锁存故障→快闪、使能中/输入缺失警告→慢闪、正常（24V+12V 均 ON 无故障）→呼吸/常亮、空闲→微光/灭。映射阈值做成宏。 |

### 4.4 tasks/（新建）

| 文件 | 状态 | 说明 |
|---|---|---|
| log_task.c/h | COPY | master 本地副本（UART 后端）。 |
| app_main.c/h | ADAPT | 初始化顺序见 §5.2；注释/横幅改 E1_SLAVER_POWER_CTU。 |
| com_task.c/h | ADAPT | 10ms：dev_rs485 读字节→srv_com_slv rx_feed/tick→tx_flush；接线 read_data=app_status_report_fill、send_frame→dev_rs485_send；ctrl 回调解析 0x10→`srv_pwr_ctrl_request_outputs()` + `drv_pwm_set_duty(DRV_PWM_CH_FILL,…)`；清锁存命令→`srv_pwr_ctrl_clear_latch()`。init 中 `drv_pwm_init()`（供 FILL 通道，幂等）。 |
| power_task.c/h | ADAPT | 1ms `srv_pwr_ctrl_step(1)`；10ms 分频 `app_fault_policy_step(10)`（沿用 master 分频结构）。init 接线母线电压回调（读 srv_adc_get_latest 缓存）。 |
| sample_task.c/h | COPY | 10ms `srv_adc_trigger()+srv_adc_step()`。 |
| led_task.c/h | COPY | master 原样（drv_led + srv_signal + app_status_indicator）。 |

不移植：fan_task；master 中 com_task 里的 buzzer/E-STOP 冗余回调一律删除。

### 4.5 docs/、根文件（见 §6、§7）

## 5. 核心语义设计

### 5.1 电源输出监督 FSM（srv_pwr_ctrl）

- 输入：`desired_mask`（由 0x10 覆盖写入；bit0=24V、bit1=12V_ISO、bit2=LSD1、bit3=LSD2）。
- 每路输出独立状态：`OFF → ENABLING → ON`；故障后 `LATCHED_OFF`（锁存位保留，不自动清除）。
- 门控（仅在 ENABLING 入口判）：24V 需 `aux_mv > SRV_PWR_AUX_PRESENT_MV`；12V_ISO 需 24V 已 ON；LSD1/2 需 24V 已 ON。
- 好状态（ready）判定：24V=EXT_PCOOG_24V 高、12V_ISO=ISO_PGOOD_12V 高、LSDx=使能后节点电压 `lsdX_mv < SRV_PWR_LSD_NODE_ON_MAX_MV`。
- 超时（ENABLING 内连续不满足 ready，宏默认：24V 800ms / 12V_ISO 500ms / LSD 100ms）→ 关断该路 + 置锁存位。
- API：`init(config)/step(elapsed_ms)/request_outputs(mask)/get_outputs()/latch_outputs(mask)/clear_latch()/get_latch_mask()/is_output_enabled(out)/is_any_latched()`。母线电压读取经 config 注入回调（返回 aux/motor/lsd1/lsd2 的 mV + valid），由 power_task 实现。
- 规则：显式 0x10 命令把某锁存路期望置 0，视为人工关断 → 顺带清该路锁存；`clear_latch()`（0x11）清全部锁存，FSM 依当前 desired 自动重试。
- 阈值宏（待实机确认）：`SRV_PWR_AUX_PRESENT_MV`≈20000（迟滞 ±2000）、`SRV_PWR_MOTOR_PRESENT_MV`≈20000、`SRV_PWR_LSD_NODE_ON_MAX_MV`≈3000、`SRV_PWR_READY_DEBOUNCE_MS`≈10、`SRV_PWR_LOSS_DEBOUNCE_MS`=100（防误关断）、各路使能超时如上述。

### 5.2 app_main 初始化顺序

1) `delay_init()` → 2) `log_task_init()`（banner `E1_SLAVER_POWER_CTU`）→ 3) `com_task_init()`（dev_rs485 + drv_pwm + srv_com_slv 三回调接线 + srv_pwr_det_init）→ 4) `led_task_init()` → 5) `sample_task_init()` → 6) `power_task_init()`（srv_pwr_ctrl + app_fault_policy + 母线电压回调接线）→ 7) `for(;;){ sw_timer_tick(millis()); sw_timer_task(); }`。`Core/Src/main.c` USER CODE 调 `app_main()`。

## 6. CMake / Core / .ioc 修改清单

1. **`.ioc`（CubeMX 修改后重新生成）**：
   - PA2 `LSD1_IN`、PA3 `LSD2_IN`：`GPIO_Output`，初始电平低（默认全关）。
   - NVIC 使能 `USART3_IRQn`（drv_uart RX IDLE/错误回调必需）；`USART1_IRQn` 一并使能（与 master 对齐，无害）。
   - 重新生成后：gpio.c 初值写 PA0/PA2/PA3/PA4 全低、PC5 低；it.c 自动出现 USART1/3 IRQHandler。
2. **`Core/Inc/stm32f1xx_hal_conf.h`**：`USE_HAL_UART_REGISTER_CALLBACKS 0U → 1U`（drv_uart per-instance 回调前提）。
3. **`Core/Src/main.c`**：仅 USER CODE——includes 加 `app_main.h`、USER CODE 2 调 `app_main()`。
4. **`Core/Src/stm32f1xx_it.c`**：若重生成未自动加入，手工在相应区加 `USART1/3_IRQHandler`（`HAL_UART_IRQHandler(&huart1/&huart3)`）+ 顶部 extern `huart1/huart3`；DMA1_Channel1..5 处理器 slave 已生成无需改。
5. **根 `CMakeLists.txt`**（本工程自身）：仿 master 增加 `aux_source_directory(tasks|device_drivers|service|applications)`、`add_subdirectory(../public_layer/m_middlewares m_middlewares)`、include `../public_layer/service`、链接 `m_middlewares`、源加 `../public_layer/service/srv_signal.c`、编译宏 `PRINTF_DISABLE_SUPPORT_FLOAT`/`PRINTF_DISABLE_SUPPORT_EXPONENTIAL`、POST_BUILD objcopy 出 `.hex/.bin`。其余保持骨架内容。
6. `docs/hardware_pin.md`：修正与 main.h 的出入（RS485_EN=PC5、LED_PWM=PB6、LSD1/2_IN 为输出、FILL=PB8），供后续硬件核对。
7. （可选）新增 `README.md`（简短：定位/资源/构建/版本表头），可仿 master。

## 7. RS485 协议（docs/protocol_slaver_485.md，草案待联调）

信封与 master 同构：`[ 'z' ][cmd][data_len][payload...][CRC8 0x31/初值0xFF][ '\n' ]`，总长=len+5，多字节小端；应答 cmd=`0x80|下行`；错误应答 `0x7F`，错误码 0x00 无错/0x01 未知命令/0x02 暂不支持/0x03 长度不匹配。

| 下行 | 帧 | 请求负载 | 应答 |
|---|---|---|---|
| 0x01 | 读系统状态 | 0 | `0x81` 2B：byte0 故障位 = bit0 err_24v、bit1 err_12v、bit2 err_aux(缺失)、bit3 err_motor(缺失)、bit4 err_lsd1、bit5 err_lsd2、bit6-7 保留；byte1 = bit0 24V 实际输出、bit1 12V_ISO、bit2 LSD1、bit3 LSD2、bit4 有故障锁存、其余保留 |
| 0x02 | 读电压 | 0 | `0x82` 8B：aux_mv/motor_mv/lsd1_mv/lsd2_mv (uint16 LE) |
| 0x03 | 读温度/VDDA | 0 | `0x83` 4B：mcu_temp_x100 (int16 LE)、vdda_mv (uint16 LE) |
| 0x10 | 输出控制 | 4B：`[ctrl] [0x00] [fill_lo] [fill_hi]`；ctrl 位=bit0 24V、bit1 12V_ISO、bit2 LSD1、bit3 LSD2（整帧覆盖期望态）；fill=补光亮度 0..1000 (uint16 LE) | `0x90` 1B {0x00} ACK |
| 0x11 | 清除故障锁存 | 1B {0x01} | `0x91` 1B {0x00} |
| 0x1F | 升级请求（预留） | 1B {0x01} | `0x7F` err=0x02 |

软件映射：`srv_com_slv`（protocol_parser/packer，三回调）、`com_task`（传输搬运）、`app_status_report_fill`（read_data）。故障位含义：某轨锁存/或期望开却 PGOOD 长时间不满足视为对应 err 位。命令码为草案，联调时可再调整。

## 8. 实现顺序

1. 建目录 + 从 master 工程拷基础 COPY 文件（drv_systick/drv_uart/dev_rs485/drv_log_uart/drv_led/sample_task/log_task/led_task）。
2. device_drivers 适配：drv_pwm → drv_power → drv_status →（drv_adc 若需独立则先建，见注）。
3. service：srv_adc → srv_pwr_det → srv_pwr_ctrl → srv_com_slv。
4. applications：app_status_report → app_fault_policy → app_status_indicator。
5. tasks + app_main + com_task/power_task 接线。
6. docs/protocol_slaver_485.md + hardware_pin.md 修订 + README。
7. .ioc（PA2/3 输出 + NVIC USART1/3）重生成 + hal_conf + main.c/it.c USER CODE + 根 CMakeLists。
8. 构建验证。

> 注：srv_adc 直接由 drv_adc 驱动。drv_adc 路由/通道实现存在 master `drv_adc.c` 内，slave 需按其骨架改写为 6 Rank（IN10~IN13+TEMP+VREF）。若实现中发现 master 的 `drv_adc.c` 内嵌「Rank 数/通道名/CD4051B 位」较多，可只保留下层触发/读回/回调骨架，把通道表改 6 项。

## 9. 验证

- 编译链接：`cmake --preset Debug` + `cmake --build --preset Debug`，产物 `build/Debug/E1_SLAVER_POWER_CTU.elf/.bin/.hex/.map`；无编译错误/新增警告；map 查 RAM ≤48KB。
- clangd：先 Debug 构建再索引（`.clangd` 指向 build/Debug）。
- 静态自查：RX 路径无日志回灌；drv_uart 初始化成功需 USART3 IRQ + hal_conf 回调开关；drv_pwm 20kHz 组频；ADC Rank 与 srv_adc 换算顺序一致。
- 无法上电验证项（极性/阈值/时序/命令码）集中为宏并在文档标注「待实机确认」。

## 10. 风险与待实机确认

1. LSD 输出节点故障判定阈值与方向（低边开关：ON 时节点应近 0V；节点拉高=开关开路/外部短路），实机核对。
2. AUX/MOTOR 存在阈值 20V 为默认；LM5146 最低工作输入未知，需按输入范围调整。
3. PGOOD 去抖 10ms / 运行时丢失 100ms / 使能超时 800/500/100ms 为初始默认。
4. 协议命令码/位域为草案，与上位机联调后可调。
5. RS485_EN、LED_PWM、LSD 方向与 doc 的出入已按 main.h/.ioc 修正（决策 #4）。
6. 20kHz 为 LED 与补光共用组频（TIM4），LED 呼吸/闪烁走占空比无影响。

## 11. 明确不做（范围外）

- 自主顺序上电、E-STOP/冗余、风扇/蜂鸣器/温度采集 NTC、CD4051B、flash 参数、bootloader/485 升级。
- 修改 public_layer 或其他工程 CMake/源码。
- master 工程（E1_MASTER_POWER_CTU）侧任何改动。
