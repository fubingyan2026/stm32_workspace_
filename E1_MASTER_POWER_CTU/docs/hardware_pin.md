# MCU（U201 STM32F103RCT7）引脚功能汇总表

| 引脚编号 (Pin Number) | 引脚名称 (Pin Name) | 网络名称/描述符号 (Net Name) | 引脚性质 (Type) | 功能描述 (Description) |
| :--- | :--- | :--- | :--- | :--- |
| 1 | VBAT | 3V3 | PWR | 芯片后备供电引脚，连接至系统 3.3V 主电源。 |
| 2 | PC13-TAMPER-RTC | `-` | NC | 未使用 / 悬空。 |
| 3 | PC14-OSC32_IN | `-` | NC | 未使用 / 悬空（低速外部晶振输入引脚）。 |
| 4 | PC15-OSC32_OUT | `-` | NC | 未使用 / 悬空（低速外部晶振输出引脚）。 |
| 5 | PD0-OSC_IN | OSC_IN | OSC | 主时钟输入引脚，连接外部 8MHz 高速无源晶振 (X201)。 |
| 6 | PD1-OSC_OUT | OSC_OUT | OSC | 主时钟输出引脚，连接外部 8MHz 高速无源晶振 (X201)。 |
| 7 | NRST | NRST | SYS | 芯片主硬件复位输入引脚，低电平有效，连接外部复位按键 (SW201) 及 RC 滤波电路。 |
| 8 | PC0 | NTC1_ADC | ADC | **模拟 ADC 采样输入（外部 NTC 测温）：**<br>• **分压偏置电路**：上偏置电阻 $R_{401} = 10\text{ k}\Omega$（上拉至 3.3V），下臂接入外部 NTC 电阻；后级经 $R_{403} = 22\text{ k}\Omega$ 限流及 $C_{402} = 100\text{ nF}$ 滤波。<br>• **采样分压公式**：$$V_{\text{ADC}} = 3.3\text{V} \times \frac{R_{\text{NTC1}}}{R_{\text{NTC1}} + 10\text{ k}\Omega}$$ |
| 9 | PC1 | NTC2_ADC | ADC | **模拟 ADC 采样输入（DC-DC MOS 管测温）：**<br>• **分压偏置电路**：上偏置精密电阻 $R_{402} = 10\text{ k}\Omega\ (\pm 1\%)$（上拉至 3.3V），下臂为板载贴片 NTC $R_{404}$（标称 $10\text{ k}\Omega, B=3950\text{K}$），并联 $C_{403} = 100\text{ nF}$ 滤波。<br>• **采样分压公式**：$$V_{\text{ADC}} = 3.3\text{V} \times \frac{R_{\text{NTC2}}}{R_{\text{NTC2}} + 10\text{ k}\Omega}$$（$25^\circ\text{C}$ 常温时分压比为 $1:2$，即采样电压约为 $1.65\text{ V}$）。 |
| 10 | PC2 | VIN_DC-DC_ADC | ADC | **模拟 ADC 采样输入（DC-DC 输入端电压监测）：**<br>• **电阻分压网络**：上分压电阻 $R_{936} = 220\text{ k}\Omega\ (\pm 1\%)$，下分压电阻 $R_{941} = 10\text{ k}\Omega\ (\pm 1\%)$，滤波电容 $C_{938} = 100\text{ nF}$。<br>• **分压比与公式**：$$\text{分压比} = \frac{10\text{ k}\Omega}{220\text{ k}\Omega + 10\text{ k}\Omega} = \frac{1}{23} \approx 0.04348$$$$V_{\text{ADC}} = V_{\text{IN\_DC-DC}} \times \frac{1}{23}$$（满量程 $3.3\text{ V}$ 对应最高输入监测电压约 $75.9\text{ V}$）。 |
| 11 | PC3 | VIN_ADC | ADC | **模拟 ADC 采样输入（系统总输入主母线电压监测）：**<br>• **电阻分压网络**：上分压电阻 $R_{935} = 220\text{ k}\Omega\ (\pm 1\%)$，下分压电阻 $R_{940} = 10\text{ k}\Omega\ (\pm 1\%)$，滤波电容 $C_{937} = 100\text{ nF}$。<br>• **分压比与公式**：$$\text{分压比} = \frac{10\text{ k}\Omega}{220\text{ k}\Omega + 10\text{ k}\Omega} = \frac{1}{23} \approx 0.04348$$$$V_{\text{ADC}} = V_{\text{IN}} \times \frac{1}{23}$$（用于主电源 $36\text{V}\sim58\text{V}$ 上电范围校验及欠过压保护判定）。 |
| 12 | VSSA | GND | PWR | 模拟地 (Analog Ground)。 |
| 13 | VDDA | VDDA | PWR | 模拟供电正电源引脚 (3.3V)，由 3.3V 主电经磁珠滤波网络供给。 |
| 14 | PA0-WKUP | FAN0_FG_IO | IN | 数字脉冲输入/定时器捕获引脚，检测内部机箱散热风扇 (FAN0) 的转速反馈 (FG) 信号。 |
| 15 | PA1 | FAN1_FG_IO | IN | 数字脉冲输入/定时器捕获引脚，检测外部散热风扇 (FAN1) 的转速反馈 (FG) 信号。 |
| 16 | PA2 | `-` | NC | 未使用 / 悬空。 |
| 17 | PA3 | `-` | NC | 未使用 / 悬空。 |
| 18 | VSS_4 | GND | PWR | 数字地 (Digital Ground)。 |
| 19 | VDD_4 | 3V3 | PWR | 数字正电源供电引脚 (3.3V)。 |
| 20 | PA4 | CD4051B_A | OUT | 数字输出引脚，急停状态采集多路复用器 (CD4051BPWR) 通道地址选择位 A。 |
| 21 | PA5 | CD4051B_B | OUT | 数字输出引脚，急停状态采集多路复用器 (CD4051BPWR) 通道地址选择位 B。 |
| 22 | PA6 | CD4051B_C | OUT | 数字输出引脚，急停状态采集多路复用器 (CD4051BPWR) 通道地址选择位 C。 |
| 23 | PA7 | `-` | NC | 未使用 / 悬空。 |
| 24 | PC4 | CD4051B_ADC | ADC | **模拟 ADC 采样输入（急停回路状态轮询）：**<br>• **拓扑特性**：连接至 8 选 1 模拟多路复用器 (CD4051B) 的公共输出端 (OUTIN)。<br>• **分压比**：直通选通采集（分压传输比为 $1:1$），轮询输入各急停回路节点电平（$0\sim3.3\text{ V}$），配合比较器基准进行故障诊断。 |
| 25 | PC5 | RS485_EN | OUT | 数字输出引脚，RS485 收发器 (SP3485EN) 方向收发使能控制端 (DE/!RE)。 |
| 26 | PB0 | BUZZ_PWM | OUT | 数字/PWM 输出引脚，通过驱动 MOS 管 (Q202) 控制报警蜂鸣器 (BUZZER201)。 |
| 27 | PB1 | `-` | NC | 未使用 / 悬空。 |
| 28 | PB2 | BOOT1 | SYS | 芯片启动配置引脚 BOOT1，板载下拉电阻默认配置从主 Flash 启动。 |
| 29 | PB10 | RS485_TX | COMM | 串口通信引脚，复用为 USART3_TX，连接至 RS485 收发芯片 (SP3485EN) 发送输入端 (DI)。 |
| 30 | PB11 | RS485_RX | COMM | 串口通信引脚，复用为 USART3_RX，连接至 RS485 收发芯片 (SP3485EN) 接收输出端 (RO)。 |
| 31 | VSS_1 | GND | PWR | 数字地 (Digital Ground)。 |
| 32 | VDD_1 | 3V3 | PWR | 数字正电源供电引脚 (3.3V)。 |
| 33 | PB12 | `-` | NC | 未使用 / 悬空。 |
| 34 | PB13 | `-` | NC | 未使用 / 悬空。 |
| 35 | PB14 | `-` | NC | 未使用 / 悬空。 |
| 36 | PB15 | `-` | NC | 未使用 / 悬空。 |
| 37 | PC6 | `-` | NC | 未使用 / 悬空。 |
| 38 | PC7 | `-` | NC | 未使用 / 悬空。 |
| 39 | PC8 | `-` | NC | 未使用 / 悬空。 |
| 40 | PC9 | `-` | NC | 未使用 / 悬空。 |
| 41 | PA8 | AUX_POWER_EN | OUT | 数字输出引脚，辅助电源 (AUX_POWER) 热插拔驱动控制器 (LM5069) 使能控制信号。 |
| 42 | PA9 | USART1_TX | COMM | 串口通信引脚，复用为 USART1_TX，引出至外部通信/调试接口端子 (CN201)。 |
| 43 | PA10 | USART1_RX | COMM | 串口通信引脚，复用为 USART1_RX，引出至外部通信/调试接口端子 (CN201)。 |
| 44 | PA11 | VIN_DC-DC_EN | OUT | 数字输出引脚，DC-DC 供电开关控制器 (LM5060) 功率输入前端使能控制信号。 |
| 45 | PA12 | 12V_PGOOD | IN | 数字输入引脚，板载 12V Buck 降压电源芯片 (SY8513FCC) 电源良好指示信号 (Power Good)。 |
| 46 | PA13 | SWDIO | DEBUG | SWD 调试通信引脚，串行调试数据输入/输出信号，引出至烧录调试座 (U202)。 |
| 47 | VSS_2 | GND | PWR | 数字地 (Digital Ground)。 |
| 48 | VDD_2 | 3V3 | PWR | 数字正电源供电引脚 (3.3V)。 |
| 49 | PA14 | SWCLK | DEBUG | SWD 调试通信引脚，串行调试时钟信号，引出至烧录调试座 (U202)。 |
| 50 | PA15 | `-` | NC | 未使用 / 悬空。 |
| 51 | PC10 | MOTOR_POWER_PGD | IN | 数字输入引脚，电机电源 (MOTOR_POWER) 热插拔芯片 (LM5069) 电源良好指示信号 (Power Good)。 |
| 52 | PC11 | MOTOR_POWER_EN | OUT | 数字输出引脚，电机电源 (MOTOR_POWER) 热插拔控制器 (LM5069) 主使能控制信号。 |
| 53 | PC12 | AUX_POWER_PGD | IN | 数字输入引脚，辅助电源 (AUX_POWER) 热插拔芯片 (LM5069) 电源良好指示信号 (Power Good)。 |
| 54 | PD2 | `-` | NC | 未使用 / 悬空。 |
| 55 | PB3 | `-` | NC | 未使用 / 悬空。 |
| 56 | PB4 | `-` | NC | 未使用 / 悬空。 |
| 57 | PB5 | DC-DC_24V_PGOOD | IN | 数字输入引脚，24V/20A 大功率同步降压芯片 (MP9931N) 电源良好指示信号 (Power Good)。 |
| 58 | PB6 | DC-DC_24V_EN | OUT | 数字输出引脚，24V/20A 大功率同步降压电源 (MP9931N) 运行使能控制信号。 |
| 59 | PB7 | LED_PWM | OUT | 数字/PWM 输出引脚，通过三极管 (Q201) 驱动板载系统运行状态指示灯 (LED201)。 |
| 60 | BOOT0 | BOOT0 | SYS | 芯片启动配置引脚 BOOT0，板载下拉电阻默认配置从主 Flash 启动。 |
| 61 | PB8 | FAN0_PWM_IO | OUT | PWM 输出引脚，内部机箱散热风扇 (FAN0) PWM 调速驱动控制信号。 |
| 62 | PB9 | FAN1_PWM_IO | OUT | PWM 输出引脚，外部散热风扇 (FAN1) PWM 调速驱动控制信号。 |
| 63 | VSS_3 | GND | PWR | 数字地 (Digital Ground)。 |
| 64 | VDD_3 | 3V3 | PWR | 数字正电源供电引脚 (3.3V)。 |