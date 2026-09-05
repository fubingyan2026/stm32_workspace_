该原理图（副电源管理模块）以主控 MCU（**U201 STM32F103RCT7**）为核心，各引脚功能属性及 ADC 分压比详细整理如下：

> 引脚宏/方向以 CubeMX 工程（`.ioc` / `Core/Inc/main.h`）为准，本文档仅作物理描述参考；如有出入以 main.h 为准。

---

### 一、MCU (STM32F103RCT7) 各引脚功能属性

#### 1. 模拟采集引脚（ADC）及分压比
* **Pin 8 (PC0 / ADC12_IN10) — `AUX_POWER_ADC`**
  * **功能属性**：模拟输入（ADC），用于检测副电源（AUX_POWER，标称 48V）电压。
  * **分压网络**：上分压 $R_{107} = 220\,\text{k}\Omega$（1%），下分压 $R_{110} = 10\,\text{k}\Omega$（1%），滤波电容 $C_{114} = 100\,\text{nF}$。
  * **分压比**：
    $$\frac{V_{\text{ADC}}}{V_{\text{AUX}}} = \frac{R_{110}}{R_{107} + R_{110}} = \frac{10\,\text{k}\Omega}{220\,\text{k}\Omega + 10\,\text{k}\Omega} = \frac{1}{23} \approx 0.04348$$
  * **电压计算公式**：$V_{\text{AUX}} = 23 \times V_{\text{ADC}}$（ADC 满量程 3.3V 时对应最大测量电压约 **75.9V**）。

* **Pin 9 (PC1 / ADC12_IN11) — `MOTOR_POWER_ADC`**
  * **功能属性**：模拟输入（ADC），用于检测电机电源（MOTOR_POWER，标称 48V）电压。
  * **分压网络**：上分压 $R_{108} = 220\,\text{k}\Omega$（1%），下分压 $R_{111} = 10\,\text{k}\Omega$（1%），滤波电容 $C_{115} = 100\,\text{nF}$。
  * **分压比**：
    $$\frac{V_{\text{ADC}}}{V_{\text{MOTOR}}} = \frac{R_{111}}{R_{108} + R_{111}} = \frac{10\,\text{k}\Omega}{220\,\text{k}\Omega + 10\,\text{k}\Omega} = \frac{1}{23} \approx 0.04348$$
  * **电压计算公式**：$V_{\text{MOTOR}} = 23 \times V_{\text{ADC}}$（ADC 满量程 3.3V 时对应最大测量电压约 **75.9V**）。

* **Pin 10 (PC2 / ADC12_IN12) — `LSD1_ADC`**
  * **功能属性**：模拟输入（ADC），用于检测通道 1 低边驱动输出端（LSD1，24V 回路）状态/对地电压。
  * **分压网络**：上分压 $R_{605} = 100\,\text{k}\Omega$（1%），下分压 $R_{607} = 10\,\text{k}\Omega$（1%），滤波电容 $C_{603} = 100\,\text{nF}$。
  * **分压比**：
    $$\frac{V_{\text{ADC}}}{V_{\text{LSD1}}} = \frac{R_{607}}{R_{605} + R_{607}} = \frac{10\,\text{k}\Omega}{100\,\text{k}\Omega + 10\,\text{k}\Omega} = \frac{1}{11} \approx 0.09091$$
  * **电压计算公式**：$V_{\text{LSD1}} = 11 \times V_{\text{ADC}}$（ADC 满量程 3.3V 时对应最大测量电压约 **36.3V**）。

* **Pin 11 (PC3 / ADC12_IN13) — `LSD2_ADC`**
  * **功能属性**：模拟输入（ADC），用于检测通道 2 低边驱动输出端（LSD2，24V 回路）状态/对地电压。
  * **分压网络**：上分压 $R_{606} = 100\,\text{k}\Omega$（1%），下分压 $R_{608} = 10\,\text{k}\Omega$（1%），滤波电容 $C_{604} = 100\,\text{nF}$。
  * **分压比**：
    $$\frac{V_{\text{ADC}}}{V_{\text{LSD2}}} = \frac{R_{608}}{R_{606} + R_{608}} = \frac{10\,\text{k}\Omega}{100\,\text{k}\Omega + 10\,\text{k}\Omega} = \frac{1}{11} \approx 0.09091$$
  * **电压计算公式**：$V_{\text{LSD2}} = 11 \times V_{\text{ADC}}$（ADC 满量程 3.3V 时对应最大测量电压约 **36.3V**）。

---

#### 2. 电源控制与状态监测引脚（GPIO）
* **Pin 14 (PA0) — `DC-DC_EN`**：推挽输出（GPIO Output），控制板载 24V/6A Buck 电源模块（LM5146）的使能端（高电平使能开启，低电平关闭）。
* **Pin 15 (PA1) — `24V_EXT_PGOOD`**：输入（GPIO Input），检测 LM5146 输出电源状态（高电平表示 24V 输出正常）。
* **Pin 16 (PA2) — `LSD1_IN`**：推挽输出（GPIO Output），控制低边驱动开关 Q601（ZXMS6004FFTA）栅极（高电平开通，低电平关断）。
* **Pin 17 (PA3) — `LSD2_IN`**：推挽输出（GPIO Output），控制低边驱动开关 Q602（ZXMS6004FFTA）栅极。
* **Pin 20 (PA4) — `12V_ISO_EN`**：推挽输出（GPIO Output），控制隔离 12V 模块（URB2412S-6WR3）的 CTRL 引脚（**拉低使能，拉高关闭**；实测确认，与光耦/驱动级反相有关）。
* **Pin 21 (PA5) — `12V_ISO_PGOOD`**：输入（GPIO Input），经比较器 LM193 与光耦隔离反馈，检测 12V_ISO 是否在正常窗口范围（11.4V ~ 12.6V）（**高电平正常，低电平异常**；实测确认）。
* **Pin 24 (PC5) — `RS485_EN`**：推挽输出（GPIO Output），控制 RS485 收发器（SP3485）的收发切换控制引脚。
* **Pin 59 (PB6) — `LED_PWM`**：TIM4_CH1 PWM 输出（状态灯，经数字三极管 Q201/DTC143ZCA 驱动板载蓝色 LED201，亮度由占空比控制；TIM4 组频率 20kHz）。
* **Pin 61 (PB8) — `FILL_LED_PWM`**：TIM4_CH3 PWM 输出（推荐 20kHz），输出至第 7 页补光灯驱动芯片 PT4115 的 DIM 端，经板载 RC 滤波转为模拟电平以调节补光灯电流（0 ~ 1A）。

---

#### 3. 通信与调试接口引脚
* **Pin 29 (PB10) — `RS485_TX`**：USART3_TX，RS485 发送引脚，接 SP3485 DI 端。
* **Pin 30 (PB11) — `RS485_RX`**：USART3_RX，RS485 接收引脚，接 SP3485 RO 端。
* **Pin 42 (PA9) — `USART1_TX`**：USART1_TX，调试串口发送，引至连接器 CN201。
* **Pin 43 (PA10) — `USART1_RX`**：USART1_RX，调试串口接收，引至连接器 CN201。
* **Pin 46 (PA13) — `SWDIO`**：SWD 数据调试线，引至烧录接口 U202。
* **Pin 49 (PA14) — `SWCLK`**：SWD 时钟调试线，引至烧录接口 U202。

---

#### 4. 系统电源、时钟与启动配置引脚
* **Pin 1 (VBAT)**：接系统 3.3V 电源。
* **Pin 5 (PD0) / Pin 6 (PD1)**：外接 8MHz HSE 主晶振 X201（匹配电容 20pF）。
* **Pin 7 (NRST)**：主芯片硬件复位引脚（外接 10k 上拉、100nF 下拉及复位按键 SW201）。
* **Pin 12 (VSSA) / Pin 13 (VDDA)**：模拟地与模拟电源（3.3V 经磁珠 L201 滤波）。
* **Pin 18, 31, 47, 63 (VSS)**：系统数字接地 GND。
* **Pin 19, 32, 48, 64 (VDD)**：系统 3.3V 数字供电（并联去耦电容）。
* **Pin 28 (PB2/BOOT1) & Pin 60 (BOOT0)**：启动配置引脚，分别接 10k 上拉/下拉电阻网络。

---

### 二、板级主要接口连接器（Connector）引脚功能

1. **CN201 (USART1 串口调试口，GH1.25-3P)**：
   * Pin 1: `USART1_TX`
   * Pin 2: `USART1_RX`
   * Pin 3: `GND`
2. **U202 (SWD 固件烧录接口，GH1.25-5P)**：
   * Pin 1: `3V3`
   * Pin 2: `SWCLK`
   * Pin 3: `GND`
   * Pin 4: `SWDIO`
   * Pin 5: `NRST`
3. **CN301 (RS485 通信口，5023520300-3P)**：
   * Pin 1: `RS485_A`
   * Pin 2: `RS485_B`
   * Pin 3: `GND`
4. **CN601 (24V 低边开关输出口，Micro-Fit 4P)**：
   * Pin 1: `LSD1`（低边驱动输出 1）
   * Pin 2: `LSD2`（低边驱动输出 2）
   * Pin 3: `24V_EXT`（24V 输出）
   * Pin 4: 空脚（NC）
5. **CN701 (补光灯接口，Micro-Fit 4P)**：
   * Pin 1: `FILL_LED-`
   * Pin 2: `GND_ISO`
   * Pin 3: `FILL_LED+`
   * Pin 4: `12V_ISO`