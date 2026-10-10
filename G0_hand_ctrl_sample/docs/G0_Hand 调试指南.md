# G0_Hand 调试指南

## 调试通道

- **USART1（PA9/PA10）@115200**：日志控制台，用串口助手查看 `LOG_I/W/E`。
- **USART2（PA2/PA3）@1000000**：上位机命令/协议口（`host/g0_host.py`）。
- SWD：PA13=SWDIO / PA14=SWCLK，挂 `build/RelWithDebInfo/*.elf` 在线调试。

> 排查时先确认连的是哪个口、用的哪个波特率；连错口或波特率不对只会看到乱码。

---

## 调试项

### 日志
- 无输出：确认为 USART1@115200；检查是否 `LOG_ENABLED=0` 或级别被设为 NONE；SWD 看是否停在 HardFault；新增文件后是否删过 `build/`。
- 级别调整：`tasks/log_task.c` 的 `log_set_level()`。
- 日志禁用浮点格式化（F1 无 FPU），不要用 `%f/%e`。

### 上位机 / 心跳
- 连不上或收不到 0x00 心跳：波特率必须 1000000；确认程序已跑过 `app_uart_interact_init()`（日志有 `uart_cmd_task: UART 命令任务初始化完成`）。

### 协议命令
- 无反应：日志口看是否收到对应命令；核对 CRC8（reflected 0x8C/init 0xFF）、帧尾 `\n`、帧头 `z`(0x7A)；核对载荷长度（0x02=20B，0x05 BLINK 才 4B）。

### 电机（DM4310，CAN ID 0x01）
- 不动：日志确认有“使能帧已发送…开始下发控制帧”；查 CANH/CANL、120Ω 终端、CAN1 1 Mbps/采样点 75%。
- 位置不变：目标被夹在 `SRV_USER_POS_MIN~MAX`（0.0~0.8）；位置为斜坡过渡到目标。
- 反馈异常：上位机发 0x03，看 0x04 的 state 字段。
- 在线/掉线：`daemon_task` 每 1s 打印电机守护在线状态与喂狗频率。
- 保存零位：按住 KEY1+KEY2 3s（日志“已保存电机当前位置为零点”）。

### eFuse 24V
- 看 `srv_efuse` 的“状态切换 → …(SHDN=.. PGOOD=.. FLT_N=..)”，对照引脚电平判断是 PGOOD 拉低还是 FLT_N 触发。
- 进入 FAULT：`FLT_N=1` 时 1s 后自动恢复上电；`FLT_N=0`（持续过流/短路）则打 WARN 等待，不自动恢复。
- 掉电关断：PGOOD 持续拉低 3500ms 触发关断。

### LED
- 不亮/颜色不对：确认已下发 0x05 且 action/间隔正确；示波器量 PWM 引脚（蓝=PA8、绿=PB6、红=PB7）。
- 呼吸常暗：`srv_signal_set_state` 切换状态时需保留 breath min/max 参数。

### 按键
- 无响应：KEY1=PB9 / KEY2=PB8，低电平有效，量引脚电平是否翻转；只有 PRESS/RELEASE 上报为 0x01。

### ADC
- 电压异常：VIN=PA7(IN7)、V_IMON=PB0(IN8)；打开 `srv_adc` 的 `SRV_ADC_LOG_ENABLE` 可每 1s 打印遥测；换算按 VREF=3300mV/4095，先核对分压比。
