# Plan: G0_hand_ctrl_sample 架构适配（参照 E1_Hand_G474）

## 目标

把 `G0_hand_ctrl_sample`（STM32F103C8Tx，Cortex-M3，64KB Flash/20KB RAM，目前只有 CubeMX 外设初始化、无应用代码）适配为与 `E1_Hand_G474` 相同的分层架构：`device_drivers/` + `tasks/` + `service/` + 引入 `public_layer/m_middlewares` + 共享 service/task + CMakeLists 接入。CAN 协议先搭骨架占位；日志 Flash 落盘完整启用（需为 hal_flash 新增 F1 驱动）；添加 build.bat。

## 关键结论（已调研确认）

- **G0 外设**（来自 `.ioc` + `Core/Src/*.c`）：CAN1 经典 bxCAN 1Mbps（PA11/PA12，`hcan` 句柄，CubeMX 已使能 RX0/RX1/SCE 中断）、USART2 115200（PA2/PA3，RX DMA circular + TX DMA normal，即控制台串口）、ADC1 2ch（PA7=ADC1_IN7 VIN_ADC，PB0=ADC1_IN8 V_IMON，DMA normal）、TIM1 CH1 PWM（PA8）、TIM4 CH1/CH2 PWM（PB6/PB7）、KEY1/KEY2（PB9/PB8）、eFuse GPIO。
- **F1 HAL 兼容性**：`HAL_UARTEx_ReceiveToIdle_DMA` / `HAL_UARTEx_RxEventCallback` / `HAL_UARTEx_GetRxEventType` 均存在（stm32f1xx_hal_uart.h:762/784/764）。F1 无 `UART_CLEAR_OREF/NEF/FEF/PEF`、无 `UART_RXDATA_FLUSH_REQUEST`（F4/G4 特有），需用 `__HAL_UART_CLEAR_OREFLAG/NEFLAG/FEFLAG/PEFLAG` 清标志。F1 bxCAN API 与 F4 相同（`HAL_CAN_ConfigFilter/Start/AddTxMessage/GetRxMessage/ActivateNotification`）。
- **G0 `USE_HAL_UART_REGISTER_CALLBACKS=0`**：只有 1 路串口（USART2 控制台），采用 E1_Master_Power_Manage 的 drv_log_uart 全局 weak 回调风格（`HAL_UART_TxCpltCallback`/`HAL_UARTEx_RxEventCallback` 直接定义），无需改 hal_conf。
- **hal_flash 无 F1 驱动**：需新增 `drv_stm32f1_flash.c/.h`，并在 `hal_flash.h/.c` 加 `HAL_FLASH_CHIP_STM32F1` 分支。F1 页擦除 1KB（`FLASH_PAGE_SIZE=0x400`），`HAL_FLASHEx_Erase`（TypeErase=FLASH_TYPEERASE_PAGES, PageAddress, NbPages）+ `HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD)`。ring_storage 已有 `RING_STORAGE_SECTOR_1K`。
- **日志 Flash 区域**（G0 只有 64KB Flash）：预留末尾 8KB = `0x0800E000`，`SRV_LOG_FLASH_AREA_SIZE=8*1024`，`SRV_LOG_FLASH_SECTOR_SIZE=RING_STORAGE_SECTOR_1K`（8 扇区，≈49 条 WARN/ERROR）。
- **注意**：工作区约束「不要修改 CMakeLists.txt」针对 E1_Hand_G474；用户本次明确要求适配 G0 的 CMakeLists，属当前指令优先，仅改 `G0_hand_ctrl_sample/CMakeLists.txt`。

## 改动清单（按实现顺序）

### 1. 共享库：新增 F1 Flash 驱动（public_layer 原件）

文件：`public_layer/device_drivers/hal_flash/drv_stm32f1_flash.c` + `.h`

- `.h` 参照 `drv_stm32f4_flash.h`：`#ifdef HAL_FLASH_CHIP_STM32F1` 守卫，导出 `f1_sectors[]`、`f1_dev`。
- `.c` 参照 `drv_stm32f4_flash.c`：
  - `#ifdef HAL_FLASH_CHIP_STM32F1` 守卫；`#include "stm32f1xx_hal.h"`。
  - 扇区表：F103C8 64KB = 64 × 1KB 页（base=0x08000000 + 0x400×i）。
  - `f1_erase`：`HAL_FLASH_Unlock()` → 清 `FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR`（`__HAL_FLASH_CLEAR_FLAG`）→ `FLASH_EraseInitTypeDef{TypeErase=FLASH_TYPEERASE_PAGES, PageAddress, NbPages}` → `HAL_FLASHEx_Erase` → 读回校验 → `HAL_FLASH_Lock()`。
  - `f1_write`：32-bit 字粒度（与 F4 相同，含非 4B 尾部补 `0xFFFFFFFF` + 读回校验）。
  - `f1_read` 直读；`f1_erase_size_at` 返回 `0x400`；`cache_invalidate` 空实现。
  - caps：`addr=0x08000000`、`total_size=64*1024`、`erase_size=0x400`、`write_gran=HAL_FLASH_WRITE_GRAN_32`、`erase_size_uniform=true`、`has_ecc/wp/crc=false`。

修改 `public_layer/device_drivers/hal_flash/hal_flash.h`：
- 选型 #if 列表加入 `!defined(HAL_FLASH_CHIP_STM32F1)`；注释更新。

修改 `public_layer/device_drivers/hal_flash/hal_flash.c`：
- include 分支加 `#elif defined(HAL_FLASH_CHIP_STM32F1) #include "drv_stm32f1_flash.h"`。
- 设备实例分支加 `#elif defined(HAL_FLASH_CHIP_STM32F1) extern hal_flash_dev_t f1_dev; #define FLASH_DEV f1_dev`。

### 2. G0 device_drivers 层

新建目录 `G0_hand_ctrl_sample/device_drivers/`（5 个驱动，风格/头注释/日志开关宏对齐 E1_Hand）：

1. `drv_systick.c/.h` — 直接复用 E1_Hand 版本（SysTick 延时/`millis`/`micros`，`delay_init()`）。F1 SystemCoreClock=72MHz 自动适配。
2. `drv_log_uart.c/.h` — 基于 `E1_Master_Power_Manage/device_drivers/drv_log_uart.c`（全局 weak 回调风格）：
   - `LOG_HUART = &huart2`，`DRV_LOG_UART_RX_CIRC_BUF_SIZE = 256U`（RAM 受限）。
   - 清标志改用 `__HAL_UART_CLEAR_OREFLAG/NEFLAG/FEFLAG/PEFLAG`，**删除** `__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST)`（F1 无此宏）。
   - `drv_log_uart_init()` 内对 `huart2->hdmarx` 运行时切 circular 保留（CubeMX 已是 circular，幂等无害）。
   - `HAL_UARTEx_RxEventCallback` 内 `kfifo_move_in` 按 DMA 计数器同步（同 F4）。
3. `drv_can.c/.h` — 基于 `E1_Master_Power_Manage/device_drivers/drv_can.c`（经典 bxCAN）：
   - 句柄表 `[DRV_CAN_CH_1] = &hcan`（G0 是 `hcan` 不是 `hcan1`）。
   - **删除** STB 引脚控制（`P_CAN_STB` 不存在于 G0）。保留 filter 全通过、RX FIFO0 中断、`HAL_CAN_RxFifo0MsgPendingCallback`、TX 忙限频、`drv_can_is_bus_off/recover`。
   - 保留 `drv_can_tx_all_done`（bxCAN 3 邮箱）以便后续协议用。
4. `drv_adc.c/.h` — 新建单实例版本（F1 仅 ADC1）：
   - 句柄 `&hadc1`；`dma_buf[2]`；路由表 `VIN_ADC→idx0`、`V_IMON→idx1`（对应 CubeMX Rank1/2）。
   - `drv_adc_trigger()` 用 `HAL_ADC_Start_DMA(&hadc1, dma_buf, 2)`；`HAL_ADC_ConvCpltCallback` 清 busy。
   - 原始值接口 `drv_adc_read_raw(ch)`（12-bit，无 VREFINT 校准，srv 层做简单换算）。
5. `drv_pwm.c/.h` — 新建：封装 TIM1_CH1（PA8）+ TIM4_CH1/CH2（PB6/PB7），`drv_pwm_init()` 启动 PWM（`HAL_TIM_PWM_Start`），`drv_pwm_set_duty(ch, 0..1023)`（`__HAL_TIM_SET_COMPARE`），供 led_task 用。

### 3. G0 service 层

新建目录 `G0_hand_ctrl_sample/service/`：

1. `srv_can.c/.h` — **协议骨架占位**（用户已确认）：
   - 定义占位帧 ID 常量（如 `SRV_CAN_ID_CTRL 0x010` / `SRV_CAN_ID_ACK 0x011`，注释「待协议文档」）。
   - `srv_can_init()` / `srv_can_on_rx(const drv_can_msg_t*)` / `srv_can_process()` / `srv_can_send_heartbeat()`：on_rx 打印 RX 帧（限频）、process 空实现、heartbeat 发送一帧 0x010（带 tick），保证链路可验证。
2. `srv_adc.c/.h` — 薄换算层：`srv_adc_init/trigger/step/get_latest`；`vin_mv = raw*3300/4095`、`imon_raw` 透传；`step` 每周期 LOG_D 打印。
3. `srv_signal` / `srv_log_flash` 复用 public_layer 原件，不在本目录复制。

### 4. G0 task 层

新建目录 `G0_hand_ctrl_sample/tasks/`：

1. `app_main.c/.h` — 主入口（对齐 E1_Hand/E1_Master 模式）：
   ```c
   delay_init();
   log_task_init();          /* 内部含 drv_log_uart_init + log_init */
   srv_log_flash_init();     /* 依赖 log 已初始化 */
   can_task_init();
   led_task_init();
   sample_task_init();
   for (;;) { sw_timer_tick(millis()); sw_timer_task(); }
   ```
2. `can_task.c/.h` — 对齐 E1_Hand：`drv_can_init()` → `srv_can_init()` → 注册 `drv_can_register_rx_callback` → sw_timer 10ms：`drv_can_poll_status` + `srv_can_process()` + 周期心跳（`srv_can_send_heartbeat()` 1s）。RX 回调转发 `srv_can_on_rx`。
3. `led_task.c/.h` — 对齐 E1_Hand：`srv_signal_init(millis)`，注册 1 个呼吸 LED（TIM1_CH1），`write_output` → `drv_pwm_set_duty(DRV_PWM_TIM1_CH1, value)`；sw_timer 10ms 调 `srv_signal_task_refresh()`。
4. `sample_task.c/.h` — 对齐 E1_Master：sw_timer 10ms：`srv_adc_trigger()` + `srv_adc_step()`。

### 5. Core 接入

- `G0_hand_ctrl_sample/Core/Src/main.c`：USER CODE Includes 加 `#include "app_main.h"`；USER CODE 2 内 `MX_*_Init()` 之后调用 `app_main()`（对齐 E1_Hand）。
- 中断已就绪（`stm32f1xx_it.c` 已有 DMA1_CH1/CH6/CH7、CAN RX0/RX1/SCE、USART2、SysTick 处理器），无需改。

### 6. CMakeLists.txt 适配（仅 G0 根 CMakeLists，不改 cmake/stm32cubemx）

参照 `E1_Hand_G474/CMakeLists.txt`：
- `add_subdirectory(../public_layer/m_middlewares m_middlewares)`。
- `aux_source_directory`：`tasks`、`device_drivers`、`service`、`../public_layer/device_drivers/hal_flash`。
- `target_sources`：`${DEV_DRV} ${DEV_DRV_HAL_FLASH} ${SERVICE} ${TASKS}` + 共享 `../public_layer/service/srv_signal.c`、`../public_layer/service/srv_log_flash.c`、`../public_layer/task/log_task.c`。
- `target_include_directories`：`tasks/device_drivers/service` + `../public_layer/device_drivers/hal_flash`、`../public_layer/service`、`../public_layer/task`。
- `target_compile_definitions`：
  ```
  HAL_FLASH_CHIP_STM32F1
  PRINTF_DISABLE_SUPPORT_FLOAT
  PRINTF_DISABLE_SUPPORT_EXPONENTIAL
  SRV_LOG_FLASH_AREA_START=0x0800E000U
  SRV_LOG_FLASH_AREA_SIZE=8*1024U
  SRV_LOG_FLASH_SECTOR_SIZE=RING_STORAGE_SECTOR_1K
  ```
- `target_link_libraries(... stm32cubemx m_middlewares)`。
- POST_BUILD hex/bin（`${CMAKE_OBJCOPY}`，同 E1_Hand）。

### 7. build.bat + .clangd

- 复制 `E1_Hand_G474/build.bat` 到 `G0_hand_ctrl_sample/build.bat`（自动探测 `%LOCALAPPDATA%\stm32cube\bundles`；默认 RelWithDebInfo）。
- 更新 `G0_hand_ctrl_sample/.clangd`：`CompilationDatabase: build/RelWithDebInfo`（与 build.bat 默认一致，对齐 E1_Hand）。

## 验证

1. `G0_hand_ctrl_sample/build.bat`（或 `build.bat -t Debug`）编译零错误零警告。
2. 检查 `build/RelWithDebInfo/G0_hand_ctrl_sample.elf/.hex/.bin` 生成。
3. 串口 115200（USART2 PA2/PA3）看到启动横幅 + 周期 ADC 日志 + CAN 心跳发送日志。
4. CAN 分析仪接入 CAN1（PA11/PA12）观察到周期 0x010 心跳帧；向 0x010 发帧，控制台打印 RX 日志（验证 RX/TX 链路）。
5. 上电后 `log`/`help` 控制台命令可用；触发 WARN 后 `log` 能 dump 出 Flash 记录（验证 F1 flash 驱动 + srv_log_flash）。
6. clangd 索引正常（先构建后索引）。

## 风险 / 注意

- F1 无 FPU：已禁用浮点 printf（`PRINTF_DISABLE_SUPPORT_FLOAT`），日志中禁止 `%f`。
- F1 20KB RAM 紧张：`drv_log_uart` RX 缓冲用 256B；如 Flash 链接超限，调小 `SRV_LOG_FLASH_AREA_SIZE` 或改 `log` TX 缓冲（`LOG_DEFAULT_TX_BUFFER_SIZE` 为共享默认 4KB，不建议改共享文件）。
- `HAL_CAN_GetTxMailboxesFreeLevel` / `HAL_CAN_GetRxMessage` 在 F1 HAL 存在（与 F4 同族 API），实现时如编译报错以 F1 头文件签名修正。
- 修改 public_layer 时只在原件上编辑；不动 `cmake/stm32cubemx/CMakeLists.txt` 与 E1_Hand 工程。

## 不做（本次范围外）

- 具体手套 CAN 协议内容（仅骨架占位，待协议文档）。
- KEY1/KEY2 / eFuse 相关驱动与逻辑（`main.h` 已有引脚，留待后续）。
- 其他工程（E1_Hand/E1_Master/boot）不改动（除非 hal_flash 新增 F1 分支的共享编辑，不影响现有 F4/G4/H7/G0 选型）。
