# E1 移植 stm32_g474_boot 升级功能 —— 迁移记录与操作手册

> 本文档记录将 `stm32_g474_boot`（STM32G474 bootloader）的升级功能移植到
> `E1_Master_Power_Manage`（STM32F407 主控电源板）的**完整过程**与**操作细节**。
> 升级机制本身（分区布局 / 协议 / 流程）见 [boot_upgrade.md](boot_upgrade.md)；
> 本文侧重「怎么迁移的」和「迁移后怎么操作」。

---

## 1. 背景与目标

- **源**：`stm32_g474_boot` —— 基于 CAN/CAN FD + 双 A/B 分区 + ring_storage 元数据的独立 bootloader。
  升级代码集中在 `service/boot/`（boot_transport / boot_fsm / boot_flash）+ `tasks/boot_task.c`。
- **目标**：让 E1（F407 主控电源板）具备**通过 CAN 现场升级自身固件**的能力，
  并遵循 E1 的 **6 层架构**（tasks → applications → services → middleware → device_drivers → HAL）。
- **用户拍板的方向**：
  1. **完整 A/B 双镜像**：工程产出 Boot + App 两个固件镜像，升级闭环完整。
  2. **新增 CAN ID `0x003`** 作为进 boot 触发命令。

### 迁移前 E1 的现状（关键约束）
- App 固件 84KB，**链接占满整个 1MB Flash**，无任何分区/升级代码。
- 已有 `srv_boot_ctrl`：管理 boot metadata（`boot_metadata_t` 字节契约与 g474_boot 完全一致）、
  `request_boot()` 置 upgrade_flag 复位——即「App 请求进 boot」半程。
- 已有 `hal_flash`（F4 驱动，`HAL_FLASH_CHIP_STM32F4`）、`ring_storage_port_hal()`、`drv_system_reset()`、
  `srv_param_store`（参数分区）、共享 `m_middlewares`（fsm / msg_fifo / sw_timer / ring_storage / crc）。

---

## 2. G474 vs F407 关键差异与适配决策

| 维度 | G474（源） | F407（目标） | 适配处理 |
|------|-----------|-------------|---------|
| CAN 外设 | FDCAN，经典/FD 8-64B | bxCAN，**仅经典 8B** | 协议帧（START=8B/METADATA=7B/DATA=2+6B/DATA_END=8B/ACK=8B）全兼容；`s_supported_frame_sizes` 裁为 `{8}`；`drv_can_msg_t.dlc` 为 `uint8_t` 0-8，兼容 |
| CAN ID | `0x701/0x702` | 不变 | 与 E1 现有 `0x001/0x002/0x200-0x202` 无冲突 |
| Flash 总容量 | 128KB，4KB 页 | 1MB，**不规则扇区**（16K×4+64K+128K×7） | App 分区定为 128KB 单扇区；metadata/参数区 256KB（2×128K 扇区，ring_storage 要求 ≥2 扇区） |
| App 现状 | 独立工程（boot 与 app 分离） | 本工程即 App，占满 1MB | App 重定位到 `0x08020000`，向量表 VTOR 重定位 |
| boot→app 跳转 | 代码被注释 | 启用 | 启用向量表跳转 + 增加 `fw_size>0` 与 SP/PC 合法性校验 |
| CAN 驱动 API | `drv_can_init(ch, &hfdcan)` | `drv_can_init()`（无参） | 适配调用；复用 E1 的 RX 回调注册 |
| Bus-Off 自恢复 | 有 | **无** | 为 `drv_can` 新增 `drv_can_is_bus_off()` / `drv_can_recover()`（bxCAN `ESR.BOFF` + Stop/Start） |
| 系统复位 | `HAL_NVIC_SystemReset()`（注释） | 已有 `drv_system_reset()` | boot 的 `reset_cb` 复用 |
| 硬件 ID | `0x0001` | `0x0002` | 防误刷 G474 固件；上位机 GUI 可配 |
| metadata 扇区 | `caps.erase_size`（G4 均匀页） | F4 非均匀 → 不能直接用 | `boot_flash` 的 ring_storage 显式 `RING_STORAGE_SECTOR_128K` |
| 构建 | 单一镜像 | 需双镜像 | CMake 双目标 + 每目标链接脚本/宏 |

### 最终 Flash 分区布局（F407，1MB）

| 区域 | 地址 | 大小 | 扇区 | 管理者 |
|------|------|------|------|--------|
| BOOT | `0x08000000` | 128KB | S0-S4 | Boot 镜像 |
| App A | `0x08020000` | 128KB | S5 | App 镜像链接处 |
| App B | `0x08040000` | 128KB | S6 | A/B 对侧 |
| Metadata | `0x08060000` | 256KB | S7-S8 | `boot_flash`(Boot) / `srv_boot_ctrl`(App) 共享 |
| APP 参数 | `0x080A0000` | 256KB | S9-S10 | `srv_param_store` |
| 空闲 | `0x080E0000` | 128KB | S11 | 预留 |

> 迁移导致的**既有代码地址变更**：`srv_boot_ctrl` metadata `0x080C0000 → 0x08060000`；
> `srv_param_store` 参数区 `0x08080000 → 0x080A0000`（避免与 Boot/App 分区重叠）。

---

## 3. 迁移过程（分步）

### 步骤 1 —— 摸底
- 精读源项目 `service/boot/*`、`tasks/boot_task.c`、`drv_can.h`、上位机 `protocol.py`（确认协议帧字节布局与校验算法）。
- 确认 E1 现状：App 体积 84KB、CAN 驱动结构、`srv_boot_ctrl` 字节契约、链接脚本、CMake 结构、工具链位置。
- 确认关键事实：`caps.addr = 0x08000000`、F4 `write_gran = 32-bit`、ring_storage 需 ≥2 扇区、
  `-ffunction-sections + --gc-sections` 已启用（Boot 复用整个 CubeMX Core 也不臃肿）。

### 步骤 2 —— 移植 service 层 `service/boot/`
| 文件 | 移植方式 | 适配点 |
|------|---------|--------|
| `boot_transport.{h,c}` | 近乎原样 | 帧长集合 `{8,12,...,64}` → `{8}`；修正 METADATA 字段偏移注释 |
| `boot_fsm.{h,c}` | 原样 | `BOOT_FLASH_APP_SIZE` 由 `boot_flash.h` 提供（新值 128KB） |
| `boot_flash.{h,c}` | 适配 | 分区常量 `BOOT=0x20000/APP=0x20000/META=0x40000`；ring_storage 配置 `start=base+0x60000, sector=128K, write_gran=caps`；改用 `ring_storage_port_hal()` |

> **移植中修正的一个转录错误**：METADATA 帧的 checksum 在 `data[1..4]`、version 在 `data[5..6]`
> （无 seq 字节），与上位机 `struct.pack(">BIH", ...)` 完全对应——初版转录时误写成 `data[2..5]/[6..7]`，已按源项目修正。

### 步骤 3 —— 移植 task 层 `tasks/boot_task.{h,c}`
- `drv_can_init()` 无参适配。
- **启用跳转**（源项目被注释）：`__disable_irq(); __set_MSP(app_sp); ((void(*)())app_pc)();`，
  并新增跳转前合法性校验：
  - `meta.fw_size == 0`（首烧无 App）→ 留在 boot 模式；
  - 初始 SP 需在 `[RAM_START, RAM_END]`（**允许 == RAM_END**，因 `_estack` = RAM 末尾）；
  - 初始 PC 需在 App 分区 Flash 范围 `[0x08020000, 0x08040000)`。
- `reset_cb` → `drv_system_reset()`；`hw_compat_id = 0x0002`；保留 A/B 切换、msg_fifo、sw_timer、bus-off 轮询。

### 步骤 4 —— 双镜像入口 `tasks/app_main.c`
- 用编译宏 `E1_BUILD_BOOT` 分流同一文件（避免新增入口文件被 App 的 `aux_source_directory` 误收导致重复 `app_main`）：
  - Boot 分支：`delay_init → log_task_init → boot_task_try_boot_app()（失败则 boot_task_init()）→ 主循环`。
  - App 分支：原流程不变。

### 步骤 5 —— App 侧接入
- `tasks/can_task.c`：新增 `0x003`（1 字节 `0x01`）进 boot 命令。遵循 ISR 极简约定——ISR 内**只置标志**
  `s_enter_boot_requested`（volatile），主循环 `can_timer_cb` 检测后调 `srv_boot_ctrl_request_boot()`
  （Flash 写 + 复位不能放 ISR）。
- `srv_boot_ctrl.c` / `srv_param_store.c`：分区地址迁移（见 §2）。

### 步骤 6 —— 驱动补充 `device_drivers/drv_can.{h,c}`
- 新增 `drv_can_is_bus_off()`（`CAN->ESR.BOFF`）、`drv_can_recover()`（`HAL_CAN_Stop/Start` + 重使能 RX 中断）。

### 步骤 7 —— 构建改造（双镜像）
- **链接脚本**：`STM32F407XX_FLASH.ld` ORIGIN 改 `0x08020000`/128KB（App）；新增 `STM32F407XX_BOOT.ld`（`0x08000000`/128KB）。
- **toolchain** `cmake/gcc-arm-none-eabi.cmake`：移除全局 `-T` 与 `-Map=`（改每目标指定，避免两个镜像争用同一 map）。
- **`cmake/stm32cubemx/CMakeLists.txt`**：CubeMX 源/库挂接改为 foreach 附加到 App 与 Boot 两个目标。
- **根 `CMakeLists.txt`**：新增 `E1_Boot` 目标（boot 服务 + boot_task + 最小驱动集 + hal_flash）；
  App 目标排除 `tasks/boot_task.c`、追加 `USER_VECT_TAB_ADDRESS`/`VECT_TAB_OFFSET=0x00020000U`；
  每目标独立 `-T` / `-Map` / POST_BUILD hex+bin。

### 步骤 8 —— 编译修复（过程中的问题）
| 问题 | 原因 | 修复 |
|------|------|------|
| `boot_flash.h: No such file` | boot_task.c 在 tasks/ 下找不到 `service/boot/*.h` | Boot 目标 include path 增加 `service/boot` |
| `__disable_irq/__set_MSP` 隐式声明 | 未包含 CMSIS core 头 | boot_task.c 增加 `#include "main.h"`（拉入 `core_cm4.h`） |
| `$<TARGET_FILE_BASE_NAME>` 求值失败 | 该 genex 需参数 | 改用字面 map 名 `E1_Master_Power_Manage.map` / `E1_Boot.map` |
| App VTOR 重定位不生效 | `system_stm32f4xx.c` 在**共享** `STM32_Drivers` OBJECT 库，收不到 App 目标宏 | 将该文件移到每目标源列表 `MX_Application_Src`，App/Boot 各自编译副本 |
| 跳转校验误拒合法 App | `_estack = 0x20020000`（RAM 末尾），`app_sp >= RAM_END` 误判 | 放宽为 `app_sp > RAM_END`（允许 == RAM_END） |

### 步骤 9 —— 验证
- Debug 与 Release 双配置构建通过，boot 代码**零警告**（`-Wall`）。
- 体积：Debug App 86KB / Boot 63KB（Release 50KB / 37KB），均远小于 128KB 分区。
- 链接地址：App `.isr_vector`@`0x08020000`、Boot@`0x08000000`。
- **VTOR 确认**：反汇编 App 的 `SystemInit` 见 `str r2,[r3,#8]`（r2=`0x08020000` → SCB->VTOR）；
  Boot 的 `SystemInit` 无 VTOR 写入（保持 `0x08000000`）。

---

## 4. 操作手册

### 4.1 构建
```bash
cmake --preset Debug && ninja -C build/Debug     # 或 build.bat / build.sh
cmake --preset Release && ninja -C build/Release
```
产物在 `build/<Config>/`：`E1_Master_Power_Manage.{elf,hex,bin}`（App，@0x08020000）与
`E1_Boot.{elf,hex,bin}`（Boot，@0x08000000）。

### 4.2 首次烧录（SWD，一次性）
| 镜像 | 地址 | 说明 |
|------|------|------|
| `E1_Boot.bin` | `0x08000000` | 启动决策 + 升级接收 |
| `E1_Master_Power_Manage.bin` | `0x08020000` | 主控电源固件（首版 App） |

> 只烧 Boot 不烧 App 也可以：上电后 Boot 判定无有效 App（`fw_size=0`）→ 停留在升级模式，等首次 CAN 烧写。

### 4.3 现场升级（CAN）
1. **上位机**：运行 `stm32_g474_boot/updata_tool/flash_tool`（PySide6 GUI，无需改动）。
   - **HW Compat ID 填 `0x0002`**（默认 0x0001 必须改，否则 Boot 回 NACK `HW_MISMATCH`）。
   - **帧长度选 `8`**（经典 CAN）。
   - 打开待升级的 `E1_Master_Power_Manage.bin`。
2. **触发进 boot**：向主电源板发 `0x003`，1 字节 `0x01` → App 置 upgrade_flag 复位。
3. **烧写**：点「开始升级」，按 `0x701 START(METADATA→DATA_START→DATA→DATA_END)×N → VERIFY → REBOOT` 流程，
   固件写入**对侧**分区，整包 32-bit 累加和校验通过后 Boot 写 Metadata 并复位。
4. 复位后 Boot 校验新分区 → **跳转新 App**。下次升级自动切到另一分区（A/B 交替）。

### 4.4 验证清单
- [ ] `ninja -C build/Debug` 同时产出 `E1_Master_Power_Manage.*` 与 `E1_Boot.*`
- [ ] 上电日志：Boot 打印「校验通过，跳转到分区 X」→ App 打印「系统启动」+ 状态帧 `0x001` 正常
- [ ] 发 `0x003` → 复位后 Boot 打印「升级标志置位，进入 Bootloader 模式」
- [ ] 上位机 `hw_id=0x0002`、帧长 8 走完整升级 → Boot 打印「Metadata 写入成功」→ 自动复位跳转
- [ ] 再升级一次，确认目标分区在 A/B 间交替（日志「当前分区 X, 目标分区 Y」）
- [ ] 异常路径：`hw_id` 填错 → NACK `HW_MISMATCH`；拔线超 6s → 全局超时回 IDLE

### 4.5 回退与恢复
- **A/B 回退**：升级失败时旧固件仍在对侧分区（未被擦除），重新走升级流程即可；Boot 校验失败会留在升级模式。
- **救砖**：Boot 区未损坏时可用 CAN 重刷；若 Boot 区也损坏，需 SWD 重烧 Boot。
- **无看门狗自动回退**（与 g474_boot 一致）：新 App 运行后崩溃不会自动回退到旧版，属已知限制。

---

## 5. 已知说明 / 后续

- **代码位置**：Boot 升级栈（`boot_transport`/`boot_fsm`/`boot_flash`）与 `boot_task` 已从 E1 的
  `service/boot/`、`tasks/` **迁移到共享层** `../public_layer/service/boot/` 与 `../public_layer/task/`
  （跨工程复用；依赖 E1 的 `drv_can`/`drv_systick`/`log_task` 接口，其他工程接入时需提供同名接口）。
  上文迁移步骤中的旧路径为**历史记录**，当前以 public_layer 为准。
- **联调修复 ① A/B 切换判定**：`boot_task_init` 曾用 `upgrade_flag==0` 判「有无有效 App」，导致 0x003 触发
  （upgrade_flag=1）时永远写 A。改为以 **`fw_size>0`** 判定（[boot_task.c](../public_layer/task/boot_task.c)）。
- **联调修复 ③ A/B 链接地址缺陷（运行槽提升）**：App 固定链接于 A（`0x08020000`），提交到 B 后 Boot 直接跳
  B 的内嵌 PC（仍是 A 链接地址）→ 落到 A 的旧/空区，取指失败进 `Default_Handler`。改为 `boot_flash_promote_to_a()`：
  Boot 校验通过后若活动分区为 B，先拷贝 B→A 再从 A 启动（Boot 跑在 `0x08000000` 区，写 A 安全）。
- **联调修复 ② REBOOT ACK 发送时序**：`handler_reboot_pending` 原在 `send_ack()`（仅入队）后立即 `reset_cb()`，
  复位打断在途 ACK，上位机收不到 REBOOT 应答而超时。改为 FSM 置 `reset_requested`，由 `boot_task`
  在 ACK 真正发出后 `drv_system_reset()`（`boot_fsm_take_reset()`）；等待方式为 `drv_can_tx_all_done()`
  （bxCAN TX 邮箱全部空闲，`TSR.TME` 硬件置位）配合 `millis()` 有界轮询，非固定延时。
- **后续增强（跳转/复位前日志排空）**：`log_task_flush()` 排空 log 缓冲并等待 UART DMA 完成；
  在 Boot→App 跳转、REBOOT/回滚复位、App→Boot（0x003）复位前调用，确保最后几句日志
  （"跳转分区"、"请求进入 bootloader"）不被复位打断丢失。
- **后续增强（Boot 蓝色 LED 状态指示）**：Boot 复用 `srv_signal.c`（已上移 `../public_layer/service/`）+ `led_task.c`（`E1_BUILD_BOOT` 变体）
  + `drv_led.c`，按升级状态驱动蓝色 LED（IDLE 慢闪/传输快闪/校验速闪/重启常亮）；
  `led_task_init` 经 `boot_task_get_state()` 读取升级 FSM 状态。
- **后续增强（初始等待超时）**：Boot 进入升级模式后未收到 START，先于 `BOOT_IDLE_WARN_MS`（默认 10s）打
  警告日志，至 `BOOT_IDLE_ROLLBACK_MS`（默认 30s）仍无指令且存在有效上个版本则复位跳回上个版本，
  否则保持等待（[boot_task.c](../public_layer/task/boot_task.c) `boot_rollback_to_prev()`）。
- **后续增强（失败/取消自动回滚）**：升级会话失败或 CANCEL 回到 IDLE 后，经 `BOOT_ROLLBACK_DELAY_MS`
  （默认 2000ms，可调）无新会话，即清除 `upgrade_flag` 并复位，Boot 跳转上个已提交版本
  （[boot_task.c](../public_layer/task/boot_task.c) `boot_check_rollback()`/`boot_rollback()`）。
- **后续增强（Boot 心跳 beacon）**：Boot 在 IDLE 态每 1s 发一帧 beacon（`0x702`，命令 `0x09`，携带
  `hw_id`），上位机据此自动判断「设备是否已在 Boot」——收到则跳过 `0x003`，未收到才发 `0x003` 触发。
  实现于 `boot_transport_build_beacon()` + `boot_fsm_tick()`（IDLE 分支），上位机 `worker.py` 新增
  `_wait_for_beacon()`/`_send_trigger()` 探测阶段。帧定义见 [boot_upgrade.md](boot_upgrade.md) §4。
- `reboot_counts` 在 Boot 决策与 App 启动时各 +1（两处 ring_storage 独立实例），单次上电计 2 次——仅信息计数，无功能影响。
- Boot 镜像复用了整个 CubeMX `main()`（全部 `MX_*_Init`），二进制偏大（63KB）但 < 128KB 且已启用 gc-sections；如需瘦身可做精简 main。
- 升级协议细节（帧格式 / 校验算法 / 超时）见 [boot_upgrade.md](boot_upgrade.md) 与
  源项目 `stm32_g474_boot/boot_protocol_spec.md`（本工程协议与其一致，仅经典 CAN 8 字节）。
