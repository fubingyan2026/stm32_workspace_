# E1_CTU_BOOT：STM32F103 RCT 双分区 RS485 YMODEM Bootloader 实现计划

## 1. 目标与背景（已与用户确认的决策）

为兄弟工程 `E1_MASTER_POWER_CTU` / `E1_SLAVER_POWER_CTU`（均为 STM32F103RCTx，256KB Flash / 48KB RAM）制作一套共用的 RS485 固件升级 Bootloader，位于本工作区 `E1_CTU_BOOT/` 工程内。

已确认决策：

| # | 决策 | 结论 |
|---|------|------|
| 1 | 双分区模型 | **单链接 A + B 暂存提升**：App 永远按 A 分区地址链接；新固件由 Boot 经 YMODEM 下载到 B 槽 → 整包累加和校验 → `promote_to_a` 拷回 A → 复位从 A 启动。B 始终保留一份完整已验证镜像，断电打断提升时下次上电 Boot 自动从 B 重建 A（防半砖）。**不做跨版本自动回滚**（A/B 最终内容相同）。 |
| 2 | Flash 布局 | `BOOT 0x08000000 32K / AppA 0x08008000 96K / AppB 0x08020000 96K / 保留 0x08038000~0x0803DFFF 24K / Meta 0x0803E000 8K`（见 §4）。 |
| 3 | 传输协议 | 标准 **YMODEM**（接收端角色，CRC-16/xmodem 模式），485 单从机点对点，115200-8N1，PC5(DE)/USART3(PB10/PB11)。 |
| 4 | 复用策略 | **复用 `public_layer` 共享 boot 栈**：`service/boot/boot_flash.{c,h}`、`device_drivers/hal_flash/*`（**新增 F1 驱动**）、`m_middlewares/Third_Party/ring_storage/*`、`algorithm/crc.c`；不复用 CAN 专用 `boot_fsm`/`boot_transport`/`task/boot_task`（新写 485/YMODEM 版本）。 |
| 5 | 交付范围 | **仅实现 `E1_CTU_BOOT` 工程**（+ `public_layer` 两处向后兼容小改动，见 §5）。兄弟 App 的改造与兄弟 host GUI 集成仅输出文档契约与后续任务清单，不在本次实现内。 |
| 6 | 代码约定 | 分层架构（tasks→service→device_drivers，webkit/MISRA 风格、中文 doxygen）、全程静态分配、共享代码改 `public_layer` 原件。本工程无日志依赖：全工程编译加 `-DLOG_ENABLED=0`（log 宏自动剥离为空，无需 log.c/log_task）。 |

关键事实核实（实现依据）：
- F103RC 在 CubeMX/驱动层统一编译宏 `STM32F103xE`，Flash 页 **全片均匀 2KB**（`FLASH_PAGE_SIZE=0x800`），HAL 页擦除/`FLASHEx_Erase` 可直接使用。
- F1 Flash 编程粒度：HAL 支持 `FLASH_TYPEPROGRAM_HALFWORD`/`FLASH_TYPEPROGRAM_WORD`（32-bit），用 WORD 编程 + 读回校验即可。`ring_storage` 的 `RING_STORAGE_WRITE_GRAN_32` 与 WORD 编程匹配（其注释标注 STM32F1=32bit）。
- 两个兄弟板 485 引脚/收发器/PC5/LED(PB6) 完全一致，一套 boot 二进制可通用。
- App 侧升级命令已预留：Master `0x11`、Slaver `0x1F`（当前应答"不支持"），兄弟 host 已有 `CMD_UPGRADE` 常量。
- 现有兄弟固件尺寸：Master Debug ~72KB（Release 更小），Slaver Release ~38KB；96KB 槽位（≈0x18000）在 2KB 页均匀区全部对齐。

## 2. 交付目录结构（目标）

```
E1_CTU_BOOT/
├─ Core/                       # CubeMX 生成（仅改 USER CODE 区）：main() 调 app_main()
├─ device_drivers/             # 已拷贝存在（F103 本地驱动，可在本工程内小改）
│  ├─ drv_uart.c/h             # 调整 RX 缓冲 ≥2048 且 HT/IDLE 双事件同步（YMODEM 1KB 包需要）
│  ├─ dev_rs485.c/h            # 保持（半双工 DE 自动控制）
│  ├─ drv_systick.c/h          # 保持
│  └─ (drv_led / drv_log_uart 若编译不过则从 build 中排除，不用于 boot)
├─ tasks/                      # 新增
│  ├─ app_main.c/h             # 入口：delay_init → boot_task_try_boot_app() → boot_task_init() 主循环
│  └─ boot_task.c/h            # 启动决策 + 升级会话粘合层（485+YMODEM+flash 状态机驱动）
├─ service/
│  ├─ boot_ymodem.c/h          # 新增：YMODEM 接收器（纯协议，无平台依赖）
│  └─ boot_flash_f1.c/h        # 可选薄封装：分区常量 + boot_metadata_t 访问（若直接引用 public 的 boot_flash 则不需要）
├─ docs/
│  └─ boot_485_ymodem.md       # 升级交互流程、YMODEM 用法、metadata 契约（App/host 集成依据）
├─ host/                       # 新增（仅本仓库内 bring-up 用，不属于兄弟 GUI）
│  └─ ymodem_send.py           # pyserial YMODEM 发送脚本（等 'C' → 发 .bin，供联调验证）
├─ E1_CTU_BOOT.ioc             # 保持
├─ STM32F103XX_FLASH.ld        # 保持（Boot 自身 0x08000000 起）
├─ CMakeLists.txt              # 修改：加入本地 device_drivers/tasks/service + ../public_layer 引用（见 §8）
└─ .gitignore / .clangd        # 保持

public_layer/                   # 共享层（仅加新文件/向后兼容分支，不影响其他工程）
├─ device_drivers/hal_flash/
│  ├─ drv_stm32f1_flash.c/h    # 新增：F103 驱动（仿 drv_stm32g4_flash 模板）
│  ├─ hal_flash.h              # 微改：选型宏清单加 HAL_FLASH_CHIP_STM32F1
│  └─ hal_flash.c              # 微改：#include 分支 + extern f1_dev / FLASH_DEV 分支
└─ service/boot/boot_flash.c   # 微改：BOOT_META_OFFSET 用 #ifndef 包一层（默认值不变），
                               #       供编译期把 meta 放到 0x0803E000（保留用户选定布局）
```

## 3. 模块职责与关键行为

### 3.1 上电启动决策 `boot_task_try_boot_app()`（在跳转前完成，不初始化 485）

1. `boot_flash_init()`（内部 `hal_flash_init` + `ring_storage` 加载 meta、reboot_counts+1 并保存）。
2. 读 `boot_metadata_t`。判定优先级（flag 语义见 §4.3）：
   - **case flag==2（提交中断恢复）**：若 `byte_sum(B[0..fw_size])==fw_checksum` 且 fw_size 合法 → `boot_flash_promote_to_a(B, fw_size)` 重建 A；成功 → 写 meta flag=0 → 复位。B 校验失败 → 写 flag=0，转入下一条决策。
   - **case flag==1 或 flag==2-恢复失败**：返回"进入升级模式"（先 `boot_flash_erase_partition(B)` 丢弃残留，再进 YMODEM 会话）。
   - **case flag==0 且 A 有效**：A 有效判定 = `fw_size∈[16, BOOT_FLASH_APP_SIZE]` 且 `byte_sum(A[0..fw_size])==fw_checksum` 且向量表 sanity（栈顶在 RAM 区、复位向量在 A 区且 bit0=1）。有效 → 跳 A（返回 true，不复位）。
   - **case flag==0 且 A 无效**：若 `byte_sum(B)==fw_checksum`（B 是最近完整镜像）→ 从 B 重建 A（同 promote）→ meta 归位 → 跳 A；否则进入升级模式并**常驻等待**（板为空/损坏恢复）。
3. 跳转函数 `boot_jump(uint32_t addr)`：`__disable_irq()` → `__set_MSP((uint32_t)*addr)` → 以 `(*(void(*)(void))(*(addr+4)))()` 执行；跳转前无需恢复外设（App 自行初始化）。
   - 注：不做上电驻留监听（无 dwell）——正常升级由 App 命令触发；出厂/空片自动常驻等待；最终兜底为 BOOT0 拉高进 ROM Bootloader（USART1）。

### 3.2 YMODEM 升级会话（boot 常驻后，task 主循环驱动）

- 采用 `m_middlewares/framework` 风格的一个轻量状态机（可不用 fsm 库，用 enum+switch），状态：`WAIT_HEADER → RECV_DATA → FINALIZE → (COMMIT/RESET 或 ERROR/CANCEL)`。
- 会话入口：擦除 B（整槽 96K=48 页）→ 每 ~1s 周期发送 `'C'` 等待首包。
- 包解析规则：
  - 首字节 `SOH(0x01)`=128B 数据 / `STX(0x02)`=1024B 数据（两者都支持）；校验序号 `seq == (~seq_byte)`，数据区 2 字节 CRC-16（poly 0x1021，初值 0）通过才处理。
  - Block 0：头包，可选解析文件名/长度（长度字段在数据区偏移 124..127 3 字节大端）。不做强制解析（兼容 Tera Term 等通用工具）。
  - Block N≥1：数据。**先写 Flash 后回 ACK**：将数据块（或按实际长度）写入 B 的连续偏移；落笔前若该 2KB 页未擦除则先擦页（懒擦除，避免整槽先擦带来的"AC K 后超时"重发覆盖已编程区问题）。块 CRC 错 → NAK（不写）；写+读回失败 → 发 CAN(0x18) 中止。
  - 防重复写：若收到的块序号 ≤ 已 ACK 的序号（host 超时重发）→ 直接 ACK 不重复写。
  - 结束：EOT(0x04) → 回 NAK；再次 EOT → 回 ACK，进入 FINALIZE。
- 长度口径：fw_size = 收到的实际数据字节累计（不依赖头包长度；若头包给了长度则以长度截断尾部 0x1A 填充）。fw_checksum = 收到的所有数据字节的 32-bit 累加和（uint32 溢出自然截断，与 `boot_flash_compute_checksum` 的 `sum & 0xFFFFFFFF` 一致）。
- 超时：整包间无活动 >10s → CANCEL 会话：写 meta flag=0 → 复位（有有效 A 则回 A，否则留在常驻等待）。块内分包无活动 >3s → 重发当前期望的 ACK/NAK 状态（等待 host 重发）。
- **FINALIZE 提交序列（唯一提交点，全部先落盘后改指针）**：
  1. 写 meta：`{upgrade_flag=2, fw_size, fw_checksum, version=旧值+1, boot_partition=A}`（ring_storage 原子帧提交）。
  2. `boot_flash_promote_to_a(B, fw_size)`（擦 A → 1KB/2KB 分块从 B 读写入 A，逐块读回校验）。
  3. 写 meta：`{upgrade_flag=0, ...}`。
  4. `NVIC_SystemReset()`。
  - 断电于 1~3 步之间：下次上电按 §3.1 flag==2/1 分支自愈。
- 升级期间 LED 指示（PB6，通过 htim4 CCR 或简单 GPIO）：等待='C' 慢闪；收块中快闪/常亮；擦写/提升中常亮；完成后 3 短闪再复位。不做其它应用输出。

## 4. Flash 布局常量与 metadata 契约

### 4.1 布局（用户选定，页全部 2KB 对齐）

| 区域 | 地址 | 大小 | 说明 |
|------|------|------|------|
| BOOT | `0x08000000` | `0x8000`(32K) | Boot 自身（本次工程链接区，不改 ld） |
| App A | `0x08008000` | `0x18000`(96K) | 运行槽（App 固定链接地址，兄弟工程后续任务） |
| App B | `0x08020000` | `0x18000`(96K) | 暂存/备份槽（YMODEM 写入目标） |
| 保留 | `0x08038000` | `0x6000`(24K) | 预留（后续 App 参数/扩展用） |
| Meta | `0x0803E000` | `0x2000`(8K) | ring_storage 区域（4×2KB 页），仅用 2 页即可，留磨损均衡余量 |

### 4.2 编译期宏（E1_CTU_BOOT CMake `target_compile_definitions`）

```
HAL_FLASH_CHIP_STM32F1
STM32F103xE  USE_HAL_DRIVER   (现有 CubeMX 已有)
LOG_ENABLED=0                       # 剥离全部 log 依赖
BOOT_FLASH_BOOT_SIZE=0x8000U
BOOT_FLASH_APP_SIZE=0x18000U
BOOT_FLASH_META_SIZE=0x2000U
BOOT_META_OFFSET=0x3E000U           # 需配合 §5.3 的 boot_flash.c #ifndef 小改
BOOT_META_SECTOR_SIZE=0x800U        # F1 均匀 2KB 页
```
Boot 自身仍链接到整片 `0x08000000`（Flash 256K 起始），只实际占用 <32KB。

### 4.3 metadata（`boot_metadata_t`，与公共 boot_flash/srv_boot_ctrl 字节契约一致）

字段：`magic=0x424F4F54("BOOT") / boot_partition(A) / upgrade_flag / version(u16) / fw_size(u32) / fw_checksum(u32 累加和) / reboot_counts(u32) / reserved`。

`upgrade_flag` 语义（本设计定义，写入 docs 契约）：
- `0` 正常（A 为当前运行镜像，fw_size/checksum 指 A）；
- `1` App 请求升级/下载中（B 内容无效，可整槽重来）；
- `2` 提交中（B 已完整且 checksum 已写入 meta，A 可能被提升打断——下电可自动续）。

## 5. `public_layer` 改动（均向后兼容，不影响现有 F4/G4/G0/H7 消费者）

1. **新增 `device_drivers/hal_flash/drv_stm32f1_flash.{c,h}`**：以 `drv_stm32g4_flash.c` 为模板实现 `hal_flash_ops_t`：
   - caps：`addr=0x08000000, total_size=0x40000(256K), erase_size=0x800, write_gran=HAL_FLASH_WRITE_GRAN_32, erase_size_uniform=true, has_write_protect=false, has_ecc=false, has_crc=false`。
   - `init`：设置 `f1_priv.page_size=0x800`，同步 caps.erase_size。
   - `erase(offset,size)`：`HAL_FLASH_Unlock` → 逐 2KB 页 `HAL_FLASHEx_Erase`（F103xE 均匀页）→ `HAL_FLASH_Lock`；仿 g4 驱动加"chunk=0 死锁防御"。要求调用侧 offset/size 页对齐（hal_flash 上层已检查）。
   - `write(offset,buf,size)`：按 32-bit WORD 编程（`HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,...)`）并逐字读回校验；尾部不足 4B 用 0xFF 填充；`HAL_FLASH_Unlock/Lock` 包夹。
   - `read`：memcpy 式直读；`cache_invalidate`：空实现（F1 无 D/I-Cache）。
2. **`hal_flash.h`**：默认分支守卫加入 `&& !defined(HAL_FLASH_CHIP_STM32F1)`；`hal_flash.c` 增加 `#elif defined(HAL_FLASH_CHIP_STM32F1)` include `drv_stm32f1_flash.h` 及 `extern f1_dev / #define FLASH_DEV f1_dev`。
3. **`service/boot/boot_flash.c`**：将 `#define BOOT_META_OFFSET ...` 改为
   `#ifndef BOOT_META_OFFSET #define BOOT_META_OFFSET (BOOT+2*APP) #endif`（默认值不变，允许 F1 布局经编译宏把 meta 放到 0x0803E000）。其余不动。

## 6. E1_CTU_BOOT 本地改动

### 6.1 Core（USER CODE 区）
- `Core/Src/main.c`：`/* USER CODE BEGIN 2 */ app_main();`（仿 G474 boot：`main()` 先 CubeMX 初始化，再进 app_main）。在 `stm32f1xx_it.c`/`dma.c` 无需改（HAL 已配好 DMA/USART/IDLE 中断）。

### 6.2 device_drivers（本工程本地副本，仅 boot 使用，可改）
- `drv_uart.c`：`DRV_UART_RX_BUF_SIZE` 由 256 → **2048**（容纳整包 STX 1KB+），并让 RX 事件回调在 **HT 与 IDLE 都执行 `kfifo_move_in`**（当前只同步 IDLE，会被 1KB 连发冲垮）；保留 TX 队列逻辑。
- `drv_led.c`（若保留）：不依赖不存在于本工程的 `drv_pwm`；要么删除该文件，要么将 boot 指示灯做成直接操作 `htim4`/GPIO 的小函数放 `boot_task` 内部。

### 6.3 tasks/service（新增）
- `tasks/app_main.c/h`：初始化 `delay_init()`；调 `boot_task_try_boot_app()`（true 则永不返回）；false 则 `boot_task_init()` 并进入 `while(1){ boot_task_poll(); }`（可带 1ms sw_timer 风格周期）。
- `service/boot_ymodem.c/h`：无平台依赖的 YMODEM 接收器：
  - 接口：`boot_ymodem_init(ctx,cfg)`、`boot_ymodem_feed(ctx, byte)`（按字节流驱动）、`boot_ymodem_poll(ctx, now_ms)`（超时/'C'心跳调度）。
  - 向上回调/返回：包就绪回调写 flash、`NAK/ACK/CAN/'C'` 输出需求回调、完成事件（总长+累加和）。
  - CRC-16/xmodem 增量实现放本模块（避免依赖公共 crc 表）。
- `tasks/boot_task.c/h`：胶水层——`boot_task_try_boot_app()`（§3.1 决策+跳转）；`boot_task_init()`（485 初始化：`dev_rs485_init()`；LED 指示）；`boot_task_poll()`（搬运 `dev_rs485_rx_read()` → ymodem feed；周期 `dev_rs485_tx_flush()`；定时 tick；YMODEM 完成 → 调 §3.2 提交序列，异常 → 复位回 A 或留驻）。
- 升级 flash 写入路径：接收块 → `boot_flash_write_block(B, off, data, len)`；每进入新 2KB 页前先 `boot_flash_erase_partition` 不可行（整槽先擦后写会丢 B），故在 boot_task 内维护"当前已擦页偏移"，页内首写前对 B 的该页单独 erase（可调用 hal_flash_erase 的页级接口或给 boot_flash 增加 `boot_flash_erase_range()`，实现时以最小改动优先，私有即可）。

### 6.4 docs 与 host（新增，bring-up 用）
- `docs/boot_485_ymodem.md`：YMODEM 流程、时序图、`upgrade_flag` 契约、Flash 布局表、Tera Term/脚本用法、给兄弟 App/host 的集成说明与后续任务清单（App 需：链接至 0x08008000 + 设置 VTOR + 实现 0x11/0x1F 跳 boot；host 需：YMODEM 发送 + 升级按钮 + 等待重启后回连）。
- `host/ymodem_send.py`：pyserial 实现 YMODEM 发送（等 'C'，发 block0+文件，处理 EOT 两次），命令 `python host/ymodem_send.py COMx file.bin`。

## 7. 行为/时序汇总（写进 doc 前与实现核对）

| 项 | 值 |
|----|----|
| 485 波特率/格式 | 115200 8N1 |
| 'C' 心跳 | 等待头包时每 1000ms 一帧 |
| 包间超时 | 3000ms（重发提示） |
| 会话总超时 | 10000ms 无活动 → 清 flag 复位 |
| 每包 CRC | CRC-16 CCITT(0x1021, init 0, no final xor) |
| 结束 | EOT→NAK，EOT→ACK |
| 中止 | 收 CAN(0x18) 或连续 NAK 计数>10 → 复位回 A |
| 最大固件 | ≤96KB（0x18000），若超发 → CAN 中止并提示 |

## 8. 构建接线（必须改 E1_CTU_BOOT/CMakeLists.txt）

> 工作区 AGENTS.md 有"不修改 CMakeLists.txt"的约定——那针对成熟工程（CubeMX 再生成区）；本工程 root `CMakeLists.txt` 头部注明 "generated only once / user may modify"，兄弟 Master/Slaver 也是以同样方式加入 `../public_layer` 引用。本计划要求在 E1_CTU_BOOT 自己的 CMakeLists 中按兄弟工程相同方式新增源与 include（若用户不允，请先显式解除该限制）。

- `target_compile_definitions(... PRIVATE HAL_FLASH_CHIP_STM32F1 LOG_ENABLED=0 BOOT_FLASH_...=... BOOT_META_OFFSET=0x3E000U BOOT_META_SECTOR_SIZE=0x800U)`。
- include：`device_drivers`、`tasks`、`service`、`Core/Inc`、`../public_layer/m_middlewares`(及所需子目录 log/utils/framework/algorithm)、`../public_layer/service`、`../public_layer/service/boot`、`../public_layer/device_drivers/hal_flash`。
- 新增源：
  - 本地：`device_drivers/drv_uart.c dev_rs485.c drv_systick.c`、`tasks/app_main.c boot_task.c`、`service/boot_ymodem.c`。
  - 共享：`../public_layer/device_drivers/hal_flash/hal_flash.c drv_stm32f1_flash.c ring_storage_port.c`、`../public_layer/service/boot/boot_flash.c`、`../public_layer/m_middlewares/Third_Party/ring_storage/ring_storage.c rs_crc32.c`、`../public_layer/m_middlewares/algorithm/crc.c`、`../public_layer/m_middlewares/utils/kfifo.c`、`../public_layer/m_middlewares/framework/msg_fifo.c`。
- 链接：`cmake --preset Debug && cmake --build --preset Debug`（Windows 同兄弟工程用法）。产物 `build/Debug/E1_CTU_BOOT.elf/.bin`。
- 校验 Boot 体积 ≤24KB（预留 ≥8KB 增长）；若超标削减 HAL（如去掉不用的 tim/adc 源）或升 Release。

## 9. 验证计划

1. **编译**：Debug/Release 双构建通过、零告警（`-Wall`），检查 `.map`：boot 代码 + 数据落在 `0x08000000..0x08007FFF` 内。
2. **静态检查**：clangd 先 Debug 构建后索引无报错；MISRA 要点自查（无 malloc、模块内实例化 handle、公共 API 首行参数校验）。
3. **单元级（可选，本仓库无测试基建，最少做）**：CRC-16/xmodem 用 `"123456789"`→`0x31C3` 自检宏；YMODEM 包解析做边界自检。
4. **硬件 bring-up（SWD 首次烧录）**：
   a. 空片/抹掉 A/B/Meta 后烧 Boot → 观察 Boot 常驻等待（LED 慢闪），PC 端 `host/ymodem_send.py` 发送一个链接在 `0x08008000` 的最小测试 App（如点灯）→ 应完成下载→提升→复位→App 点亮；
   b. 通过测试 App 的 485 上报确认启动正常；再次用脚本发第二版 → 验证升级后运行新版本；
   c. **断电注入**：分别在下载中、提升中（提升循环加调试断点或中途断电）断电 → 上电应自动从 B 重建/重入并最终可运行或回到可升级态；
   d. **CRC 破坏**：改一个字节重新发 → 应 NAK/中止且旧 App 仍可运行；
   e. 在两兄弟硬件各验证一次引脚配置一致性与 USART3 时序。
5. **回归**：连续 10 次升级循环 + 中途随机断电，统计无"双坏"状态。

## 10. 风险与注意

- **ring_storage 写入粒度 vs F1 编程**：以 `write_gran=32`(WORD) 对齐 ring_storage；F1 驱动按 WORD 成对半字/直接 WORD 编程。若实现期发现 ring_storage 存在非 2 对齐写路径，退路 = 本地实现"双页 double-buffer"meta 写模块，字节布局不变（doc 标注为契约），并跳过公共 `boot_flash`/ring_storage 的 meta 部分（分区操作仍可复用 boot_flash 其余部分或全本地化）。
- **RX 溢出**：YMODEM 1KB 包会瞬时涌入；必须按 §6.2 将 RX 缓冲扩到 ≥2048 并启用 HT 同步，否则丢包。
- **重复编程**：已编程页不可再写——靠"先校验后写/重复包直接 ACK/页懒擦除"规避。
- **boot 自身可靠性**：boot 区内不做任何自擦写；promote 仅擦 A/B（运行代码位于 0x08000000 区，安全）。
- **F1 HAL 页擦**：因 `STM32F103xE` 宏 flash 全片 2KB 均匀页，勿把兄弟 App（若曾按 1KB 页假设）逻辑带入 F1 驱动。
- **兄弟 App 后续任务（本次不实现，必须写进 docs）**：链接脚本 Flash ORIGIN 改 `0x08008000`、LENGTH `0x18000`；`SCB->VTOR` 置 `0x08008000`（system_stm32f1xx.c 的 `VECT_TAB_OFFSET` 或启动早期）；实现升级命令（置 flag=1→复位）；若 App 需要参数存储请规划使用保留区。
- **CMakeLists 修改许可**：§8 违反工作区"不改 CMakeLists"的默认记录，需用户许可（见构建接线注）。

## 11. 实施顺序（建议执行顺序）

1. `public_layer`：新增 F1 flash 驱动 + hal_flash 选型分支 + boot_flash.c 的 meta offset #ifndef（§5）。
2. E1_CTU_BOOT：CMake 接线 + 宏定义（§8），先让空工程编过。
3. 本地 drv_uart RX 缓冲/HT 同步改造（§6.2）。
4. `service/boot_ymodem` 协议接收器 + CRC16（§6.3）。
5. `tasks/app_main` + `boot_task`：启动决策、跳转、提交序列、超时/LED（§3、§6.3）。
6. `host/ymodem_send.py` + `docs/boot_485_ymodem.md`（§6.4）。
7. 按 §9 验证；生成 `build/Debug/E1_CTU_BOOT.bin` 交付烧录。
