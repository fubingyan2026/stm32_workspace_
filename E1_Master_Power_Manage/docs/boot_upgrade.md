# E1_Master_Power_Manage Boot / 固件升级

> 迁移过程与操作手册见 [boot_migration.md](boot_migration.md)。
> 协议帧/时序/超时/状态矩阵的**正式规范**见 [upgrade_protocol.md](upgrade_protocol.md)。

本文档描述主控电源板（STM32F407）的 Bootloader 与 CAN 固件升级机制。
代码移植自 `stm32_g474_boot`（`service/boot/*` + `tasks/boot_task.*`），适配 STM32F407
（经典 bxCAN 8 字节帧 + 128KB 扇区），并接入本工程的 6 层架构。

## 1. 架构：双镜像

工程一次构建产出两个固件镜像：

| 镜像 | 链接地址 | 大小 | 内容 | 说明 |
|------|----------|------|------|------|
| **E1_Master_Power_Manage**（App） | `0x08020000` | 128KB | 现有主控电源固件 | 升级对象；上电由 Boot 校验后跳转 |
| **E1_Boot**（Boot） | `0x08000000` | 128KB | 启动决策 + CAN 升级接收 | 首烧/升级后不再变动 |

- **Boot 镜像**：`tasks/app_main.c` 的 `E1_BUILD_BOOT` 分支 + `tasks/boot_task.*` + `service/boot/*` +
  最小驱动集（systick / log_uart / can）。启动即做「跳转 or 升级」决策，决策失败则进入升级接收循环。
- **App 镜像**：现有全部任务/服务/应用，仅新增 0x003 进 boot 命令；向量表重定位到分区
  （`USER_VECT_TAB_ADDRESS` + `VECT_TAB_OFFSET=0x00020000U`，`system_stm32f4xx.c` 于 SystemInit 内写 `SCB->VTOR`）。

## 2. Flash 分区布局（F407，1MB）

| 区域 | 地址 | 大小 | 扇区 | 管理模块 |
|------|------|------|------|----------|
| BOOT | `0x08000000` | 128KB | S0-S4 | Boot 镜像 |
| App A | `0x08020000` | 128KB | S5 | Boot 写入 / App 链接 |
| App B | `0x08040000` | 128KB | S6 | Boot 写入（A/B 对侧） |
| Metadata | `0x08060000` | 256KB | S7-S8 | `boot_flash`（Boot）与 `srv_boot_ctrl`（App）共享 |
| APP 参数 | `0x080A0000` | 256KB | S9-S10 | `srv_param_store` |
| 空闲 | `0x080E0000` | 128KB | S11 | 预留 |

- `boot_flash`（Boot 侧）与 `srv_boot_ctrl`（App 侧）各自持有独立 ring_storage 实例，但**同一区域、同一
  `boot_metadata_t` 字节契约**（`magic=0x424F4F54` + partition/upgrade_flag/version/fw_size/fw_checksum/reboot_counts）。
  两者不同时运行（Boot 模式 or App 模式），字段布局不得增删。
- `boot_metadata_t` 在 `service/boot/boot_flash.h` 与 `service/srv_boot_ctrl.c` 各定义一份，逐字节一致。

## 3. 升级流程

```
主机                                   主电源板
 │ 0x003 (进入升级模式)                  │
 ├──────────────────────────────────────>│ App: srv_boot_ctrl_request_boot()
 │                                      │     置 upgrade_flag=1 → 系统复位
 │                                      │ Boot: 判定 upgrade_flag≠0 → 升级模式
 │ 0x701 START (fw_size/hw_id=0x0002/frame=8)
 ├──────────────────────────────────────>│ Boot: 擦除对侧分区 → ACK
 │ 0x701 METADATA (整包 checksum/version)│
 ├──────────────────────────────────────>│ Boot: 记录元数据 → ACK
 │ 0x701 DATA_START/0x03 DATA/DATA_END   │
 ├───（1KB Block × N，逐块校验）────────>│ Boot: 写 Flash + 读回校验 → ACK
 │ 0x701 VERIFY                          │
 ├──────────────────────────────────────>│ Boot: 整包 32-bit 累加和 → ACK
 │ 0x701 REBOOT                          │
 ├──────────────────────────────────────>│ Boot: 写 Metadata(新分区/upgrade_flag=0) → 复位
 │                                      │ Boot: 校验通过 → 跳转新 App 分区
```

- **进 boot 触发（直接 0x003，beacon 确认）**：上位机**直接发 `0x003`**（1 字节 `0x01`）触发——
  `0x003` 对已在 Boot 的设备无害（Boot 忽略之），无需先探测。随后等 Boot 心跳 beacon
  （`0x702`，含 `hw_id` 核对，防刷错板）确认进入 Boot；省去探测窗口，App→Boot 约 1s。
  App 收到 `0x003` 后其 `can_task` 在 ISR 仅置标志，主循环 `can_timer_cb` 调
  `srv_boot_ctrl_request_boot()`（Flash 写 + 复位不能放 ISR）。
- **目标分区**：Boot 以 **`meta.fw_size > 0`**（而非 `upgrade_flag`）判定「当前分区是否已有有效 App」，
  已有则写**对侧**分区，首次（fw_size=0）写 A。0x003 触发时 `upgrade_flag=1` 属正常——不能据此误判为无 App
  （否则 A/B 永不切换）。
- **运行槽提升（关键）**：App 固定链接于 A（`0x08020000`），只能从 A 运行。升级写对侧分区并 REBOOT 后，
  Boot 校验通过，若活动分区为 B 则先 `boot_flash_promote_to_a()` 把 B 拷贝到 A（Boot 跑在 `0x08000000` 区，
  写 A 安全），再从 A 启动——保证新固件真正运行。直接跳 B 会落到 A 的旧链接地址，取指失败进 `Default_Handler`。
- **跳转条件**（`boot_task_try_boot_app`）：`magic` 有效 且 `upgrade_flag==0` 且 `fw_size>0` 且
  分区 32-bit 累加和 == `meta.fw_checksum` 且向量表合法（SP 在 RAM、PC 在 App 分区），否则进入升级模式。
- **状态指示（Boot 蓝色 LED）**：Boot 复用 `srv_signal` + `led_task`（`E1_BUILD_BOOT` 变体）驱动蓝色 LED
  指示升级状态——IDLE 慢闪（等待）→ START/DATA 快闪（传输中）→ VERIFY 中速闪（校验中）→ REBOOT 前常亮。
- **失败/取消自动回滚**：升级会话（收到过 START）失败或被 CANCEL 回到 IDLE 后，等待
  `BOOT_ROLLBACK_DELAY_MS`（[boot_task.c](../tasks/boot_task.c)，默认 2000ms，可调）内无新会话，
  即清除 `upgrade_flag`（保留已提交分区）并复位——Boot 校验已提交（上个）分区并跳转，恢复旧版本。
  该窗口内主机可重新发起升级（新 START 会取消回滚倒计时）。刚进入 boot 的初始 IDLE 不触发回滚。

## 4. 升级协议（CAN 0x701 / 0x702）

协议与 `stm32_g474_boot`（`boot_protocol_spec.md`）一致，仅使用**经典 CAN 8 字节帧**：

- `0x701` 主机 → 板卡；`0x702` 板卡 → 主机（标准帧，DLC≤8）。
- 帧头 2 字节：`[Command][Sequence]`；单帧载荷 = `frame_size - 2`（=6 字节）。
- 命令：START(0x01) / METADATA(0x02) / DATA(0x03) / VERIFY(0x04) / REBOOT(0x05) / CANCEL(0x06) /
  DATA_START(0x07) / DATA_END(0x08)；应答 ACK(0x10) / NACK(0x11)。
- **Boot 心跳 beacon（E1 扩展，0x702）**：`[0x09][hw_id_H][hw_id_L][填充]`，8 字节经典 CAN。
  Boot 处于 IDLE 态时每 1s 发送一帧，供上位机探测「设备是否已在升级模式」；
  进入 START/DATA 后停发。携带 `hw_id` 供上位机核对目标板（防刷错板）。
- 块大小 1024B；每块 `DATA_START(块号)` → N×`DATA(seq)` → `DATA_END(16-bit 累加和+尾数据)`。
- 校验：块级 16-bit 累加和（全 1024B 求和 & 0xFFFF）；整包 32-bit 累加和（& 0xFFFFFFFF）。
- `hw_compat_id`：**0x0002**（G474 为 0x0001，避免误刷）。
- 超时：全局 6s 无活动复位；块内帧间隔 100ms。

## 5. 上位机工具

上位机已随工程存放于 [updata_tool/flash_tool](../updata_tool/flash_tool/README.md)
（移植自 `stm32_g474_boot`，已按 E1 协议适配）：

- **HW Compat ID** 默认 `0x0002`（E1 Boot 的硬件兼容 ID，勿改回 0x0001）。
- E1 为经典 bxCAN，**仅支持 8 字节帧**（CAN FD 已从界面移除）。
- 波特率默认 `1,000,000 bps`（E1 CAN1 = 1Mbps）。
- 流程：先发 0x003（或直接烧 Boot 后上电即进升级模式），再按上述协议烧写 App 的 `.bin`。

运行：
```bash
cd updata_tool && python -m flash_tool
```

## 6. 首次烧录 / 刷写

- **首次烧录**：用 SWD/J-Link 将 `E1_Boot.bin` 烧到 `0x08000000`，`E1_Master_Power_Manage.bin` 烧到 `0x08020000`。
- **现场升级**：Boot 已驻留后，走 CAN 0x003 + 0x701/0x702 流程，无需 SWD。
- **A/B 回退**：新固件写入对侧分区并提交后，旧固件仍在另一分区；若新 App 校验失败，Boot 会留在升级模式可重新烧录。
  （无看门狗自动回退——与 g474_boot 行为一致。）

## 7. 关键代码位置

| 文件 | 职责 |
|------|------|
| [tasks/boot_task.c](../tasks/boot_task.c) | 启动决策 + 升级接收（胶水层） |
| [tasks/app_main.c](../tasks/app_main.c) | `E1_BUILD_BOOT` 分支（Boot 入口）/ App 入口 |
| [service/boot/boot_transport.{h,c}](../service/boot/boot_transport.c) | 协议帧编解码 |
| [service/boot/boot_fsm.{h,c}](../service/boot/boot_fsm.c) | 升级状态机 |
| [service/boot/boot_flash.{h,c}](../service/boot/boot_flash.c) | 分区擦写/校验/Metadata |
| [service/srv_boot_ctrl.c](../service/srv_boot_ctrl.c) | App 侧 Metadata 管理 + `request_boot()` |
| [tasks/can_task.c](../tasks/can_task.c) | 0x003 进 boot 命令 |
| [STM32F407XX_FLASH.ld](../STM32F407XX_FLASH.ld) / [STM32F407XX_BOOT.ld](../STM32F407XX_BOOT.ld) | App/Boot 链接脚本 |
