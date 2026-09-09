# E1_CTU_BOOT：RS485 YMODEM Bootloader 协议与契约

- **作者**：maximillian
- **日期**：2026-09-07
- **目标**：`E1_MASTER_POWER_CTU` / `E1_SLAVER_POWER_CTU`（STM32F103RCTx，256KB Flash）共用的 RS485 固件升级 Bootloader
- **传输**：标准 **YMODEM**（CRC-16/xmodem），115200-8N1，USART3（PB10/PB11）+ PC5(DE)

---

## 1. Flash 分区布局

| 区域 | 起始地址 | 大小 | 说明 |
|------|----------|------|------|
| Boot | `0x08000000` | 32K（0x8000） | 本 boot（Release 构建 ~27KB） |
| App A | `0x08008000` | 96K（0x18000） | **运行槽**：App 固定链接地址，只从 A 启动 |
| App B | `0x08020000` | 96K（0x18000） | 暂存/备份槽：YMODEM 新固件下载目标 |
| 保留 | `0x08038000` | 24K（0x6000） | 预留（后续 App 参数区等） |
| Metadata | `0x0803E000` | 8K（0x2000） | `boot_metadata_t`（ring_storage，4×2KB 页） |

编译期宏（与 `public_layer/service/boot/boot_flash.{c,h}` 对齐）：
`BOOT_FLASH_BOOT_SIZE=0x8000U`、`BOOT_FLASH_APP_SIZE=0x18000U`、
`BOOT_FLASH_META_SIZE=0x2000U`、`BOOT_META_OFFSET=0x3E000U`、
`BOOT_META_SECTOR_SIZE=0x800U`。

> F103xE Flash 全片 2KB 均匀页；A/B 边界（0x08020000/0x08038000/0x0803E000）均 2KB 对齐。

## 2. 运行模型（单链接 A + B 暂存提升）

App 永远按 **A 槽（0x08008000）** 链接（一套链接脚本即可）。
升级流程：

```
App 收到升级命令（Master 0x06 / Slaver 0x06，命令码两板统一，z 帧带各自 addr）→ 置 metadata upgrade_flag=1 → 复位
Boot 启动 → 读到 flag=1 → 擦除 B → YMODEM 接收新固件到 B
   ↓ 传输完成（EOT/EOT）
写 metadata {flag=2, fw_size, fw_checksum}  →  promote_to_a(B→A)  →  写 {flag=0}  →  复位
Boot 启动 → A 校验通过 → 跳转 A 运行新固件
```

- B 槽始终保留一份完整已验证镜像；提升（A 擦写）中途断电 → 下次上电按 `flag=2`
  自动从 B 重建 A（防半砖，无需主机干预）。
- **不做跨版本自动回滚**：A/B 最终内容相同；旧版本被新版本覆盖。
- 出厂空片/两槽均损坏时，Boot 常驻升级模式等待主机。

### 2.1 `boot_metadata_t`（与 srv_boot_ctrl 共享的字节契约）

| 字段 | 类型 | 说明 |
|------|------|------|
| `magic` | u32 | `0x424F4F54` ("BOOT") |
| `boot_partition` | u8 | 恒 `BOOT_PARTITION_A`（本模型固定从 A 运行） |
| `upgrade_flag` | u8 | 0=正常；1=下载中；2=提交中（可自愈续提交） |
| `version` | u16 | 固件版本（Boot 每成功提交 +1；语义占位，可由 App/host 后续扩展） |
| `fw_size` | u32 | 有效固件字节数（A 校验区间长度） |
| `fw_checksum` | u32 | `sum(A[0..fw_size))` 32-bit 累加和（`&0xFFFFFFFF`） |
| `reboot_counts` | u32 | 上电次数（Boot 管理） |
| `reserved` | u32 | 预留 |

> App 侧将来实现"跳转 Boot"服务（如 `srv_boot_ctrl`）时，必须复用同一布局与地址
> （0x0803E000，ring_storage，4×2KB 页，sector_size=0x800，write_gran=32）。

## 3. 传输协议细节（YMODEM 接收端）

| 项 | 值 |
|----|----|
| 模式 | CRC-16/xmodem（poly 0x1021，init 0，发送端低字节在前） |
| 邀请 | 等待头包时每 1000ms 发 `'C'` |
| 数据包 | 支持 `SOH`(128B)/`STX`(1024B)；序号+反码、CRC 校验 |
| block0 | 头包（文件名/长度字段），可选解析长度（数据区 [124..126] 大端） |
| 应答 | 校验失败 `NAK`；成功落 Flash 后 `ACK` |
| 重复块 | ACK 丢失重发的同序号块 → 直接重复 `ACK` 不重写 |
| 结束 | `EOT`→`NAK`，`EOT`→`ACK` → 文件结束，Boot 进入 300ms 提交窗口 |
| 中止 | 收到 `CAN`(0x18) → 清 flag 复位回 App（或回升级态） |
| 会话超时 | 传输中途 >20s 无字节 → 清 flag 复位 |
| 最长固件 | ≤ 96KB（超出拒绝） |

Boot 自身的 fw_size/checksum 口径：优先按 block0 长度字段截掉末块 `0x1A` 填充；
若头包无长度，则按含填充的累计长度计（自洽，不影响运行）。

## 4. 主机工具用法

本仓库 `host/` 提供三层实现（协议层与界面分离，风格参考兄弟工程 host）：

| 文件 | 说明 |
|------|------|
| `boot_protocol.py` | YMODEM 传输协议层（无 GUI 依赖），含"请求进入 Boot"z 帧辅助 |
| `boot_host.py` | PySide6 上位机（GUI）：串口监听 / 目标设备 / 固件选择 / 进度与日志 |
| `ymodem_send.py` | CLI 发送（复用协议层） |

CLI：

```
python host/ymodem_send.py COM5 E1_MASTER_POWER_CTU.bin 115200 [boot|master|slaver]
```

GUI：

```
python host/boot_host.py
```

也可以使用支持 YMODEM 的终端（Tera Term：File → YMODEM → Send）：
接收端（板端）已进入升级模式并周期发 `'C'`，选择文件发送即可。

### 触发板端进入升级模式

- 已运行正常 App 的板：由 **App 侧**收到升级命令后写 `upgrade_flag=1` 并复位
  （兄弟 App 实现后置，见 §5）。
- 空片/无法启动的板：直接上电，Boot 检测无有效 App → 常驻升级模式 → 直接用工具发文件。
- 兜底：BOOT0 拉高进入 ST ROM Bootloader（USART1）可用 ST 工具整片恢复。

## 5. 兄弟工程后续集成清单（本 boot 仓库不含，按本契约实现）

**App（Master/Slaver 各自）：**
1. 链接脚本：FLASH ORIGIN=`0x08008000`、LENGTH=`0x18000`；工程启动早期设
   `SCB->VTOR = 0x08008000`（F103 支持 VTOR）。
2. `srv_com_mst/srv_com_slv` 将预留的升级命令（Master `0x06` / Slaver `0x06`，命令码两板已统一）
   改为：校验 metadata（`upgrade_flag=1` 写入 0x0803E000 区域）→ 应答 ACK → 复位；
   无 flash 参数存储的工程需引入与 Boot 相同的 `boot_flash`/`ring_storage`/`hal_flash`
   （选 `HAL_FLASH_CHIP_STM32F1`）以复用同一字节契约。
3. 应用侧记录/展示自身版本可由本契约 `version` 扩展。

> **z 帧寻址（V5.0.0 起）**：两兄弟板共用统一帧头（无 addr 字节），设备区分在 payload 首字节 ID：
> `[z][cmd][len][payload][crc][\n]`（E1_MASTER ID=0x01、E1_SLAVER ID=0x02）。
> 升级请求即定向发送：`z 06 02 01 01 crc \n`（Master，payload=[0x01,0x01]）/
> `z 06 02 02 01 crc \n`（Slaver，payload=[0x02,0x01]）。
> Boot 本身的 YMODEM 传输仍无地址（单点直连/独立进入升级模式）。

**Host GUI（`ctu_host/ctu_host.py` 双板调试上位机 + `boot_host.py` 升级工具）：**
1. 复用 §4 的 YMODEM 发送逻辑（或 pip 的 ymodem 库）。
2. 升级按钮流程：发定向升级命令（payload 首字节=目标 ID：Master `z 06 02 01 01` / Slaver `z 06 02 02 01`）
   → 等待板复位（约 1s）→ 进入 YMODEM 发送 →
   发送完成后等待 ~3s（Boot 提升+复位）→ 重新以 0x01 读状态确认新固件运行。

## 6. 版本记录

| 版本 | 日期 | 变更 |
|------|------|------|
| V1.0.0 | 2026-09-07 | 初始版：布局/契约/YMODEM 流程/后续集成清单 |
| V1.1.0 | 2026-09-08 | 集成同步：App 升级请求 z 帧带设备地址（Master addr=0x01 / Slaver addr=0x02）；YMODEM 本体不变 |
| V1.2.0 | 2026-09-08 | 命令码连续化同步：Master 升级 0x11→0x05、Slaver 升级 0x1F→0x06 |
| V1.3.0 | 2026-09-08 | 命令码两板统一同步：Master 升级 0x05→0x06（与 Slaver 一致）；Host GUI 文件名单更新为 `ctu_host/ctu_host.py` + `boot_host.py` |
