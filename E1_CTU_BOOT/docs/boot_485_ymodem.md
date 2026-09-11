# E1_CTU_BOOT：RS485 寻址分块 Bootloader 协议与契约

- **作者**：maximillian
- **日期**：2026-09-11
- **目标**：`E1_MASTER_POWER_CTU` / `E1_SLAVER_POWER_CTU`（STM32F103RCTx，256KB Flash）共用的 RS485 固件升级 Bootloader
- **传输**：**z 帧寻址分块协议**（复用兄弟工程信封 + 块内 CRC16），115200-8N1，USART3（PB10/PB11）+ PC5(DE)

---

## 1. Flash 分区布局

| 区域 | 起始地址 | 大小 | 说明 |
|------|----------|------|------|
| Boot | `0x08000000` | 32K（0x8000） | 本 boot（Release 构建 ~27KB） |
| App A | `0x08008000` | 96K（0x18000） | **运行槽**：App 固定链接地址，只从 A 启动 |
| App B | `0x08020000` | 96K（0x18000） | 暂存/备份槽：新固件下载目标 |
| 保留 | `0x08038000` | 24K（0x6000） | 预留（后续 App 参数区等） |
| Metadata | `0x0803E000` | 8K（0x2000） | `boot_metadata_t`（ring_storage，4×2KB 页） |

编译期宏（与本地 `service/boot_flash.{c,h}` 对齐）：
`BOOT_FLASH_BOOT_SIZE=0x8000U`、`BOOT_FLASH_APP_SIZE=0x18000U`、
`BOOT_FLASH_META_SIZE=0x2000U`、`BOOT_META_OFFSET=0x3E000U`、
`BOOT_META_SECTOR_SIZE=0x800U`。

> F103xE Flash 全片 2KB 均匀页；A/B 边界（0x08020000/0x08038000/0x0803E000）均 2KB 对齐。

## 2. 运行模型（单链接 A + B 暂存提升）

App 永远按 **A 槽（0x08008000）** 链接（一套链接脚本即可）。
升级流程：

```
App 收到升级命令（Master 0x06 / Slaver 0x06，两板命令码统一）→ 置 metadata upgrade_flag=1 + 本机 ID → 复位
Boot 启动 → 读到 flag=1 → 静默等待主机寻址 SELECT(0x06)
   ↓ START(0x08) 擦除 B → DATA(0x09)×N 写入 B → END(0x0A)
写 metadata {flag=2, fw_size, fw_checksum}  →  promote_to_a(B→A)  →  写 {flag=0}  →  复位
Boot 启动 → A 校验通过 → 跳转 A 运行新固件
```

> **多节点/半双工安全**：Boot 绝不主动发送（无 'C' 心跳），只在被寻址帧
> （payload 首字节 = 本机 ID，或 0x00 广播）校验通过后回一帧；同一总线多个
> 节点同时处于 Boot 也不会争总线。总线上只有主机发起通信。

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
| `reserved` | u32 | 低字节 = 本机设备 ID（App 请求升级时写入；Boot 未定时可由广播学习） |

> App 侧将来实现"跳转 Boot"服务（如 `srv_boot_ctrl`）时，必须复用同一布局与地址
> （0x0803E000，ring_storage，4×2KB 页，sector_size=0x800，write_gran=32）。

## 3. 传输协议细节（z 帧寻址分块）

信封（与 MASTER/SLAVER 完全一致，CRC8 覆盖 `z..payload`，poly 0x8C 初值 0xFF）：

```
[ 'z' ][ cmd ][ len ][ payload... ][ CRC8 ][ '\n' ]
payload[0] = 目标设备 ID（0x01=MASTER / 0x02=SLAVER / 0x00=广播）
```

| 命令 | 方向 | payload | 应答 |
|------|------|---------|------|
| `0x06` SELECT | 主机→板 | `[id][0x01]` | `0x86 [id][err]` 选中本机 |
| `0x08` START | 主机→板 | `[id][size u32][sum u32]` | `0x88 [id][err]` 擦除 B 槽 |
| `0x09` DATA | 主机→板 | `[id][blk u16][crc16 u16][data ≤248B]` | `0x89 [id][err][blk u16]` |
| `0x0A` END | 主机→板 | `[id]` | `0x8A [id][err]` → 校验并提交复位 |
| `0x0B` ABORT | 主机→板 | `[id]` | `0x8B [id][err]` → 放弃并复位 |

- 数据块：`blk` 为 16 位块号（偏移 = blk × 248），块内 CRC16/xmodem；设备校验
  通过、写入并读回校验后才 ACK；错误/超时由主机重发该块。
- `err`：0x00=OK / 0x01=帧长 / 0x02=状态 / 0x03=块号 / 0x04=CRC16 / 0x05=Flash / 0x06=长度容量。
- 总校验 `sum` = 固件字节 32 位累加和（`&0xFFFFFFFF`），与 Boot metadata 一致。
- 设备 ID 未知（空片）时接受广播或任意 ID 并学习（写入 meta.reserved）。
- 最长固件 ≤ 96KB；`data` 上限 248B，单帧 ≤ 258B。

## 4. 主机工具用法

本仓库 `host/` 提供三层实现（协议层与界面分离）：

| 文件 | 说明 |
|------|------|
| `boot_protocol.py` | 寻址分块协议层（无 GUI 依赖）：帧编解码 + `BootProtoSender` + 升级请求 |
| `boot_host.py` | PySide6 上位机（GUI）：串口 / 目标设备 / 固件选择 / 进度与日志 |
| `boot_send.py` | CLI 发送（复用协议层） |

CLI：

```
python host/boot_send.py COM5 E1_MASTER_POWER_CTU.bin 115200 master [--req]
```

GUI：

```
python host/boot_host.py
```

> 与 YMODEM 不同，本协议数据帧不兼容 Tera Term 等通用工具，须使用本仓库 host。

### 触发板端进入升级模式

- 上位机**统一先发一帧 `0x06`**（与设备 ID 定向）：
  - 目标在运行 App：App 视为升级请求 → 写标志并复位进入 Boot；
  - 目标已在 Boot：Boot 视为 `SELECT` → 选中会话并回 `0x86`（幂等，可重复）。
- 广播 ID `0x00` 对运行中的 App 无效（App 按 ID 过滤），仅用于已在 Boot/空片。
- 空片/无法启动的板：直接上电，Boot 静默等待被寻址 → 直接用工具发（可先 0x06 选中）。
- 兜底：BOOT0 拉高进入 ST ROM Bootloader（USART1）可用 ST 工具整片恢复。

## 5. 兄弟工程后续集成清单（本 boot 仓库不含，按本契约实现）

**App（Master/Slaver 各自）：**
1. 链接脚本：FLASH ORIGIN=`0x08008000`、LENGTH=`0x18000`；工程启动早期设
   `SCB->VTOR = 0x08008000`（F103 支持 VTOR）。
2. `srv_com_mst/srv_com_slv` 将预留的升级命令（Master `0x06` / Slaver `0x06`，命令码两板已统一）
   改为：写 metadata（`upgrade_flag=1` 且 `reserved` 低字节=本机 ID 0x01/0x02）→ 应答 ACK → 复位；
   升级跳转服务 `srv_boot_ctrl` 复用 Boot 工程持有的 `boot_flash`/`ring_storage`/`hal_flash`
   （选 `HAL_FLASH_CHIP_STM32F1`）以复用同一字节契约。
3. 应用侧记录/展示自身版本可由本契约 `version` 扩展。
4. `0x07` 读 Boot/固件信息（App 侧实现）：只读 metadata（`boot_flash_peek_metadata`，不写 Flash）
   返回 app_ver/meta_ver/fw_size/fw_checksum/reboot/flags，便于升级前确认。
   **Boot 传输命令 `0x08–0x0F` 保留**，App 不得占用。

> **z 帧寻址**：两兄弟板共用统一帧头（无 addr 字节），设备区分在 payload 首字节 ID：
> `[z][cmd][len][payload][crc][\n]`（E1_MASTER ID=0x01、E1_SLAVER ID=0x02）。
> 升级请求：`z 06 02 01 01 crc \n`（Master，payload=[0x01,0x01]）/
> `z 06 02 02 01 crc \n`（Slaver，payload=[0x02,0x01]）。
> **Boot 模式下的分块数据传输也带同一设备 ID**（见 §3），因此一主多从、多节点
> 同时处于 Boot 也可安全寻址升级。

**Host（`boot_host.py` / `boot_send.py`）：**
1. 升级流程：发定向升级命令（payload 首字节=目标 ID）→ 等 App 复位 → `SELECT(0x06)`
   选中 Boot → `START(0x08)` → 循环 `DATA(0x09)`（按块应答/重传）→ `END(0x0A)` →
   等待 Boot 提交复位 → 重新读状态确认新固件运行。

## 6. 版本记录

| 版本 | 日期 | 变更 |
|------|------|------|
| V1.0.0 | 2026-09-07 | 初始版：布局/契约/YMODEM 流程/后续集成清单 |
| V1.1.0 | 2026-09-08 | 集成同步：App 升级请求 z 帧带设备地址（Master addr=0x01 / Slaver addr=0x02）；YMODEM 本体不变 |
| V1.2.0 | 2026-09-08 | 命令码连续化同步：Master 升级 0x11→0x05、Slaver 升级 0x1F→0x06 |
| V1.3.0 | 2026-09-08 | 命令码两板统一同步：Master 升级 0x05→0x06（与 Slaver 一致）；Host GUI 文件名单更新为 `ctu_host/ctu_host.py` + `boot_host.py` |
| V2.0.0 | 2026-09-11 | **方案 B**：Boot 升级传输由 YMODEM 改为 z 帧寻址分块（SELECT/START/DATA/END/ABORT，含设备 ID + 块 CRC16）；取消 `'C'` 心跳与自发发送，支持多节点同时 Boot；App 请求升级时写 meta.reserved=本机 ID；host 更新为 `boot_protocol.py`+`boot_host.py`+`boot_send.py` |
