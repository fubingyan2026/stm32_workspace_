# srv_log_flash — WARN/ERROR 日志 Flash 持久化服务

将 log 模块的 **WARN/ERROR** 级别日志持久化到 Flash，掉电不丢失，供故障排查回溯。

采用 **"每条日志 = 一个 ring_storage 帧"** 模型：整块 28KB Flash 退化为真环形日志，写满整个区域才回绕擦除最旧扇区，实际保留 **204 条**。

---

## 目录

- [设计模型](#设计模型)
- [数据流](#数据流)
- [Flash 布局](#flash-布局)
- [容量计算](#容量计算)
- [关键设计点](#关键设计点)
- [API 参考](#api-参考)
- [配置参数](#配置参数)
- [控制台命令](#控制台命令)
- [依赖模块](#依赖模块)
- [调参权衡](#调参权衡)

---

## 设计模型

| 项 | 值 |
|----|----|
| 存储内容 | 仅 WARN（级别 2）及以上，ERROR（级别 1） |
| 存储单位 | 每条日志 = 1 个 ring_storage 帧 |
| 区域 | 0x08019000 ~ 0x08020000，28KB = **7 × 4KB 扇区** |
| 最大保留 | **204 条**（写满才回绕擦最旧） |
| 掉电安全 | 每帧独立 CRC + commit_magic，最多丢"正在写的那一条" |

为什么不用"整帧快照"：旧模型（V5）把 40 条打包成一帧 4KB 快照，扇区只能装 2 帧，每 2 次落盘就触发 GC 擦扇区，区域再大也不增加条数。改为每条一帧后帧体积 136B，4KB 扇区能装 30 帧，环形日志才真正成立。

---

## 数据流

```
log_log(WARN/ERROR)                        ← 任意上下文，可能 ISR
   │
   ▼
log_format_output()                        ← log.c，可在 ISR 执行
   │
   ▼
srv_log_flash_sink()                       ← 仅 RAM 入队（kfifo SPSC，无锁）
   │        ├── 剥离 ANSI 颜色码
   │        ├── UTF-8 截断回退 + 强制 \r\n
   │        └── kfifo_put（队满丢新，保留队内既有）
   ▼
srv_log_flash_step()                       ← log_task 的 10ms sw_timer，主循环
   │        ├── 窥视队首 → s_record（不弹出）
   │        ├── ring_storage_save()（每条一帧）
   │        └── 成功才 kfifo_skip 出队，失败保留下次重试
   ▼
ring_storage_save() → Flash
```

**核心约束**：sink 可能 ISR 上下文（见 log.c `log_format_output` 注释），因此 sink 内**禁止任何 Flash 操作**（毫秒级擦写不能进中断），只做 RAM 入队；所有 Flash 写都收敛在主循环 step。

---

## Flash 布局

```
0x08019000 ┬ 扇区 0 (4KB)  ─┐
0x0801A000 ┬ 扇区 1 (4KB)   │ 7 × 4KB
0x0801B000 ┬ 扇区 2 (4KB)   │ = 28KB
0x0801C000 ┬ 扇区 3 (4KB)   │ 环形轮转 + 懒擦除
0x0801D000 ┬ 扇区 4 (4KB)   │
0x0801E000 ┬ 扇区 5 (4KB)   │
0x0801F000 ┬ 扇区 6 (4KB)  ─┘
0x08020000 ┘ (128KB Flash 边界)
```

每个扇区内按版本号递增顺序追加帧：

```
扇区起始
  ├── 帧 vN   [帧头 28B | KV 数据 7B+98B | 帧尾 8B]  = 136B
  ├── 帧 vN+1 ...
  ├── ...（每扇区 30 帧 = 4080B）
  └── 0xFF...（剩余 16B 空白）
```

| 帧内字段 | 大小 | 说明 |
|---------|------|------|
| magic | 4B | 0x52535446 ("RSTF") |
| version | 4B | 全局单调递增，作时间序 |
| frame_len | 4B | 帧总逻辑大小 |
| kv_count | 4B | KV 数量（恒为 1） |
| header_crc32 | 4B | 帧头校验 |
| KV 数据 | 105B | key="logs"(4) + value=记录(98) |
| data_crc32 + commit_magic | 8B | 数据校验 + 原子提交点 |

单条记录 `srv_log_flash_record_t` = `level(1) + len(1) + text[96]` = **98B**。

---

## 容量计算

```
帧 Flash 体积 = 对齐8B(28 帧头尾 + 7 KV + 98 记录) = 136B
每扇区帧数   = floor(4096 / 136) = 30
最大保留条数 = 7 × 30 − (7−1) = 204
             （N 个扇区装满后相邻扇区边界各重复 1 帧，故减 N−1）
```

| 配置 | 帧体积 | 每扇区 | 扇区数 | 最大保留 |
|------|--------|--------|--------|---------|
| 96B 行 / 4KB 扇区 / 28KB（当前） | 136B | 30 | 7 | **204** |
| 96B 行 / 8KB 扇区 / 24KB | 136B | 60 | 3 | 178 |
| 64B 行 / 4KB 扇区 / 28KB | 104B | 39 | 7 | 267 |
| 128B 行 / 4KB 扇区 / 28KB | 168B | 24 | 7 | 162 |

`SRV_LOG_FLASH_MAX_FRAMES`（dump 收集数组上限）= 28672/136 + 2 = 212 ≥ 204。

---

## 关键设计点

### ISR 安全

- sink 只做 `kfifo_put`（SPSC 无锁，仅写 `in` 索引），可在 ISR 执行。
- step 主循环 `kfifo_peek` + `ring_storage_save` + `kfifo_skip`（仅写 `out` 索引）。
- 单写者单读者，生产/消费索引不交叉，无锁安全。

### 掉电保护

每帧三重校验：帧头 CRC + 数据 CRC + `commit_magic`（最后写入）。断电最多导致"正在写的那一条"帧无效，被自动跳过，**上一帧完整保留**。不丢整批。

### 限流与丢策略

- `FLUSH_MIN_MS=200ms`：两次落盘最小间隔，防高频错误触发连续擦写。
- 待落盘队列（kfifo 1KB ≈ 10 条）：突发错误先入队缓冲；**队满丢新**（保留队内既有记录），与 log TX 路径语义一致。
- 落盘失败保持队首，下个周期重试。

### 跨重启时间序

版本号跨重启全局递增（ring_storage 持久化），而 `millis()` 每次上电归零。因此 dump 按**版本号**排序输出，`W (4260)` 出现在 `W (320)` 前是跨重启的正常现象，不是乱序。

### 旧布局帧自动跳过

旧固件（V5，value 3932B）残留帧因 value 长度与当前注册值（98B）不匹配，`rs_parse_and_load_kv` 不复制（`val_len > value_len` 时 break），dump 靠 `len==0` 哨兵跳过。

---

## API 参考

| 函数 | 调用方 | 说明 |
|------|--------|------|
| `srv_log_flash_init()` | app_main（log_task_init 之后） | hal_flash + ring_storage 初始化，注册单条记录 KV，注册 log 落盘回调 |
| `srv_log_flash_step()` | log_task 的 sw_timer（10ms） | 队列有记录且到限流间隔时落盘一条 |
| `srv_log_flash_dump()` | log_task 命令 `log` | 遍历全部有效帧 → 按版本排序 → 输出 |
| `srv_log_flash_clear()` | log_task 命令 `logclear` | 整区擦除 + 丢弃待落盘队列 |

接线方式（log 模块提供落盘钩子，本服务注册回调）：

```c
/* log.h 提供的钩子（log.c 内部 log_format_output 调用） */
typedef void (*log_flash_sink_cb_t)(log_level_t level, const char* line, uint16_t len);
log_error_t log_set_flash_sink_cb(log_flash_sink_cb_t cb);

/* 本服务在 init 中注册 */
log_set_flash_sink_cb(srv_log_flash_sink);
```

依赖 ring_storage 新增的 3 个公共接口：

| 接口 | 用途 |
|------|------|
| `ring_storage_foreach_frame(ctx, cb, arg)` | dump 遍历所有有效帧（帧头 CRC + commit_magic 已校验） |
| `ring_storage_load_frame(ctx, frame_addr)` | dump 按地址加载指定帧（完整校验含数据 CRC） |
| `ring_storage_wipe(ctx)` | clear 整区擦除 + 重置运行时状态 |

---

## 配置参数

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `SRV_LOG_FLASH_ENABLE` | 1 | 总开关 |
| `SRV_LOG_FLASH_AREA_START` | 0x08019000 | 区域起点（4KB 对齐，终点为 128KB Flash 边界） |
| `SRV_LOG_FLASH_AREA_SIZE` | 28KB | 区域大小 = 7 × 4KB 扇区 |
| `SRV_LOG_FLASH_SECTOR_SIZE` | RING_STORAGE_SECTOR_4K | 逻辑扇区（= G4 单 Bank 物理页） |
| `SRV_LOG_FLASH_LINE_MAX` | 96 | 单条记录文本上限（当前最长 ~95B） |
| `SRV_LOG_FLASH_PENDING_BUFFER_SIZE` | 1024 | 待落盘队列缓冲（2 的幂） |
| `SRV_LOG_FLASH_FLUSH_MIN_MS` | 200 | 两次落盘最小间隔 |
| `SRV_LOG_FLASH_VERSION` | 6 | 布局版本标记 |

`SRV_LOG_FLASH_FRAME_FLASH_SIZE` / `MAX_FRAMES` / `MAX_RECORDS` 均为**编译期自动核算**，改上面任一宏会跟着重算，无需手动维护。

---

## 控制台命令

通过 USART1 控制台（log_task 命令解析）：

```
log        打印 Flash 中存储的 WARN/ERROR 日志（带总数/容量）
logclear   清空 Flash 日志（整区擦除）
```

dump 输出示例：

```
===== Flash 日志 (WARN/ERROR) 7/204 条 =====
W (4260) srv_can: 电机 0x21 长时间无响应（掉线或断电）
E (1200) drv_can: ch1 ERROR-PASSIVE (tx err accumulating, no ACK?)
...
===== 结束 =====
```

---

## 依赖模块

- `ring_storage`（m_middlewares/Third_Party）— 帧存储 / 环形轮转 / 断电保护
- `hal_flash` + `drv_stm32g4_flash`（public_layer/device_drivers）— Flash 读写擦
- `kfifo`（m_middlewares/utils）— ISR 安全的待落盘队列
- `log`（m_middlewares/log）— 落盘钩子 + 输出通道

---

## 调参权衡

| 目标 | 手段 | 代价 |
|------|------|------|
| 更多条数 | 调小 `LINE_MAX`（96→64 → 267 条） | 超长日志被截断（当前最长 ~95B，需先确认） |
| 更多条数 | 扩大区域 / 换 16KB 扇区 | 占用更多 Flash，需 16KB 对齐重排起点 |
| 故障风暴捕获率 | 调小 `FLUSH_MIN_MS`（200→50） | 落盘频率 ↑，磨损 ↑（仍远低于旧快照模型） |
| 减少 RAM | 调小 `PENDING_BUFFER_SIZE` | 突发错误缓冲变小，更易丢新 |

RAM 占用（当前配置）：`s_record` 98B + `s_frame_buf` 162B + 待落盘队列 1KB + dump 数组 212×8B ≈ **3KB**。
