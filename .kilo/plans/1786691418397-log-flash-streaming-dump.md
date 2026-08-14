# 计划：Flash 日志 dump 流式输出 + dump 期间屏蔽实时日志

> 本文件包含两阶段：**阶段一（已实施）** 流式 dump 背压续传；**阶段二（本计划）**
> dump 期间屏蔽实时日志，避免干扰 flash 历史日志输出。

---

## 阶段一（已实施，背景参考）

- `log_tx_avail()` 新增于 log 中间件；`srv_log_flash_dump()` 改为收集+排序+启动状态机；
  `srv_log_flash_dump_step()` 由 log_task 10ms sw_timer 驱动，`log_tx_avail() >= 128B` 时
  逐条续传，停滞 >2s 中止；`srv_log_flash_clear()` 中止进行中的 dump。
- E1_Hand_G474 / stm32_g474_boot / E1_Master_Power_Manage 均已构建通过。

## 问题（阶段二）

dump 与实时日志共用同一 TX kfifo（`log.c:474` 的 `kfifo_put` 与 dump 的 `log_write` 同缓冲），
UART 按 FIFO 顺序发送。故 dump 期间：
1. 任何 `LOG_*`（主循环/ISR，如 CAN 错误、电机服务日志）会插在两条 flash 记录之间，
   历史记录与实时记录视觉混排、无法区分。
2. 实时日志洪泛占满 TX 缓冲时，`log_tx_avail()` 长期 <128B，dump 停等，超 2s 看门狗误中止。

**用户决策：dump 期间屏蔽实时日志（`log_log`/`log_hexdump` 的 TX 输出），dump 自身
（`log_write` 通道）不受影响；实时 WARN/ERROR 仍走落盘钩子持久化，不丢失。**

---

## 阶段二：实现方案

### 核心思路

屏蔽必须落在 log 中间件的 TX 写入路径（srv_log_flash 无法阻止其他模块调用 `LOG_*`）。
新增全局开关 `log_hold_output(bool)`：置位期间 `log_format_output()` 跳过
`kfifo_put(&s_tx_fifo, ...)`，但**保留落盘钩子调用**；`log_hexdump()` 直接丢弃；
`log_write()` 不受影响（dump 的专用通道）。默认 false，对 Boot 及其它工程零行为变化。

### 改动 1：`public_layer/m_middlewares/log/log.h`

在 `log_set_timestamp_enable` 声明附近新增：

```c
/**
 * @brief 暂停/恢复格式化日志（LOG_*/hexdump）的 TX 输出
 * @param hold true=暂停（仅限 log_write 仍可输出），false=恢复
 * @note  用于 Flash 日志 dump 期间屏蔽实时日志、避免与历史记录混排。
 *        暂停仅屏蔽 TX 队列写入；若已注册落盘钩子，WARN/ERROR 仍会持久化不丢失。
 *        log_write() / log_tx_*() 不受影响。
 */
log_error_t log_hold_output(bool hold);
```

### 改动 2：`public_layer/m_middlewares/log/log.c`

1. 新增静态标志：
   ```c
   /** @brief TX 输出暂停标志（dump 期间屏蔽实时日志） */
   static bool s_output_held = false;
   ```
2. 新增实现（镜像 `log_set_color_enable` 风格，`log.c:192` 附近）：
   ```c
   log_error_t log_hold_output(bool hold)
   {
       if (!s_initialized) {
           return LOG_ERROR_UNINITIALIZED;
       }
       s_output_held = hold;
       return LOG_OK;
   }
   ```
3. `log_format_output()`（`log.c:474`）：把 kfifo_put 加门控，落盘钩子（477-479）不动：
   ```c
   if (!s_output_held) {
       kfifo_put(&s_tx_fifo, (const uint8_t*)buf, offset);
   }
   /* 落盘钩子保持不变：暂停 TX 期间 WARN/ERROR 仍持久化 */
   if (s_flash_sink_cb != NULL) {
       s_flash_sink_cb(level, buf, offset);
   }
   ```
4. `log_hexdump()`（`log.c:323`）：函数起始（len==0 检查之后）加
   `if (s_output_held) { return LOG_OK; }`，整段丢弃（实时诊断输出，不触发落盘钩子）。

> 并发说明：`s_output_held` 为单字节标志，主循环写、ISR 读（`log_format_output` 可能在
> ISR 执行），与现有 `s_current_level` 同模式，无需临界区。

### 改动 3：`public_layer/service/srv_log_flash.c`

持有/释放必须与 `s_dump_active` 状态完全同步（置 true 必配 hold true，置 false 必配 hold false）。

1. `srv_log_flash_dump()`：在确认 `s_dump_count > 0` 后、**打印 header 之前**置位，缩小时隙：
   ```c
   (void)log_hold_output(true);
   ```
   （count==0 分支提前 return，不置位。）
2. `srv_log_flash_dump_step()` 两处释放：
   - 停滞中止分支（`s_dump_active = false` 之前）：`(void)log_hold_output(false);`
   - 正常完成分支（打印"===== 结束 =====" 后）：`(void)log_hold_output(false);`
3. `srv_log_flash_clear()`：与 `s_dump_active = false` 一同 `(void)log_hold_output(false);`

> dump 内部自打印（header/记录/结束语/中止语）全部走 `log_write` 或
> `SRV_LOG_FLASH_LOG_STR`（即 `log_write`），不受 hold 影响，输出完整。
> `srv_log_flash_step()` 落盘失败时的 `SRV_LOG_FLASH_LOG_E` 属 `log_log` 路径会被暂停显示，
> 但其 ERROR 级别仍经落盘钩子持久化，可事后回查。

### 改动 4：`public_layer/service/srv_log_flash_README.md`

- 读取路径小节补充一句："dump 进行期间通过 `log_hold_output(true)` 屏蔽实时 `LOG_*`
  输出，仅 dump（`log_write` 通道）与实时 WARN/ERROR 落盘钩子继续工作；dump 结束/中止/clear 时恢复。"
- 配置表中可补一行 `log_hold_output`（非宏，仅说明）。

---

## 数据流（阶段二后）

```
log 命令 → srv_log_flash_dump()   log_hold_output(true) → 收集/排序 → 打印头 → s_dump_active=true
每个 10ms tick：
  log_timer_cb  TX 块排空 → srv_log_flash_dump_step()（背压逐条 log_write）
                期间所有 LOG_*/hexdump 被 hold 屏蔽（WARN/ERROR 仍落盘）
全部输出 → "===== 结束 =====" → log_hold_output(false)
停滞 >2s  → "超时中止" → log_hold_output(false)
logclear  → 中止 dump → log_hold_output(false)
```

## 边界与失败模式

- **重复 `log` 命令**：dump() 重收集、hold(true) 幂等。
- **`logclear` 竞态**：clear 释放 hold，实时日志立即恢复。
- **LOG_OUTPUT_NONE 停滞**：中止路径释放 hold，不再长期静默。
- **Boot / 其它工程**：srv_log_flash 不编入 Boot；hold 默认 false，零行为变化；
  E1_Master_Power_Manage 共用同一 srv_log_flash.c/log.c，自动获得该特性。
- **hold 卡死风险**：置 true 的唯一入口是 dump()（count>0），释放覆盖全部
  `s_dump_active=false` 路径（中止/完成/clear），且停滞看门狗保证中止必发生，
  不存在长期静默路径。
- **代价**：dump 期间（约 2s）控制台无实时日志；ERROR 不显示但已落盘，可随后回查。

## 验证计划

1. 构建：E1_Hand_G474 `cmd /c build.bat`；回归 stm32_g474_boot 与 E1_Master_Power_Manage
   （log.c 改动影响共享中间件，需确认 Boot 目标 E1_BUILD_BOOT 下无新符号引用）。
2. 功能：制造 ≥204 条 WARN/ERROR + 期间持续打印 INFO 日志，发 `log`：
   - 输出为干净连续的 flash 历史块（无 INFO 插入），首尾标记完整；
   - dump 结束后实时日志立即恢复；
   - 期间新产生的 WARN/ERROR 未被丢弃（clear 后重新 dump 可看到，或对比计数）。
3. 竞态：dump 进行中 `logclear` → 实时日志立即恢复。
4. 停滞：LOG_OUTPUT_NONE 下 `log` → 约 2s 后"超时中止"且实时日志恢复。
