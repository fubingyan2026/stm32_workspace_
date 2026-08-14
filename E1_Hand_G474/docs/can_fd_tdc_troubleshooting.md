# CAN2 CAN FD 发送失败排查总结（PA430 测试）

## 1. 问题现象
- PA430（Motorevo）伺服在 FDCAN2（PB12 RX / PB13 TX，仲裁 1M / 数据 5M）上运行 MIT 测试。
- 发 CAN FD 帧时：PB13 只发几个电平就中止（bit 错误），CANH/CANL 无差分输出，
  检测器收不到任何帧，日志持续 ERROR-PASSIVE → BUS-OFF。
- 改用经典 CAN（DLC 8，全程 1M）后完全正常。

## 2. 根因
FDCAN2 未使能发送延迟补偿（TDC，Transmitter Delay Compensation）。
- 5M 数据段位时间仅 200ns，与收发器环路延迟（TXD→总线→RXD，约 100~250ns）同量级。
- 无 TDC 时，FDCAN 位监测（bit monitoring）在采样点读到的是上一比特 → bit 错误 → 帧在数据段开始即中止。
- 经典 CAN（1M 位时间 1µs）环路延迟可忽略，故不受影响。

## 3. 修复
`device_drivers/drv_can.c` 的 `drv_can_init()`，每通道在 `HAL_FDCAN_Start` 前（State==READY）：
```c
HAL_FDCAN_ConfigTxDelayCompensation(s_hfdcan[ch], DRV_CAN_TDC_OFFSET, DRV_CAN_TDC_FILTER);
HAL_FDCAN_EnableTxDelayCompensation(s_hfdcan[ch]);  /* 置 DBTP.TDC */
```
- `DRV_CAN_TDC_OFFSET 8U`（TDCO，mtq=6.25ns @160MHz）、`DRV_CAN_TDC_FILTER 4U`（TDCF）。
- 控制器自动测量环路延迟（PSR.TDCV），TDCO 仅附加余量（SSP = TDCV + TDCO）。
- 两路 FDCAN 都启用（TDC 只作用于 FD 数据段，经典 CAN 不受影响）。

## 4. 关键诊断方法（经验）
1. **错误计数**：TEC 每帧 +8、rec=0，结合帧发送节拍确认「发帧但无人应答/位错误」。
2. **波形判断**：「PB13 只发几比特即中止」= bit 错误中止（位监测不匹配），
   不是固件没发完整帧——固件正确送入 TX FIFO 并开始发送，中止是硬件位错误的响应。
3. **PSR.LEC 陷阱**：LEC 在每次读 PSR 后复位为 7（No Change），5ms 轮询下几乎恒读到 7，
   不能用轮询读 LEC 判断错误类型。
4. **经典 CAN 隔离**：经典与 FD 的仲裁段（1M）完全相同，经典正常 + FD 失败 → 定位 FD 数据段（5M）。
5. **TDC 规则**：数据速率 >5Mbps 时收发器环路延迟不可忽略，必须使能 TDC。

## 5. 顺带完成 / 踩坑
- 双 CAN 并行：苇熠(HT) 测试在 CAN1、PA430 在 CAN2 同时运行
  （原单模式宏会顶替掉 CAN1 的苇熠测试）。
- drv_can 增加 FDCAN2 通道支持（句柄表 + `DRV_CAN_CH_2`）。
- 曾用「ECR 优先读 LEC」尝试捕获错误类型，但引入「bus-off 后 TEC 冻结 → PSR 永不读 →
  恢复序列不触发」回归，已回退为每轮读 PSR。
- 保留诊断开关：经典 CAN 测试 `SRV_PA430_CLASSIC_TEST`（默认 0）、
  环回自测 `DRV_CAN_LOOPBACK_TEST`（默认 0）。

## 6. 改动文件清单
- `device_drivers/drv_can.c`：FDCAN2 支持、TDC 使能、诊断日志（act/lec/tec/rec）、环回自测开关。
- `device_drivers/drv_can.h`：新增 `DRV_CAN_CH_2`。
- `service/srv_pa430_torque_test.c/.h`：走 FDCAN2、经典 CAN 诊断开关。
- `tasks/can_task.c`：双通道并行路由（CH_1→srv_can，CH_2→pa430）。
- `service/srv_can.c`：移除 PA430 路由分支。
