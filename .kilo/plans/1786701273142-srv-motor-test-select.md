# 统一 CAN1 电机测试模块选择头文件（srv_motor_test_select.h）

## 目标

当前 CAN1(FDCAN1) 上三个测试模块靠两个正交宏分别选择，接线分散：
- `service/srv_ht_test_mode.h` 的 `SRV_HT_TEST_MODE_TORQUE`（1=ht_torque，0=ht_temp）；
- `service/srv_tongzhi_torque_test.h` 的 `SRV_TONGZHI_TORQUE_TEST_ENABLE`（当前被硬编码为 1）。

本次重构：**将 `srv_ht_test_mode.h` 重命名为 `service/srv_motor_test_select.h`**，
用「一个枚举 + 一个选择宏」统一表示并切换 CAN1 上运行的测试模块；`can_task.c` / `srv_can.c`
只按该枚举接线。PA430 走 CAN2，保持 `SRV_PA430_TORQUE_TEST_ENABLE` 独立不变。

## 已确认决策

| 项 | 决策 |
|----|------|
| 选择宏 | `SRV_MOTOR_TEST_SELECT`，可被 `-DSRV_MOTOR_TEST_SELECT=...` 覆盖 |
| 默认值 | `SRV_MOTOR_TEST_TONGZHI`（保持当前烧录行为不变） |
| 枚举 | `srv_motor_test_sel_t`，三个成员：HT_TORQUE / HT_TEMP / TONGZHI |
| PA430 | 不在本选择内，继续由 `srv_pa430_torque_test.h` 的 `SRV_PA430_TORQUE_TEST_ENABLE` 控制 |
| 旧宏 | `SRV_HT_TEST_MODE_TORQUE`、`SRV_TONGZHI_TORQUE_TEST_ENABLE` 全部移除，无兼容别名 |

## 新头文件 `service/srv_motor_test_select.h`（由 srv_ht_test_mode.h 重命名改写）

```c
#ifndef __SRV_MOTOR_TEST_SELECT_H
#define __SRV_MOTOR_TEST_SELECT_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- 枚举：表示当前运行的 CAN1 测试模块 --- */
typedef enum {
    SRV_MOTOR_TEST_SEL_HT_TORQUE = 0, /* 苇熠 位置模式往复耐久（srv_ht_torque_test） */
    SRV_MOTOR_TEST_SEL_HT_TEMP   = 1, /* 苇熠 速度模式测试（srv_ht_temp_test） */
    SRV_MOTOR_TEST_SEL_TONGZHI   = 2, /* 良志(ODrive) 位置模式往复（srv_tongzhi_torque_test） */
    SRV_MOTOR_TEST_SEL_NUM
} srv_motor_test_sel_t;

/* --- 预处理选择值（#if 比较用，值须与枚举一致） --- */
#define SRV_MOTOR_TEST_HT_TORQUE 0
#define SRV_MOTOR_TEST_HT_TEMP   1
#define SRV_MOTOR_TEST_TONGZHI   2

/* --- 当前选择（命令行 -DSRV_MOTOR_TEST_SELECT=SRV_MOTOR_TEST_HT_TORQUE 等可覆盖） --- */
#ifndef SRV_MOTOR_TEST_SELECT
#define SRV_MOTOR_TEST_SELECT SRV_MOTOR_TEST_TONGZHI
#endif

/* --- 便捷判定宏（供 #if 使用） --- */
#define SRV_MOTOR_TEST_IS_HT_TORQUE (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_HT_TORQUE)
#define SRV_MOTOR_TEST_IS_HT_TEMP   (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_HT_TEMP)
#define SRV_MOTOR_TEST_IS_TONGZHI   (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_TONGZHI)

#ifdef __cplusplus
}
#endif

#endif /* __SRV_MOTOR_TEST_SELECT_H */
```

> 说明：`SRV_MOTOR_TEST_SELECT` 默认值用**宏值**而非枚举成员，因为枚举常量不是预处理器
> 符号，`#if` 无法比较；枚举类型 `srv_motor_test_sel_t` 供运行时表示/switch 使用，两者值一一对应。

## 改动清单（按顺序执行）

1. **重命名并重写头文件**
   - `git mv service/srv_ht_test_mode.h service/srv_motor_test_select.h`，再按上面内容改写。
   - 删除后不应再有任何文件 `#include "srv_ht_test_mode.h"`。

2. **`tasks/can_task.c`**
   - `#include "srv_ht_test_mode.h"` → `#include "srv_motor_test_select.h"`。
   - 顶部接线块改为：
     ```c
     #if SRV_MOTOR_TEST_IS_TONGZHI
     #include "srv_tongzhi_torque_test.h"
     #define CAN1_TEST_INIT srv_tongzhi_torque_test_init
     #define CAN1_TEST_STEP srv_tongzhi_torque_test_step
     #define CAN1_TEST_ON_RX srv_tongzhi_torque_test_on_rx
     #define CAN1_TEST_USE_SRV_CAN 0
     #elif SRV_MOTOR_TEST_IS_HT_TORQUE
     #include "srv_ht_torque_test.h"
     #define CAN1_TEST_INIT srv_ht_torque_test_init
     #define CAN1_TEST_STEP srv_ht_torque_test_step
     #define CAN1_TEST_ON_RX srv_can_on_rx
     #define CAN1_TEST_USE_SRV_CAN 1
     #elif SRV_MOTOR_TEST_IS_HT_TEMP
     #include "srv_ht_temp_test.h"
     #define CAN1_TEST_INIT srv_ht_temp_test_init
     #define CAN1_TEST_STEP srv_ht_temp_test_step
     #define CAN1_TEST_ON_RX srv_can_on_rx
     #define CAN1_TEST_USE_SRV_CAN 1
     #else
     #error "SRV_MOTOR_TEST_SELECT 值无效"
     #endif
     ```
   - `can_task_init()`：`#if !SRV_TONGZHI_TORQUE_TEST_ENABLE` 包住的 `srv_can_init()` 改为
     `#if CAN1_TEST_USE_SRV_CAN`。
   - `can_timer_cb()`：同样把 `srv_can_process()` 的 `#if !SRV_TONGZHI_TORQUE_TEST_ENABLE`
     改为 `#if CAN1_TEST_USE_SRV_CAN`。
   - 顶部注释与 RX 分发注释同步改为引用 `srv_motor_test_select.h`。

3. **`service/srv_can.c`**
   - `#include "srv_ht_test_mode.h"` → `#include "srv_motor_test_select.h"`。
   - 测试帧路由宏改为：
     ```c
     #if SRV_MOTOR_TEST_IS_HT_TORQUE
     #include "srv_ht_torque_test.h"
     #define HT_TEST_ON_RX srv_ht_torque_test_on_rx
     #elif SRV_MOTOR_TEST_IS_HT_TEMP
     #include "srv_ht_temp_test.h"
     #define HT_TEST_ON_RX srv_ht_temp_test_on_rx
     #else
     /* 良志(TONGZHI)：CAN1 帧由 can_task 直连 srv_tongzhi_torque_test_on_rx，
        srv_can 不参与路由；占位实现仅保证编译 */
     static bool srv_can_ht_rx_placeholder(const drv_can_msg_t* msg)
     {
         (void)msg;
         return false;
     }
     #define HT_TEST_ON_RX srv_can_ht_rx_placeholder
     #endif
     ```
   - 文件头注释（第 10-13 行）同步更新。

4. **`service/srv_tongzhi_torque_test.h`**
   - 删除 `#define SRV_TONGZHI_TORQUE_TEST_ENABLE 1`（第 37 行）与下方 `#ifndef ... 0` 块
     （第 44-50 行，含「接线开关」注释段）——选择权移交 `srv_motor_test_select.h`。
   - 注释里引用 `srv_ht_test_mode.h` 的地方改为 `srv_motor_test_select.h`。

5. **`service/srv_pa430_torque_test.h`**
   - 第 44 行注释中引用 `srv_ht_test_mode.h` 处改为 `srv_motor_test_select.h`。

6. **文档**
   - `AGENTS.md` 第 33 行：CAN1 测试模块选择改为引用 `service/srv_motor_test_select.h`
     的 `SRV_MOTOR_TEST_SELECT`（HT_TORQUE/HT_TEMP/TONGZHI 三选一）。
   - `docs/srv_ht_torque_test.md` 第 10 行：激活方式改为
     `srv_motor_test_select.h` 中 `SRV_MOTOR_TEST_SELECT = SRV_MOTOR_TEST_HT_TORQUE`。

## 验证

1. **默认构建**：`cmd /c build.bat` 通过；默认 TONGZHI，`can_task.o` 引用
   `srv_tongzhi_torque_test_*`、不引用 `srv_can_on_rx`，行为与当前烧录固件一致。
2. **覆盖构建（三种都试）**：
   - `cmake --preset Debug -DSRV_MOTOR_TEST_SELECT=SRV_MOTOR_TEST_HT_TORQUE`（或命令行 `-D`）
     后构建：`can_task.o` 引用 `srv_ht_torque_test_*` + `srv_can_on_rx`；
   - `SRV_MOTOR_TEST_HT_TEMP` 同理引用 `srv_ht_temp_test_*`；
   - 两个 HT 模式 `SRV_MOTOR_TEST_USE_SRV_CAN` 路径均编译，无未定义符号。
   - 注意：预设用 `cmake --preset Debug`，命令行宏用 `CMAKE_C_FLAGS` 追加
     `-DSRV_MOTOR_TEST_SELECT=SRV_MOTOR_TEST_HT_TORQUE`（实现时确认 CMake 写法）。
3. **运行时**：分别烧录三种选择，控制台出现对应模块日志（`ht_torque` / `ht_temp` /
   `tongzhi_test` 启动行），且同一时刻只有选中模块参与 CAN1 收发。
4. **回归**：PA430（FDCAN2）路径不受影响，`SRV_PA430_TORQUE_TEST_ENABLE` 照旧。

## 边界 / 风险

- 枚举与宏值必须一一对应：改枚举序号的提交必须同步改 `SRV_MOTOR_TEST_HT_*` 三个宏，否则 `#if` 误判。
- `srv_can.c` 在 TONGZHI 模式下仍会编译，占位 `HT_TEST_ON_RX` 只在编译期存在、运行期不会被调用
  （can_task 在 TONGZHI 下不调用 `srv_can_on_rx`）。
- `SRV_MOTOR_TEST_SELECT` 若在命令行用枚举名（如 `=SRV_MOTOR_TEST_TONGZHI`）传值，因枚举名不是
  宏，`#if` 会当作 0 误判——文档与注释须强调**命令行覆盖应使用宏名或整型字面量**。
