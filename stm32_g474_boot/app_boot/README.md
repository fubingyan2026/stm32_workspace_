# G474 App 适配 Boot 引导 — 操作手册

本目录提供 STM32G474 App 工程接入 `stm32_g474_boot` 引导所需的**全部文件**。

## 目录文件

| 文件 | 来源 | 用途 |
|---|---|---|
| `STM32G474XX_APP.ld` | 新建 | App 链接脚本 (0x08010000, 48K) |
| `srv_boot_ctrl.c/h` | 新建 | App 侧 Boot 控制服务 (metadata 管理 + request_boot) |
| `hal_flash.h` / `hal_flash_base.h` / `hal_flash.c` | public_layer | Flash 抽象层 |
| `drv_stm32g4_flash.h` / `drv_stm32g4_flash.c` | public_layer | G474 Flash 驱动 (64-bit 双字编程) |
| `drv_stm32f4_flash.h` / `drv_stm32h7_flash.h` | public_layer | 其他芯片 stub (hal_flash.c 所需) |
| `ring_storage.h` / `ring_storage_port.h` / `ring_storage.c` | public_layer | Flash KV 存储 (metadata 区) |
| `ring_storage_port_hal.h` / `ring_storage_port.c` | public_layer | ring_storage → hal_flash 桥接层 |

> 所有库文件已去除 log 依赖（`LOG_ENABLE=0`），无需 log 模块即可编译。

## Flash 布局

```
0x08000000 ┌──────────────┐
           │   Bootloader │ 64 KB  (0x00000 - 0x0FFFF)
0x08010000 ├──────────────┤
           │   App        │ 48 KB  (0x10000 - 0x1BFFF)  ← App 链接于此
0x0801C000 ├──────────────┤
           │   Metadata   │ 16 KB  (0x1C000 - 0x1FFFF)  ring_storage KV
0x08020000 └──────────────┘
```

---

## 操作步骤

### 1. 复制文件

将本目录下**所有文件**复制到 App 工程，放在 `app_boot/` 子目录下（名称随意）。

### 2. 替换链接脚本

删除 App 工程原有的 `STM32G474XX_FLASH.ld`（如存在），`app_boot/STM32G474XX_APP.ld` 替代。

### 3. 修改 CMakeLists.txt

```cmake
# ===== App 链接分区 0x08010000 =====
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-T,${CMAKE_SOURCE_DIR}/app_boot/STM32G474XX_APP.ld
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
)

# ===== app_boot 库文件 =====
aux_source_directory(${CMAKE_SOURCE_DIR}/app_boot APP_BOOT)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_BOOT}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    app_boot
)

# ===== 编译宏 =====
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    HAL_FLASH_CHIP_STM32G4 
    # App 向量表重定位到 0x08010000
    # (system_stm32g4xx.c 的 SystemInit 中 SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET)
    USER_VECT_TAB_ADDRESS
    VECT_TAB_OFFSET=0x00010000U
)
```

> **无需**额外添加 `public_layer` include 路径或源文件——所有依赖已在本目录内。

### 4. App 代码调用

```c
#include "srv_boot_ctrl.h"

// 启动时调用一次（幂等）
srv_boot_ctrl_init();

// 需要升级时调用（例如收到 CAN ID 0x003）
srv_boot_ctrl_request_boot();  // 置 upgrade_flag → 保存 → 系统复位
```

---

## 升级流程

```
App 运行中 → srv_boot_ctrl_request_boot()
                ↓ 置 upgrade_flag=1, 保存到 ring_storage
              NVIC_SystemReset()
                ↓
Boot 启动 → 读 metadata → upgrade_flag=1 → 进入升级模式（不跳 App）
                ↓
上位机 CAN 升级 (START → DATA → VERIFY → REBOOT)
                ↓
Boot 写 metadata (upgrade_flag=0, fw_checksum) → NVIC_SystemReset()
                ↓
Boot 启动 → upgrade_flag=0, checksum OK → 跳转 App
```

---

## CAN 触发进 Boot（参考 E1_Master_Power_Manage）

App 运行中收到 CAN ID `0x003`（载荷首字节 `0x01`）即请求进入 bootloader。Flash 操作不能放 ISR，采用 **ISR 置标志 → 主循环消费** 模式。

### 1. 定义

```c
#define BOOT_REQUEST_CAN_ID    (0x003U)  /**< 进 boot 命令 CAN ID */
#define BOOT_REQUEST_LEN       (1U)      /**< 最少载荷字节数 */
#define BOOT_REQUEST_MAGIC     (0x01U)   /**< 载荷首字节 magic */
```

### 2. ISR 回调（仅置标志，不操作 Flash）

```c
static volatile bool s_enter_boot_requested;  /**< ISR 置位，主循环消费 */

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    (void)ch;
    if (!msg) return;

    /* 进 boot 命令（0x003, 载荷首字节 0x01）：仅置标志 */
    if (msg->id == BOOT_REQUEST_CAN_ID
        && msg->dlc >= BOOT_REQUEST_LEN
        && msg->data[0] == BOOT_REQUEST_MAGIC) {
        s_enter_boot_requested = true;
        return;
    }

    /* ... 其他 CAN 帧处理 ... */
}
```

### 3. 主循环消费（sw_timer 回调中执行 Flash 写 + 复位）

```c
#include "srv_boot_ctrl.h"

static void can_timer_cb(void* user_data)
{
    (void)user_data;

    /* 消费进 boot 请求（Flash 写 + 复位，不能放 ISR） */
    if (s_enter_boot_requested) {
        s_enter_boot_requested = false;
        LOG_W("can", "收到 bootloader 请求，准备跳转");
        if (srv_boot_ctrl_request_boot() != SRV_BOOT_CTRL_OK) {
            LOG_E("can", "进入 bootloader 请求失败");
        }
        return; /* 复位后不可达 */
    }

    /* ... 其他周期性任务 ... */
}
```

### 流程时序

```
上位机                               App                         Bootloader
  │                                   │                              │
  │── CAN 0x003 [0x01] ──────────────>│                              │
  │                                   │ ISR: s_enter_boot = true     │
  │                                   │ 主循环: request_boot()       │
  │                                   │   → upgrade_flag=1, save     │
  │                                   │   → NVIC_SystemReset()       │
  │                                   X                              │
  │                                                          启动 → │
  │                                                          upgrade_flag=1
  │                                                          进入升级模式
  │                                   │                              │
  │── CAN 0x701 START ──────────────────────────────────────────────>│
  │<── CAN 0x702 ACK ────────────────────────────────────────────────│
  │                        ... 升级流程 ...                          │
```

### 设计要点

| 要点 | 说明 |
|---|---|
| ISR 仅置标志 | `s_enter_boot_requested` 是 `volatile bool`，ISR 写、主循环读，无锁 |
| Flash 写放主循环 | `ring_storage_save()` 涉及 Flash 擦写（~16ms GC）、不可重入锁，绝不能放 ISR |
| `0x003` 对 Boot 无害 | Boot 的 CAN 滤波器只收 `0x701`，`0x003` 直接被硬件丢弃，不会干扰升级会话 |
| 无需先探测 | 上位机直接发 `0x003`，随后等 Boot 心跳 beacon（`0x702` 每 200ms）确认设备已进入升级模式 |

## 参考

- `../boot_protocol_spec.md` — CAN 升级协议帧格式
- `../CLAUDE.md` — Bootloader 工程架构
- `E1_Master_Power_Manage/tasks/can_task.c` — 完整 CAN 触发进 boot 实现
