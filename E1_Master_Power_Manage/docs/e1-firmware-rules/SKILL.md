
---
name: e1-firmware-rules
description: 强制执行 STM32 裸机项目的五层架构、自包含驱动、无锁多实例服务及 WebKit C 编码规范。
---
# 统一架构与编程规范指南

## 1. 核心架构与依赖红线

本系统基于 STM32裸机，单线程协作式调度：`main() -> app_main() -> while(1) { sw_timer_task(); }`。

### 1.1 分层架构与依赖限制

| 层级 (由顶向下) | 允许 Include 的层 | 严禁 Include 的层 | 核心职责 |
| :--- | :--- | :--- | :--- |
| **tasks/** | `service/`, `device_drivers/`, `m_middlewares/`, `Core/` | 无 | 系统初始化、桥接回调、配置并运行 `sw_timer`（调度） |
| **service/** (`srv_`) | `m_middlewares/`, **绑定的** `device_drivers/`, `Core/` 基本类型 | 其他无关 `service/`、任何 `tasks/` | 纯业务逻辑/状态机/算法。**严禁包含 `sw_timer`** |
| **m_middlewares/** | 其他 `m_middlewares/`、C 标准库 | 任何硬件相关层（`device_drivers/`, `Core/`, `service/`, `tasks/`） | 平台无关通用库（如无锁队列、FSM 引擎、滤波算法） |
| **device_drivers/** (`drv_`) | `Core/` (`main.h`, `hal`), `m_middlewares/` | `service/`, `tasks/` | HAL 接口语义化包装，**唯一允许接触引脚和外设句柄的层** |
| **Core/** | CubeMX 生成层 | 业务代码 | 硬件抽象层（HAL）与中断入口 |

---

## 2. 关键设计规约

### 2.1 驱动自包含规约 (`device_drivers/`)
*   **无参初始化**：`drv_xxx_init(void)` 必须无参。引脚配置和 HAL 句柄（如 `&htim1`）必须内置在驱动 `.c` 文件的宏或静态硬件配置表中，严禁通过参数传入。
*   **逻辑通道**：对外暴露枚举通道（如 `DRV_ADC_CH_VIN`），内部通过查表映射到物理通道。
*   **中断处理**：HAL 弱回调函数必须在对应驱动 `.c` 内重写，只做“解析通道、更新缓冲/FIFO”等最小工作。

### 2.2 服务解耦规约 (`service/`)
*   **不自建定时器**：严禁自建 `sw_timer`。由 Task 层周期调用 `srv_xxx_step(uint16_t elapsed_ms)` 驱动。
*   **风格 A (回调注入)**：涉及总线复用或异构设备时，硬件接口抽象为函数指针，初始化时从上层注入。不包含任何 `drv_` 头文件。
*   **风格 B (直连驱动)**：专属单一驱动时，允许直接 `#include "drv_xxx.h"`。
*   **多实例防踩踏**：**严禁在多实例服务 `.c` 中使用静态单例变量**（如静态 FIFO/缓冲区）。所有状态、队列和缓存必须声明在实例上下文 `srv_xxx_context_t` / 句柄结构体内。

### 2.3 编排器规约 (`tasks/`)
*   **静态分配**：必须在 `tasks` 的 `.c` 中以 `static` 声明所有的驱动和业务上下文句柄，避免动态分配。
*   **顺序初始化**：在 `app_main.c` 中按依赖顺序同步初始化各 Task（例：先 CAN/ADC，后时序控制）。

---

## 3. C 语言编码规范 (WebKit & MISRA)

### 3.1 格式与排版规范
*   **缩进**：一律使用 **4 个空格**，禁止使用 Tab 键。单行限制 **100 字符**。
*   **大括号规则**：
    *   **函数定义**：左大括号**换行另起**（Allman 风格）。
    *   **控制语句**（`if/for/while/switch`）：左大括号与关键字**同行**（K&R 风格）。
*   **代码编写顺序**：
    1. 文件头注释 $\to$ 2. Includes $\to$ 3. 私有宏/常量 $\to$ 4. 私有变量 $\to$ 5. 私有函数声明 $\to$ 6. 导出函数实现 $\to$ 7. 私有静态函数实现。

### 3.2 命名与注释
*   **命名法**：一律使用**小写蛇形 + 模块前缀**。类型以 `_t` 结尾，回调指针以 `_cb_t` 结尾。
*   **错误码**：首个成员必须为 `MODULE_OK = 0`，其余成员大写蛇形并带详细中文 Doxygen 注释。
*   **注释**：所有公共 API、关键结构体必须使用中文 Doxygen 注释（含 `@brief`, `@param`, `@return`）。

### 3.3 内存与安全国防线
*   **零动态分配**：运行期禁止 `malloc` / `free`。
*   **分配初始化**：若由于框架限制不得不动态分配内存，**分配后必须立即 `memset` 清零**（防范状态机判定垃圾值死机）。
*   **防御性校验**：所有公共接口入口必须优先进行非空、越界和初始化状态校验，异常时返回对应错误码。
*   **未使用参数**：显式调用 `(void)param;` 消除编译警告。

---

## 4. 极简骨架模板

### 4.1 驱动层 (drv_xxx)

#### `drv_xxx.h`
```c
#ifndef __DRV_XXX_H
#define __DRV_XXX_H

#include <stdint.h>

typedef enum { DRV_XXX_CH_0 = 0, DRV_XXX_CH_MAX } drv_xxx_ch_t;
typedef enum { DRV_XXX_OK = 0, DRV_XXX_ERR_PARAM = -1, DRV_XXX_ERR_INIT = -2 } drv_xxx_error_t;

drv_xxx_error_t drv_xxx_init(void); /* 必须无参 */
drv_xxx_error_t drv_xxx_write(drv_xxx_ch_t ch, uint16_t val);

#endif
```

#### `drv_xxx.c`
```c
#include "drv_xxx.h"
#include "main.h" /* 唯一允许引用 HAL 层的层 */

typedef struct { TIM_HandleTypeDef* htim; uint32_t ch; } drv_hw_t;

static const drv_hw_t s_hw[DRV_XXX_CH_MAX] = { [DRV_XXX_CH_0] = { &htim10, TIM_CHANNEL_1 } };
static bool s_init = false;

drv_xxx_error_t drv_xxx_init(void)
{
    if (s_init) return DRV_XXX_OK;
    for (uint32_t i = 0; i < DRV_XXX_CH_MAX; ++i) {
        HAL_TIM_PWM_Start(s_hw[i].htim, s_hw[i].ch);
    }
    s_init = true;
    return DRV_XXX_OK;
}

drv_xxx_error_t drv_xxx_write(drv_xxx_ch_t ch, uint16_t val)
{
    if (!s_init) return DRV_XXX_ERR_INIT;
    if (ch >= DRV_XXX_CH_MAX) return DRV_XXX_ERR_PARAM;
    __HAL_TIM_SET_COMPARE(s_hw[ch].htim, s_hw[ch].ch, val);
    return DRV_XXX_OK;
}
```

### 4.2 服务层 (srv_xxx - 回调注入风格)

#### `srv_xxx.h`
```c
#ifndef __SRV_XXX_H
#define __SRV_XXX_H

#include <stdbool.h>
#include <stdint.h>

typedef void (*srv_xxx_write_cb_t)(uint16_t val);
typedef enum { SRV_XXX_OK = 0, SRV_XXX_ERR_NULL = -1 } srv_xxx_error_t;

typedef struct { srv_xxx_write_cb_t write_cb; } srv_xxx_config_t;

typedef struct {
    srv_xxx_config_t config;
    uint16_t val;
    bool is_init; /* 多实例字段独立存储，严禁在 .c 中定义全局 static */
} srv_xxx_context_t;

srv_xxx_error_t srv_xxx_init(srv_xxx_context_t* ctx, const srv_xxx_config_t* cfg);
srv_xxx_error_t srv_xxx_step(srv_xxx_context_t* ctx, uint16_t elapsed_ms);

#endif
```

#### `srv_xxx.c`
```c
#include "srv_xxx.h" /* 严禁包含 stm32f4xx_hal.h 和 drv_xxx.h */

srv_xxx_error_t srv_xxx_init(srv_xxx_context_t* ctx, const srv_xxx_config_t* cfg)
{
    if (!ctx || !cfg || !cfg->write_cb) return SRV_XXX_ERR_NULL;
    ctx->config = *cfg;
    ctx->val = 0;
    ctx->is_init = true;
    return SRV_XXX_OK;
}

srv_xxx_error_t srv_xxx_step(srv_xxx_context_t* ctx, uint16_t elapsed_ms)
{
    if (!ctx || !ctx->is_init) return SRV_XXX_ERR_NULL;
    ctx->val += elapsed_ms;
    ctx->config.write_cb(ctx->val);
    return SRV_XXX_OK;
}
```

### 4.3 编排层 (xxx_task)

#### `xxx_task.c`
```c
#include "xxx_task.h"
#include "drv_xxx.h"
#include "srv_xxx.h"
#include "sw_timer.h"

#define TASK_PERIOD_MS 10U

static srv_xxx_context_t s_srv_ctx; /* 静态分配上下文 */
static sw_timer_t s_timer;

static void bridge_write(uint16_t val) { drv_xxx_write(DRV_XXX_CH_0, val); }
static void timer_cb(void* arg) { (void)arg; srv_xxx_step(&s_srv_ctx, TASK_PERIOD_MS); }

void xxx_task_init(void)
{
    drv_xxx_init();
    srv_xxx_init(&s_srv_ctx, &(srv_xxx_config_t){ .write_cb = bridge_write });
    
    sw_timer_init(&s_timer, &(sw_timer_config_t){
        .priority = SW_TIMER_PRIO_NORMAL,
        .callback = timer_cb,
        .user_data = NULL
    });
    sw_timer_start(&s_timer, TASK_PERIOD_MS, 0);
}
```