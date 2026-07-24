# 主电源板 ↔ 从电源板 CAN 通讯协议

## 总线

主从板共享同一条 **P_CAN 总线**（CAN1, 1 Mbps）。主电源板作为控制器，从电源板执行指令并回复 ACK。

| CAN ID | 方向 | 说明 |
|--------|------|------|
| `0x001` | 主机 ↔ 主电源板 | 系统状态 + 控制指令（详见 power_can_protocol.md） |
| `0x002` | **主电源板 → 从电源板** | 从板控制帧（HSD 输出，带 ACK 重试） |

---

## 0x002 — 从板控制协议

### 控制帧（主电源板 → 从电源板，6 字节）

| 字节 | 字段 | 描述 |
|------|------|------|
| 0-4 | 保留 | 未使用（填 0x00） |
| 5 | `ctrl_byte` | bit4=HSD1_12V, bit5=reserved, bit6=valid_mask |

**byte5 控制位详解：**

| 位 | 名称 | 描述 |
|----|------|------|
| 4 | `hsd1_12v_on` | HSD1 12V 通道控制：1=开, 0=关 |
| 5 | `reserved_channel` | 保留（未来扩展） |
| 6 | `valid_mask` | 始终为 1，标识此帧为有效控制帧 |

### ACK 帧（从电源板 → 主电源板，8 字节）

从板收到 0x002 后，回复相同 ID 的 8 字节帧作为 ACK。主电源板不校验内容，仅用于握手确认。

| 字节 | 描述 |
|------|------|
| 0-7 | 任意内容（主电源板仅确认收到） |

---

## 通讯时序

### 状态机

| 当前状态 | 触发条件 | 下一状态 | 动作 |
|---------|---------|---------|------|
| IDLE | `srv_can_slv_request()` | PENDING | 打包 0x002 帧并发送 |
| PENDING | 收到 0x002 ACK (len=8) | ACKED | 停止重试 |
| PENDING | 50ms 超时 | PENDING (保持) | 重发 0x002 帧 |
| ACKED | 下次 `srv_can_slv_request()` | IDLE → PENDING | 发送新帧 |

### 典型流程

| 步骤 | 主电源板 | 方向 | 从电源板 |
|------|---------|------|---------|
| 1 | `can_task_set_slave_ctrl(true, false)` → `srv_can_slv_request()` | | |
| 2 | 发送 0x002 (byte5=0x50: HSD1_12V=ON) | → | |
| 3 | 等待 ACK，启动 50ms 重试计时 | | |
| 4 | | ← | 回复 0x002 ACK (8 bytes) |
| 5 | 收到 ACK → 状态 SLAVE_ACKED | | |

### 重试参数

| 参数 | 值 | 说明 |
|------|----|------|
| `SRV_CAN_SLV_RETRY_MS` | **50 ms** | 无 ACK 时重试间隔 |
| 重试策略 | 持续重试直到收到 ACK | 不限制重试次数 |
| ACK 判定 | ID=0x002 且 len=8 且状态为 PENDING | |

---

## 软件架构

### 服务层 `srv_can_slv`

```c
// 从板控制数据结构
typedef struct {
    bool hsd1_12v_on;        // HSD1 12V
    bool reserved_channel;   // 保留
} srv_can_slv_ctrl_t;

// 初始化（注入 send_frame + get_ctrl 回调）
srv_can_slv_init(&cfg);

// 请求发送控制帧
srv_can_slv_request();   // 读取 get_ctrl → 打包 0x002 → 发送

// 周期任务（每 10ms 调用）
srv_can_slv_task();      // 检查超时 → 重试

// RX 处理
srv_can_slv_process_rx(id, data, len);  // 检测 ACK
```

### 任务层 `can_task`

```c
// 设置从板控制状态（外部调用）
can_task_set_slave_ctrl(hsd_12v_on, hsd_24v_on);

// 内部：can_task timer callback
//   srv_can_slv_task()      ← 每 10ms 重试检查
//   srv_can_slv_process_rx() ← CAN RX 中断 → ACK 检测
```

---

## 使用场景

HSD 控制伴随电源上电时序和急停状态：

| 场景 | 操作 |
|------|------|
| 上电完成 | `can_task_set_slave_ctrl(true, false)` — 开启 HSD1_12V |
| 急停按下 | `can_task_set_slave_ctrl(false, false)` — 关闭 HSD |
