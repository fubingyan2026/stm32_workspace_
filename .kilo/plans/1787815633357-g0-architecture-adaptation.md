# Plan: G0 service 层 — UART 命令协议收发（srv_uart_rx_cmd / srv_uart_tx_cmd）

## 目标

基于 `public_layer/m_middlewares/protocol_tools` 的 `protocol_parser` / `protocol_packer` 库，创建 G0 的串口命令服务层：

- `service/srv_uart_rx_cmd.c/.h` — 接收解析：轮询 `drv_uart_rx_read` 喂给 parser，`get_len_cb`/`check_cb` 解析完整帧后回调上抛。
- `service/srv_uart_tx_cmd.c/.h` — 发送打包：`protocol_packer_pack` 生成帧后经 `drv_uart_send` 发出。
- `tasks/uart_cmd_task.c/.h` — sw_timer 周期轮询：喂 RX + `protocol_parser_tick` + 可选 `drv_uart_recover`。

**协议（用户定义，收发一致）**：`[z][cmd][data_len][payload...][crc][\n]`
- header：`'z'`(0x7A) 1 字节；cmd 1 字节；data_len 1 字节（可变长负载上限 255）；payload data_len 字节；crc 1 字节（CRC8）；footer：`'\n'`(0x0A) 1 字节。
- 帧总长 = data_len + 5（1+1+1+data_len+1+1）。

**已确认决策**：支持 255 字节负载（需把 `drv_uart.c` 的 `DRV_UART_TX_BUF_SIZE` 128→264）；创建 `uart_cmd_task`。

## 关键结论（已调研）

- **packer 帧结构** = `[header][data][checksum][footer]`；`fill_len_cb(buffer, payload_len)` 在 data 拷贝后回调，可在原地写长度字段。因此 `srv_uart_tx_cmd_send` 构造 data 块 `[cmd][data_len占位][payload]`（2+payload_len 字节），由 `fill_len_cb` 把 `data_len` 字段填成 `payload_len_arg - 2`。
- **parser 行为**：header 匹配后先 `kfifo_peek` 到 `output_buffer`，`get_len_cb(buffer, len)` 需返回「完整帧总长」；`check_cb(buffer, len)` 收到含 header+payload+crc+footer 的整帧，crc 字节位于 `len-2`（footer `\n` 在 `len-1`）。
- **CRC8**：`crc.c` 的 `get_CRC8_check_sum(data, len, CRC8_INIT=0xFF)`（DJI 表驱动）。协议约定 CRC 覆盖 `header+cmd+data_len+payload`（crc 前所有字节）：
  - packer `checksum_cb`：`checksum_out[0] = get_CRC8_check_sum(buffer, offset, 0xFF)`，`*checksum_len_out = 1`。
  - parser `check_cb`：计算 `get_CRC8_check_sum(buffer, len-2, 0xFF)` 与 `buffer[len-2]` 比较。
- **kfifo** 输入缓冲须为 2 的幂（`kfifo_init` 自动向上取整，但按 512 静态分配稳妥）。
- **协议库已编入**：`m_middlewares/CMakeLists.txt` 已含 `protocol_tools` 与 `algorithm`（crc.c），G0 已链接 m_middlewares，无需改 CMake 收集；仅需在 include 路径中确认 `protocol_tools`（m_middlewares PUBLIC include 已含）。
- **drv_uart**：管理 `DRV_UART_CH_2`（USART2）；`drv_uart_rx_read/available`、`drv_uart_send/is_tx_busy`、`drv_uart_recover` 均可用。TX 缓冲 128B→264B 以容纳最大帧 260B。

## 改动清单（按实现顺序）

### 1. `device_drivers/drv_uart.c`（唯一驱动改动）

- `DRV_UART_TX_BUF_SIZE` 由 `128U` 改为 `264U`（容纳最大帧 260B + 余量）。仅此一处。

### 2. `service/srv_uart_tx_cmd.h/.c`（发送打包）

常量：
- `SRV_UART_TX_CMD_MAX_PAYLOAD 255U`
- `SRV_UART_TX_CMD_HEADER {'z'}` / `SRV_UART_TX_CMD_FOOTER {'\n'}`，`HEADER_LEN=1` / `FOOTER_LEN=1`
- `SRV_UART_TX_CMD_MAX_FRAME (SRV_UART_TX_CMD_MAX_PAYLOAD + 5) = 260U`
- 静态缓冲：`s_pack_out[SRV_UART_TX_CMD_MAX_FRAME]`（packer output_buffer）

API（`srv_uart_tx_cmd.h`）：
```c
typedef enum { SRV_UART_TX_CMD_OK=0, SRV_UART_TX_CMD_ERROR_NULL_PTR, SRV_UART_TX_CMD_ERROR_UNINITIALIZED, SRV_UART_TX_CMD_ERROR_INVALID_PARAM, SRV_UART_TX_CMD_ERROR_TX_BUSY, SRV_UART_TX_CMD_ERROR_INTERNAL } srv_uart_tx_cmd_error_t;
void srv_uart_tx_cmd_init(void);
srv_uart_tx_cmd_error_t srv_uart_tx_cmd_send(uint8_t cmd, const uint8_t* data, uint8_t data_len);
```

实现要点：
- `srv_uart_tx_cmd_init()`：`protocol_packer_init(&s_packer, &cfg)`，cfg 含 header/footer/checksum_len=1/output_buffer、`fill_len_cb`、`checksum_cb`。
- `srv_uart_tx_cmd_send(cmd, data, data_len)`：
  1. 校验：`data_len > 255` → INVALID_PARAM；`data_len>0 && !data` → NULL_PTR；未初始化 → UNINITIALIZED。
  2. 静态 `s_cmd_buf[2+255]`：`[0]=cmd`、`[1]=0`(占位)、`[2..]=payload`。
  3. `protocol_packer_pack(&s_packer, s_cmd_buf, 2+data_len, &frame, &frame_len)`。
  4. `drv_uart_send(DRV_UART_CH_2, frame, frame_len)`；TX_BUSY 映射为 `SRV_UART_TX_CMD_ERROR_TX_BUSY`，pack 错误映射为 INTERNAL/INVALID_PARAM。
- `fill_len_cb(uint8_t* buffer, uint16_t payload_len_arg)`：`buffer[HEADER_LEN+1] = (uint8_t)(payload_len_arg - 2);`（即 data_len 字段 = 实际 payload 长度）。
- `checksum_cb(const uint8_t* data, uint16_t len, uint8_t* out, uint16_t* out_len)`：`out[0] = get_CRC8_check_sum((uint8_t*)data, len, CRC8_INIT); *out_len = 1; return PROTOCOL_PACKER_OK;`

### 3. `service/srv_uart_rx_cmd.h/.c`（接收解析）

常量：
- 同 TX：header/footer 定义、`SRV_UART_RX_CMD_MAX_PAYLOAD 255U`、`SRV_UART_RX_CMD_MAX_FRAME 260U`
- `SRV_UART_RX_CMD_INPUT_BUF_SIZE 512U`（kfifo 输入，2 的幂）
- `SRV_UART_RX_CMD_OUTPUT_BUF_SIZE 264U`（parser output_buffer，须 ≥ 260）
- 静态缓冲：`s_input[512]`、`s_output[264]`

API（`srv_uart_rx_cmd.h`）：
```c
typedef void (*srv_uart_rx_cmd_cb_t)(uint8_t cmd, const uint8_t* data, uint8_t data_len);
typedef enum { SRV_UART_RX_CMD_OK=0, SRV_UART_RX_CMD_ERROR_NULL_PTR, SRV_UART_RX_CMD_ERROR_UNINITIALIZED, SRV_UART_RX_CMD_ERROR_INTERNAL } srv_uart_rx_cmd_error_t;
void srv_uart_rx_cmd_init(srv_uart_rx_cmd_cb_t callback);
void srv_uart_rx_cmd_step(void); /* 喂数据 + 解析 + 分派回调（task 周期调用） */
void srv_uart_rx_cmd_tick(void); /* 转发 protocol_parser_tick（空闲超时） */
```

实现要点：
- `srv_uart_rx_cmd_init(callback)`：保存回调；`protocol_parser_init(&s_parser, &cfg)`，cfg 含 header/footer、input_buffer=s_input、output_buffer=s_output、`get_len_cb`、`check_cb`、各长度字段。
- `srv_uart_rx_cmd_step()`：
  1. `drv_uart_rx_available(DRV_UART_CH_2)` 非 0 → 读入临时 `s_rx_buf[32]`（循环 `drv_uart_rx_read`）→ `protocol_parser_feed`。
  2. 循环 `protocol_parser_parse(&s_parser, &len, &frame)`：
     - 返回 `PROTOCOL_PARSER_OK` → 校验 `len>=5 && frame[0]=='z' && frame[len-1]=='\n'`，提取 `cmd=frame[1]`、`data_len=frame[2]`、`data=&frame[3]`，调回调 `srv_uart_rx_cmd_cb_t(cmd, data, data_len)`；继续解析下一帧。
     - `INCOMPLETE` / `IDLE_TIMEOUT` → break（等待更多数据 / 丢垃圾字节）。
     - 其他错误（CHECKSUM/FOOTER_MISMATCH/HEADER_MISMATCH 等）→ 限频 LOG_W 后继续（parser 已自行跳过垃圾）。
  3. 帧字段校验：`frame[2] == len-5`（data_len 与实际长度一致），不一致按 INVALID 丢弃并 LOG_W。
- `get_len_cb(uint8_t* buffer, uint16_t len)`：若 `len < HEADER_LEN+2`（不足 cmd+data_len）返回 0（不完整）；否则 `return (uint16_t)buffer[2] + 5;`（data_len+5 总长）。
- `check_cb(uint8_t* buffer, uint16_t len)`：`get_CRC8_check_sum(buffer, len-2, 0xFF)` 与 `buffer[len-2]` 相等 → `PROTOCOL_PARSER_OK`，否则 `PROTOCOL_PARSER_ERROR_CHECKSUM`。
- `srv_uart_rx_cmd_tick()`：`protocol_parser_tick(&s_parser)`。

### 4. `tasks/uart_cmd_task.h/.c`（轮询接入）

- `uart_cmd_task_init()`：
  - `srv_uart_tx_cmd_init()` + `srv_uart_rx_cmd_init(srv_uart_rx_cmd_task_cb)`（RX 回调：LOG_I 打印 cmd/data_len，预留业务分发，暂默认透传打印）。
  - sw_timer 10ms：回调里 `srv_uart_rx_cmd_step()` + `srv_uart_rx_cmd_tick()` + `(void)drv_uart_recover(DRV_UART_CH_2);`（周期错误兜底恢复）。
- `tasks/app_main.c`：include `uart_cmd_task.h`，在 `key_task_init()` 之后加 `uart_cmd_task_init();`。

### 5. CMakeLists

- 无需新增源收集（`service/`、`tasks/` 已 `aux_source_directory`）。include 路径已含 `../public_layer/service`；`protocol_tools`/`algorithm` 头由 m_middlewares PUBLIC include 提供。
- 确认 `crc.h`、`protocol_parser.h`、`protocol_packer.h` 可被 `#include`（m_middlewares PUBLIC 含 `protocol_tools` 与 `algorithm`，已核实 line 58/60）。

## 验证

1. `G0_hand_ctrl_sample/build.bat` 编译零错误零警告。
2. 串口 USART2（PA2/PA3, 115200）连接 PC 串口助手：
   - 发送 `5A 01 02 11 22 <crc> 0A`（cmd=0x01, data_len=2, payload=11 22），控制台打印 RX 回调（cmd/data_len/payload），无 CHECKSUM/FOOTER 报错。
   - 发送错误 CRC / 缺帧尾帧 → 打印限频 WARN，随后正确帧仍能解析（错位自愈）。
   - 发送变长帧（data_len=0 与 data_len=255 各一帧）验证可变长支持。
   - 通过调试或临时 LOG 调用 `srv_uart_tx_cmd_send` 回发一帧，用串口助手校验 `[z][cmd][data_len][payload][crc][\n]` 与收端 CRC 一致。
3. CRC8 双向一致性：RX 收到的帧用 `verify_CRC8_check_sum` 校验逻辑为「计算 len-2 字节 vs buffer[len-2]」，TX 侧按同字节范围生成，保证互通。

## 风险 / 注意

- RAM：新增 ~512+264+260+257+32 ≈ 1.3KB（当前 62% 使用 → 约 69%），仍在 20KB 内；如紧张可将 INPUT_BUF_SIZE 降至 512→256（但需 ≥ 260 容纳最大帧，故保持 512）。
- `protocol_parser_feed` 在输入缓冲溢出时 `kfifo_reset` 并返回 `BUFFER_OVERFLOW`（step 中忽略即可，parser 自行恢复）。
- `drv_uart_send` 有 TX_BUSY 与 `len>264` 返回；最大帧 260B < 264 缓冲，OK。
- CRC8 仅 1 字节，抗干扰有限；如需更强可后续升级 CRC16（不改帧格式需同步对端）。
- 帧头 `'z'`/帧尾 `'\n'` 为固定字节；payload 内若出现 0x7A/0x0A 无需转义（长度由 data_len 字段决定，parser 按长度切帧）。

## 不做（范围外）

- 具体命令业务分发（RX 回调目前只打印，业务命令表后续由应用层定义）。
- CRC16/加密/转义等增强。
- CAN 与 UART 命令互通（后续可按需桥接）。
