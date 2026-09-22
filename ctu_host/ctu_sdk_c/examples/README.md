# ctu_sdk_c 示例使用说明

本目录提供两个可直接编译运行的示例，覆盖 SDK 最常用的两条使用路径：

| 示例 | 可执行文件 | 作用 | 是否写板端 Flash |
|------|-----------|------|------------------|
| [monitor.c](monitor.c) | `ctu_example_monitor` | 周期轮询两板状态/电压/温度，打印异常与通信统计 | **否**（只读） |
| [upgrade.c](upgrade.c) | `ctu_example_upgrade` | 固件升级：预检 → 0x06 邀请 → SELECT/START/DATA/END，含进度与中止 | **是**（会改写 App 分区） |

> 更完整的命令行工具（含 `scan`/`volt`/`temp`/`info`/`buzzer`/`output`/`clear-latch` 等全部命令）见
> [../tools/ctu_cli.c](../tools/ctu_cli.c)，接口细节见 [../docs/API.md](../docs/API.md)。

---

## 1. 前置条件

- Linux / WSL，已构建出串口设备（如 `/dev/ttyUSB0`）
- 串口默认 **115200-8N1**；两板（MASTER + SLAVER）挂在同一条 RS485 总线上
- **串口访问权限**：设备节点通常是 `root:dialout 660`，当前用户需在 `dialout` 组内或用 `sudo` 运行
  ```bash
  sudo usermod -aG dialout $USER   # 重新登录 / wsl --shutdown 后生效
  ```
  权限不足时程序会直接给出提示：`无法打开串口 xxx: Permission denied（权限不足…）`

## 2. 构建

在 `ctu_sdk_c/` 目录：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

示例默认参与构建（`-DCTU_BUILD_EXAMPLES=ON`，默认即 ON）。产物：

```
build/ctu_example_monitor
build/ctu_example_upgrade
```

只想编库、不编示例：`cmake -S . -B build -DCTU_BUILD_EXAMPLES=OFF`。

**串口名可简写**：`ttyUSB0` 会被自动解析为 `/dev/ttyUSB0`（两者都可）。

---

## 3. `ctu_example_monitor` — 周期轮询监测

### 用法

```bash
./build/ctu_example_monitor <串口> [波特率]
```

| 参数 | 必需 | 说明 |
|------|------|------|
| `<串口>` | 是 | 设备名，如 `/dev/ttyUSB0` 或简写 `ttyUSB0` |
| `[波特率]` | 否 | 默认 `115200`（任何受支持的数值：9600/19200/38400/57600/115200/230400/460800/921600） |

```bash
./build/ctu_example_monitor /dev/ttyUSB0
./build/ctu_example_monitor ttyUSB0 460800
```

### 行为

- 每轮对 **两块板** 依次查询：状态 `0x01` → 电压 `0x02` → 温度 `0x03`（共 6 次请求）
- 每轮结束后**休眠 500 ms**（间隔固定在示例代码内，见 `monitor.c` 的 `nanosleep`）
- 启动时会打印所有级别的日志（含 `[tx]`/`[rx]` 原始帧十六进制），便于观察总线报文
- `Ctrl+C`（SIGINT）退出，打印 `已停止`，进程返回 `0`

退出码：

| 退出码 | 含义 |
|--------|------|
| `0` | 正常退出（Ctrl+C） |
| `1` | 客户端初始化失败 / 串口打不开 |
| `2` | 参数错误（缺少串口参数） |

### 输出示例

```
已连接 /dev/ttyUSB0
  [tx] TX  7A 01 01 01 69 0A
  [rx] RX  7A 81 03 01 01 00 E0 0A
  [tx] TX  7A 02 01 01 8D 0A
  [rx] RX  7A 82 05 01 39 96 52 95 E5 0A
  [tx] TX  7A 03 01 01 26 0A
  [rx] RX  7A 83 07 01 FE EE 8D 0C 61 0D BF 0A
master  状态=异常  VIN=38457 mV  MCU=34.25 °C
  ! slaver cmd=0x01 失败: 等待应答超时
tx=6 rx=3 丢包=3 (50.0%)
^C
已停止
```

字段含义：

| 输出 | 说明 |
|------|------|
| `master 状态=异常/正常` | `ctu_master_status_has_fault()`；异常时会打印该行但**不列出具体异常项**（示例从简，逐项明细请用 `ctu_cli status`） |
| `VIN=… mV` | MASTER 的 VIN 电压（SLAVER 行打印 `AUX=`） |
| `MCU=… °C` | MCU 温度 |
| `! <dev> cmd=0xNN 失败: …` | 单条查询失败（超时/设备错误应答等），**不会中断轮询**，输出到 stderr |
| `tx= rx= 丢包= (x%)` | 累计请求数 / 成功应答数 / 失败数 与丢包率 |

### 注意

- 示例**固定轮询两块板**。如果只接了其中一块，另一块每次都会超时 → 丢包率约 **50%**，这属正常现象，不代表链路有问题。
  只关心单板时请改用 `ctu_cli -p <串口> monitor --device master`。
- 示例的日志回调会打印每一帧的十六进制，输出量较大；批量/长期监测建议用 `ctu_cli monitor`（无原始帧）。

---

## 4. `ctu_example_upgrade` — 固件升级

> ⚠️ **该示例会真正写板端 Flash（App 分区），并让板子复位运行新固件。** 执行前请确认目标设备与固件版本。

### 用法

```bash
./build/ctu_example_upgrade <串口> <master|slaver> <固件.bin>
```

| 参数 | 必需 | 说明 |
|------|------|------|
| `<串口>` | 是 | 设备名，支持简写 `ttyUSB0` |
| `<master\|slaver>` | 是 | 目标设备，**只接受这两个小写字符串** |
| `<固件.bin>` | 是 | **App 分区镜像**（非 Boot），可为绝对或相对路径 |

波特率固定为 `115200`（示例未提供波特率参数；需要其它波特率请用 `ctu_cli upgrade -b …`）。

### 固件从哪来

由固件工程构建产生，例如：

```
E1_MASTER_POWER_CTU/build/RelWithDebInfo/E1_MASTER_POWER_CTU.bin
E1_SLAVER_POWER_CTU/build/RelWithDebInfo/E1_SLAVER_POWER_CTU.bin
E1_SLAVER_POWER_CTU/build/Release/E1_SLAVER_POWER_CTU.bin
```

```bash
./build/ctu_example_upgrade /dev/ttyUSB0 master \
  ../E1_MASTER_POWER_CTU/build/RelWithDebInfo/E1_MASTER_POWER_CTU.bin
```

镜像要求（预检会自动判定，不通过则**不会访问总线**）：

- 栈顶指针落在 RAM 区间 `0x20000000 ~ 0x2000C000`
- 复位向量落在 AppA 区间 `[0x08008000, 0x08020000)` 且为奇地址（Thumb）
- 文件 ≤ 96 KB（`CTU_FIRMWARE_MAX_SIZE`）

### 找不到文件时的辅助

若只给文件名而当前目录没有，程序会**自动向上最多 4 级**在各级 `build` 产物目录中查找同名文件
（跳过 CMake 内部产物，优先 `RelWithDebInfo` / `Release`），并给出可直接复制的完整命令：

```
固件 E1_SLAVER_POWER_CTU.bin  0 B  校验和 0x00000000  无法打开文件: No such file or directory
提示: 已在其它目录找到同名固件，请改用该路径重试:
  ./build/ctu_example_upgrade /dev/ttyUSB0 slaver <找到的路径>
```

- `<找到的路径>` 取决于你的目录结构与同名文件的位置（程序会选优先匹配项）
- 一处都没找到时改为提示手动给出路径：
  ```
  提示: 当前目录没有该文件，请给出固件路径，例如
    ./build/ctu_example_upgrade /dev/ttyUSB0 master ../../E1_MASTER_POWER_CTU/build/RelWithDebInfo/E1_MASTER_POWER_CTU.bin
  ```
- 该分支只在**预检失败**时出现，因此不会访问总线、更不会写板

### 输出示例

```
固件 ../E1_MASTER_POWER_CTU/build/RelWithDebInfo/E1_MASTER_POWER_CTU.bin  60232 B  校验和 0x005C50A4  预检通过
· 发送 0x06 邀请 / 等待 Boot
· 选中设备并进入升级会话
· 擦除暂存区并开始下载
· 传输固件数据
进度  57.7%
进度 100.0%
· 提交固件（校验→提升/写入暂存槽）
进度 100.0%
master 升级完成：../E1_MASTER_POWER_CTU/build/RelWithDebInfo/E1_MASTER_POWER_CTU.bin
```

- **阶段**（`· xxx`）与**进度**（`进度 xx.x%`）都输出到 **stderr**
- 本示例的日志回调默认不输出（`config.user` 未置位），所以看不到 `[info]/[tx]` 明细；
  需要看原始帧请用 `ctu_cli -v upgrade --device master --file <bin>`

### 中止

升级过程中按 `Ctrl+C`：

1. 示例调用 `ctu_client_request_cancel()`
2. SDK 向板端发送 `ABORT`（0x0B）并返回 `CTU_ERROR_CANCELED`
3. 打印 `升级已中止`，进程返回 `1`

板端会停留在 Bootloader（升级会话有约 10 s 空闲超时，主机消失后自动退出），再次运行本示例会重新发起 `0x06` 邀请。

### 退出码

| 退出码 | 含义 |
|--------|------|
| `0` | 升级成功 |
| `1` | 预检失败 / 串口打不开 / 升级失败 / 被中止 |
| `2` | 参数错误（缺参数、设备名非法） |

---

## 5. 参数速查

```bash
# 轮询监测（只读）
./build/ctu_example_monitor <串口> [波特率]

# 固件升级（写 Flash，波特率固定 115200）
./build/ctu_example_upgrade <串口> <master|slaver> <固件.bin>
```

对应的 SDK 调用（便于改成自己的程序）：

| 示例 | 主要 API |
|------|----------|
| monitor | `ctu_client_init/open` → `ctu_poller_init` → 循环 `ctu_poller_poll_once` + `ctu_poller_get_stats` → `ctu_client_deinit` |
| upgrade | `ctu_boot_check_file` → `ctu_client_init/open` → `ctu_client_upgrade`（+ `ctu_client_request_cancel`）→ `ctu_client_deinit` |

---

## 6. 常见问题

| 现象 | 原因 / 处理 |
|------|-------------|
| `无法打开串口 …: Permission denied（权限不足…）` | 用户不在 `dialout` 组：`sudo usermod -aG dialout $USER` 后重新登录，或用 `sudo` 运行 |
| `无法打开串口 …: No such file or directory` | 设备名不对或未插入；用 `./build/ctu_cli ports` 列出实际设备；也可直接用简写 `ttyUSB0` |
| 升级示例报 `向量表栈顶 0x… 不在 RAM 范围（疑似非本 App 固件）` | 传入了 Boot 镜像或 `.hex`/`.elf` 等非 App 分区 `.bin`；请用固件工程的 App `.bin` |
| 某块板一直 `失败: 等待应答超时` | 该板未上电/未接在总线上/地址不符；先用 `./build/ctu_cli -p <串口> scan` 判断是单板还是整条总线无应答 |
| 丢包率长期 50% | 只接了一块板，而 monitor 固定轮询两块；改用 `ctu_cli monitor --device master` 只轮询在线的那块 |
| `未知设备: xxx（可选 master / slaver）` | 升级示例只接受小写 `master`/`slaver`（`0x01`/`主` 等别名仅在 `ctu_cli` 中支持） |
| 升级中途断开/断电 | 板端停留在 Bootloader，重新执行升级即可（0x06 邀请幂等） |
| 需要看升级报文的原始字节 | 用 `./build/ctu_cli -p <串口> -v upgrade --device <dev> --file <bin>` |

---

## 7. 相关文档

- 接口文档：[../docs/API.md](../docs/API.md)
- SDK 说明与构建：[../README.md](../README.md)
- 共享中间件来源与同步：[../middleware/README.md](../middleware/README.md)
