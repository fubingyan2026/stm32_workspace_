# ctu_sdk_c — E1 CTU 电源板 RS485 C SDK

E1_MASTER_POWER_CTU（`0x01`）与 E1_SLAVER_POWER_CTU（`0x02`）共用一条 RS485 总线时的
主机侧 **C11 SDK**（Linux/POSIX）。覆盖上位机全部功能：状态/电压/温度/固件信息读取、
蜂鸣器与输出控制、清除故障锁存、升级请求、**完整固件升级（分块传输，支持中止）**、
周期轮询与丢包统计，并提供命令行工具。

- 无动态内存、无线程依赖、无第三方库（termios + poll + glob）
- **协议解析复用共享中间件 `protocol_parser`**（与固件端同一实现），副本随包提供于
  `middleware/`，SDK 可脱离仓库独立部署
- 遵循仓库 `MODULE_CODING_GUIDE.md`（WebKit 风格 + MISRA 倾向 + 中文 Doxygen）
- 接口细节见 **[docs/API.md](docs/API.md)**

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

无外部依赖，克隆本目录即可构建。

产物：`build/libctu_sdk.a`（`ctu::sdk`）、`build/ctu_cli`、`build/ctu_example_monitor`、`build/ctu_example_upgrade`。

严格模式（告警视为错误）：

```bash
cmake -S . -B build -DCTU_WARNINGS_AS_ERRORS=ON && cmake --build build -j
```

## 快速开始

```c
#include <stdio.h>
#include "ctu/ctu.h"

int main(void)
{
    ctu_client_t client;
    ctu_client_config_t config = { 0 };
    ctu_master_status_t status;

    ctu_client_init(&client, &config);
    if (ctu_client_open(&client, "/dev/ttyUSB0", 115200) != CTU_OK) {
        ctu_client_deinit(&client);
        return 1;
    }

    if (ctu_client_read_master_status(&client, &status) == CTU_OK) {
        printf("异常=%d 急停=%d\n", ctu_master_status_has_fault(&status), status.estop);
    }
    ctu_client_set_outputs(&client, 0x03U, 500U, NULL);       /* SLAVER: 24V + 12V_ISO, 补光 50% */
    ctu_client_upgrade(&client, CTU_DEVICE_MASTER, "fw.bin"); /* 固件升级 */

    ctu_client_deinit(&client);
    return 0;
}
```

编译：`gcc main.c -Iinclude build/libctu_sdk.a -o app`

## 命令行

```bash
./build/ctu_cli ports
./build/ctu_cli -p /dev/ttyUSB0 scan
./build/ctu_cli -p /dev/ttyUSB0 status --device master
./build/ctu_cli -p /dev/ttyUSB0 output --on 24v,12v --duty 500 --preserve
./build/ctu_cli -p /dev/ttyUSB0 upgrade --device slaver --file fw.bin
./build/ctu_cli -p /dev/ttyUSB0 monitor --interval 0.5
```

## 示例

- [examples/monitor.c](examples/monitor.c) — 周期轮询 + 统计打印
- [examples/upgrade.c](examples/upgrade.c) — 带进度与 Ctrl+C 中止的升级

## 接硬件自检

```bash
./build/ctu_cli ports                                      # 列串口（支持简写 ttyUSB0）
./build/ctu_cli -p /dev/ttyUSB0 scan                       # 两板在线探测
./build/ctu_cli -p /dev/ttyUSB0 -v status --device master  # -v 打印收发帧十六进制
./build/ctu_cli -p /dev/ttyUSB0 upgrade --device slaver --file fw.bin
```

打开失败会给出具体原因（如 `Permission denied（权限不足…）`）；串口默认 115200-8N1。

## 目录

```
ctu_sdk_c/
  include/ctu/        公共头文件（ctu.h 为总入口）
  src/                实现（protocol / transport / boot / client / poller）
  middleware/         随包发布的共享中间件副本（protocol_parser + kfifo）
  tools/ctu_cli.c     命令行工具
  examples/           示例
  docs/API.md         接口文档
```

`middleware/` 为 `public_layer/m_middlewares` 的**字节级副本**（协议解析用），
来源、同步方式与已知告警见 `middleware/README.md`：

| 中间件 | 用途 |
|--------|------|
| `protocol_parser` | z 帧流式解析（与固件端同一实现） |
| `kfifo` | 解析器内部环形缓冲 |
