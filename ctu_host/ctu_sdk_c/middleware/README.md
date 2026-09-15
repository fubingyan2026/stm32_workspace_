# middleware — 随包发布的共享中间件副本

本目录是 `public_layer/m_middlewares` 中**协议解析相关中间件的字节级副本**，
目的是让 `ctu_sdk_c` 可以脱离仓库其余部分**独立部署**到 Linux 目标机。

> 注意：仓库 `AGENTS.md` 的约定是「共享代码不复制、直接改 `public_layer` 原件」。
> 本 SDK 是明确要求可独立分发的交付物，因此按需做成**随包副本**；除此之外的
> 工程仍应遵守该约定。若后续需要修改中间件行为，请先在 `public_layer` 原件上修改
> 并回归固件侧，再同步到此目录。

## 副本内容

| 文件 | 来源 |
|------|------|
| `protocol_tools/protocol_parser.c` | `public_layer/m_middlewares/protocol_tools/protocol_parser.c` |
| `protocol_tools/protocol_parser.h` | `public_layer/m_middlewares/protocol_tools/protocol_parser.h` |
| `utils/kfifo.c` | `public_layer/m_middlewares/utils/kfifo.c` |
| `utils/kfifo.h` | `public_layer/m_middlewares/utils/kfifo.h` |

复制方式：直接文件拷贝，**内容未做任何修改**（SHA-256 与原件一致）。
固件端 `E1_MASTER_POWER_CTU/service/srv_com_mst.c` 使用的是同一份原件，因此
主机侧与板端解析行为完全一致。

## 同步方法

```bash
cp ../../public_layer/m_middlewares/protocol_tools/protocol_parser.{c,h} middleware/protocol_tools/
cp ../../public_layer/m_middlewares/utils/kfifo.{c,h}                     middleware/utils/
```

同步后重新构建，并接硬件做一次读写自检：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCTU_WARNINGS_AS_ERRORS=ON
cmake --build build -j
./build/ctu_cli -p /dev/ttyUSB0 scan
./build/ctu_cli -p /dev/ttyUSB0 -v status --device master
```

## 已知告警（上游原件自带，未修改）

用 `-Wall -Wextra` 编译本目录会看到 3 条良性告警，属原件既有情况：

| 位置 | 告警 |
|------|------|
| `protocol_parser.c` `parser_check_header()` | `kfifo_len`/`kfifo_get` 丢弃 `const` 限定符（`const protocol_parser_context_t*` → 非 const 形参） |
| `protocol_parser.c` `parser_print_hex()` | 空实现，`-Wunused-function` |

因此 CMake 中该目标只开 `-Wall -Wextra`、不纳入 SDK 的 `-Werror`。
如需彻底消除，应在 `public_layer` 原件修正后再同步（SDK 侧不单方面改动副本）。
