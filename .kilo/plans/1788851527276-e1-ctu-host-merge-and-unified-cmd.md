# E1_MASTER/SLAVER 上位机合并 + 主/副板命令码统一 实现计划

## 1. 目标与已确认决策

将 `E1_MASTER_POWER_CTU/host/`（ctu_host.py + ctu_protocol.py）与 `E1_SLAVER_POWER_CTU/host/`（slv_host.py + slv_protocol.py）两套上位机合并为**一个窗口同屏并列**的调试工具，放在仓库根新建共享目录 `ctu_host/`；同时把两板 RS485 命令码统一为 **SLAVER 现行表**（仅改 MASTER 固件）。

已与用户确认的决策：

| # | 决策 | 结论 |
|---|------|------|
| 1 | 界面形态 | **双板同屏并列**：顶部共用连接栏，左侧 E1_MASTER 面板、右侧 E1_SLAVER 面板，底部共用日志区；自动轮询同时查两块板 |
| 2 | 文件位置 | 仓库根新建 `ctu_host/`（含 `ctu_protocol.py` + `ctu_host.py` + `README.md`） |
| 3 | 旧文件处置 | 合并版验证通过后删除原两工程 `host/` 下 4 个旧文件及 `__pycache__`（git 历史可回溯） |
| 4 | 统一命令表基准 | **统一为 SLAVER 现行表**，见 §3；MASTER 固件按此表对齐，SLAVER 固件与协议文档命令号不变 |
| 5 | MASTER 0x05/0x06 | **0x05 清除故障锁存 = 实现**（调 `app_fault_policy_reset()`）；**0x06 升级 = 暂应答不支持**（等后续 Boot 适配任务，与 SLAVER 一致接入 E1_CTU_BOOT） |
| 6 | 范围外 | 不合并 E1_CTU_BOOT/host/boot_host.py（升级工具保持独立）；MASTER 完整升级链路（AppA 重链/VTOR/srv_boot_ctrl）另行计划 |

## 2. 现实约束（已核实，勿再"公用=全同"）

- 两板 z 帧信封完全一致：`['z' 0x7A][addr][cmd][data_len][payload][CRC8]['\n']`，总长=data_len+6，CRC8(poly 0x31, init 0xFF)，小端，两板按 header `{'z',addr}` 过滤（E1_MASTER=0x01、E1_SLAVER=0x02）。
- 现状命令码确实不同（已核对 `srv_com_mst.h`/`srv_com_slv.h` + 两份协议文档）：
  - MASTER：0x02=读温度 / 0x03=读电压 / 0x05=升级(不支持)
  - SLAVER：0x02=读电压 / 0x03=读温度+VDDA / 0x05=清锁存 / 0x06=升级
- 状态/电压/温度/控制的 **数据段布局按板不同**（合并后仍需按板解码，这正符合"数据段不一样"）。

## 3. 统一命令表（目标，两板一致；应答 = 0x80|cmd）

| cmd | 帧名 | MASTER 数据段（应答） | SLAVER 数据段（应答） |
|-----|------|----------------------|----------------------|
| 0x01 | 读系统状态 | 0x81：2B（b0: estop/r12/r24/rvin/raux/rmotor；b1: fan0/fan1/ntc1/ntc2） | 0x81：2B（b0: err_24v/12v/aux/motor/lsd1/lsd2；b1: out_24v/12v/lsd1/lsd2/latch） |
| 0x02 | 读电压 | 0x82：4B vin/vin_dcdc (u16 mV) | 0x82：8B aux/motor/lsd1/lsd2 (u16 mV) |
| 0x03 | 读温度 | 0x83：6B ntc1/ntc2/mcu (i16 ×100℃) | 0x83：4B mcu_temp(i16×100)+vdda(u16 mV) |
| 0x04 | 控制 | 0x84：负载1B buzzer_duty(0-50) | 0x84：负载4B mask+0x00+duty u16(0-1000) |
| 0x05 | 清除故障锁存 | 0x85：负载1B magic=0x01；**实现**（清保护锁存） | 0x85：负载1B magic=0x01；清故障锁存 |
| 0x06 | 升级请求 | 负载1B magic=0x01；**应答不支持 0x7F err=0x02** | 0x86：ACK 后跳 Bootloader |

## 4. 任务清单

### 阶段 A：MASTER 固件命令码对齐（改动最小面）

改动文件（仅 E1_MASTER_POWER_CTU，SLAVER 固件零改动）：

1. `E1_MASTER_POWER_CTU/service/srv_com_mst.h`
   - 版本 V3.0.0 → V4.0.0；命令枚举改为：`READ_STATUS=0x01, READ_VOLT=0x02, READ_TEMP=0x03, CTRL=0x04, RESET_LATCH=0x05, UPGRADE=0x06`（注释同步：0x02 电压 4B、0x03 温度 6B、0x05 清锁存、0x06 升级暂不支持）。
   - 新增回调类型 `typedef void (*srv_com_mst_reset_cb_t)(void);`；`srv_com_mst_config_t` 增加可选成员 `.reset_latch`。
   - 顶部 Doxygen 注释补充 0x05/0x06 语义及"命令码与 E1_SLAVER 统一"说明。
2. `E1_MASTER_POWER_CTU/service/srv_com_mst.c`
   - `init()`：保存可选 `reset_latch`（允许 NULL）。
   - `com_handle_frame()`：`READ_TEMP`/`READ_VOLT` 两个 case 的打包体对调（0x02→volt 4B，0x03→temp 6B）。
   - 新增 `case SRV_COM_MST_CMD_RESET_LATCH`：`plen==1 && payload[0]==0x01` 时，若有 reset 回调则调用并应答 0x85+[0x00]，否则应答不支持；非法长度应答 BAD_LEN。
   - `case SRV_COM_MST_CMD_UPGRADE`（0x06）：应答不支持（原 0x05 的不支持逻辑迁移至此）。
3. `E1_MASTER_POWER_CTU/tasks/com_task.c`
   - include `app_fault_policy.h`；新增 `static void com_reset_latch(void){ app_fault_policy_reset(); }`；cfg 增加 `.reset_latch = com_reset_latch`；注释 0x10→0x04 字样顺手修正（现注释残留 0x10/0x11 旧号，一并改为 0x04/0x05）。
4. `E1_MASTER_POWER_CTU/docs/protocol_master_485.md`
   - 命令表更新为 §3 统一表；0x82 改为"电压应答(4B)"、0x83 改为"温度应答(6B)"并交换两小节正文；新增 0x85 清锁存小节、0x86 升级（预留，本阶段应答不支持，将来对接 E1_CTU_BOOT）；交互示例用新号（`z 01 04 01 32` 蜂鸣器等）；版本记录加 V4.0.0（与 SLAVER 命令码统一：0x02 电压/0x03 温度、新增 0x05 清锁存、升级 0x05→0x06）。
   - 顺带修正文档正文中任何残留旧号（如"升级 0x05"相关描述）。
5. 引用同步（搜 `0x05|0x06|升级` 全仓库确认）：
   - `E1_MASTER_POWER_CTU/docs/调试工作总结.md`：命令列表（现残留 0x02 温度/0x03 电压/0x10/0x11）改为统一表；host 引用改为新 `ctu_host/` 路径。
   - `E1_CTU_BOOT/host/boot_protocol.py`：`UPGRADE_CMD` 中 master 0x05→0x06；`DEVICE_NAMES`/docstring 同步（`Master ... 0x01/0x06`）。
   - `E1_CTU_BOOT/host/boot_host.py`：复选框文案 `0x01/0x05、0x02/0x06` → `0x01/0x06、0x02/0x06`（即两板均为 0x06）。
   - `E1_CTU_BOOT/docs/boot_485_ymodem.md`：Master 升级命令 0x05→0x06（§升级流程/§后续任务清单/版本记录补一行）；Host GUI 文件名单更新为合并后的 `ctu_host/ctu_host.py`。

### 阶段 B：合并上位机（新建 `ctu_host/`）

新建目录 `ctu_host/`（仓库根），三个文件：

6. `ctu_host/ctu_protocol.py` — 合并自两套 protocol（以"公共信封 + 按板数据段"组织）：
   - 公共：`DEV_ADDR_MASTER/SLAVER`、统一命令常量（§3）、`CMD_REPLY_FLAG/CMD_ERR`、`ERR_TEXT`、CRC8 表与 `crc8()`、`build_frame()`、`FrameParser`、`parse_addr/cmd/payload`、`unpack_i16_le/u16_le`、`frame_hex`；`CMD_NAME`（按统一表：0x05 清除故障锁存、0x06 升级请求）。
   - 按板数据段（函数名以 `mst_`/`slv_` 区分，保留两套现有 keys/文本）：
     - 状态解码：`decode_mst_status(payload)`、`decode_slv_status(payload)` 及各自 text（沿用现 ctu_protocol `status_items`/`decode_status` 与 slv_protocol `status_items`/`decode_status_text` 逻辑与 key 命名）；
     - 电压/温度解码：`decode_mst_volt`/`decode_slv_volt`、`decode_mst_temp`/`decode_slv_temp`；
     - 构建：`build_read_status/volt/temp(addr)`（公共，0x01/0x02/0x03）、`build_mst_ctrl(buzzer_duty, addr)`、`build_slv_ctrl(mask, duty, addr)`、`build_reset_latch(addr)`（公共 0x05）。
   - 模块 docstring 注明：统一命令表、两板数据段差异、与两份 `docs/protocol_*_485.md` 对齐。
7. `ctu_host/ctu_host.py` — PySide6 + pyserial 单窗口双板并列：
   - 顶部连接栏：串口/刷新/波特率(默认115200)/连接断开 + `自动轮询` + 间隔 spin(100-2000ms, 默认500) + `显示收发帧`(默认勾选)。
   - 主体左右两列（`QGroupBox` 分列，固定宽度相当）：
     - **左 = E1_MASTER (0x01)**：查询按钮（读状态/读电压/读温度）；控制区＝蜂鸣器滑块(0-50)+发送控制帧+静音+**清除保护锁存(0x05)**；数据槽位（沿用 ctu_host 15 行，未读取保持 "--" 不折叠，estop/轨/风扇/NTC 着色规则不变）。
     - **右 = E1_SLAVER (0x02)**：查询按钮（读状态/读电压/读温度含VDDA）；控制区＝24V/12V_ISO/LSD1/LSD2 勾选 + 补光滑块(0-1000,0.1%)+发送输出控制/全开/全关/清除锁存(0x05)；数据槽位（沿用 slv_host 17 行）。
     - 每列头部各放一个"轮询"复选（默认勾选），供只挂一块板时停用另一块。
   - 底部：共用日志区（清空按钮），`SerialWorker(QThread)` 单实例 + `FrameParser`；按 `parse_addr` 把应答路由到对应面板解码刷新。
   - 收发帧日志沿用 master 版格式：`RX {E1_MASTER/E1_SLAVER}: {hex} [{名称}]`，名称取 `CMD_NAME[base]+应答/命令`，0x7F→错误应答；`显示收发帧` 勾选且**非自动轮询时**才输出（沿用 slv_host 防刷屏策略）；TX 手动按钮带 tag。
   - 自动轮询：每 tick 依次向两块启用轮询的板发 `build_read_status→volt→temp`（log=False）；应答按 addr 分别刷新两面板数据槽。控制/清锁存按钮为手动发送（log=True）。
   - 复用现有 `STYLE` 配色；窗口标题 `E1 CTU 电源板 RS485 调试上位机`。
8. `ctu_host/README.md`：运行方式 `python ctu_host.py`、依赖 `pyserial + PySide6`、统一命令表简述、两板协议文档链接、注意事项（MASTER 需先烧新固件才支持 0x05；0x06 升级见 E1_CTU_BOOT）。

### 阶段 C：删除旧文件 + 收尾文档

9. 删除：`E1_MASTER_POWER_CTU/host/ctu_host.py`、`ctu_protocol.py`、`__pycache__/`；`E1_SLAVER_POWER_CTU/host/slv_host.py`、`slv_protocol.py`、`__pycache__/`。
10. 引用清理：
    - `E1_SLAVER_POWER_CTU/README.md` 目录树删 `host/` 行并指向 `ctu_host/`；`E1_SLAVER_POWER_CTU/docs/调试工作总结.md` host 表改述为合并工具并提统一表。
    - `E1_MASTER_POWER_CTU/docs/调试工作总结.md`（见任务 5）同步。
    - 需要时在仓库根 `AGENTS.md` 工作区布局一节补一行 `ctu_host/`（共享 485 调试上位机，两板命令码统一）。

## 5. 实施顺序

1. 阶段 A 固件 + 文档（先定契约）。
2. 阶段 B 合并上位机实现。
3. 语法/导入自检（见 §6），通过后删除旧文件（阶段 C）。
4. 主板上位机使用前需烧录新固件（rollout 见 §7）。

## 6. 验证计划

1. **固件编译**：`E1_MASTER_POWER_CTU` 用现有方式（`build.bat` 或 `cmake --preset Debug && cmake --build --preset Debug`）Debug+Release 均通过、无告警；不改任何 CMakeLists。
2. **Python 静态**：`python -m py_compile ctu_protocol.py ctu_host.py`；`python -c "import ctu_host, ctu_protocol"` 无导入错误（PySide6/pyserial 已装，`__pycache__` 显示 cpython-312 存在）。
3. **协议单元自检**（离线脚本或 `python -c`）：用两板协议文档中的示例字节往返验证——`build_frame`+CRC8、状态/电压/温度解码（如 master 温度 6B→3 个 i16；slave 电压 8B→4 个 u16）、master 0x04 负载 1B、slave 0x04 负载 4B、0x05 负载 `[0x01]`。
4. **GUI 冒烟**：条件允许时 `QT_QPA_PLATFORM=offscreen` 实例化 MainWindow 一次，确认布局不抛异常。
5. **硬件联调（用户执行）**：两板同挂 485 总线 → 连接 → 自动轮询两板数据实时刷新；MASTER 蜂鸣器控制/清锁存、SLAVER 输出控制/清锁存；收发帧日志颜色与含义正确；只挂单板时关掉另一板轮询无报错。

## 7. 风险与注意

- **时序/兼容**：命令码改动是**不兼容升级**——MASTER 板必须烧新固件后，新上位机对 0x02/0x03/0x05 的语义才正确；旧固件与新上位机混用会电压温度对调。SLAVER 固件无变化。
- **boot_host**：master 升级命令号 0x05→0x06 需同步（boot_protocol.py/boot_host.py/boot doc），否则未来触发升级会错发 0x05（现语义=清锁存）。master 0x06 在固件完成 Boot 适配前仍应答不支持，boot_host 对 master 的"请求升级"预期失败，属已知未实现项。
- **0x05 安全**：master 清除保护锁存等同急停释放沿的自动解锁；仅建议在确认安全时使用（文档中注明）。
- **自动轮询总线占用**：12 帧/周期在 115200 下开销小；两块板按地址应答互不冲突。
- 不修改任何 `CMakeLists.txt`、`Core/` 生成代码、`public_layer/`。
