# ctu_flasher — E1 CTU 固件打包 / J-Link 烧录工具

把 **E1_CTU_BOOT**（Bootloader）与 **E1_MASTER_POWER_CTU / E1_SLAVER_POWER_CTU**（App）
的固件按 Flash 布局拼接为可直接烧录的整包，并调用 **SEGGER J-Link Commander** 烧录到目标板。

- 单个 Python 文件、仅用标准库 + tkinter（`ctu_flasher.py`），无需安装第三方包；
- 固件预检：分区容量、向量表（SP/复位向量）、App 签名（`A+0x200` 魔数 `0x41505031`）；
- 烧录方式：整包（Boot+App，可选全片擦除）或仅 App（不动 Boot，调试用）。

## Flash 布局（见 `E1_CTU_BOOT/docs/boot_485_ymodem.md`）

| 区域 | 起始地址 | 大小 | 说明 |
|------|----------|------|------|
| Boot | `0x08000000` | 32K (0x8000) | E1_CTU_BOOT |
| App A | `0x08008000` | 96K (0x18000) | E1_MASTER / E1_SLAVER App（固定链接地址） |
| App B | `0x08020000` | 96K | 485 升级暂存槽（本工具不烧） |
| Metadata | `0x0803E000` | 8K | Boot 元数据（本工具不烧，首次上电由 Boot 生成） |

整包 = `Boot` + `0xFF` 补齐到 0x8000 + `App`，从 `0x08000000` 起一次写完。

## 运行

```bash
python ctu_flasher.py
```

要求：已安装 SEGGER J-Link 软件（自动检测 `C:\Program Files\SEGGER\JLink_V*\JLink.exe`，
也可手动浏览）。USB-RS485 与本工具无关，烧录走 J-Link SWD。

使用流程：

1. 「自动检测」从各工程 `build/**` 取最新 `.bin`（找不到时会尝试
   `ctu_host/ctu_sdk_c/build/` 下的示例固件），或手动选择三个 `.bin`；
2. 「打包固件」→ 输出目录生成 `E1_MASTER_POWER_CTU_merged.bin`、
   `E1_SLAVER_POWER_CTU_merged.bin`；
3. 选目标板（MASTER / SLAVER）→「开始烧录」。
4. 「仅擦除全片」：连接目标后只执行全片擦除、不写任何固件（会二次确认）。
   擦除后板上无固件可运行，需重新烧录；仅 App 模式绝不会全片擦除（保护 Boot）。

## 路径记忆（自动保存，下次直接用）

固件路径、J-Link 路径、输出目录与烧录设置会**改动即自动保存**（600ms 防抖），
窗口关闭/重启后自动恢复；每个固件输入框（Boot / MASTER App / SLAVER App / JLink.exe）
还提供**最近使用下拉**（每个最多 10 条，去重置顶）。

- 配置文件：`%USERPROFILE%\.ctu_flasher.json`（界面底部显示实际路径）；删除该文件即恢复默认。
- 启动时若上次的固件路径已失效（文件被移动/删除），日志会给出告警，其余设置仍会恢复。
- 也可点「自动检测」按各工程 `build/**` 重新取最新 `.bin`。

## 打包为 exe（可选）

```bash
build_exe.bat
# 或
python -m PyInstaller --noconfirm --clean ctu_flasher.spec
```

产物为单文件 `dist\ctu_flasher.exe`（无控制台窗口），目标机无需安装 Python。

## 注意

- 整包模式勾选「烧录前全片擦除」会清空整片 Flash（含 App B / Metadata），
  量产首次烧录建议全片擦除；**仅 App 模式会自动禁用全片擦除**，避免擦掉 Boot。
- App 必须带 `.app_sig` 签名（链接脚本固定放在 `App+0x200`）。Boot 若以
  `BOOT_SKIP_APP_VERIFY=0`（量产完整校验）构建，还需要 metadata 中的
  `fw_size/fw_checksum` 与 App 匹配；直接用 J-Link 烧 App 调试时，
  Boot 应为 `BOOT_SKIP_APP_VERIFY=1`。工具会对缺失签名给出告警。
- 日常 485 在线升级请使用 `ctu_host`（上位机）/`boot_host.py`，本工具面向
  产线/调试阶段的整片烧录。
