# E1_Master_Power_Manage 固件升级上位机（flash_tool）

自 `stm32_g474_boot/updata_tool` 移植，按 E1 协议（经典 CAN 8 字节帧、HW_ID=0x0002）适配。
基于 CANable 2.5（USB-CAN）与 PySide6。

## 依赖
- Python 3.9+，`pip install PySide6 pyusb`
- CANable 设备（本仓库已随附 `canable_sdk` 与 Windows `libusb-1.0.dll`）

## 打包为 exe（Windows）
需要 `pip install pyinstaller`。在项目根目录运行：

```bash
# Windows (CMD)
updata_tool\build_exe.bat
# 或直接：
python -m PyInstaller updata_tool/UpdataTool.spec --noconfirm --clean \
    --workpath updata_tool/build --distpath updata_tool/dist
```

产物： `updata_tool/dist/UpdataTool.exe`（单文件，约 47 MB，内含 `libusb-1.0.dll`，可直接拷贝分发）。
打包配置见 [../UpdataTool.spec](../UpdataTool.spec)：入口为项目根目录 `flash_gui.py`，
`libusb-1.0.dll` 打包到根目录以便 `canable_sdk/__init__.py` 的 `os.add_dll_directory` 命中；
`cangui` 被排除（本工具不需要通用 CANable GUI）。

## 运行
```bash
cd updata_tool
python -m flash_tool            # 或 python -m updata_tool.flash_tool
```

## 使用（E1 主控电源板）
1. **扫描并连接** CANable 设备；波特率选 **1,000,000 bps**（E1 CAN1 = 1Mbps）。
2. **HW Compat ID** 已默认 `0x0002`（E1 Boot 的硬件兼容 ID；勿改回 0x0001，否则 Boot 回 NACK `HW_MISMATCH`）。
3. **CAN ID** 默认 `0x701`（Host→Node；Node→Host 自动 `0x702`）。
4. 选择待升级的 `E1_Master_Power_Manage.bin`（App 镜像，链接于 `0x08020000`）。
5. 点「开始升级」。工具**直接发送 `0x003`**（1 字节 `0x01`）触发进入升级模式——
   `0x003` 对已在 Boot 的设备无害（Boot 忽略之），无需先探测。随后等 Boot 心跳 beacon
   （`0x702`，含 `hw_id` 核对）确认后开始握手，App→Boot 约 1s。
   （App 未处理 0x003 或设备异常时，会报「未检测到 Boot 心跳」。）
6. 握手后按 START → METADATA → (DATA_START→DATA→DATA_END)×N → VERIFY → REBOOT 流程，
   固件写入**对侧** A/B 分区，整包 32-bit 累加和校验通过后 Boot 自动复位并跳转新 App。

> E1 为经典 bxCAN，**仅支持 8 字节帧**（CAN FD 已从界面移除）；
> 升级协议细节见 `docs/boot_upgrade.md`，迁移记录见 `docs/boot_migration.md`。
