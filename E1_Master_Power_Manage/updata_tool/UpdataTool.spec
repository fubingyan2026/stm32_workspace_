# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec — 将 E1 固件升级上位机（flash_tool）打包为单个 exe。
#
# 运行（在项目根目录）：
#   python -m PyInstaller updata_tool/UpdataTool.spec --noconfirm --clean \
#       --workpath updata_tool/build --distpath updata_tool/dist
#
# 产物： updata_tool/dist/UpdataTool.exe （含 libusb-1.0.dll，可独立分发）

import os

# SPECPATH 是本 spec 所在目录（updata_tool/）
here = os.path.abspath(SPECPATH)
project_root = os.path.abspath(os.path.join(here, ".."))   # E1_Master_Power_Manage/

entry = os.path.join(project_root, "flash_gui.py")          # 顶层启动脚本
libusb = os.path.join(here, "libusb-1.0.dll")               # pyusb 后端 DLL
icon = os.path.join(here, "cangui", "logo.ico")             # exe 图标（CANable logo）

a = Analysis(
    [entry],
    pathex=[project_root, here],
    binaries=[],
    # libusb-1.0.dll 放打包根目录：canable_sdk/__init__.py 里
    # os.add_dll_directory(父目录) 在 onefile 下即 _MEIPASS，可直接命中。
    datas=[(libusb, ".")],
    hiddenimports=["usb.backend.libusb1"],                  # pyusb libusb1 后端
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        "cangui",        # 另一个独立 GUI（通用 CANable），不打包
        "tkinter",       # 无需 Tk
        "matplotlib", "PIL", "numpy",
    ],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="UpdataTool",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,          # GUI 程序，不弹黑窗口
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=icon if os.path.exists(icon) else None,
)
