# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path


BOOT_HOST_DIR = Path(SPECPATH).resolve().parent / 'E1_CTU_BOOT' / 'host'

a = Analysis(
    ['ctu_host.py'],
    pathex=[str(BOOT_HOST_DIR)],
    binaries=[],
    datas=[],
    hiddenimports=['boot_protocol'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='ctu_host',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
