#!/usr/bin/env bash
# 将 E1 固件升级上位机（updata_tool/flash_tool）打包为单个 exe。
# 产物： updata_tool/dist/UpdataTool.exe
set -euo pipefail

cd "$(dirname "$0")/.."

python -m PyInstaller updata_tool/UpdataTool.spec --noconfirm --clean \
    --workpath updata_tool/build --distpath updata_tool/dist

echo
echo "打包完成： updata_tool/dist/UpdataTool.exe"
echo "单文件 exe，内含 libusb-1.0.dll，可直接拷贝分发。"
