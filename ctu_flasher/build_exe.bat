@echo off
REM 打包 E1 CTU 固件打包/烧录工具为单文件 Windows exe（产物: dist\ctu_flasher.exe）
REM 依赖: python + pyinstaller（tkinter 为标准库）
setlocal
cd /d "%~dp0"

python -m PyInstaller --noconfirm --clean ctu_flasher.spec
if errorlevel 1 (
    echo.
    echo [失败] 打包未完成，请检查上方 PyInstaller 输出。
    exit /b 1
)

echo.
echo [完成] %~dp0dist\ctu_flasher.exe
endlocal
