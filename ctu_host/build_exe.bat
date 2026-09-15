@echo off
REM 打包 E1 CTU 上位机为单文件 Windows exe（产物: dist\ctu_host.exe）
REM 依赖: pip install -r requirements.txt pyinstaller
setlocal
cd /d "%~dp0"

python -m PyInstaller --noconfirm --clean ctu_host.spec
if errorlevel 1 (
    echo.
    echo [失败] 打包未完成，请检查上方 PyInstaller 输出。
    exit /b 1
)

echo.
echo [完成] %~dp0dist\ctu_host.exe
endlocal
