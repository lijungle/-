@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ============================================================
echo MSPM0 Keil 工程环境一键配置
echo ============================================================
echo.
echo 本脚本将：
echo   1. 设置用户环境变量 MSPM0_SDK / SYSCONFIG
echo   2. 按本机 MSPM0 SDK 路径修补 Keil 工程 include/lib 路径
echo   3. 运行 SysConfig 生成验证
echo.

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\setup_mspm0_env.ps1"
set RET=%ERRORLEVEL%

if not "%RET%"=="0" (
    echo.
    echo 配置失败，请检查上方错误信息。
    pause
    exit /b %RET%
)

echo.
echo 配置成功。请关闭并重新打开 Keil / VSCode，再编译工程。
pause
exit /b 0
