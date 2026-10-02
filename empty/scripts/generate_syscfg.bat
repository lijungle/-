@echo off
rem 显式指定器件和封装，防止器件覆盖参数导致封装被重置
call "D:\ti\sysconfig_1.23.1\sysconfig_cli.bat" -s "D:\ti\mspm0_sdk_2_04_00_06\.metadata\product.json" -d MSPM0G3507 -p "LQFP-64(PM)" --compiler keil -o "%~dp0.." "%~dp0..\empty.syscfg"
exit /b %errorlevel%
