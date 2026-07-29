@echo off
setlocal

set "SYSCONFIG_CLI=E:\NUEDC\sysconfig_cli.bat"
set "SDK_PRODUCT=E:\NUEDC\mspm0_sdk_2_02_00_05\.metadata\product.json"
set "PROJECT_DIR=%~dp0"
set "PROJECT_DIR=%PROJECT_DIR:~0,-1%"

if not exist "%SYSCONFIG_CLI%" (
    echo SysConfig CLI not found: %SYSCONFIG_CLI%
    exit /b 1
)

if not exist "%SDK_PRODUCT%" (
    echo MSPM0 SDK product file not found: %SDK_PRODUCT%
    exit /b 1
)

call "%SYSCONFIG_CLI%" -o "%PROJECT_DIR%" -s "%SDK_PRODUCT%" --compiler keil --device "MSPM0G350X" --package "LQFP-64(PM)" --part "Default" "%PROJECT_DIR%\empty_mspm0g3507.syscfg"
exit /b %ERRORLEVEL%
