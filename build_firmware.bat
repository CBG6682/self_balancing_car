@echo off
REM ==============================================================================
REM Build Firmware Script for STM32F401CCU6 Self-Balancing Robot
REM Target: Windows (ARM GNU Toolchain + CMake + Ninja / MinGW Makefiles)
REM ==============================================================================

echo ==========================================================
echo  [BUILD] Compiling STM32F401CCU6 Balance Robot Firmware
echo ==========================================================

REM 1. Check cmake
where cmake >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] 'cmake' chua duoc tim thay. Vui long cai dat CMake 3.22+.
    exit /b 1
)

REM 2. Check arm-none-eabi-gcc
where arm-none-eabi-gcc >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] 'arm-none-eabi-gcc' chua duoc tim thay trong PATH.
    echo         Vui long cai dat Arm GNU Toolchain.
    exit /b 1
)

set BUILD_DIR=%~dp0build\Debug

echo [INFO] Cau hinh CMake...
cmake --preset Debug
if %ERRORLEVEL% NEQ 0 (
    cmake -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE="%~dp0cmake\gcc-arm-none-eabi.cmake"
)

echo [INFO] Dang bien dich firmware...
cmake --build "%BUILD_DIR%"

if exist "%BUILD_DIR%\self_balancing_car.bin" (
    echo ----------------------------------------------------------
    echo [SUCCESS] Bien dich firmware thanh cong!
    echo   File BIN: %BUILD_DIR%\self_balancing_car.bin
    echo   File HEX: %BUILD_DIR%\self_balancing_car.hex
    echo ----------------------------------------------------------
) else (
    echo [ERROR] Bien dich that bai! Vui long kiem tra loi phia tren.
    exit /b 1
)
echo ==========================================================
