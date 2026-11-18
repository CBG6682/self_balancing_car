@echo off
REM ==============================================================================
REM Master Pipeline Execution Script: STM32 Self-Balancing Robot
REM Target: Windows (Command Prompt / PowerShell)
REM ==============================================================================

setlocal enabledelayedexpansion

set PYTHON_EXEC=%~dp0venv\Scripts\python.exe

if not exist "%PYTHON_EXEC%" (
    echo [WARN] Chua tim thay virtual environment tai venv\Scripts\python.exe.
    echo [INFO] Dang tu dong chay setup.bat...
    call "%~dp0setup.bat"
)

echo ==========================================================
echo  [PIPELINE] STM32 Self-Balancing Robot Toolchain
echo ==========================================================

if "%1"=="--selftest" goto RUN_SELFTEST
if "%1"=="--diag" goto RUN_SELFTEST
if "%1"=="-d" goto RUN_SELFTEST
if "%1"=="--check" goto RUN_CHECK
if "%1"=="-c" goto RUN_CHECK
if "%1"=="--calib" goto RUN_CALIB
if "%1"=="-s" goto RUN_CALIB
if "%1"=="--deadzone" goto RUN_DEADZONE
if "%1"=="-dz" goto RUN_DEADZONE
if "%1"=="--offset" goto RUN_OFFSET
if "%1"=="-o" goto RUN_OFFSET
if "%1"=="--tune" goto RUN_TUNE
if "%1"=="-t" goto RUN_TUNE
if "%1"=="--monitor" goto RUN_MONITOR
if "%1"=="-m" goto RUN_MONITOR
if "%1"=="--record" goto RUN_RECORD
if "%1"=="-r" goto RUN_RECORD
if "%1"=="--sim" goto RUN_SIM
if "%1"=="--autotune" goto RUN_AUTOTUNE
if "%1"=="--build" goto REBUILD
if "%1"=="-b" goto REBUILD
if "%1"=="--stlink" goto RUN_STLINK
if "%1"=="-st" goto RUN_STLINK
if "%1"=="--flash" goto RUN_FLASH
if "%1"=="-f" goto RUN_FLASH

echo Chon cong cu thuc thi:
echo   1) [KHUYEN NGHI] Kiem tra tong the phan cung (Tools\system_self_test.py)
echo   2) Kiem tra ket noi USB-UART / ESP32 Bridge  (Tools\check_connection.py)
echo   3) Hieu chuan cam bien MPU6050 ^& Encoder     (Tools\live_sensor_calib.py)
echo   4) Do Deadzone tung dong co GA25             (Tools\find_deadzone.py)
echo   5) Tim goc Offset can bang co khi            (Tools\find_zero_balance.py)
echo   6) Tinh chinh PID truc tiep                  (Tools\live_balance_tuning.py)
echo   7) Giam sat ^& gui lenh qua ESP32 Bridge      (Tools\esp32_tune_monitor.py)
echo   8) Ghi du lieu telemetry 100Hz (CSV)         (Tools\record_session.py)
echo   9) Mo phong vat ly ^& khoi tao LQR PID        (Tools\sim_and_init_pid.py)
echo  10) Toi uu hoa PID tu dong (ARX/ITAE)         (Tools\auto_tune_pid.py)
echo  11) Bien dich firmware STM32                  (build_firmware.bat)
echo  12) Nap firmware qua ST-Link V2/V3 (SWD)      (Tools\flash_stlink.py)
echo  13) Nap firmware qua USB HID Bootloader       (Tools\flash_hid.py)
echo   0) Thoat
echo ----------------------------------------------------------
set /p choice="Nhap lua chon [0-13]: "

if "%choice%"=="1" goto RUN_SELFTEST
if "%choice%"=="2" goto RUN_CHECK
if "%choice%"=="3" goto RUN_CALIB
if "%choice%"=="4" goto RUN_DEADZONE
if "%choice%"=="5" goto RUN_OFFSET
if "%choice%"=="6" goto RUN_TUNE
if "%choice%"=="7" goto RUN_MONITOR
if "%choice%"=="8" goto RUN_RECORD
if "%choice%"=="9" goto RUN_SIM
if "%choice%"=="10" goto RUN_AUTOTUNE
if "%choice%"=="11" goto REBUILD
if "%choice%"=="12" goto RUN_STLINK
if "%choice%"=="13" goto RUN_FLASH
if "%choice%"=="0" exit /b 0
echo [ERROR] Lua chon khong hop le!
exit /b 1

:RUN_SELFTEST
"%PYTHON_EXEC%" "%~dp0Tools\system_self_test.py"
exit /b 0

:RUN_CHECK
"%PYTHON_EXEC%" "%~dp0Tools\check_connection.py"
exit /b 0

:RUN_CALIB
"%PYTHON_EXEC%" "%~dp0Tools\live_sensor_calib.py"
exit /b 0

:RUN_DEADZONE
"%PYTHON_EXEC%" "%~dp0Tools\find_deadzone.py"
exit /b 0

:RUN_OFFSET
"%PYTHON_EXEC%" "%~dp0Tools\find_zero_balance.py"
exit /b 0

:RUN_TUNE
"%PYTHON_EXEC%" "%~dp0Tools\live_balance_tuning.py"
exit /b 0

:RUN_MONITOR
"%PYTHON_EXEC%" "%~dp0Tools\esp32_tune_monitor.py"
exit /b 0

:RUN_RECORD
"%PYTHON_EXEC%" "%~dp0Tools\record_session.py"
exit /b 0

:RUN_SIM
"%PYTHON_EXEC%" "%~dp0Tools\sim_and_init_pid.py"
exit /b 0

:RUN_AUTOTUNE
"%PYTHON_EXEC%" "%~dp0Tools\auto_tune_pid.py"
exit /b 0

:REBUILD
call "%~dp0build_firmware.bat"
exit /b 0

:RUN_STLINK
"%PYTHON_EXEC%" "%~dp0Tools\flash_stlink.py" "%~dp0build\Debug\self_balancing_car.bin"
exit /b 0

:RUN_FLASH
"%PYTHON_EXEC%" "%~dp0Tools\flash_hid.py" "%~dp0build\Debug\self_balancing_car.bin"
exit /b 0
