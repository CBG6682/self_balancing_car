@echo off
REM ==============================================================================
REM Master Pipeline Execution Script: Self-Balancing Robot
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
echo  [PIPELINE] STM32 Self-Balancing Robot Automated Toolchain
echo ==========================================================

if "%1"=="--selftest" goto RUN_SELFTEST
if "%1"=="--diag" goto RUN_SELFTEST
if "%1"=="-d" goto RUN_SELFTEST
if "%1"=="--all" goto RUN_ALL
if "%1"=="-a" goto RUN_ALL
if "%1"=="--check" goto RUN_CHECK
if "%1"=="-c" goto RUN_CHECK
if "%1"=="--hardware" goto RUN_HW
if "%1"=="-hw" goto RUN_HW
if "%1"=="--motor" goto RUN_MOTOR
if "%1"=="-m" goto RUN_MOTOR
if "%1"=="--sensor" goto RUN_SENSOR
if "%1"=="-s" goto RUN_SENSOR
if "%1"=="--bridge" goto RUN_BRIDGE
if "%1"=="-b" goto RUN_BRIDGE
if "%1"=="--loopback" goto RUN_LOOPBACK
if "%1"=="-lb" goto RUN_LOOPBACK
if "%1"=="--flash" goto RUN_FLASH
if "%1"=="-f" goto RUN_FLASH
if "%1"=="--stlink" goto RUN_STLINK
if "%1"=="-st" goto RUN_STLINK
if "%1"=="--sim" goto RUN_SIM
if "%1"=="--step2" goto RUN_SIM
if "%1"=="--data" goto RUN_DATA
if "%1"=="--step4" goto RUN_DATA
if "%1"=="--tune" goto RUN_TUNE
if "%1"=="--step5" goto RUN_TUNE
if "%1"=="--test" goto RUN_TEST
if "%1"=="--step6" goto RUN_TEST

echo Chon che do chay:
echo   1) [KHUYEN NGHI] Kiem tra TONG THE tung linh kien co xac nhan (Tools\system_self_test.py)
echo   2) Chay TOAN BO quy trinh Auto-Tuning (Mo phong -> Thu thap -> Toi uu -> Build)
echo   3) Mo phong Vat ly & Khoi tao LQR PID
echo   4) Thu thap / Sinh du lieu telemetry (CSV)
echo   5) Nhan dang he thong ARX & Toi uu hoa PID
echo   6) Kiem thu kich ban dieu khien & Ve do thi phan hoi
echo   7) Chi bien dich firmware STM32 (build_firmware.bat)
echo   8) Kiem tra phan cung & Tune truc tiep (Tools\hardware_check.py)
echo   9) Kiem tra dong co tuong tac (Tools\motor_test_interactive.py)
echo  10) Bang theo doi cam bien IMU & Encoder (Tools\sensor_test_live.py)
echo  11) Kiem tra thong tuyen UART ESP32 ^<--^> STM32 (Tools\test_bridge_live.py)
echo  12) Nap firmware STM32 qua USB HID Type-C (Tools\flash_hid.py)
echo  13) Nap firmware STM32 qua ST-Link V2/V3 (Tools\flash_stlink.py)
echo  14) Chan doan & Lang nghe ket noi USB-UART (Tools\check_connection.py)
echo   0) Thoat
echo ----------------------------------------------------------
set /p choice="Nhap lua chon [1-14, 0]: "

if "%choice%"=="1" goto RUN_SELFTEST
if "%choice%"=="2" goto RUN_ALL
if "%choice%"=="3" goto RUN_SIM
if "%choice%"=="4" goto RUN_DATA
if "%choice%"=="5" goto RUN_TUNE
if "%choice%"=="6" goto RUN_TEST
if "%choice%"=="7" goto REBUILD
if "%choice%"=="8" goto RUN_HW
if "%choice%"=="9" goto RUN_MOTOR
if "%choice%"=="10" goto RUN_SENSOR
if "%choice%"=="11" goto RUN_BRIDGE
if "%choice%"=="12" goto RUN_FLASH
if "%choice%"=="13" goto RUN_STLINK
if "%choice%"=="14" goto RUN_CHECK
if "%choice%"=="0" exit /b 0
echo [ERROR] Lua chon khong hop le!
exit /b 1

:RUN_SIM
echo.
echo >>> [1/4] Mo phong Vat ly & Khoi tao LQR PID...
"%PYTHON_EXEC%" "%~dp0Tools\sim_and_init_pid.py"
if "%choice%"=="2" exit /b 0
goto :eof

:RUN_DATA
echo.
echo >>> [2/4] Thu Thap Du Lieu / Sinh Du Lieu Gia Lap...
"%PYTHON_EXEC%" "%~dp0Tools\data_acquisition.py"
if "%choice%"=="3" exit /b 0
goto :eof

:RUN_TUNE
echo.
echo >>> [3/4] Nhan Dang He Thong & Toi Uu Hoa PID...
"%PYTHON_EXEC%" "%~dp0Tools\auto_tune_pid.py"
if "%choice%"=="4" (
    call :REBUILD
    exit /b 0
)
goto :eof

:RUN_TEST
echo.
echo >>> [4/4] Kiem Thu Kich Ban Dieu Khien Tu Dong & Ve Do Thi...
"%PYTHON_EXEC%" "%~dp0Tools\test_control_sequence.py"
if "%choice%"=="5" exit /b 0
goto :eof

:REBUILD
echo.
echo >>> [AUTO-BUILD] Bien dich lai firmware STM32...
call "%~dp0build_firmware.bat"
goto :eof

:RUN_CHECK
echo.
echo >>> [CHECK] Chan doan va Lang nghe ket noi USB-UART...
"%PYTHON_EXEC%" "%~dp0Tools\check_connection.py"
exit /b 0

:RUN_HW
echo.
echo >>> [HARDWARE] Kiem tra phan cung va Tune truc tiep...
"%PYTHON_EXEC%" "%~dp0Tools\hardware_check.py"
exit /b 0

:RUN_MOTOR
echo.
echo >>> [MOTOR] Kiem tra dong co tuong tac...
"%PYTHON_EXEC%" "%~dp0Tools\motor_test_interactive.py"
exit /b 0

:RUN_SENSOR
echo.
echo >>> [SENSOR] Bang theo doi cam bien IMU va Encoder...
"%PYTHON_EXEC%" "%~dp0Tools\sensor_test_live.py"
exit /b 0

:RUN_BRIDGE
echo.
echo >>> [BRIDGE] Kiem tra thong tuyen UART thoi gian thuc...
"%PYTHON_EXEC%" "%~dp0Tools\test_bridge_live.py"
exit /b 0

:RUN_LOOPBACK
echo.
echo >>> [LOOPBACK] Kiem tra toan bo tin hieu Loopback la PING...
"%PYTHON_EXEC%" "%~dp0Tools\test_stm32_loopback.py"
exit /b 0

:RUN_FLASH
echo.
echo >>> [FLASH] Nap firmware STM32 qua USB HID Bootloader...
"%PYTHON_EXEC%" "%~dp0Tools\flash_hid.py" "%~dp0build\Debug\self_balancing_car.bin"
exit /b 0

:RUN_STLINK
echo.
echo >>> [ST-LINK] Nap firmware STM32 qua ST-Link V2/V3 (SWD)...
"%PYTHON_EXEC%" "%~dp0Tools\flash_stlink.py" "%~dp0build\Debug\self_balancing_car.bin"
exit /b 0

:RUN_SELFTEST
echo.
echo >>> [SELF-TEST] Kiem tra tong the tung linh kien co xac nhan...
"%PYTHON_EXEC%" "%~dp0Tools\system_self_test.py"
exit /b 0

:RUN_ALL
call :RUN_SIM
call :RUN_DATA
call :RUN_TUNE
call :RUN_TEST
call :REBUILD
echo.
echo ==========================================================
echo  [HOAN THANH TRON VEN CHUOI PIPELINE AUTO-TUNING]
echo   - Header C da duoc cap nhat: Modules\Inc\pid_AutoConfig.h
echo   - Firmware da duoc bien dich: build\Debug\self_balancing_car.bin
echo   - Do thi kiem thu da duoc luu: Tools\test_result_plot.png
echo ==========================================================
exit /b 0
