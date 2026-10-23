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

if "%1"=="--all" goto RUN_ALL
if "%1"=="-a" goto RUN_ALL
if "%1"=="--sim" goto RUN_SIM
if "%1"=="--step2" goto RUN_SIM
if "%1"=="--data" goto RUN_DATA
if "%1"=="--step4" goto RUN_DATA
if "%1"=="--tune" goto RUN_TUNE
if "%1"=="--step5" goto RUN_TUNE
if "%1"=="--test" goto RUN_TEST
if "%1"=="--step6" goto RUN_TEST

echo Chon che do chay:
echo   1) Chay TOAN BO quy trinh (Mo phong -> Thu thap -> Toi uu -> Kiem thu -> Rebuild Firmware)
echo   2) Mo phong Vat ly & Khoi tao LQR PID
echo   3) Thu thap / Sinh du lieu telemetry (CSV)
echo   4) Nhan dang he thong ARX & Toi uu hoa PID
echo   5) Kiem thu kich ban dieu khien & Ve do thi phan hoi
echo   6) Chi bien dich firmware STM32 (build_firmware.bat)
echo   0) Thoat
echo ----------------------------------------------------------
set /p choice="Nhap lua chon [1-6, 0]: "

if "%choice%"=="1" goto RUN_ALL
if "%choice%"=="2" goto RUN_SIM
if "%choice%"=="3" goto RUN_DATA
if "%choice%"=="4" goto RUN_TUNE
if "%choice%"=="5" goto RUN_TEST
if "%choice%"=="6" goto REBUILD
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
