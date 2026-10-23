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
if "%1"=="--step2" goto RUN_STEP2
if "%1"=="--step4" goto RUN_STEP4
if "%1"=="--step5" goto RUN_STEP5
if "%1"=="--step6" goto RUN_STEP6

echo Chon che do chay:
echo   1) Chay TOAN BO quy trinh (Step 2 -> 4 -> 5 -> 6 -> Rebuild Firmware)
echo   2) Chay Step 2: Mo phong & Tinh toan LQR PID khoi diem
echo   3) Chay Step 4: Thu thap / Sinh du lieu telemetry (CSV)
echo   4) Chay Step 5: Nhan dang he thong ARX & Toi uu hoa PID
echo   5) Chay Step 6: Kiem thu kịch ban dieu khien & Ve do thi phan hoi
echo   6) Chi bien dich firmware STM32 (build_firmware.bat)
echo   0) Thoat
echo ----------------------------------------------------------
set /p choice="Nhap lua chon [1-6, 0]: "

if "%choice%"=="1" goto RUN_ALL
if "%choice%"=="2" goto RUN_STEP2
if "%choice%"=="3" goto RUN_STEP4
if "%choice%"=="4" goto RUN_STEP5
if "%choice%"=="5" goto RUN_STEP6
if "%choice%"=="6" goto REBUILD
if "%choice%"=="0" exit /b 0
echo [ERROR] Lua chon khong hop le!
exit /b 1

:RUN_STEP2
echo.
echo >>> [1/4] CHAY STEP 2: Mo phong Vat ly & Khoi tao LQR PID...
"%PYTHON_EXEC%" "%~dp0Tools\step2_sim_and_init_pid.py"
if "%choice%"=="2" exit /b 0
goto :eof

:RUN_STEP4
echo.
echo >>> [2/4] CHAY STEP 4: Thu Thap Du Lieu / Sinh Giu Lieu Gia Lap...
"%PYTHON_EXEC%" "%~dp0Tools\step4_data_acquisition.py"
if "%choice%"=="3" exit /b 0
goto :eof

:RUN_STEP5
echo.
echo >>> [3/4] CHAY STEP 5: Nhan Dang He Thong & Toi Uu Hoa PID...
"%PYTHON_EXEC%" "%~dp0Tools\step5_auto_tune_pid.py"
if "%choice%"=="4" (
    call :REBUILD
    exit /b 0
)
goto :eof

:RUN_STEP6
echo.
echo >>> [4/4] CHAY STEP 6: Kiem Thu Kich Ban Dieu Khien Tu Dong & Ve Do Thi...
"%PYTHON_EXEC%" "%~dp0Tools\step6_test_control_sequence.py"
if "%choice%"=="5" exit /b 0
goto :eof

:REBUILD
echo.
echo >>> [AUTO-BUILD] Bien dich lai firmware STM32...
call "%~dp0build_firmware.bat"
goto :eof

:RUN_ALL
call :RUN_STEP2
call :RUN_STEP4
call :RUN_STEP5
call :RUN_STEP6
call :REBUILD
echo.
echo ==========================================================
echo  [HOAN THANH TRON VEN CHUOI PIPELINE AUTO-TUNING]
echo   - Header C da duoc cap nhat: Modules\Inc\pid_AutoConfig.h
echo   - Firmware da duoc bien dich: build\Debug\self_balancing_car.bin
echo   - Do thi kiem thu da duoc luu: Tools\test_result_plot.png
echo ==========================================================
exit /b 0
