@echo off
REM ==============================================================================
REM Setup Environment for Self-Balancing Robot Tools
REM Target: Windows (Command Prompt / PowerShell)
REM ==============================================================================

echo ==========================================================
echo  [SETUP] Self-Balancing Robot - Environment Installation
echo ==========================================================

REM 1. Check Python
where python >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Python chua duoc cai dat hoac chua them vao PATH.
    echo         Vui long cai dat Python 3.8+ tu python.org.
    exit /b 1
)

REM 2. Create Virtual Environment
set VENV_DIR=%~dp0venv
if not exist "%VENV_DIR%" (
    echo [INFO] Dang tao virtual environment tai %VENV_DIR%...
    python -m venv "%VENV_DIR%"
) else (
    echo [INFO] Virtual environment da ton tai tai %VENV_DIR%.
)

REM 3. Install Requirements
echo [INFO] Cai dat thu vien Tools/requirements.txt...
"%VENV_DIR%\Scripts\python.exe" -m pip install --upgrade pip
"%VENV_DIR%\Scripts\python.exe" -m pip install -r "%~dp0Tools\requirements.txt"

echo ==========================================================
echo  [SETUP COMPLETED] Moi truong Python tren Windows da san sang!
echo  Ban co the chay build_firmware.bat hoac run_pipeline.bat.
echo ==========================================================
