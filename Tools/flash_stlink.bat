@echo off
setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set PROJECT_ROOT=%~dp0..
set PYTHON_EXEC=%PROJECT_ROOT%\venv\Scripts\python.exe

if not exist "%PYTHON_EXEC%" (
    set PYTHON_EXEC=python
)

"%PYTHON_EXEC%" "%SCRIPT_DIR%flash_stlink.py" %*
