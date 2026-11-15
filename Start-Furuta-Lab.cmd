@echo off
setlocal
set "FURUTA_TASK_ROOT=%~dp0"
if not exist "%FURUTA_TASK_ROOT%.venv\Scripts\pythonw.exe" (
    echo Run Setup-Furuta-Lab.ps1 once to install the local environment.
    pause
    exit /b 1
)
start "" "%FURUTA_TASK_ROOT%.venv\Scripts\pythonw.exe" "%FURUTA_TASK_ROOT%desktop\main.py"
