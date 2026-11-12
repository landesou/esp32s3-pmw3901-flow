@echo off
REM PMW3901 optical-flow host launcher.
REM NOTE: keep this file ASCII-only. cmd.exe parses .bat in the OEM codepage,
REM and UTF-8 Chinese bytes can be mis-read as command separators.
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" goto run

echo [first run] creating venv and installing dependencies, please wait...
python -m venv .venv
if errorlevel 1 goto err
".venv\Scripts\python.exe" -m pip install -U pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto err

:run
".venv\Scripts\python.exe" flow_host.py %*
exit /b %errorlevel%

:err
echo.
echo Install failed. Check network, or run manually:
echo   python -m venv .venv
echo   .venv\Scripts\python.exe -m pip install -r requirements.txt
pause
exit /b 1
