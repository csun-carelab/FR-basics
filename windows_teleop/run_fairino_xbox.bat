@echo off
echo Starting Fairino Xbox Controller...
echo Robot IP: 192.168.58.2
echo.
py "%~dp0fairino_xbox_windows.py"
echo.
echo ============================================
echo Script exited. Press any key to close...
pause >nul
