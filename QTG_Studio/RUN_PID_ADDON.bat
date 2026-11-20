@echo off
cd /d "%~dp0"
python pid_launcher.py
if errorlevel 1 pause
