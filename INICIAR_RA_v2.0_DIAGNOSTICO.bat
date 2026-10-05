@echo off
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0INICIAR_RA_v2.0_DIAGNOSTICO.ps1"
pause
