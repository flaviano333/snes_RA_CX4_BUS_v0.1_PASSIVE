@echo off
setlocal
if "%~2"=="" (
  echo Uso: INSTALAR_ROM_UMA_VEZ.bat "CX4_SYSCLK_RESET_HYBRID_V6_GENERIC.uf2" "C:\ROMs\Mega Man X2.sfc"
  exit /b 1
)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0INSTALAR_ROM_UMA_VEZ.ps1" -Uf2Generico "%~1" -RomPath "%~2"
