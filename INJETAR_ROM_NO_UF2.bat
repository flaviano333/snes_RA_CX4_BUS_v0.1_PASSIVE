@echo off
setlocal
if "%~2"=="" (
  echo Uso: INJETAR_ROM_NO_UF2.bat "CX4_LAST_TRY_BOOTBACK_HLE_GENERIC_SEM_ROM.uf2" "C:\ROMs\Mega Man X2.sfc"
  exit /b 1
)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0INJETAR_ROM_NO_UF2.ps1" -Uf2Path "%~1" -RomPath "%~2"
