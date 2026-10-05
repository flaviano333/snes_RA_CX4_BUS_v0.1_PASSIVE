@echo off
setlocal
if "%~1"=="" (
  echo Uso: INJETAR_ROM_NO_UF2.bat firmware_generico.uf2 "Mega Man X2 ^(USA^).sfc"
  exit /b 1
)
if "%~2"=="" (
  echo Uso: INJETAR_ROM_NO_UF2.bat firmware_generico.uf2 "Mega Man X2 ^(USA^).sfc"
  exit /b 1
)
py -3 "%~dp0tools\inject_rom_into_uf2.py" "%~1" "%~2"
