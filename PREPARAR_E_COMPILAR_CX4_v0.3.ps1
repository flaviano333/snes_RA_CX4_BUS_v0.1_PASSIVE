param(
    [Parameter(Mandatory=$true)]
    [string]$RomPath
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root
if (-not (Test-Path $RomPath)) { throw "ROM nao encontrada: $RomPath" }
if (-not $env:PICO_SDK_PATH) { throw "PICO_SDK_PATH nao esta definido." }

New-Item -ItemType Directory -Force -Path third_party | Out-Null
Write-Host "Baixando o core CX4 LLE permissivo (ares -> snesrecomp)..."
Invoke-WebRequest -UseBasicParsing `
  "https://raw.githubusercontent.com/RetroPortingToolKit/snesrecomp/main/runner/src/snes/cx4.c" `
  -OutFile "third_party/cx4.c"
Invoke-WebRequest -UseBasicParsing `
  "https://raw.githubusercontent.com/RetroPortingToolKit/snesrecomp/main/runner/src/snes/cx4.h" `
  -OutFile "third_party/cx4.h"

py -3 tools/patch_upstream_cx4.py third_party/cx4.c
py -3 tools/prepare_rom.py "$RomPath" cx4_game_rom.bin

if (Test-Path build_cx4_v03) { Remove-Item -Recurse -Force build_cx4_v03 }
cmake -S . -B build_cx4_v03 -G Ninja
cmake --build build_cx4_v03
Write-Host ""
Write-Host "Pronto: build_cx4_v03\snes_rp2350b_capture.uf2"
Write-Host "A ROM foi incorporada SOMENTE na sua build local; ela nao faz parte deste pacote."
