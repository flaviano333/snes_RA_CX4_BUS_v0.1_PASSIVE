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
if (Test-Path build_cx4_v038) { Remove-Item -Recurse -Force build_cx4_v038 }
cmake -S . -B build_cx4_v038 -G Ninja
cmake --build build_cx4_v038

$generic = Join-Path $root "build_cx4_v038\snes_rp2350b_capture.uf2"
$out = Join-Path $root "build_cx4_v038\snes_rp2350b_capture_CX4_WITH_ROM.uf2"
py -3 tools/inject_rom_into_uf2.py "$generic" "$RomPath" -o "$out"
Write-Host ""
Write-Host "Pronto: $out"
Write-Host "A ROM foi injetada apenas no UF2 local e nao foi adicionada ao repositorio."
