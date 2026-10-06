param(
    [Parameter(Mandatory=$true)][string]$Uf2Generico,
    [Parameter(Mandatory=$true)][string]$RomPath,
    [string]$OutputPath = ".\MMX2_ROM_SLOT_UMA_VEZ.uf2"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root
if (-not (Test-Path $Uf2Generico)) { throw "UF2 generico nao encontrado: $Uf2Generico" }
if (-not (Test-Path $RomPath)) { throw "ROM nao encontrada: $RomPath" }
py -3 tools/install_rom_slot.py "$Uf2Generico" "$RomPath" -o "$OutputPath"
