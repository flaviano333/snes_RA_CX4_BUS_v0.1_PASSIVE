param(
    [Parameter(Mandatory=$true)][string]$Uf2Path,
    [Parameter(Mandatory=$true)][string]$RomPath,
    [string]$OutputPath = ""
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root
if (-not (Test-Path $Uf2Path)) { throw "UF2 nao encontrado: $Uf2Path" }
if (-not (Test-Path $RomPath)) { throw "ROM nao encontrada: $RomPath" }
if ($OutputPath) {
    py -3 tools/inject_rom_into_uf2.py "$Uf2Path" "$RomPath" -o "$OutputPath"
} else {
    py -3 tools/inject_rom_into_uf2.py "$Uf2Path" "$RomPath"
}
