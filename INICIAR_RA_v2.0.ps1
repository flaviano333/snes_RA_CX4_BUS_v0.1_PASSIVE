$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$port = "COM7"
$roms = @(Get-ChildItem -File | Where-Object { $_.Extension -ieq ".sfc" -or $_.Extension -ieq ".smc" })

if ($roms.Count -eq 0) {
    Write-Host "ERRO: coloque a ROM suportada pelo RetroAchievements nesta pasta." -ForegroundColor Red
    Read-Host "Enter para sair"
    exit 1
}
if ($roms.Count -gt 1) {
    Write-Host "ERRO: deixe somente UMA ROM .sfc/.smc nesta pasta." -ForegroundColor Red
    Read-Host "Enter para sair"
    exit 1
}

Write-Host "SNES RA v2.0 WRAMSEL + READ-REPAIR"
Write-Host "Firmware v2.0 obrigatorio: GP39=/WRAMSEL + READ-REPAIR"
Write-Host ("Porta: " + $port)
Write-Host ("ROM RA: " + $roms[0].Name)
Write-Host "HYBRID-POLL continua ativo; firmware agora repara bytes WRAM KNOWN usando leituras fisicas qualificadas."
Write-Host ""

py -3 .\ra_usb2snes_bridge.py --port $port --rom $roms[0].FullName --poll-gap-ms 8 --max-snapshot-age-ms 50
