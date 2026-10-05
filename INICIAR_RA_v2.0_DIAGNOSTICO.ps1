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

Write-Host "SNES RA v2.0 WRAMSEL + READ-REPAIR - CHEESE PROBE"
Write-Host "Mostra poll/snapshot, READ-REPAIR e probe dedicado de $1558/$155C."
Write-Host ("Porta: " + $port)
Write-Host ("ROM RA: " + $roms[0].Name)
Write-Host ""

py -3 .\ra_usb2snes_bridge.py --port $port --rom $roms[0].FullName --poll-gap-ms 8 --max-snapshot-age-ms 50 --trace-polls --trace-wramsel --trace-cheese --trace-ra --trace-snapshots
