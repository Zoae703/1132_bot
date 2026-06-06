param(
    [Parameter(Mandatory = $true)]
    [string]$ElfFile,

    [string]$InterfaceCfg = 'interface/cmsis-dap.cfg',

    [string]$TargetCfg = 'target/stm32f4x.cfg',

    [switch]$ProbeOnly
)

$candidatePaths = @()

if ($env:OPENOCD) {
    $candidatePaths += $env:OPENOCD
}

$candidatePaths += @(
    'C:\OpenOCD\bin\openocd.exe',
    'C:\openocd\bin\openocd.exe',
    'C:\Program Files\OpenOCD\bin\openocd.exe',
    'C:\Program Files (x86)\OpenOCD\bin\openocd.exe',
    'C:\ST\OpenOCD\bin\openocd.exe',
    'C:\ST\STM32CubeCLT_1.20.0\OpenOCD\bin\openocd.exe'
)

$openocd = $candidatePaths | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1

if (-not $openocd) {
    $openocd = Get-ChildItem -Path 'C:\' -Filter openocd.exe -Recurse -ErrorAction SilentlyContinue |
        Select-Object -First 1 -ExpandProperty FullName
}

if (-not $openocd) {
    Write-Host 'openocd.exe not found. Make sure it is installed on C drive or added to PATH / OPENOCD.'
    exit 1
}

Write-Host "openocd: $openocd"

if ($ProbeOnly) {
    exit 0
}

& $openocd -f $InterfaceCfg -f $TargetCfg -c "program $ElfFile verify reset exit"
exit $LASTEXITCODE