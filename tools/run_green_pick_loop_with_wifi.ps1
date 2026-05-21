param(
    [int]$Attempts = 20,
    [double]$IntervalSeconds = 5.0,
    [string]$TargetClass = 'green'
)

$ErrorActionPreference = 'Stop'

$Workspace = Split-Path -Parent $PSScriptRoot
$LogDir = Join-Path $Workspace 'logs'
$OutLog = Join-Path $LogDir 'run_green_pick_loop.out.log'
$ErrLog = Join-Path $LogDir 'run_green_pick_loop.err.log'
$RobotSsid = 'HW-9E5ACFD8'
$RestoreSsid = 'TJ-WIFI'
$Interface = 'WLAN'

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

function Connect-WifiProfile {
    param([string]$Ssid)
    for ($attempt = 1; $attempt -le 3; $attempt++) {
        Write-Host "Connecting WiFi profile: $Ssid (attempt $attempt)"
        netsh wlan disconnect interface="$Interface" | Out-Host
        Start-Sleep -Seconds 2
        netsh wlan connect name="$Ssid" ssid="$Ssid" interface="$Interface" | Out-Host
        Start-Sleep -Seconds 8
        $state = netsh wlan show interfaces
        $state | Out-Host
        $ssidPattern = 'SSID\s+:\s+{0}(\s|$)' -f [regex]::Escape($Ssid)
        if ($state -match $ssidPattern) {
            return
        }
    }
    throw "Failed to connect WiFi profile: $Ssid"
}

function Wait-RobotPing {
    $deadline = (Get-Date).AddSeconds(45)
    while ((Get-Date) -lt $deadline) {
        if (Test-Connection -ComputerName 192.168.149.1 -Count 1 -Quiet -ErrorAction SilentlyContinue) {
            Write-Host 'Robot ping ok'
            return
        }
        Start-Sleep -Seconds 2
    }
    throw 'Robot ping timed out'
}

try {
    Set-Location $Workspace
    Connect-WifiProfile $RobotSsid
    Wait-RobotPing

    Write-Host "Running $Attempts repeated $TargetClass pick attempts..."
    $proc = Start-Process -FilePath python -ArgumentList @(
        'tools/run_green_pick_loop.py',
        '--target-class', $TargetClass,
        '--attempts', "$Attempts",
        '--interval', "$IntervalSeconds"
    ) `
        -WorkingDirectory $Workspace `
        -RedirectStandardOutput $OutLog `
        -RedirectStandardError $ErrLog `
        -PassThru `
        -NoNewWindow `
        -Wait

    Write-Host "Loop exit code: $($proc.ExitCode)"
    if (Test-Path $OutLog) { Get-Content $OutLog | Out-Host }
    if (Test-Path $ErrLog) { Get-Content $ErrLog | Out-Host }
    if ($proc.ExitCode -ne 0) {
        throw "green pick loop failed with exit code $($proc.ExitCode)"
    }
}
finally {
    Write-Host 'Restoring campus WiFi...'
    try { Connect-WifiProfile $RestoreSsid }
    catch { Write-Warning "Failed to restore $RestoreSsid automatically: $($_.Exception.Message)" }
}
