$ErrorActionPreference = 'Stop'

$Workspace = Split-Path -Parent $PSScriptRoot
$LogDir = Join-Path $Workspace 'logs'
$OutLog = Join-Path $LogDir 'run_green_pick.out.log'
$ErrLog = Join-Path $LogDir 'run_green_pick.err.log'
$RobotSsid = 'HW-9E5ACFD8'
$RestoreSsid = 'TJ-WIFI'
$Interface = 'WLAN'

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

function Connect-WifiProfile {
    param([string]$Ssid)
    Write-Host "Connecting WiFi profile: $Ssid"
    netsh wlan connect name="$Ssid" ssid="$Ssid" interface="$Interface" | Out-Host
    Start-Sleep -Seconds 8
    netsh wlan show interfaces | Out-Host
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

    Write-Host 'Running real green-block pick...'
    $proc = Start-Process -FilePath python -ArgumentList @('tools/run_green_pick.py') `
        -WorkingDirectory $Workspace `
        -RedirectStandardOutput $OutLog `
        -RedirectStandardError $ErrLog `
        -PassThru `
        -NoNewWindow `
        -Wait

    Write-Host "Pick exit code: $($proc.ExitCode)"
    if (Test-Path $OutLog) { Get-Content $OutLog | Out-Host }
    if (Test-Path $ErrLog) { Get-Content $ErrLog | Out-Host }
    if ($proc.ExitCode -ne 0) {
        throw "green pick failed with exit code $($proc.ExitCode)"
    }
}
finally {
    Write-Host 'Restoring campus WiFi...'
    try { Connect-WifiProfile $RestoreSsid }
    catch { Write-Warning "Failed to restore $RestoreSsid automatically: $($_.Exception.Message)" }
}
