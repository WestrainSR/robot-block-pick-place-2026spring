param(
    [string]$TargetClass = 'gray',
    [string]$PlaceClass = '',
    [string]$YoloModel = 'tongji',
    [string]$YoloClasses = 'gray,yellow,grass,blue',
    [double]$YoloConf = 0.20,
    [double]$TargetRobotX = 0.1452,
    [double]$TargetRobotY = -0.0093,
    [double]$RobotXTolerance = 0.010,
    [double]$RobotYTolerance = 0.010,
    [double]$PlaceTargetRobotX = 0.1452,
    [double]$PlaceTargetRobotY = 0.0,
    [double]$PlaceRobotXTolerance = 0.015,
    [double]$PlaceRobotYTolerance = 0.015,
    [string]$PlaceSteps = '',
    [bool]$HoldAfterPlace = $true,
    [string]$HoldPlaceSteps = '1,2',
    [Nullable[bool]]$GraspCheckEnabled = $null,
    [bool]$OpenGripperBeforeApproach = $true,
    [int]$GripperOpenPosition = 200,
    [double]$GripperOpenDuration = 0.30,
    [double]$MaxLinearSpeed = 0.08,
    [double]$MaxAngularSpeed = 0.25,
    [double]$CmdPulse = 0.06,
    [int]$Timeout = 90
)

$ErrorActionPreference = 'Stop'

$Workspace = Split-Path -Parent $PSScriptRoot
$LogDir = Join-Path $Workspace 'logs'
$OutLog = Join-Path $LogDir 'headless_grasp_trial.out.log'
$ErrLog = Join-Path $LogDir 'headless_grasp_trial.err.log'
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

    Write-Host "Running headless grasp trial for $TargetClass..."
    $pythonArgs = @(
        'tools/headless_grasp_trial.py',
        '--target-class', $TargetClass,
        '--place-class', $PlaceClass,
        '--yolo-model', $YoloModel,
        '--yolo-classes', $YoloClasses,
        '--yolo-conf', "$YoloConf",
        '--target-robot-x', "$TargetRobotX",
        '--target-robot-y', "$TargetRobotY",
        '--robot-x-tolerance', "$RobotXTolerance",
        '--robot-y-tolerance', "$RobotYTolerance",
        '--place-target-robot-x', "$PlaceTargetRobotX",
        '--place-target-robot-y', "$PlaceTargetRobotY",
        '--place-robot-x-tolerance', "$PlaceRobotXTolerance",
        '--place-robot-y-tolerance', "$PlaceRobotYTolerance",
        '--place-steps', $PlaceSteps,
        $(if ($HoldAfterPlace) { '--hold-after-place' } else { '--no-hold-after-place' }),
        '--hold-place-steps', $HoldPlaceSteps,
        $(if ($null -ne $GraspCheckEnabled) { if ($GraspCheckEnabled) { '--grasp-check-enabled' } else { '--no-grasp-check-enabled' } }),
        $(if ($OpenGripperBeforeApproach) { '--open-gripper-before-approach' } else { '--no-open-gripper-before-approach' }),
        '--gripper-open-position', "$GripperOpenPosition",
        '--gripper-open-duration', "$GripperOpenDuration",
        '--max-linear-speed', "$MaxLinearSpeed",
        '--max-angular-speed', "$MaxAngularSpeed",
        '--cmd-pulse', "$CmdPulse",
        '--timeout', "$Timeout"
    )

    Push-Location $Workspace
    try {
        $output = & python @pythonArgs 2>&1 | Tee-Object -FilePath $OutLog
        $exitCode = $LASTEXITCODE
    }
    finally {
        Pop-Location
    }
    Set-Content -Path $ErrLog -Value ''

    Write-Host "Headless trial exit code: $exitCode"
    if ($exitCode -ne 0) {
        throw "headless grasp trial failed with exit code $exitCode"
    }
}
finally {
    Write-Host 'Restoring campus WiFi...'
    try { Connect-WifiProfile $RestoreSsid }
    catch { Write-Warning "Failed to restore $RestoreSsid automatically: $($_.Exception.Message)" }
}
