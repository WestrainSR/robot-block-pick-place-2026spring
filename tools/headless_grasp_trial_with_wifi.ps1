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
    [string]$PlaceBlindForwardEnabled = 'true',
    [double]$PlaceBlindForwardDistance = 0.02,
    [double]$PlaceBlindForwardSpeed = 0.04,
    [double]$PlaceBlindForwardMaxSeconds = 1.0,
    [string]$PlaceSteps = '',
    [string]$HoldAfterPlace = 'true',
    [string[]]$HoldPlaceSteps = @(),
    [string]$PlaceUsePickGeometry = 'false',
    [string[]]$PlacePickSteps = @('1', '2'),
    [string]$LShapePushEnabled = 'true',
    [string]$LShapePushPose = '518,196,176,597,500,335',
    [string]$LShapePushPoseAction = 'horizontal',
    [int]$LShapePushPoseStep = 1,
    [double]$LShapePushPoseDuration = 1.0,
    [string]$LShapePushServoOrder = '5,4,3,2,1',
    [int]$LShapePushWristServoIndex = 4,
    [int]$LShapePushWristPosition = 108,
    [int]$LShapePushGripperPosition = -1,
    [double]$LShapePushDistance = 0.05,
    [double]$LShapePushSpeed = 0.04,
    [double]$LShapePushMaxSeconds = 2.0,
    [string]$LShapePushReleaseBefore = 'true',
    [string]$LShapePushCloseAfter = 'true',
    [int]$LShapePushClosePosition = 500,
    [double]$LShapePushCloseDuration = 0.35,
    [string]$LShapePushLiftAction = 'navigation_pick_ai',
    [string[]]$LShapePushLiftSteps = @('5', '6'),
    [string]$GraspCheckEnabled = '',
    [string]$OpenGripperBeforeApproach = 'true',
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

function Convert-ToOptionalBool {
    param(
        [AllowNull()][object]$Value,
        [AllowNull()][object]$Default,
        [string]$Name
    )
    if ($null -eq $Value) {
        return $Default
    }
    if ($Value -is [bool]) {
        return [bool]$Value
    }
    $text = ([string]$Value).Trim().TrimEnd('`')
    if ($text -eq '') {
        return $Default
    }
    switch -Regex ($text.ToLowerInvariant()) {
        '^(true|1|yes|y|on|\$true)$' { return $true }
        '^(false|0|no|n|off|\$false)$' { return $false }
        default {
            throw "Invalid boolean value for ${Name}: ${Value}. Use true/false, 1/0, or omit the parameter."
        }
    }
}

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
    $holdAfterPlaceValue = Convert-ToOptionalBool -Value $HoldAfterPlace -Default $true -Name 'HoldAfterPlace'
    $placeUsePickGeometryValue = Convert-ToOptionalBool -Value $PlaceUsePickGeometry -Default $false -Name 'PlaceUsePickGeometry'
    $placeBlindForwardEnabledValue = Convert-ToOptionalBool -Value $PlaceBlindForwardEnabled -Default $true -Name 'PlaceBlindForwardEnabled'
    $lShapePushEnabledValue = Convert-ToOptionalBool -Value $LShapePushEnabled -Default $false -Name 'LShapePushEnabled'
    $lShapePushReleaseBeforeValue = Convert-ToOptionalBool -Value $LShapePushReleaseBefore -Default $true -Name 'LShapePushReleaseBefore'
    $lShapePushCloseAfterValue = Convert-ToOptionalBool -Value $LShapePushCloseAfter -Default $true -Name 'LShapePushCloseAfter'
    $openGripperBeforeApproachValue = Convert-ToOptionalBool -Value $OpenGripperBeforeApproach -Default $true -Name 'OpenGripperBeforeApproach'
    $graspCheckEnabledValue = Convert-ToOptionalBool -Value $GraspCheckEnabled -Default $null -Name 'GraspCheckEnabled'
    Connect-WifiProfile $RobotSsid
    Wait-RobotPing

    Write-Host "Running headless grasp trial for $TargetClass..."
    Write-Host ("L-shape push: enabled={0}, pose_action={1}, pose_step={2}, servo_order={3}, wrist{4}={5}, gripper={6}, distance={7}m, speed={8}m/s, max={9}s, close_after={10}" -f `
        $lShapePushEnabledValue, $LShapePushPoseAction, $LShapePushPoseStep, $LShapePushServoOrder, $LShapePushWristServoIndex, $LShapePushWristPosition, $LShapePushGripperPosition, `
        $LShapePushDistance, $LShapePushSpeed, $LShapePushMaxSeconds, $lShapePushCloseAfterValue)
    $holdPlaceStepsArg = ($HoldPlaceSteps -join ',').Trim()
    $placePickStepsArg = ($PlacePickSteps -join ',').Trim()
    $lShapePushLiftStepsArg = ($LShapePushLiftSteps -join ',').Trim()
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
        $(if ($placeBlindForwardEnabledValue) { '--place-blind-forward-enabled' } else { '--no-place-blind-forward-enabled' }),
        '--place-blind-forward-distance', "$PlaceBlindForwardDistance",
        '--place-blind-forward-speed', "$PlaceBlindForwardSpeed",
        '--place-blind-forward-max-seconds', "$PlaceBlindForwardMaxSeconds",
        $(if ($holdAfterPlaceValue) { '--hold-after-place' } else { '--no-hold-after-place' }),
        $(if ($placeUsePickGeometryValue) { '--place-use-pick-geometry' } else { '--no-place-use-pick-geometry' }),
        $(if ($lShapePushEnabledValue) { '--l-shape-push-enabled' } else { '--no-l-shape-push-enabled' }),
        '--l-shape-push-pose-action', $LShapePushPoseAction,
        '--l-shape-push-pose-step', "$LShapePushPoseStep",
        '--l-shape-push-pose-duration', "$LShapePushPoseDuration",
        '--l-shape-push-servo-order', $LShapePushServoOrder,
        '--l-shape-push-wrist-servo-index', "$LShapePushWristServoIndex",
        '--l-shape-push-wrist-position', "$LShapePushWristPosition",
        '--l-shape-push-gripper-position', "$LShapePushGripperPosition",
        '--l-shape-push-distance', "$LShapePushDistance",
        '--l-shape-push-speed', "$LShapePushSpeed",
        '--l-shape-push-max-seconds', "$LShapePushMaxSeconds",
        $(if ($lShapePushReleaseBeforeValue) { '--l-shape-push-release-before' } else { '--no-l-shape-push-release-before' }),
        $(if ($lShapePushCloseAfterValue) { '--l-shape-push-close-after' } else { '--no-l-shape-push-close-after' }),
        '--l-shape-push-close-position', "$LShapePushClosePosition",
        '--l-shape-push-close-duration', "$LShapePushCloseDuration",
        '--l-shape-push-lift-action', $LShapePushLiftAction,
        $(if ($null -ne $graspCheckEnabledValue) { if ($graspCheckEnabledValue) { '--grasp-check-enabled' } else { '--no-grasp-check-enabled' } }),
        $(if ($openGripperBeforeApproachValue) { '--open-gripper-before-approach' } else { '--no-open-gripper-before-approach' }),
        '--gripper-open-position', "$GripperOpenPosition",
        '--gripper-open-duration', "$GripperOpenDuration",
        '--max-linear-speed', "$MaxLinearSpeed",
        '--max-angular-speed', "$MaxAngularSpeed",
        '--cmd-pulse', "$CmdPulse",
        '--timeout', "$Timeout"
    )
    if ($PlaceSteps -ne '') {
        $pythonArgs += @('--place-steps', $PlaceSteps)
    }
    if ($holdPlaceStepsArg -ne '') {
        $pythonArgs += @('--hold-place-steps', $holdPlaceStepsArg)
    }
    if ($placePickStepsArg -ne '') {
        $pythonArgs += @('--place-pick-steps', $placePickStepsArg)
    }
    if ($LShapePushPose -ne '') {
        $pythonArgs += @('--l-shape-push-pose', $LShapePushPose)
    }
    if ($lShapePushLiftStepsArg -ne '') {
        $pythonArgs += @('--l-shape-push-lift-steps', $lShapePushLiftStepsArg)
    }

    Push-Location $Workspace
    try {
        & python @pythonArgs 2>&1 | Tee-Object -FilePath $OutLog
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
