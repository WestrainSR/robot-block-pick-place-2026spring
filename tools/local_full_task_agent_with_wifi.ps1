param(
    [ValidateSet('run', 'stop', 'status', 'command')]
    [string]$Action = 'run',
    [string]$TargetClass = 'gray',
    [string]$TargetSequence = '',
    [string]$PlaceClass = 'glass',
    [string]$YoloModel = 'tongji',
    [string]$YoloClasses = 'gray,yellow,grass,blue',
    [double]$YoloConf = 0.20,
    [string]$StartNavigation = 'false',
    [string]$StartBase = 'false',
    [string]$StartCamera = 'false',
    [string]$StartYolo = 'true',
    [string]$UseNav = 'true',
    [string]$UseArm = 'true',
    [ValidateSet('odom', 'nav2')]
    [string]$NavMode = 'odom',
    [double]$OdomMaterialX = 1.03,
    [double]$OdomMaterialY = -1.03,
    [double]$OdomMaterialYaw = -0.7853981633974483,
    [double]$OdomFeedX = 0.15,
    [double]$OdomFeedY = -1.07,
    [double]$OdomFeedYaw = 3.141592653589793,
    [double]$OdomReturnX = 1.03,
    [double]$OdomReturnY = -1.03,
    [double]$OdomReturnYaw = -0.7853981633974483,
    [double]$PickTargetRobotX = 0.1422,
    [double]$PickTargetRobotY = -0.01,
    [double]$PickRobotXTolerance = 0.005,
    [double]$PickRobotYTolerance = 0.002,
    [double]$PlaceTargetRobotX = 0.175,
    [double]$PlaceTargetRobotY = 0.01,
    [double]$PlaceRobotXTolerance = 0.005,
    [double]$PlaceRobotYTolerance = 0.002,
    [string]$GraspCheckEnabled = 'false',
    [double]$MaxLinearSpeed = 0.08,
    [double]$MaxAngularSpeed = 0.25,
    [double]$CmdPulse = 0.06,
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
    [string]$LShapePushLiftAction = '',
    [string[]]$LShapePushLiftSteps = @('5', '6'),
    [double]$Timeout = 0,
    [string]$RobotSsid = 'HW-9E5ACFD8',
    [string]$RestoreSsid = 'TJ-WIFI',
    [string]$Interface = 'WLAN',
    [switch]$NoRestore,
    [string[]]$ExtraArgs = @()
)

$ErrorActionPreference = 'Stop'

$Workspace = Split-Path -Parent $PSScriptRoot
$LogDir = Join-Path $Workspace 'logs'
$OutLog = Join-Path $LogDir 'local_full_task_agent.out.log'
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

function Add-BoolFlag {
    param(
        [System.Collections.ArrayList]$Args,
        [string]$Name,
        [bool]$Value
    )
    if ($Value) {
        [void]$Args.Add("--$Name")
    }
    else {
        [void]$Args.Add("--no-$Name")
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

$startNavigationValue = Convert-ToOptionalBool -Value $StartNavigation -Default $false -Name 'StartNavigation'
$startBaseValue = Convert-ToOptionalBool -Value $StartBase -Default $false -Name 'StartBase'
$startCameraValue = Convert-ToOptionalBool -Value $StartCamera -Default $false -Name 'StartCamera'
$startYoloValue = Convert-ToOptionalBool -Value $StartYolo -Default $true -Name 'StartYolo'
$useNavValue = Convert-ToOptionalBool -Value $UseNav -Default $true -Name 'UseNav'
$useArmValue = Convert-ToOptionalBool -Value $UseArm -Default $true -Name 'UseArm'
$graspCheckEnabledValue = Convert-ToOptionalBool -Value $GraspCheckEnabled -Default $false -Name 'GraspCheckEnabled'
$lShapePushEnabledValue = Convert-ToOptionalBool -Value $LShapePushEnabled -Default $true -Name 'LShapePushEnabled'
$lShapePushReleaseBeforeValue = Convert-ToOptionalBool -Value $LShapePushReleaseBefore -Default $true -Name 'LShapePushReleaseBefore'
$lShapePushCloseAfterValue = Convert-ToOptionalBool -Value $LShapePushCloseAfter -Default $true -Name 'LShapePushCloseAfter'

$agentArgs = [System.Collections.ArrayList]@(
    'tools/local_full_task_agent.py',
    $Action,
    '--timeout', "$Timeout",
    '--target-class', $TargetClass,
    '--place-class', $PlaceClass,
    '--yolo-model', $YoloModel,
    '--yolo-classes', $YoloClasses,
    '--yolo-conf', "$YoloConf",
    '--nav-mode', $NavMode,
    '--odom-material-x', "$OdomMaterialX",
    '--odom-material-y', "$OdomMaterialY",
    '--odom-material-yaw', "$OdomMaterialYaw",
    '--odom-feed-x', "$OdomFeedX",
    '--odom-feed-y', "$OdomFeedY",
    '--odom-feed-yaw', "$OdomFeedYaw",
    '--odom-return-x', "$OdomReturnX",
    '--odom-return-y', "$OdomReturnY",
    '--odom-return-yaw', "$OdomReturnYaw",
    '--pick-target-robot-x', "$PickTargetRobotX",
    '--pick-target-robot-y', "$PickTargetRobotY",
    '--pick-robot-x-tolerance', "$PickRobotXTolerance",
    '--pick-robot-y-tolerance', "$PickRobotYTolerance",
    '--place-target-robot-x', "$PlaceTargetRobotX",
    '--place-target-robot-y', "$PlaceTargetRobotY",
    '--place-robot-x-tolerance', "$PlaceRobotXTolerance",
    '--place-robot-y-tolerance', "$PlaceRobotYTolerance",
    '--max-linear-speed', "$MaxLinearSpeed",
    '--max-angular-speed', "$MaxAngularSpeed",
    '--visual-servo-command-seconds', "$CmdPulse",
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
    '--l-shape-push-close-position', "$LShapePushClosePosition",
    '--l-shape-push-close-duration', "$LShapePushCloseDuration"
)

if ($TargetSequence -ne '') {
    [void]$agentArgs.Add('--target-sequence')
    [void]$agentArgs.Add($TargetSequence)
}
if ($LShapePushPose -ne '') {
    [void]$agentArgs.Add('--l-shape-push-pose')
    [void]$agentArgs.Add($LShapePushPose)
}
if ($LShapePushLiftAction -ne '') {
    [void]$agentArgs.Add('--l-shape-push-lift-action')
    [void]$agentArgs.Add($LShapePushLiftAction)
}
$liftStepsArg = ($LShapePushLiftSteps -join ',').Trim()
if ($liftStepsArg -ne '') {
    [void]$agentArgs.Add('--l-shape-push-lift-steps')
    [void]$agentArgs.Add($liftStepsArg)
}

Add-BoolFlag -Args $agentArgs -Name 'start-navigation' -Value $startNavigationValue
Add-BoolFlag -Args $agentArgs -Name 'start-base' -Value $startBaseValue
Add-BoolFlag -Args $agentArgs -Name 'start-camera' -Value $startCameraValue
Add-BoolFlag -Args $agentArgs -Name 'start-yolo' -Value $startYoloValue
Add-BoolFlag -Args $agentArgs -Name 'use-nav' -Value $useNavValue
Add-BoolFlag -Args $agentArgs -Name 'use-arm' -Value $useArmValue
Add-BoolFlag -Args $agentArgs -Name 'grasp-check-enabled' -Value $graspCheckEnabledValue
Add-BoolFlag -Args $agentArgs -Name 'l-shape-push-enabled' -Value $lShapePushEnabledValue
Add-BoolFlag -Args $agentArgs -Name 'l-shape-push-release-before' -Value $lShapePushReleaseBeforeValue
Add-BoolFlag -Args $agentArgs -Name 'l-shape-push-close-after' -Value $lShapePushCloseAfterValue

foreach ($extra in $ExtraArgs) {
    [void]$agentArgs.Add($extra)
}

try {
    Set-Location $Workspace
    if ($Action -ne 'command') {
        Connect-WifiProfile $RobotSsid
        Wait-RobotPing
    }

    Write-Host "Running local full-task agent: action=$Action"
    $oldErrorActionPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & python @agentArgs 2>&1 | Tee-Object -FilePath $OutLog
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $oldErrorActionPreference
    }
    Write-Host "Local full-task agent exit code: $exitCode"
    if ($exitCode -ne 0) {
        throw "local full-task agent failed with exit code $exitCode"
    }
}
finally {
    if ($Action -ne 'command' -and -not $NoRestore) {
        Write-Host 'Restoring campus WiFi...'
        try { Connect-WifiProfile $RestoreSsid }
        catch { Write-Warning "Failed to restore $RestoreSsid automatically: $($_.Exception.Message)" }
    }
}
