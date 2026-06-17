# 机器人端架构对齐与 L 型拼接使用报告

更新时间：2026-06-17

## 1. 当前策略

我们现在由本地电脑主导任务，但不拆队友已经部署的机器人端框架。

机器人端继续保留 `competition_pick_place` 的完整流程：

```text
NAV_TO_MATERIAL
SEARCH_TARGET
PICK
ALIGN_PLACE
PLACE
L_SHAPE_PUSH
ADVANCE_AFTER_PICK
NAV_TO_FEED
RELEASE
NAV_HOME
DONE
```

本地只负责通过 SSH 进入机器人 Docker 容器，启动同一个 ROS2 launch，并把我们已经调好的抓取、放置和 L 型拼接参数传进去。

## 2. 本地新增主控入口

新增文件：

- `tools/local_full_task_agent.py`
- `tools/local_full_task_agent_with_wifi.ps1`

推荐使用 PowerShell 包装器，因为它会自动切换到机器人 WiFi，结束后恢复校园网：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\local_full_task_agent_with_wifi.ps1 -Action run
```

只打印将要发给机器人的完整命令，不连接机器人：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\local_full_task_agent_with_wifi.ps1 -Action command
```

查看机器人端任务和 YOLO 状态：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\local_full_task_agent_with_wifi.ps1 -Action status
```

停止本任务节点和 YOLO，不停止相机和底盘：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\local_full_task_agent_with_wifi.ps1 -Action stop
```

## 3. 默认任务参数

当前默认任务是：

- 抓取目标：`gray`
- 放置目标：命令里可以写 `glass`，本地 Agent 会转成 YOLO 类别 `blue`
- YOLO 模型：`tongji`
- YOLO 类别：`gray,yellow,grass,blue`
- 不启动 base：`start_base=false`
- 不启动 camera：`start_camera=false`
- 启动 YOLO：`start_yolo=true`
- 使用 odom 导航：`nav_mode=odom`
- 关闭夹爪成功检测：`grasp_check_enabled=false`

不启动 base/camera 是为了避免重复打开 Aurora 相机。队友的总流程里如果已经启动底盘和相机，我们这里只启动任务节点和 YOLO。

## 4. 抓取与放置标定

当前默认值已经对齐前面实机成功的抓取逻辑：

```text
pick_target_robot_x_m=0.1422
pick_target_robot_y_m=-0.01
pick_robot_x_tolerance_m=0.005
pick_robot_y_tolerance_m=0.002

place_target_robot_x_m=0.175
place_target_robot_y_m=0.01
place_robot_x_tolerance_m=0.005
place_robot_y_tolerance_m=0.002

max_linear_speed=0.08
max_angular_speed=0.25
visual_servo_command_seconds=0.06
```

调抓取位置时，主要改：

```powershell
-PickTargetRobotX 0.1422
-PickTargetRobotY -0.01
```

调放置位置时，主要改：

```powershell
-PlaceTargetRobotX 0.175
-PlaceTargetRobotY 0.01
```

## 5. L 型拼接参数

L 型拼接在 `PLACE` 后执行。当前推荐流程是：红石方块放到玻璃方块上后先松爪，然后把末端调整为接近水平，再向前推一小段距离：

```text
l_shape_push_enabled=true
l_shape_push_pose=518,196,176,597,500,335
l_shape_push_pose_action=horizontal
l_shape_push_pose_step=1
l_shape_push_wrist_servo_index=4
l_shape_push_wrist_position=108
l_shape_push_gripper_position=-1
l_shape_push_distance_m=0.05
l_shape_push_speed_mps=0.04
l_shape_push_max_seconds=2.0
l_shape_push_release_before=true
l_shape_push_close_after=true
l_shape_push_lift_steps=5,6
```

当前默认已经固化为现场确认正确的完整铲车姿态：

```powershell
-LShapePushPose '518,196,176,597,500,335'
-LShapePushGripperPosition -1
```

如果只想做小范围腕关节微调，优先调：

```powershell
-LShapePushWristPosition 108
```

推块距离优先调：

```powershell
-LShapePushDistance 0.05
```

推之前是否先松手：

```powershell
-LShapePushReleaseBefore true
-LShapePushReleaseBefore false
```

推完是否重新夹住：

```powershell
-LShapePushCloseAfter true
-LShapePushCloseAfter false
```

## 6. 完整运行示例

当前推荐命令：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\local_full_task_agent_with_wifi.ps1 `
  -Action run `
  -TargetClass gray `
  -PlaceClass glass `
  -YoloModel tongji `
  -YoloClasses gray,yellow,grass,blue `
  -YoloConf 0.20 `
  -PickTargetRobotX 0.1422 `
  -PickTargetRobotY -0.01 `
  -PickRobotXTolerance 0.005 `
  -PickRobotYTolerance 0.002 `
  -PlaceTargetRobotX 0.175 `
  -PlaceTargetRobotY 0.01 `
  -PlaceRobotXTolerance 0.005 `
  -PlaceRobotYTolerance 0.002 `
  -LShapePushEnabled true `
  -LShapePushReleaseBefore true `
  -LShapePushPose '518,196,176,597,500,335' `
  -LShapePushGripperPosition -1 `
  -LShapePushDistance 0.05 `
  -LShapePushSpeed 0.04 `
  -LShapePushCloseAfter true
```

如果已经手动切到了机器人 WiFi，也可以直接跑 Python：

```powershell
python tools\local_full_task_agent.py run `
  --target-class gray `
  --place-class glass `
  --pick-target-robot-x 0.1422 `
  --pick-target-robot-y -0.01 `
  --place-target-robot-x 0.175 `
  --place-target-robot-y 0.01 `
  --l-shape-push-enabled=true `
  --l-shape-push-wrist-position 108
```

## 7. 部署方式

机器人端只需要部署 `competition_pick_place` 包：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\deploy_competition_package_with_wifi.ps1
```

部署脚本会：

1. 切到 `HW-9E5ACFD8`
2. 上传本地 `competition_pick_place`
3. 在 `MentorPi` 容器里重新 `colcon build`
4. 恢复 `TJ-WIFI`

## 8. 已完成本地验证

已完成：

- Python 静态编译通过
- PowerShell 主控入口 `-Action command` 可生成完整命令
- 命令中确认包含 `start_base:=false`、`start_camera:=false`
- 命令中确认 `place_class:=blue`
- 命令中确认抓取标定、放置标定、L 型水平推块参数均已透传

实机运行前需要确认：

- 队友的底盘、相机基础流程已经启动，或者现场明确允许手动启动 base/camera
- Aurora 相机没有被重复 launch
- 玻璃方块在 YOLO 中按 `blue` 类别识别
