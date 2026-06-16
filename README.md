# 方块拾取闭环控制项目

这是工程智能课程机器人方块拾取任务的工作仓库。当前仓库已经从早期的红/绿/蓝三类方案，演进为面向 `tongji` OpenVINO 模型的四类目标方案：

```text
gray / yellow / grass / blue
```

其中 `grass` 兼容早期文档中的 `green` 叫法。后续运行、调参和部署应以本 README、`docs/项目交接文档.md`、`docs/当前运行手册.md`、`docs/无实时画面抓取调参流程.md` 为准；早期红绿蓝文档保留为历史记录。

## 当前主线

核心实现位于：

```text
competition_pick_place/
  competition_pick_place/competition_node.py
  launch/competition_run.launch.py
  config/classes.yaml
  config/competition_waypoints.yaml
```

当前方案：

- YOLO 负责识别目标类别和二维检测框。
- 深度图负责估计目标距离；深度不可用时回退到检测框面积。
- 底盘通过 P/MPC 视觉伺服做中心和距离闭环。
- 机械臂复用原厂动作组 `navigation_pick_init_ai` / `navigation_pick_ai` / `navigation_place`。
- 夹爪闭合后读取 `/controller_manager/servo_states`，通过舵机位置判断是否夹到；失败会重新对准并重试。

## 推荐调试入口

目前最稳的调参方式不是实时 UI，而是 headless 单次试验：机器人执行一次抓取，同时保存关键帧、YOLO 框、日志和检测 CSV，结束后自动拉回本地。

详细步骤见：

```text
docs/当前运行手册.md
```

```powershell
python tools\headless_grasp_trial.py --target-class grass
```

常用目标：

```powershell
python tools\headless_grasp_trial.py --target-class gray
python tools\headless_grasp_trial.py --target-class yellow
python tools\headless_grasp_trial.py --target-class grass
python tools\headless_grasp_trial.py --target-class blue
```

结果包保存到：

```text
runs/grasp_headless/*.tar.gz
```

重点看：

```text
frame_*.jpg       带 YOLO 框的关键帧
pick.log          完整抓取日志
pick_tail.log     末尾日志
detections.csv    每帧检测类别、置信度、中心和面积
session.log       相机、检测、最终状态
```

## 本地 UI

如果需要可视化控制面板：

```powershell
python tools\robot_yolo_control_ui.py --port 8090
```

浏览器打开：

```text
http://127.0.0.1:8090
```

UI 提供：

- 启动/停止视觉；
- 启动/停止抓取；
- 查看机器人端关键进程数量；
- 显示 YOLO 叠加画面；
- 调整目标类别、目标深度、面积回退阈值、MPC 周期、速度、夹爪阈值等参数。

## 部署

部署 ROS2 包到机器人：

```powershell
python tools\deploy_competition_package_to_robot.py
```

部署 OpenVINO 模型到机器人：

```powershell
python tools\deploy_openvino_to_robot.py `
  --local-dir model\best_openvino_model `
  --model-name tongji `
  --xml model\best_openvino_model\tongji.xml `
  --bin model\best_openvino_model\tongji.bin
```

如果使用带 WiFi 自动切换的脚本，执行结束后应确认电脑已回到校园网，避免连接机器人热点后失去互联网。

## 直接 ROS2 运行

在机器人 Docker 容器 `MentorPi` 内：

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch competition_pick_place competition_run.launch.py \
  target_class:=grass \
  dry_run:=false \
  stop_after_pick:=true \
  start_navigation:=false \
  start_yolo:=true \
  use_nav:=false \
  use_arm:=true \
  yolo_model:=tongji \
  yolo_classes:=gray,yellow,grass,blue \
  control_mode:=mpc \
  closed_loop_pick:=true
```

完整导航、放置和返航需要先标定 `competition_pick_place/config/competition_waypoints.yaml`。当前该文件仍是占位零坐标，不能直接用于真实导航。

## 目录说明

```text
competition_pick_place/   ROS2 闭环抓取包
tools/                    部署、训练、验证、UI、headless 调参脚本
docs/                     当前说明、历史方案、课程资料和报告
model/                    当前 tongji OpenVINO 模型
deployment/               早期 competition_blocks OpenVINO 模型
datasets/                 采集和训练数据，通常不纳入 Git
runs/                     训练输出和抓取调试结果，通常不纳入 Git
reports/                  模型可视化报告，通常不纳入 Git
```

## 重要注意

- 本仓库包含机器人连接信息和课程现场资料，应保持私有。
- 当前工作区有未提交改动，主要是深度辅助、四类目标和 headless 调参相关内容；不要随手 `git reset`。
- README 之前的红绿蓝描述已经过期；需要复用红绿蓝模型时，应显式切换 `yolo_model:=competition_blocks` 和 `yolo_classes:=red,green,blue`。
