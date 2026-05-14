# 方块拾取任务编码 Prompt

下面这段 prompt 可直接交给后续编码执行者或 AI 编程代理使用。目标是在机器人上实现比赛任务，不从零造轮子，优先改造 Hiwonder LanderPi 官方 ROS2 示例。

```text
你是一个机器人 ROS2 工程师。当前任务是在 Hiwonder LanderPi 机器人上完成“单件磁吸方块识别抓取速度竞赛”。

硬件与连接信息：
- 机器人 WiFi：HW-9E5ACFD8
- 机器人 IP：192.168.149.1
- WiFi 密码：hiwonder
- 系统用户：pi
- 系统密码：raspberrypi
- SSH/WinSCP 端口：22

竞赛目标：
机器人从启停区出发，自主导航到原料存放区，在四类磁吸方块中识别抽签指定目标类型，抓取目标方块，导航到目标存放区，将方块放置到色环中心附近，最后返回启停区。四类目标统一命名为：
- redstone
- glass
- glowstone
- grass

最高优先级要求：
1. 不要从零实现底盘、相机、机械臂、Nav2 或深度定位。
2. 优先复用、复制并改造机器人本地已有的 Hiwonder LanderPi 官方示例，例如 navigation_transport、automatic_pick、color_sorting、yolov8/yolov11_detect、navigation、slam、kinematics、controller 相关包。
3. 保留官方示例原文件，不要直接破坏原始代码。新建 competition_pick_place 包，或在 example 下新建 competition_pick_place 目录。
4. 最终必须支持单条命令启动，例如：
   ros2 launch competition_pick_place competition_run.launch.py target_class:=grass map_name:=competition_map
5. 代码要包含超时、停车、识别失败、抓取失败处理，不能无限卡住。

第一步：连接与环境检查
1. SSH 登录 pi@192.168.149.1。
2. 执行 ~/.stop_ros.sh 关闭默认 APP 服务，避免硬件占用。
3. 确认 ROS2 环境：
   whoami
   pwd
   ls ~
   ls ~/ros2_ws
   ls ~/ros2_ws/src
   ros2 node list
   ros2 topic list
   ros2 service list
4. 如果 ros2 命令不可用，查找并 source setup 脚本：
   find ~ -maxdepth 3 -name 'setup.bash' -o -name 'setup.sh'
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
5. 注意课程资料区分系统终端和 ROS2 环境终端。如果机器人用 Docker 运行 ROS2，请进入正确 ROS2 终端/容器。

第二步：查找并运行官方示例
在 ~/ros2_ws/src 下查找：
   find . -iname '*transport*' -o -iname '*pick*' -o -iname '*sorting*' -o -iname '*yolo*'
   grep -R "navigation_transport\\|automatic_pick\\|color_sorting\\|yolov" -n .

重点阅读：
- navigation_transport：优先参考其导航搬运整体流程。
- automatic_pick：优先复用其三维定位、IK、机械臂抓取动作。
- color_sorting：参考夹爪动作、放置动作、视觉处理结构。
- yolov8_detect 或 yolov11_detect：参考模型加载、检测结果 topic/message。
- navigation / navigation2：参考 Nav2 launch 和地图加载方式。

先单独跑通官方 automatic_pick 或 navigation_transport，确认硬件正常。

第三步：创建比赛包
建议结构：
~/ros2_ws/src/competition_pick_place/
  package.xml
  setup.py
  resource/competition_pick_place
  competition_pick_place/
    __init__.py
    competition_node.py
    detection_adapter.py
    nav_adapter.py
    arm_adapter.py
    config_loader.py
  config/
    competition_waypoints.yaml
    classes.yaml
  launch/
    competition_run.launch.py

如果新增完整包太费时间，可以先创建一个可运行脚本版本，但最终仍要能通过 ros2 launch 单条命令启动。

第四步：实现状态机
在 competition_node.py 中实现如下状态机：
1. INIT
   - 读取参数 target_class、map_name、waypoints_yaml、model_path、confidence_threshold、dry_run。
   - 检查 target_class 必须是 redstone/glass/glowstone/grass 之一。
   - 检查导航、检测、机械臂依赖节点或服务可用。
2. NAV_TO_MATERIAL
   - 使用 Nav2 或官方导航接口移动到 material_standoff_pose。
   - 超时后停车并重试一次。
3. SEARCH_TARGET
   - 订阅 YOLO 检测结果。
   - 筛选类别等于 target_class 且置信度大于阈值的目标。
   - 优先选择深度有效、离画面中心最近、置信度最高的目标。
   - 若 8 到 12 秒未找到，原地小角度扫描 2 到 3 次。
4. ALIGN_TARGET
   - 根据目标框中心与图像中心误差，用低速底盘微调，直到目标进入机械臂可抓取区域。
   - 如果官方 automatic_pick 已经包含对齐逻辑，则直接复用官方逻辑。
5. PICK
   - 调用官方自动抓取或 IK + 舵机控制服务。
   - 抓取动作：打开夹爪，移动到抓取点上方 0.08m，下降，闭合夹爪，抬升。
   - 失败则重新检测并最多重抓一次。
6. NAV_TO_PLACE
   - 导航到 place_standoff_pose。
7. PLACE
   - 使用标定好的固定放置坐标 place_xyz_pitch。
   - 动作：移动到放置点上方，下降，打开夹爪，抬升并收回。
8. NAV_HOME
   - 导航回 return_pose 或 start_pose。
9. DONE
   - 停车，输出总用时和每阶段结果。
10. FAILSAFE
   - 立即发布零速度或调用官方停车接口。
   - 机械臂回安全位，输出失败原因。

第五步：配置文件
创建 config/classes.yaml：
classes:
  - redstone
  - glass
  - glowstone
  - grass

创建 config/competition_waypoints.yaml，先写占位值，必须现场标定后替换：
start_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0
material_standoff_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0
place_standoff_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0
return_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0
arm:
  pick_lift: 0.08
  gripper_open: 150
  gripper_close: 350
  place_xyz_pitch: [0.18, 0.0, 0.03, -90.0]

第六步：YOLO 模型
如果已有可用四类模型，直接部署到机器人，例如：
~/ros2_ws/src/yolov8_detect/weights/competition_blocks.pt

如果没有，训练四类 YOLO 模型：
- 每类至少 150 到 200 张。
- 包含四类同时出现在原料区的场景。
- 标签名必须与 redstone/glass/glowstone/grass 一致。
- 置信度要求稳定大于 0.7。

训练命令示例：
   yolo detect train model=yolo11n.pt data=data.yaml epochs=100 imgsz=640 batch=32

部署后先单独验证 YOLO launch，确认检测 topic 中能看到正确类别和置信度。

第七步：导航标定
1. 使用官方 SLAM/快速建图创建 competition_map。
2. 使用 RViz 记录 start_pose、material_standoff_pose、place_standoff_pose、return_pose。
3. 单独测试三段导航：
   - 启停区到原料区
   - 原料区到目标区
   - 目标区回启停区
4. 将最终坐标写入 competition_waypoints.yaml。

第八步：构建和运行
在 ~/ros2_ws 中执行：
   colcon build --symlink-install --packages-select competition_pick_place
   source install/setup.bash

dry-run：
   ros2 launch competition_pick_place competition_run.launch.py target_class:=grass dry_run:=true

真实运行：
   ros2 launch competition_pick_place competition_run.launch.py target_class:=grass dry_run:=false

第九步：验收标准
必须满足：
1. 单条命令启动。
2. target_class 可切换四类。
3. 能自主到原料区。
4. 能识别并选择指定类别方块。
5. 能抓起目标方块。
6. 能到目标区并放置。
7. 能返回启停区。
8. 任一阶段失败时能停车，不会持续乱动。
9. 关键日志包含时间戳、状态名、目标类别、检测置信度、导航结果、抓取结果。

第十步：输出结果
完成后请给出：
- 修改/新增的文件路径。
- 最终启动命令。
- 现场需要填入或已标定的参数。
- 已通过的测试项。
- 未完成或存在风险的项。
```

