# 方块拾取闭环控制项目

这是工程智能课程机器人方块拾取任务的交接仓库，包含：

- 任务资料与实施方案：`docs/`
- 闭环控制 ROS2 包：`competition_pick_place/`
- 现场照片：根目录下 `IMG_*.jpg` 与参考图片
- 执行记录与交接说明：`docs/闭环控制执行记录.md`、`docs/项目交接文档.md`

## 当前状态

已完成：

- 清理树莓派上一组残留代码，仅保留原厂框架和本项目新增包。
- 新增 `competition_pick_place` ROS2 包。
- 实现 Nav2 大范围闭环导航、YOLO 检测框末端视觉伺服、原厂动作组抓取/放置。
- 已在机器人 Docker 容器中构建通过。
- 已通过 `dry_run` 单命令验证完整状态机可以跑到 `DONE` 并干净退出。

未直接实跑 `dry_run:=false` 的原因：

- 机器人上尚未部署四类方块 YOLO OpenVINO 模型 `competition_blocks.xml/bin`。
- 比赛场地导航点仍是占位坐标，需要现场标定。

## 核心命令

在机器人 ROS2 容器中：

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch competition_pick_place competition_run.launch.py \
  target_class:=grass \
  dry_run:=false \
  start_navigation:=true \
  start_yolo:=true \
  yolo_model:=competition_blocks \
  map_name:=competition_map
```

抽签目标只需要替换 `target_class`：

```bash
target_class:=redstone
target_class:=glass
target_class:=glowstone
target_class:=grass
```

## 安全说明

本仓库应保持私有，因为课程现场文档中包含机器人热点、SSH 用户名和密码等信息。
