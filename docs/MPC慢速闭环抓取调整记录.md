# MPC 慢速闭环抓取调整记录

## 问题判断

实机观察到抓取阶段动作过于连续，机械臂下探、底盘微调、YOLO 延迟、ROS 控制延迟之间没有明显缓冲窗口。原逻辑虽然在预抓取步骤执行期间运行 MPC，但如果动作组每一步太快，MPC 的修正还没有充分体现在底盘运动上，机械臂就已经进入闭爪阶段。

## 本次调整

本次把抓取节奏改成更保守的分段闭环：

1. 粗对准仍然使用 YOLO + MPC。
2. 进入预抓取前，先给 MPC 一个 settle-before 窗口。
3. 每个预抓取动作步骤本身按比例放慢。
4. 预抓取动作执行期间持续 MPC 微调。
5. 每个预抓取步骤结束后，再给 MPC 一个 settle-after 窗口。
6. 最后一个预抓取步骤到达低位后，不再等待低位视觉面积确认，直接进入闭爪。

## 新增参数

这些参数已经加入 ROS launch 和节点：

```text
pick_pregrasp_time_scale
pick_pregrasp_min_step_seconds
pick_pregrasp_settle_seconds
pick_pregrasp_post_step_seconds
```

含义：

```text
pick_pregrasp_time_scale
  预抓取动作组第 1、2 步的机械臂执行时长倍率。

pick_pregrasp_min_step_seconds
  每个预抓取步骤最少给多少秒。

pick_pregrasp_settle_seconds
  每个预抓取步骤开始前，底盘只做视觉 MPC 微调的时间。

pick_pregrasp_post_step_seconds
  中间预抓取步骤结束后，底盘继续视觉 MPC 微调的时间；最后一个预抓取步骤后会跳过这个窗口并直接闭爪。
```

## 当前保守参数

UI 和脚本当前使用的保守值：

```text
wait_for_detection_stream = true
detection_stream_timeout = 20.0
detection_ready_min_messages = 1

visual_servo_period = 0.10
pick_target_area_ratio = 0.043
max_linear_speed = 0.035
max_angular_speed = 0.14
mpc_horizon = 8
mpc_dt = 0.10

pick_pregrasp_time_scale = 2.4
pick_pregrasp_min_step_seconds = 0.80
pick_pregrasp_settle_seconds = 0.70
pick_pregrasp_post_step_seconds = 0.60

pick_preclose_required = false
pick_preclose_fail_on_timeout = false
pick_visual_servo_timeout = 5.0
```

预测窗口为：

```text
mpc_horizon * mpc_dt = 8 * 0.10 = 0.80s
```

这组参数牺牲速度，优先让视觉、MPC、底盘响应和机械臂动作之间有足够时间同步。

启动抓取后，底盘会先原地等待 `/yolo_node/object_detect` 检测流真正开始发布消息。检测流 ready 之前只发零速度，不进入左右搜索；检测流 ready 后仍然找不到目标，才按搜索速度旋转寻找。

低位闭爪策略已经改为：

```text
pregrasp step 1 -> 中间 settle-after 微调
pregrasp step 2 -> 直接 close steps
```

原因是机械臂下探到最低后，相机容易看不到方块；继续用低位面积目标等待 YOLO 框会引入无效等待和闭爪延迟。

## 调参建议

如果实机仍然看不到明显微调，优先增加：

```text
pick_pregrasp_settle_seconds: 0.70 -> 1.00
pick_pregrasp_post_step_seconds: 0.60 -> 1.00
pick_pregrasp_time_scale: 2.4 -> 3.0
```

如果动作明显太慢但抓取稳定，可以逐步降低：

```text
pick_pregrasp_settle_seconds
pick_pregrasp_post_step_seconds
pick_pregrasp_time_scale
```

不要优先提高 `max_linear_speed` 和 `max_angular_speed`；这两个量会直接放大延迟带来的过冲。
