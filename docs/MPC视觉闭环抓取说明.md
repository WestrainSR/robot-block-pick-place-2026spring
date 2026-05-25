# MPC 视觉闭环抓取说明

## 当前实现

当前抓取不再是“目标到画面中心后整段执行写死动作组”。

新的流程是：

1. `navigation_pick_init_ai`
   - 使用原厂 AI 抓取初始化姿态。
2. 粗对准
   - 订阅 `/yolo_node/object_detect`。
   - 用 YOLO 框中心 `cx` 和框面积 `area` 作为视觉反馈。
   - 发布 `/controller/cmd_vel` 调整底盘，使绿色方块先进入粗抓取区域。
3. 闭环下探
   - 读取原厂 `navigation_pick_ai.d6a` 中的动作步。
   - 只执行第 1、2 步作为下探/张爪。
   - 下探动作进行的同时继续运行视觉控制，而不是等待动作结束。
   - 第 1、2 步是机械臂动作组步号，不是 MPC 只规划两次；MPC 会在动作持续时间内按控制周期反复重算。
4. 闭爪和抬升
   - 执行 `navigation_pick_ai.d6a` 第 3、4 步闭爪。
   - 执行第 5、6 步抬升。

## MPC 控制器

实现位置：

```text
competition_pick_place/competition_pick_place/competition_node.py
```

参数入口：

```text
control_mode:=mpc
closed_loop_pick:=true
pick_pregrasp_visual_servo:=true
```

控制量：

```text
linear.x
angular.z
```

状态量：

```text
center_error = detection.cx_ratio - desired_center_x_ratio
area_error   = target_area_ratio - detection.area_ratio
```

代价函数同时惩罚：

```text
center_error
area_error
速度幅值
相邻控制量变化
预测终端误差
```

为了避免在树莓派上安装额外优化器，当前 MPC 使用小规模采样滚动优化：

```text
linear.x  candidates = [-vmax, -0.5vmax, 0, 0.5vmax, vmax]
angular.z candidates = [-wmax, -0.5wmax, 0, 0.5wmax, wmax]
visual_servo_period = 0.06
horizon = 10
dt = 0.06
```

默认控制周期约 `0.06s`，也就是约 `16.7Hz`；`tools/run_green_pick.py` 的实机绿色抓取脚本使用 `visual_servo_period=0.05`，约 `20Hz`。

每个控制周期都会读取最新检测并重新求一次最低代价控制量，因此是 receding-horizon MPC。日志为了避免刷屏会降频打印，所以日志条数不能代表 MPC 求解次数。

`horizon * dt` 是预测窗口。当前默认 `10 * 0.06 = 0.60s`；实机绿色脚本是 `10 * 0.05 = 0.50s`。相比旧配置 `6 * 0.12 = 0.72s`，新配置的预测步长更接近实际控制周期，输出会更细、更及时。

## YOLO 启动等待

启动抓取后，节点会先等待 `/yolo_node/object_detect` 检测流 ready：

```text
wait_for_detection_stream=true
detection_stream_timeout=20.0
detection_ready_min_messages=1
```

检测流 ready 之前，底盘只发布零速度，避免 YOLO 节点还在启动时机器人原地左右搜索。收到检测流消息后，如果目标类别仍然不可见，才进入搜索旋转。

## 低位视觉限制

实测发现，夹爪下探到低位后，相机会出现目标不可见或被夹爪遮挡的情况。因此绿色实机脚本当前策略是：

```text
pick_preclose_required:=false
pick_preclose_fail_on_timeout:=false
```

也就是到低位后直接闭爪，不再等待低位面积目标。实机观察表明夹爪下探到最低后相机容易看不到方块，继续等待 YOLO 框会造成闭爪延迟。若后续调整相机或姿态，使低位也能稳定看见方块，可以重新打开闭爪前确认：

```text
pick_preclose_required:=true
pick_preclose_fail_on_timeout:=true
```

这样会在闭爪前强制等待低位视觉稳定。

## 当前绿色抓取参数

本地一键脚本：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/run_green_pick_with_wifi.ps1
```

脚本当前使用：

```text
target_class=green
init_action=navigation_pick_init_ai
pick_action=navigation_pick_ai
control_mode=mpc
closed_loop_pick=true
pick_pregrasp_visual_servo=true
pick_preclose_required=false
pick_preclose_fail_on_timeout=false
desired_center_x_ratio=0.50
pick_target_area_ratio=0.043
pick_preclose_center_x_ratio=0.90
pick_preclose_target_area_ratio=0.095
max_linear_speed=0.035
max_angular_speed=0.14
visual_servo_period=0.10
pick_pregrasp_time_scale=2.4
pick_pregrasp_min_step_seconds=0.80
mpc_horizon=8
mpc_dt=0.10
```

## 调参前实机日志结论

调参前的一次运行结果：

```text
run_status=done
green pick target aligned: cx=0.510, area=0.043
run visual-servo action steps navigation_pick_ai [1, 2]
visual-servo green pick pregrasp: cmd=(0.000,0.200)
visual-servo green pick pregrasp: cmd=(-0.030,0.200)
run action group steps navigation_pick_ai [3, 4] for green pick close
run action group steps navigation_pick_ai [5, 6] for green pick lift
DONE: stop_after_pick=true
```

注意：日志只能证明识别、MPC 控制、闭爪、抬升流程都成功执行。是否真正夹住方块仍需现场观察确认。
