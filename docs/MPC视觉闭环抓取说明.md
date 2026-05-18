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
horizon = 6
dt = 0.12
```

每个检测周期重新求一次最低代价控制量，因此是 receding-horizon MPC。

## 低位视觉限制

实测发现，夹爪下探到低位后，相机会出现目标不可见或被夹爪遮挡的情况。因此当前策略是：

```text
pick_preclose_required:=false
```

也就是在“下探过程中可见的最后窗口”完成 MPC 修正，然后立即闭爪。若后续调整相机或姿态，使低位也能稳定看见方块，可以改为：

```text
pick_preclose_required:=true
```

这样会在闭爪前再次等待低位视觉稳定。

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
desired_center_x_ratio=0.50
pick_target_area_ratio=0.042
pick_preclose_center_x_ratio=0.90
pick_preclose_target_area_ratio=0.073
max_linear_speed=0.06
max_angular_speed=0.20
```

## 最近一次实机日志结论

最近一次运行结果：

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
