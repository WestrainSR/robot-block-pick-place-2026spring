# 本地采集 UI 使用说明

## 功能

`tools/color_block_capture_ui.py` 是本地浏览器采集工作台。它解决三件事：

1. 展示红、绿、蓝三类方块分别要采多少、已经采多少、还差多少。
2. 展示当前图片类型的拍摄要求。
3. 通过 SSH 自动从机器人摄像头抓图，并保存到本地训练数据目录。

## 启动

在项目根目录运行：

```powershell
python tools/color_block_capture_ui.py --port 8765
```

打开：

```text
http://127.0.0.1:8765
```

## 使用顺序

1. 点击 `连接机器人热点`
2. 点击 `检测相机`
3. 按 UI 中“当前拍摄任务”的提示摆放方块
4. 点击 `拍照并保存`
5. UI 会自动切换到下一条推荐任务

采集结束后点击 `恢复 TJ-WIFI`，回到校园网。

## 数据保存位置

```text
D:\work\2026spring\工程智能\机器人\datasets\color_block_capture
```

结构：

```text
datasets/color_block_capture/
  manifest.csv
  raw/
    red/
    green/
    blue/
    mixed/
```

`manifest.csv` 记录每张图的类别、图片类型、路径和备注，后续整理 YOLO 数据集会基于它和标注文件自动处理。

## 当前采集指标

每类目标 300 张有效样本：

```text
单块正面中距           每类 50 张
距离变化               每类 50 张
角度变化               每类 50 张
光照和背景变化         每类 40 张
轻微遮挡/夹爪干扰      每类 30 张
红绿蓝三色同框         全局 80 张，同时计入三类
```

## 连接方式

UI 后端运行在本机，通过：

```text
本机浏览器 -> 本机 Python 后端 -> SSH pi@192.168.149.1 -> docker exec MentorPi -> ROS 相机 topic
```

相机 topic：

```text
/ascamera/camera_publisher/rgb0/image
```

所以采集时电脑必须能连到机器人热点 `HW-9E5ACFD8`。
