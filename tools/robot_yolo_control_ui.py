#!/usr/bin/env python3
import argparse
import base64
import json
import shlex
import socket
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:
    import paramiko
except Exception:  # pragma: no cover
    paramiko = None


ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / 'logs'
ROBOT_HOST = '192.168.149.1'
ROBOT_USER = 'pi'
ROBOT_PASSWORD = 'raspberrypi'
ROBOT_CONTAINER = 'MentorPi'
REMOTE_PICK_LOG = '/tmp/ui_pick.log'
REMOTE_YOLO_LOG = '/tmp/ui_yolo.log'
CAMERA_TOPIC = '/ascamera/camera_publisher/rgb0/image'
DETECTION_TOPIC = '/yolo_node/object_detect'


STREAM_SCRIPT = r'''
import base64
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectsInfo
from rclpy.node import Node
from sensor_msgs.msg import Image

CAMERA_TOPIC = "__CAMERA_TOPIC__"
DETECTION_TOPIC = "__DETECTION_TOPIC__"
FPS = __FPS__
JPEG_QUALITY = __JPEG_QUALITY__
STALE_SECONDS = __STALE_SECONDS__
COLORS = {
    "red": (40, 55, 230),
    "green": (70, 190, 95),
    "blue": (230, 115, 60),
}


class YoloOverlayStream(Node):
    def __init__(self):
        super().__init__("local_yolo_overlay_stream")
        self.bridge = CvBridge()
        self.latest = []
        self.latest_time = 0.0
        self.last_frame_time = 0.0
        self.create_subscription(ObjectsInfo, DETECTION_TOPIC, self.detect_cb, 10)
        self.create_subscription(Image, CAMERA_TOPIC, self.image_cb, 1)

    def detect_cb(self, msg):
        detections = []
        for obj in msg.objects:
            if len(obj.box) < 4:
                continue
            detections.append({
                "class_name": obj.class_name.strip(),
                "score": float(obj.score),
                "box": [int(v) for v in obj.box[:4]],
                "width": int(obj.width or 640),
                "height": int(obj.height or 480),
            })
        self.latest = detections
        self.latest_time = time.time()

    def image_cb(self, msg):
        now = time.time()
        if now - self.last_frame_time < 1.0 / max(1.0, FPS):
            return
        self.last_frame_time = now
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h), (230, 230, 230), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (230, 230, 230), 1)
        if now - self.latest_time <= STALE_SECONDS:
            for det in self.latest:
                x1, y1, x2, y2 = det["box"]
                x1 = max(0, min(w - 1, x1))
                x2 = max(0, min(w - 1, x2))
                y1 = max(0, min(h - 1, y1))
                y2 = max(0, min(h - 1, y2))
                color = COLORS.get(det["class_name"], (0, 210, 255))
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cx = ((x1 + x2) * 0.5) / max(1, w)
                area = max(0, x2 - x1) * max(0, y2 - y1) / max(1, w * h)
                label = f'{det["class_name"]} {det["score"]:.2f} cx={cx:.2f} area={area:.3f}'
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
                y0 = max(0, y1 - th - 8)
                cv2.rectangle(frame, (x1, y0), (min(w - 1, x1 + tw + 8), y1), color, -1)
                cv2.putText(frame, label, (x1 + 4, max(th + 2, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, "waiting for YOLO detections", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 210, 255), 2, cv2.LINE_AA)

        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            sys.stdout.write("__JPEG__" + base64.b64encode(encoded.tobytes()).decode("ascii") + "\n")
            sys.stdout.flush()


rclpy.init()
node = YoloOverlayStream()
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    node.destroy_node()
    rclpy.shutdown()
'''


INDEX_HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>机器人 YOLO 抓取控制台</title>
  <style>
    :root {
      --bg: #f4f6f8;
      --panel: #ffffff;
      --text: #182230;
      --muted: #5f6b7a;
      --line: #d8dee6;
      --accent: #2463eb;
      --danger: #c93636;
      --ok: #178a55;
      --shadow: 0 8px 24px rgba(24, 34, 48, .08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
      color: var(--text);
      background: var(--bg);
    }
    header {
      min-height: 64px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 14px 22px;
      background: var(--panel);
      border-bottom: 1px solid var(--line);
    }
    h1 { font-size: 18px; margin: 0; font-weight: 650; }
    .status { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
    .pill {
      border: 1px solid var(--line);
      background: #f9fafb;
      border-radius: 999px;
      padding: 7px 10px;
      font-size: 13px;
      color: var(--muted);
    }
    main {
      display: grid;
      grid-template-columns: minmax(520px, 1.4fr) minmax(360px, .9fr);
      gap: 16px;
      padding: 16px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: var(--shadow);
      min-width: 0;
    }
    .video {
      display: grid;
      grid-template-rows: auto minmax(360px, 1fr);
      overflow: hidden;
    }
    .toolbar, .panel-head {
      min-height: 52px;
      display: flex;
      gap: 10px;
      align-items: center;
      justify-content: space-between;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
    }
    .button-row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
    button, select, input {
      min-height: 36px;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fff;
      color: var(--text);
      font: inherit;
      padding: 0 10px;
    }
    button { cursor: pointer; font-weight: 600; }
    button.primary { background: var(--accent); color: #fff; border-color: var(--accent); }
    button.danger { background: var(--danger); color: #fff; border-color: var(--danger); }
    button.ok { background: var(--ok); color: #fff; border-color: var(--ok); }
    button:disabled { opacity: .55; cursor: wait; }
    .frame {
      min-height: 360px;
      background: #111827;
      display: grid;
      place-items: center;
      overflow: hidden;
    }
    .frame img {
      width: 100%;
      height: 100%;
      object-fit: contain;
      display: block;
    }
    .side { display: grid; grid-template-rows: auto auto 1fr; overflow: hidden; }
    .params {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      padding: 12px;
      border-bottom: 1px solid var(--line);
    }
    label { display: grid; gap: 5px; font-size: 12px; color: var(--muted); }
    label input, label select { width: 100%; color: var(--text); }
    .wide { grid-column: 1 / -1; }
    .log {
      margin: 0;
      padding: 12px;
      min-height: 280px;
      overflow: auto;
      background: #0e1726;
      color: #d5e1f3;
      font: 12px/1.5 Consolas, "Courier New", monospace;
      white-space: pre-wrap;
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .params { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <h1>机器人 YOLO 抓取控制台</h1>
    <div class="status">
      <span class="pill" id="robot">机器人: 未检测</span>
      <span class="pill" id="vision">视觉: 未启动</span>
      <span class="pill" id="pick">抓取: 空闲</span>
    </div>
  </header>
  <main>
    <section class="video">
      <div class="toolbar">
        <div class="button-row">
          <button id="check">检测连接</button>
          <button id="visionStart" class="primary">启动视觉</button>
          <button id="visionStop">停止视觉</button>
        </div>
        <div class="button-row">
          <button id="streamStart" class="ok">打开画面</button>
          <button id="streamStop">关闭画面</button>
        </div>
      </div>
      <div class="frame">
        <img id="stream" alt="YOLO 实时画面">
      </div>
    </section>
    <section class="side">
      <div class="panel-head">
        <strong>抓取控制</strong>
        <div class="button-row">
          <button id="pickStart" class="primary">启动抓取</button>
          <button id="pickStop" class="danger">结束抓取</button>
          <button id="drop">放下</button>
        </div>
      </div>
      <div class="params">
        <label>目标颜色
          <select id="target_class">
            <option value="green">green</option>
            <option value="red">red</option>
            <option value="blue">blue</option>
          </select>
        </label>
        <label>YOLO 置信度
          <input id="yolo_conf" type="number" step="0.01" min="0.1" max="0.99" value="0.70">
        </label>
        <label>中心目标 cx
          <input id="center" type="number" step="0.001" min="0" max="1" value="0.50">
        </label>
        <label>中心容差
          <input id="center_tolerance" type="number" step="0.001" min="0" max="0.5" value="0.028">
        </label>
        <label>面积目标
          <input id="target_area" type="number" step="0.001" min="0" max="0.5" value="0.042">
        </label>
        <label>面积容差
          <input id="area_tolerance" type="number" step="0.001" min="0" max="0.2" value="0.012">
        </label>
        <label>最大线速度
          <input id="max_linear" type="number" step="0.01" min="0" max="0.2" value="0.06">
        </label>
        <label>最大角速度
          <input id="max_angular" type="number" step="0.01" min="0" max="0.6" value="0.20">
        </label>
        <label>低位目标 cx
          <input id="preclose_center" type="number" step="0.001" min="0" max="1" value="0.90">
        </label>
        <label>低位面积目标
          <input id="preclose_area" type="number" step="0.001" min="0" max="0.5" value="0.073">
        </label>
        <label class="wide">闭环模式
          <select id="control_mode">
            <option value="mpc">mpc</option>
            <option value="p">p</option>
          </select>
        </label>
      </div>
      <pre class="log" id="log">准备就绪。</pre>
    </section>
  </main>
<script>
const $ = id => document.getElementById(id);
let streamOn = false;

function params() {
  return {
    target_class: $('target_class').value,
    yolo_conf: Number($('yolo_conf').value),
    center: Number($('center').value),
    center_tolerance: Number($('center_tolerance').value),
    target_area: Number($('target_area').value),
    area_tolerance: Number($('area_tolerance').value),
    max_linear: Number($('max_linear').value),
    max_angular: Number($('max_angular').value),
    preclose_center: Number($('preclose_center').value),
    preclose_area: Number($('preclose_area').value),
    control_mode: $('control_mode').value
  };
}

async function api(path, body) {
  const options = body ? {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body)
  } : {};
  const res = await fetch(path, options);
  const data = await res.json();
  if (!res.ok || !data.ok) throw new Error(data.error || res.statusText);
  return data;
}

function setBusy(button, busy) {
  button.disabled = busy;
}

function appendLog(text) {
  $('log').textContent = text || '';
  $('log').scrollTop = $('log').scrollHeight;
}

async function refreshStatus() {
  try {
    const data = await api('/api/status');
    $('robot').textContent = '机器人: ' + (data.robot ? '在线' : '离线');
    $('vision').textContent = '视觉: ' + (data.yolo ? '运行中' : '未启动');
    $('pick').textContent = '抓取: ' + (data.pick ? '运行中' : '空闲');
    if (data.log) appendLog(data.log);
  } catch (err) {
    $('robot').textContent = '机器人: 离线';
    appendLog(String(err.message || err));
  }
}

$('check').onclick = refreshStatus;
$('visionStart').onclick = async () => {
  setBusy($('visionStart'), true);
  try { await api('/api/start_vision', params()); await refreshStatus(); }
  catch (err) { appendLog(String(err.message || err)); }
  finally { setBusy($('visionStart'), false); }
};
$('visionStop').onclick = async () => {
  setBusy($('visionStop'), true);
  try { await api('/api/stop_vision'); await refreshStatus(); }
  catch (err) { appendLog(String(err.message || err)); }
  finally { setBusy($('visionStop'), false); }
};
$('streamStart').onclick = () => {
  streamOn = true;
  $('stream').src = '/stream.mjpg?ts=' + Date.now();
};
$('streamStop').onclick = () => {
  streamOn = false;
  $('stream').removeAttribute('src');
};
$('pickStart').onclick = async () => {
  setBusy($('pickStart'), true);
  try { await api('/api/start_pick', params()); await refreshStatus(); }
  catch (err) { appendLog(String(err.message || err)); }
  finally { setBusy($('pickStart'), false); }
};
$('pickStop').onclick = async () => {
  setBusy($('pickStop'), true);
  try { await api('/api/stop_pick'); await refreshStatus(); }
  catch (err) { appendLog(String(err.message || err)); }
  finally { setBusy($('pickStop'), false); }
};
$('drop').onclick = async () => {
  setBusy($('drop'), true);
  try { await api('/api/drop', params()); await refreshStatus(); }
  catch (err) { appendLog(String(err.message || err)); }
  finally { setBusy($('drop'), false); }
};

setInterval(refreshStatus, 2500);
refreshStatus();
</script>
</body>
</html>
'''


def parse_args():
    parser = argparse.ArgumentParser(description='Local web UI for robot YOLO stream and grasp control.')
    parser.add_argument('--host', default=ROBOT_HOST)
    parser.add_argument('--user', default=ROBOT_USER)
    parser.add_argument('--password', default=ROBOT_PASSWORD)
    parser.add_argument('--container', default=ROBOT_CONTAINER)
    parser.add_argument('--port', type=int, default=8090)
    return parser.parse_args()


def current_ssid() -> str:
    try:
        out = subprocess.check_output('netsh wlan show interfaces', shell=True, text=True, errors='replace', timeout=5)
    except Exception:
        return ''
    for line in out.splitlines():
        if ' SSID' in line and 'BSSID' not in line:
            return line.split(':', 1)[-1].strip()
    return ''


class Robot:
    def __init__(self, host: str, user: str, password: str, container: str):
        self.host = host
        self.user = user
        self.password = password
        self.container = container
        self.lock = threading.Lock()

    def ssh(self):
        if paramiko is None:
            raise RuntimeError('paramiko is not installed')
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(self.host, username=self.user, password=self.password, timeout=8, banner_timeout=8, auth_timeout=8)
        return client

    def docker_exec(self, script: str, timeout: int = 30) -> tuple[int, str, str]:
        command = f'docker exec -u ubuntu {shlex.quote(self.container)} bash -lc {shlex.quote(script)}'
        client = self.ssh()
        try:
            stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
            out = stdout.read().decode('utf-8', errors='replace')
            err = stderr.read().decode('utf-8', errors='replace')
            rc = stdout.channel.recv_exit_status()
            return rc, out, err
        finally:
            client.close()

    def docker_stream(self, script: str):
        command = f'docker exec -u ubuntu {shlex.quote(self.container)} bash -lc {shlex.quote(script)}'
        client = self.ssh()
        stdin, stdout, stderr = client.exec_command(command, timeout=None)
        return client, stdout, stderr


def ros_prefix() -> str:
    return 'source /opt/ros/humble/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; export need_compile=True; '


def shell_kill_helpers() -> str:
    return r'''
kill_matching() {
  PATTERN="$1"
  for PID in $(pgrep -f "$PATTERN" 2>/dev/null || true); do
    if [ "$PID" != "$$" ] && [ "$PID" != "$PPID" ]; then
      kill "$PID" 2>/dev/null || true
    fi
  done
}
'''


def require_ok(rc: int, out: str, err: str) -> None:
    if rc != 0:
        raise RuntimeError((err or out or f'command failed rc={rc}').strip())


def start_vision(robot: Robot, params: dict) -> str:
    yolo_conf = float(params.get('yolo_conf', 0.70))
    script = ros_prefix() + shell_kill_helpers() + f'''
set +e
if ! ros2 topic list 2>/dev/null | grep -F "{CAMERA_TOPIC}" >/dev/null; then
  nohup ros2 launch peripherals depth_camera.launch.py >/tmp/ui_camera.log 2>&1 &
  sleep 3
fi
kill_matching "yolov11_node"
kill_matching "python3 -u /tmp/ui_yolo_runner.py"
cat >/tmp/ui_yolo_runner.py <<'PY'
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import Node

launch_service = LaunchService()
launch_service.include_launch_description(
    LaunchDescription([
        Node(
            package='yolov11_detect',
            executable='yolov11_node',
            name='yolo_node',
            output='screen',
            parameters=[
                {{'classes': ['red', 'green', 'blue']}},
                {{'model': 'competition_blocks', 'conf': {yolo_conf:.3f}, 'start': True}},
            ],
        )
    ])
)
sys.exit(launch_service.run())
PY
nohup python3 -u /tmp/ui_yolo_runner.py > {REMOTE_YOLO_LOG} 2>&1 &
echo "vision_pid=$!"
'''
    rc, out, err = robot.docker_exec(script, timeout=20)
    require_ok(rc, out, err)
    return out.strip()


def stop_vision(robot: Robot) -> str:
    script = ros_prefix() + shell_kill_helpers() + f'''
set +e
kill_matching "yolov11_node"
kill_matching "python3 -u /tmp/ui_yolo_runner.py"
echo "vision stopped"
'''
    rc, out, err = robot.docker_exec(script, timeout=12)
    require_ok(rc, out, err)
    return out.strip()


def start_pick(robot: Robot, params: dict) -> str:
    target = shlex.quote(str(params.get('target_class', 'green')))
    control_mode = shlex.quote(str(params.get('control_mode', 'mpc')))
    center = float(params.get('center', 0.50))
    center_tol = float(params.get('center_tolerance', 0.028))
    target_area = float(params.get('target_area', 0.042))
    area_tol = float(params.get('area_tolerance', 0.012))
    max_linear = float(params.get('max_linear', 0.06))
    max_angular = float(params.get('max_angular', 0.20))
    preclose_center = float(params.get('preclose_center', 0.90))
    preclose_area = float(params.get('preclose_area', 0.073))
    script = ros_prefix() + shell_kill_helpers() + f'''
set +e
timeout 2s ros2 service call /competition_pick_place/stop std_srvs/srv/Trigger "{{}}" >/dev/null 2>&1 || true
kill_matching "competition_node"
kill_matching "yolov11_node"
kill_matching "python3 -u /tmp/ui_yolo_runner.py"
for PID in $(pgrep -f "ros2 launch competition_pick_place" 2>/dev/null || true); do
  if [ "$PID" != "$$" ] && [ "$PID" != "$PPID" ]; then kill "$PID" 2>/dev/null || true; fi
done
rm -f {REMOTE_PICK_LOG}
nohup ros2 launch competition_pick_place competition_run.launch.py \\
  target_class:={target} \\
  dry_run:=false \\
  stop_after_pick:=true \\
  exit_on_done:=true \\
  start_navigation:=false \\
  start_base:=false \\
  start_camera:=false \\
  start_yolo:=true \\
  use_nav:=false \\
  use_arm:=true \\
  yolo_model:=competition_blocks \\
  yolo_conf:={float(params.get('yolo_conf', 0.70)):.3f} \\
  init_action:=navigation_pick_init_ai \\
  pick_action:=navigation_pick_ai \\
  search_timeout:=12.0 \\
  align_timeout:=45.0 \\
  desired_center_x_ratio:={center:.4f} \\
  center_tolerance_ratio:={center_tol:.4f} \\
  pick_target_area_ratio:={target_area:.4f} \\
  area_tolerance_ratio:={area_tol:.4f} \\
  stable_frames:=4 \\
  control_mode:={control_mode} \\
  closed_loop_pick:=true \\
  pick_pregrasp_visual_servo:=true \\
  pick_preclose_required:=false \\
  pick_preclose_center_x_ratio:={preclose_center:.4f} \\
  pick_preclose_target_area_ratio:={preclose_area:.4f} \\
  pick_preclose_center_tolerance_ratio:=0.0650 \\
  pick_preclose_area_tolerance_ratio:=0.0200 \\
  pick_preclose_stable_frames:=1 \\
  angular_k:=0.80 \\
  max_linear_speed:={max_linear:.4f} \\
  max_angular_speed:={max_angular:.4f} \\
  search_angular_speed:=0.12 \\
  mpc_horizon:=6 \\
  mpc_dt:=0.12 \\
  mpc_center_response:=1.05 \\
  mpc_area_response:=0.24 \\
  mpc_center_weight:=8.0 \\
  mpc_area_weight:=26.0 \\
  mpc_velocity_weight:=0.08 \\
  mpc_delta_weight:=0.16 \\
  mpc_terminal_weight:=2.2 \\
  mpc_center_gate_ratio:=0.10 > {REMOTE_PICK_LOG} 2>&1 &
echo "pick_pid=$!"
'''
    rc, out, err = robot.docker_exec(script, timeout=18)
    require_ok(rc, out, err)
    return out.strip()


def stop_pick(robot: Robot) -> str:
    script = ros_prefix() + shell_kill_helpers() + '''
set +e
timeout 2s ros2 service call /competition_pick_place/stop std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
kill_matching "competition_node"
for PID in $(pgrep -f "ros2 launch competition_pick_place" 2>/dev/null || true); do
  if [ "$PID" != "$$" ] && [ "$PID" != "$PPID" ]; then kill "$PID" 2>/dev/null || true; fi
done
echo "pick stopped"
'''
    rc, out, err = robot.docker_exec(script, timeout=12)
    require_ok(rc, out, err)
    return out.strip()


def drop_block(robot: Robot) -> str:
    script = ros_prefix() + '''
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from servo_controller.action_group_controller import ActionGroupController
from servo_controller_msgs.msg import ServosPosition
from std_srvs.srv import Trigger

rclpy.init()
node = Node('local_ui_drop_action')
pub = node.create_publisher(ServosPosition, 'servo_controller', 1)
client = node.create_client(Trigger, '/controller_manager/init_finish')
client.wait_for_service(timeout_sec=8.0)
for _ in range(8):
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
ActionGroupController(pub, '/home/ubuntu/software/arm_pc/ActionGroups').run_action('navigation_place')
time.sleep(0.3)
node.destroy_node()
rclpy.shutdown()
PY
echo "drop done"
'''
    rc, out, err = robot.docker_exec(script, timeout=25)
    require_ok(rc, out, err)
    return out.strip()


def get_status(robot: Robot) -> dict:
    script = ros_prefix() + shell_kill_helpers() + f'''
set +e
echo "__TOPICS__"
ros2 topic list 2>/dev/null | grep -E "ascamera|object_detect|yolo|controller/cmd_vel" || true
echo "__PROCS__"
for PATTERN in "yolov11_node" "python3 -u /tmp/ui_yolo_runner.py" "competition_node" "ros2 launch competition_pick_place"; do
  for PID in $(pgrep -f "$PATTERN" 2>/dev/null || true); do
    if [ "$PID" != "$$" ] && [ "$PID" != "$PPID" ]; then
      ps -p "$PID" -o pid=,args= 2>/dev/null || true
    fi
  done
done
echo "__YOLO_LOG__"
tail -n 80 {REMOTE_YOLO_LOG} 2>/dev/null || true
echo "__PICK_LOG__"
tail -n 80 {REMOTE_PICK_LOG} 2>/dev/null || true
'''
    rc, out, err = robot.docker_exec(script, timeout=10)
    if rc != 0:
        raise RuntimeError((err or out or 'status failed').strip())
    topics = ''
    procs = ''
    log = out[-4000:]
    if '__TOPICS__' in out and '__PROCS__' in out:
        topics = out.split('__TOPICS__', 1)[1].split('__PROCS__', 1)[0]
    yolo_log = ''
    if '__PROCS__' in out and '__YOLO_LOG__' in out:
        procs = out.split('__PROCS__', 1)[1].split('__YOLO_LOG__', 1)[0]
    if '__YOLO_LOG__' in out and '__PICK_LOG__' in out:
        yolo_log = out.split('__YOLO_LOG__', 1)[1].split('__PICK_LOG__', 1)[0].strip()
    if '__PICK_LOG__' in out:
        log = out.split('__PICK_LOG__', 1)[-1].strip()
    yolo = 'yolov11_node' in procs or DETECTION_TOPIC in topics
    pick = 'competition_node' in procs
    combined_log = ''
    if yolo_log:
        combined_log += '== YOLO ==\n' + yolo_log[-2200:] + '\n'
    if log:
        combined_log += '== PICK ==\n' + log[-3000:]
    return {'robot': True, 'yolo': yolo, 'pick': pick, 'log': combined_log[-5500:], 'ssid': current_ssid()}


def make_stream_script(fps: int = 8, quality: int = 78) -> str:
    script = STREAM_SCRIPT.replace('__CAMERA_TOPIC__', CAMERA_TOPIC)
    script = script.replace('__DETECTION_TOPIC__', DETECTION_TOPIC)
    script = script.replace('__FPS__', str(fps))
    script = script.replace('__JPEG_QUALITY__', str(quality))
    script = script.replace('__STALE_SECONDS__', '0.8')
    return ros_prefix() + 'python3 -u - <<\'PY\'\n' + script + '\nPY\n'


class Handler(BaseHTTPRequestHandler):
    server_version = 'RobotYoloControlUI/0.1'

    def log_message(self, fmt, *args):
        return

    @property
    def robot(self) -> Robot:
        return self.server.robot

    def read_json(self) -> dict:
        length = int(self.headers.get('Content-Length', '0') or '0')
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        return json.loads(raw.decode('utf-8'))

    def send_json(self, data: dict, status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_error_json(self, exc: Exception, status: int = 500) -> None:
        self.send_json({'ok': False, 'error': str(exc)}, status)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/':
            body = INDEX_HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == '/api/status':
            try:
                self.send_json({'ok': True, **get_status(self.robot)})
            except Exception as exc:
                self.send_error_json(exc, HTTPStatus.BAD_GATEWAY)
            return
        if parsed.path == '/stream.mjpg':
            self.stream_mjpg(parsed)
            return
        self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        try:
            payload = self.read_json()
            if parsed.path == '/api/start_vision':
                self.send_json({'ok': True, 'message': start_vision(self.robot, payload)})
            elif parsed.path == '/api/stop_vision':
                self.send_json({'ok': True, 'message': stop_vision(self.robot)})
            elif parsed.path == '/api/start_pick':
                self.send_json({'ok': True, 'message': start_pick(self.robot, payload)})
            elif parsed.path == '/api/stop_pick':
                self.send_json({'ok': True, 'message': stop_pick(self.robot)})
            elif parsed.path == '/api/drop':
                self.send_json({'ok': True, 'message': drop_block(self.robot)})
            else:
                self.send_error(404)
        except Exception as exc:
            self.send_error_json(exc, HTTPStatus.BAD_GATEWAY)

    def stream_mjpg(self, parsed) -> None:
        query = parse_qs(parsed.query)
        fps = int(query.get('fps', ['8'])[0])
        quality = int(query.get('quality', ['78'])[0])
        client = None
        try:
            client, stdout, stderr = self.robot.docker_stream(make_stream_script(fps=fps, quality=quality))
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.end_headers()
            for raw in iter(stdout.readline, ''):
                if not raw.startswith('__JPEG__'):
                    continue
                jpg = base64.b64decode(raw[len('__JPEG__'):].strip())
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpg)}\r\n\r\n'.encode('ascii'))
                self.wfile.write(jpg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            try:
                self.send_error(502)
            except Exception:
                pass
        finally:
            if client is not None:
                client.close()


def find_free_port(preferred: int) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(('127.0.0.1', preferred))
            return preferred
        except OSError:
            pass
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        return int(s.getsockname()[1])


def main():
    args = parse_args()
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    port = find_free_port(args.port)
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.robot = Robot(args.host, args.user, args.password, args.container)
    print(f'Robot YOLO control UI: http://127.0.0.1:{port}', flush=True)
    print('WiFi switching is manual; connect to HW-9E5ACFD8 before using robot controls.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
