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
from urllib.request import urlopen

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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

CAMERA_TOPIC = "__CAMERA_TOPIC__"
DETECTION_TOPIC = "__DETECTION_TOPIC__"
FPS = __FPS__
JPEG_QUALITY = __JPEG_QUALITY__
STALE_SECONDS = __STALE_SECONDS__
COLORS = {
    "red": (40, 55, 230),
    "green": (70, 190, 95),
    "grass": (70, 190, 95),
    "gray": (255, 0, 255),
    "grey": (255, 0, 255),
    "yellow": (30, 215, 235),
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
        self.create_subscription(Image, CAMERA_TOPIC, self.image_cb, qos_profile_sensor_data)

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
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                cx = ((x1 + x2) * 0.5) / max(1, w)
                area = max(0, x2 - x1) * max(0, y2 - y1) / max(1, w * h)
                label = f'{det["class_name"]} {det["score"]:.2f} cx={cx:.2f} area={area:.3f}'
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.56, 2)
                y0 = max(0, y1 - th - 8)
                cv2.rectangle(frame, (x1, y0), (min(w - 1, x1 + tw + 8), y1), color, -1)
                cv2.putText(frame, label, (x1 + 4, max(th + 2, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 2, cv2.LINE_AA)
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
      position: relative;
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
    .frame-message {
      position: absolute;
      inset: 0;
      display: grid;
      place-items: center;
      padding: 24px;
      color: #d5e1f3;
      background: rgba(17, 24, 39, .92);
      font-size: 16px;
      text-align: center;
      line-height: 1.6;
      pointer-events: none;
    }
    .frame-message.hidden { display: none; }
    .frame-message.error { color: #ffd7d7; }
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
          <button id="reset">重置通信</button>
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
        <div class="frame-message" id="streamMessage">连接机器人 WiFi 后点击“打开画面”。热点 HW-9E5ACFD8，密码 hiwonder。</div>
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
            <option value="grass">grass</option>
            <option value="gray">gray/grey</option>
            <option value="yellow">yellow</option>
            <option value="blue">blue</option>
          </select>
        </label>
        <label>YOLO 置信度
          <input id="yolo_conf" type="number" step="0.01" value="0.20">
        </label>
        <label>YOLO 模型
          <select id="yolo_model">
            <option value="tongji">tongji</option>
            <option value="competition_blocks">competition_blocks</option>
          </select>
        </label>
        <label>中心目标 cx
          <input id="center" type="number" step="0.001" value="0.50">
        </label>
        <label>中心容差
          <input id="center_tolerance" type="number" step="0.001" value="0.028">
        </label>
        <label>面积目标
          <input id="target_area" type="number" step="0.001" value="0.043">
        </label>
        <label>面积容差
          <input id="area_tolerance" type="number" step="0.001" value="0.010">
        </label>
        <label>深度距离
          <select id="use_depth">
            <option value="true">on</option>
            <option value="false">off</option>
          </select>
        </label>
        <label>目标深度(m)
          <input id="target_depth" type="number" step="0.001" value="0.32">
        </label>
        <label>深度容差(m)
          <input id="depth_tolerance" type="number" step="0.001" value="0.025">
        </label>
        <label>深度ROI
          <input id="depth_roi" type="number" step="0.05" value="0.45">
        </label>
        <label class="wide">depth topic
          <input id="depth_topic" type="text" value="/ascamera/camera_publisher/depth0/image_raw">
        </label>
        <label>pick attempts
          <input id="pick_attempts" type="number" step="1" value="3">
        </label>
        <label>gripper gap
          <input id="gripper_gap" type="number" step="1" value="30">
        </label>
        <label>empty close
          <input id="empty_close" type="number" step="1" value="500">
        </label>
        <label>gripper delay(s)
          <input id="gripper_delay" type="number" step="0.05" value="0.35">
        </label>
        <label>最大线速度
          <input id="max_linear" type="number" step="0.005" value="0.09">
        </label>
        <label>最大角速度
          <input id="max_angular" type="number" step="0.01" value="0.35">
        </label>
        <label>visual period(s)
          <input id="visual_period" type="number" step="0.01" value="0.10">
        </label>
        <label>cmd pulse(s)
          <input id="cmd_pulse" type="number" step="0.01" value="0.04">
        </label>
        <label>adaptive timing
          <select id="adaptive_timing">
            <option value="true">on</option>
            <option value="false">off</option>
          </select>
        </label>
        <label>min period(s)
          <input id="min_period" type="number" step="0.005" value="0.035">
        </label>
        <label>max period(s)
          <input id="max_period" type="number" step="0.005" value="0.16">
        </label>
        <label>period scale
          <input id="period_scale" type="number" step="0.05" value="1.05">
        </label>
        <label>pregrasp scale
          <input id="pregrasp_scale" type="number" step="0.1" value="2.4">
        </label>
        <label>settle before(s)
          <input id="pregrasp_settle" type="number" step="0.1" value="0.7">
        </label>
        <label>settle after(s)
          <input id="pregrasp_post" type="number" step="0.1" value="0.6">
        </label>
        <label>低位目标 cx
          <input id="preclose_center" type="number" step="0.001" value="0.90">
        </label>
        <label>下探跟踪面积
          <input id="preclose_area" type="number" step="0.001" value="0.095">
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
let statusInFlight = false;

function noticeBox() {
  let box = $('notice');
  if (!box) {
    box = document.createElement('div');
    box.id = 'notice';
    box.style.margin = '12px 12px 0';
    box.style.padding = '10px 12px';
    box.style.border = '1px solid var(--line)';
    box.style.borderRadius = '6px';
    box.style.background = '#f9fafb';
    box.style.fontWeight = '650';
    box.style.fontSize = '14px';
    $('log').parentNode.insertBefore(box, $('log'));
  }
  return box;
}

function showNotice(text, kind = 'info') {
  const box = noticeBox();
  box.textContent = text;
  box.style.color = kind === 'error' ? 'var(--danger)' : (kind === 'ok' ? 'var(--ok)' : 'var(--text)');
  box.style.borderColor = kind === 'error' ? 'rgba(201,54,54,.35)' : (kind === 'ok' ? 'rgba(23,138,85,.35)' : 'var(--line)');
  box.style.background = kind === 'error' ? '#fff5f5' : (kind === 'ok' ? '#f0fbf5' : '#f9fafb');
}

function params() {
  const yoloModel = $('yolo_model').value;
  return {
    target_class: $('target_class').value,
    yolo_model: yoloModel,
    yolo_classes: yoloModel === 'tongji' ? ['gray', 'yellow', 'grass', 'blue'] : ['red', 'green', 'blue'],
    yolo_conf: Number($('yolo_conf').value),
    center: Number($('center').value),
    center_tolerance: Number($('center_tolerance').value),
    target_area: Number($('target_area').value),
    area_tolerance: Number($('area_tolerance').value),
    use_depth: $('use_depth').value === 'true',
    depth_topic: $('depth_topic').value,
    target_depth: Number($('target_depth').value),
    depth_tolerance: Number($('depth_tolerance').value),
    depth_roi: Number($('depth_roi').value),
    pick_attempts: Number($('pick_attempts').value),
    gripper_gap: Number($('gripper_gap').value),
    empty_close: Number($('empty_close').value),
    gripper_delay: Number($('gripper_delay').value),
    max_linear: Number($('max_linear').value),
    max_angular: Number($('max_angular').value),
    visual_period: Number($('visual_period').value),
    cmd_pulse: Number($('cmd_pulse').value),
    adaptive_timing: $('adaptive_timing').value === 'true',
    min_period: Number($('min_period').value),
    max_period: Number($('max_period').value),
    period_scale: Number($('period_scale').value),
    pregrasp_scale: Number($('pregrasp_scale').value),
    pregrasp_settle: Number($('pregrasp_settle').value),
    pregrasp_post: Number($('pregrasp_post').value),
    preclose_center: Number($('preclose_center').value),
    preclose_area: Number($('preclose_area').value),
    control_mode: $('control_mode').value
  };
}

async function api(path, body) {
  const controller = new AbortController();
  const timeoutMs = path.startsWith('/api/status') ? 30000 : 65000;
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  const options = body ? {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body),
    signal: controller.signal
  } : {signal: controller.signal};
  try {
    const res = await fetch(path, options);
    const data = await res.json().catch(() => ({ok: false, error: res.statusText || 'non-json response'}));
    if (!res.ok || !data.ok) throw new Error(data.error || res.statusText);
    return data;
  } catch (err) {
    if (err.name === 'AbortError') {
      throw new Error(path.startsWith('/api/status') ? '状态查询超时' : '命令执行超时');
    }
    throw err;
  } finally {
    clearTimeout(timer);
  }
}

function setBusy(button, busy) {
  button.disabled = busy;
}

function appendLog(text) {
  $('log').textContent = text || '';
  $('log').scrollTop = $('log').scrollHeight;
}

function setStreamMessage(text, kind = 'info') {
  const box = $('streamMessage');
  if (!box) return;
  box.textContent = text || '';
  box.classList.toggle('hidden', !text);
  box.classList.toggle('error', kind === 'error');
}

async function refreshStatus(options = {}) {
  if (statusInFlight) {
    if (options.force) showNotice('状态仍在查询中，请稍等几秒。', 'info');
    return;
  }
  statusInFlight = true;
  try {
    const data = await api(options.force ? '/api/status?force=1' : '/api/status');
    $('robot').textContent = '机器人: ' + (data.status_refreshing ? '查询中' : (data.robot ? '在线' : '离线'));
    $('vision').textContent = '视觉: ' + (data.yolo ? '运行中' : '未启动');
    $('pick').textContent = '抓取: ' + (data.pick ? '运行中' : '空闲');
    const processText = data.process_summary ? ('\n' + data.process_summary) : '';
    if (data.log || processText) appendLog((data.log || '') + processText);
    return data;
  } catch (err) {
    const message = String(err.message || err);
    if (message.includes('状态查询超时') || message.toLowerCase().includes('abort')) {
      appendLog('状态查询超时。抓取/视觉进程可能仍在运行；稍后点“检测连接”查看日志。');
      if (!options.quiet) {
        showNotice('状态查询超时，不代表抓取已经停止。', 'info');
      }
    } else {
      $('robot').textContent = '机器人: 离线';
      appendLog(message);
      if (!options.quiet) {
        showNotice('连接机器人失败：' + message, 'error');
      }
    }
  } finally {
    statusInFlight = false;
  }
}

$('check').onclick = () => refreshStatus({force: true});
$('visionStart').onclick = async () => {
  setBusy($('visionStart'), true);
  showNotice('正在启动视觉并清理旧进程...', 'info');
  try {
    const data = await api('/api/start_vision', params());
    showNotice('视觉启动命令已执行：' + (data.message || ''), 'ok');
    refreshStatus({quiet: true});
  }
  catch (err) { showNotice('启动视觉失败：' + String(err.message || err), 'error'); appendLog(String(err.message || err)); }
  finally { setBusy($('visionStart'), false); }
};
$('visionStop').onclick = async () => {
  setBusy($('visionStop'), true);
  showNotice('正在停止视觉进程...', 'info');
  try {
    const data = await api('/api/stop_vision');
    showNotice('视觉已停止：' + (data.message || ''), 'ok');
    refreshStatus({quiet: true});
  }
  catch (err) { showNotice('停止视觉失败：' + String(err.message || err), 'error'); appendLog(String(err.message || err)); }
  finally { setBusy($('visionStop'), false); }
};
$('streamStart').onclick = () => {
  startStream();
};
$('streamStop').onclick = async () => {
  streamOn = false;
  $('stream').removeAttribute('src');
  setStreamMessage('实时画面已关闭。再次打开前请确认已连接 HW-9E5ACFD8。', 'info');
  try {
    const data = await api('/api/stop_stream', {});
    showNotice('实时画面已关闭：' + (data.message || ''), 'ok');
  } catch (err) {
    showNotice('本地画面已关闭；后台流关闭返回异常：' + String(err.message || err), 'info');
  }
  refreshStatus({quiet: true});
};

async function startStream() {
  if (streamOn) {
    streamOn = false;
    $('stream').removeAttribute('src');
    await api('/api/stop_stream', {}).catch(() => null);
    showNotice('正在替换旧视频流...', 'info');
  }
  setBusy($('streamStart'), true);
  setStreamMessage('正在检测机器人连接...', 'info');
  showNotice('正在检测机器人连接并打开实时画面...', 'info');
  try {
    const status = await api('/api/status?force=1');
    $('robot').textContent = '机器人: ' + (status.robot ? '在线' : '离线');
    $('vision').textContent = '视觉: ' + (status.yolo ? '运行中' : '未启动');
    $('pick').textContent = '抓取: ' + (status.pick ? '运行中' : '空闲');
    if (!status.robot) {
      const msg = '机器人未连接。请先把电脑 WiFi 切到 HW-9E5ACFD8，密码 hiwonder，然后再点“打开画面”。';
      setStreamMessage(msg, 'error');
      showNotice(msg, 'error');
      appendLog((status.log || msg) + '\n' + (status.process_summary || ''));
      return;
    }
    streamOn = true;
    if (!status.pick && !status.yolo) {
      setStreamMessage('机器人已连接，正在启动视觉节点...', 'info');
      const vision = await api('/api/start_vision', params());
      showNotice('视觉已准备：' + (vision.message || ''), 'ok');
    }
    setStreamMessage('正在连接机器人摄像头和 YOLO 画面...', 'info');
    $('stream').src = '/stream.mjpg?ts=' + Date.now();
  } catch (err) {
    streamOn = false;
    $('stream').removeAttribute('src');
    const msg = '打开画面失败：' + String(err.message || err);
    setStreamMessage(msg, 'error');
    showNotice(msg, 'error');
    appendLog(String(err.message || err));
  } finally {
    setBusy($('streamStart'), false);
  }
}

$('stream').onload = () => {
  if (streamOn) {
    setStreamMessage('', 'info');
    showNotice('实时画面已打开。', 'ok');
  }
};
$('stream').onerror = () => {
  if (!streamOn) return;
  streamOn = false;
  $('stream').removeAttribute('src');
  const msg = '画面流打开失败。请确认已连接 HW-9E5ACFD8，并点击“检测连接”查看 camera/yolo 日志。';
  setStreamMessage(msg, 'error');
  showNotice(msg, 'error');
  refreshStatus({quiet: true, force: true});
};
$('reset').onclick = async () => {
  setBusy($('reset'), true);
  streamOn = false;
  $('stream').removeAttribute('src');
  setStreamMessage('正在重置本地视频流和机器人端 YOLO/抓取进程...', 'info');
  showNotice('正在重置通信和机器人端残留进程...', 'info');
  try {
    await api('/api/stop_stream', {}).catch(() => null);
    const data = await api('/api/reset', {});
    showNotice('重置完成：' + (data.message || ''), 'ok');
    setStreamMessage('重置完成。需要画面时再点“打开画面”。', 'info');
    refreshStatus({quiet: true, force: true});
  } catch (err) {
    const msg = '重置失败：' + String(err.message || err);
    showNotice(msg, 'error');
    setStreamMessage(msg, 'error');
    appendLog(String(err.message || err));
  } finally {
    setBusy($('reset'), false);
  }
};
$('pickStart').onclick = async () => {
  setBusy($('pickStart'), true);
  showNotice('正在启动抓取：会先清理旧视觉/抓取进程...', 'info');
  streamOn = false;
  $('stream').removeAttribute('src');
  setStreamMessage('正在启动抓取链路，启动后会自动重新打开实时画面。', 'info');
  try {
    const data = await api('/api/start_pick', params());
    showNotice('抓取已启动：' + (data.message || ''), 'ok');
    streamOn = true;
    setStreamMessage('抓取已启动，正在重新连接 YOLO 实时画面...', 'info');
    $('stream').src = '/stream.mjpg?ts=' + Date.now();
    refreshStatus({quiet: true});
  }
  catch (err) { showNotice('启动抓取失败：' + String(err.message || err), 'error'); appendLog(String(err.message || err)); }
  finally { setBusy($('pickStart'), false); }
};
$('pickStop').onclick = async () => {
  setBusy($('pickStop'), true);
  showNotice('正在结束抓取并清理 competition/yolo/launch 进程...', 'info');
  try {
    const data = await api('/api/stop_pick');
    showNotice('抓取已结束：' + (data.message || ''), 'ok');
    refreshStatus({quiet: true});
  }
  catch (err) { showNotice('结束抓取失败：' + String(err.message || err), 'error'); appendLog(String(err.message || err)); }
  finally { setBusy($('pickStop'), false); }
};
$('drop').onclick = async () => {
  setBusy($('drop'), true);
  showNotice('正在执行放下动作...', 'info');
  try {
    const data = await api('/api/drop', params());
    showNotice('放下动作已执行：' + (data.message || ''), 'ok');
    refreshStatus({quiet: true});
  }
  catch (err) { showNotice('放下失败：' + String(err.message || err), 'error'); appendLog(String(err.message || err)); }
  finally { setBusy($('drop'), false); }
};

setInterval(refreshStatus, 5000);
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


def tcp_reachable(host: str, port: int = 22, timeout: float = 0.8) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


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
            channel = stdout.channel
            deadline = time.time() + max(1, timeout)
            out_chunks = []
            err_chunks = []
            while True:
                while channel.recv_ready():
                    out_chunks.append(channel.recv(65536))
                while channel.recv_stderr_ready():
                    err_chunks.append(channel.recv_stderr(65536))
                if channel.exit_status_ready():
                    while channel.recv_ready():
                        out_chunks.append(channel.recv(65536))
                    while channel.recv_stderr_ready():
                        err_chunks.append(channel.recv_stderr(65536))
                    rc = channel.recv_exit_status()
                    out = b''.join(out_chunks).decode('utf-8', errors='replace')
                    err = b''.join(err_chunks).decode('utf-8', errors='replace')
                    return rc, out, err
                if time.time() >= deadline:
                    channel.close()
                    out = b''.join(out_chunks).decode('utf-8', errors='replace')
                    err = b''.join(err_chunks).decode('utf-8', errors='replace')
                    return 124, out, (err + f'\ncommand timed out after {timeout}s').strip()
                time.sleep(0.05)
        finally:
            client.close()

    def docker_stream(self, script: str):
        command = f'docker exec -u ubuntu {shlex.quote(self.container)} bash -lc {shlex.quote(script)}'
        client = self.ssh()
        stdin, stdout, stderr = client.exec_command(command, timeout=None)
        return client, stdout.channel


def ros_prefix() -> str:
    return 'source /home/ubuntu/ros2_ws/.robotrc; '


def shell_kill_helpers() -> str:
    return r'''
matching_pids() {
  PATTERN="$1"
  ps -eo pid=,comm=,args= 2>/dev/null | while read -r PID COMM ARGS; do
    [ -z "$PID" ] && continue
    [ "$PID" = "$$" ] && continue
    [ "$PID" = "$PPID" ] && continue
    case "$COMM" in
      bash|sh|dash|timeout|pgrep|grep|ps) continue ;;
    esac
    printf '%s\n' "$ARGS" | grep -F -- "$PATTERN" >/dev/null 2>&1 && printf '%s\n' "$PID"
  done
}
kill_matching() {
  PATTERN="$1"
  SIGNAL="${2:-TERM}"
  for PID in $(matching_pids "$PATTERN"); do
    kill "-$SIGNAL" "$PID" 2>/dev/null || true
  done
}
force_kill_matching() {
  PATTERN="$1"
  kill_matching "$PATTERN" TERM
  sleep 0.2
  kill_matching "$PATTERN" KILL
}
count_matching() {
  PATTERN="$1"
  COUNT=0
  for PID in $(matching_pids "$PATTERN"); do
    COUNT=$((COUNT + 1))
  done
  echo "$COUNT"
}
reset_ui_processes() {
  timeout 2s ros2 service call /competition_pick_place/stop std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
  force_kill_matching "competition_node"
  force_kill_matching "ros2 launch competition_pick_place"
  force_kill_matching "yolov11_node"
  force_kill_matching "python3 -u /tmp/ui_yolo_runner.py"
  force_kill_matching "local_yolo_overlay_stream"
  rm -f /tmp/ui_pick.log /tmp/ui_yolo.log
}
'''


def camera_helpers() -> str:
    return f'''
CAMERA_TOPIC="{CAMERA_TOPIC}"
camera_frame_ready() {{
  timeout 4s ros2 topic echo --once "$CAMERA_TOPIC" >/dev/null 2>&1
}}
ensure_camera() {{
  if camera_frame_ready; then
    echo "camera_ready=1"
    return 0
  fi
  force_kill_matching "depth_camera.launch.py"
  force_kill_matching "camera_publisher"
  force_kill_matching "ascamera"
  nohup ros2 launch peripherals depth_camera.launch.py >/tmp/ui_camera.log 2>&1 &
  echo "camera_pid=$!"
  for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
    if camera_frame_ready; then
      echo "camera_ready=1"
      return 0
    fi
    sleep 1
  done
  echo "camera_ready=0"
  echo "__CAMERA_LOG__"
  tail -n 80 /tmp/ui_camera.log 2>/dev/null || true
  return 1
}}
'''


def require_ok(rc: int, out: str, err: str) -> None:
    if rc != 0:
        raise RuntimeError((err or out or f'command failed rc={rc}').strip())


def reset_robot(robot: Robot) -> str:
    script = ros_prefix() + shell_kill_helpers() + '''
set +e
reset_ui_processes
sleep 0.5
echo "robot ui reset done"
echo "pick_launch_processes=$(count_matching 'ros2 launch competition_pick_place')"
echo "competition_processes=$(count_matching 'competition_node')"
echo "yolo_processes=$(count_matching 'yolov11_node')"
echo "vision_runner_processes=$(count_matching 'python3 -u /tmp/ui_yolo_runner.py')"
echo "stream_processes=$(count_matching 'local_yolo_overlay_stream')"
'''
    rc, out, err = robot.docker_exec(script, timeout=10)
    require_ok(rc, out, err)
    return out.strip()


def start_vision(robot: Robot, params: dict) -> str:
    yolo_conf = float(params.get('yolo_conf', 0.20))
    yolo_model = str(params.get('yolo_model', 'tongji')).strip() or 'tongji'
    yolo_classes = params.get('yolo_classes') or ['gray', 'yellow', 'grass', 'blue']
    if isinstance(yolo_classes, str):
        yolo_classes = [part.strip() for part in yolo_classes.split(',') if part.strip()]
    yolo_classes = [str(item).strip() for item in yolo_classes if str(item).strip()]
    if not yolo_classes:
        yolo_classes = ['gray', 'yellow', 'grass', 'blue']
    model_literal = repr(yolo_model)
    classes_literal = repr(yolo_classes)
    script = ros_prefix() + shell_kill_helpers() + camera_helpers() + f'''
set +e
reset_ui_processes
ensure_camera || exit 20
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
                {{'classes': {classes_literal}}},
                {{'model': {model_literal}, 'conf': {yolo_conf:.3f}, 'start': True}},
            ],
        )
    ])
)
sys.exit(launch_service.run())
PY
nohup python3 -u /tmp/ui_yolo_runner.py > {REMOTE_YOLO_LOG} 2>&1 &
echo "vision_pid=$!"
for i in 1 2 3 4 5 6 7 8 9 10; do
  ros2 topic list 2>/dev/null | grep -F "{DETECTION_TOPIC}" >/dev/null && break
  sleep 1
done
echo "vision_runner_processes=$(count_matching 'python3 -u /tmp/ui_yolo_runner.py')"
echo "yolo_processes=$(count_matching 'yolov11_node')"
echo "pick_processes=$(count_matching 'competition_node')"
echo "detection_topic_present=$(ros2 topic list 2>/dev/null | grep -F "{DETECTION_TOPIC}" >/dev/null && echo 1 || echo 0)"
'''
    rc, out, err = robot.docker_exec(script, timeout=45)
    require_ok(rc, out, err)
    return out.strip()


def stop_vision(robot: Robot) -> str:
    script = ros_prefix() + shell_kill_helpers() + f'''
set +e
force_kill_matching "yolov11_node"
force_kill_matching "python3 -u /tmp/ui_yolo_runner.py"
force_kill_matching "local_yolo_overlay_stream"
sleep 1
echo "vision stopped"
echo "vision_runner_processes=$(count_matching 'python3 -u /tmp/ui_yolo_runner.py')"
echo "yolo_processes=$(count_matching 'yolov11_node')"
echo "stream_processes=$(count_matching 'local_yolo_overlay_stream')"
'''
    rc, out, err = robot.docker_exec(script, timeout=12)
    require_ok(rc, out, err)
    return out.strip()


def start_pick(robot: Robot, params: dict) -> str:
    target = shlex.quote(str(params.get('target_class', 'grass')))
    control_mode = shlex.quote(str(params.get('control_mode', 'mpc')))
    yolo_model = shlex.quote(str(params.get('yolo_model', 'tongji')).strip() or 'tongji')
    yolo_classes = params.get('yolo_classes') or ['gray', 'yellow', 'grass', 'blue']
    if isinstance(yolo_classes, str):
        yolo_classes = [part.strip() for part in yolo_classes.split(',') if part.strip()]
    yolo_classes = [str(item).strip() for item in yolo_classes if str(item).strip()] or ['gray', 'yellow', 'grass', 'blue']
    yolo_classes_arg = shlex.quote(','.join(yolo_classes))
    center = float(params.get('center', 0.50))
    center_tol = float(params.get('center_tolerance', 0.028))
    target_area = float(params.get('target_area', 0.043))
    area_tol = float(params.get('area_tolerance', 0.010))
    use_depth = str(bool(params.get('use_depth', True))).lower()
    depth_topic = shlex.quote(str(params.get('depth_topic', '/ascamera/camera_publisher/depth0/image_raw')).strip())
    target_depth = float(params.get('target_depth', 0.32))
    depth_tolerance = float(params.get('depth_tolerance', 0.025))
    depth_roi = float(params.get('depth_roi', 0.45))
    pick_attempts = max(1, int(params.get('pick_attempts', 3)))
    gripper_gap = max(0, int(params.get('gripper_gap', 30)))
    empty_close = int(params.get('empty_close', 500))
    gripper_delay = float(params.get('gripper_delay', 0.35))
    max_linear = float(params.get('max_linear', 0.09))
    max_angular = float(params.get('max_angular', 0.35))
    visual_period = float(params.get('visual_period', 0.10))
    cmd_pulse = float(params.get('cmd_pulse', 0.04))
    adaptive_timing = str(bool(params.get('adaptive_timing', True))).lower()
    min_period = float(params.get('min_period', 0.035))
    max_period = float(params.get('max_period', 0.16))
    period_scale = float(params.get('period_scale', 1.05))
    pregrasp_scale = float(params.get('pregrasp_scale', 2.4))
    pregrasp_settle = float(params.get('pregrasp_settle', 0.7))
    pregrasp_post = float(params.get('pregrasp_post', 0.6))
    preclose_center = float(params.get('preclose_center', 0.90))
    preclose_area = float(params.get('preclose_area', 0.095))
    script = ros_prefix() + shell_kill_helpers() + camera_helpers() + f'''
set +e
reset_ui_processes
ensure_camera || exit 20
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
  yolo_model:={yolo_model} \\
  yolo_classes:={yolo_classes_arg} \\
  yolo_conf:={float(params.get('yolo_conf', 0.20)):.3f} \\
  min_score:={float(params.get('yolo_conf', 0.20)):.3f} \\
  init_action:=navigation_pick_init_ai \\
  pick_action:=navigation_pick_ai \\
  search_timeout:=12.0 \\
  align_timeout:=45.0 \\
  wait_for_detection_stream:=true \\
  detection_stream_timeout:=20.0 \\
  detection_ready_min_messages:=1 \\
  wait_for_target_before_search:=true \\
  use_depth_distance:={use_depth} \\
  depth_topic:={depth_topic} \\
  camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info \\
  use_robot_frame_distance:=true \\
  camera_tilt_deg:=45.0 \\
  camera_height_m:=0.22 \\
  camera_offset_x_m:=0.06 \\
  depth_roi_pixels:=15 \\
  depth_stale_seconds:=0.800 \\
  depth_unit_scale:=0.001 \\
  depth_roi_scale:={depth_roi:.3f} \\
  depth_sample_grid:=5 \\
  depth_min_valid_samples:=20 \\
  depth_min_m:=0.080 \\
  depth_max_m:=1.500 \\
  pick_target_depth_m:={target_depth:.3f} \\
  pick_target_robot_x_m:={target_depth:.3f} \\
  pick_target_robot_y_m:=0.0 \\
  pick_robot_x_tolerance_m:={depth_tolerance:.3f} \\
  pick_robot_y_tolerance_m:=0.025 \\
  pick_depth_tolerance_m:={depth_tolerance:.3f} \\
  pick_preclose_target_depth_m:=-1.0 \\
  desired_center_x_ratio:={center:.4f} \\
  center_tolerance_ratio:={center_tol:.4f} \\
  pick_target_area_ratio:={target_area:.4f} \\
  area_tolerance_ratio:={area_tol:.4f} \\
  stable_frames:=4 \\
  control_mode:={control_mode} \\
  closed_loop_pick:=true \\
  pick_visual_servo_timeout:=5.0 \\
  visual_servo_period:={visual_period:.3f} \\
  visual_servo_command_seconds:={cmd_pulse:.3f} \\
  adaptive_servo_timing:={adaptive_timing} \\
  visual_servo_min_period:={min_period:.3f} \\
  visual_servo_max_period:={max_period:.3f} \\
  visual_servo_period_scale:={period_scale:.3f} \\
  require_fresh_detection_for_control:=true \\
  pick_pregrasp_visual_servo:=true \\
  pick_pregrasp_time_scale:={pregrasp_scale:.3f} \\
  pick_pregrasp_min_step_seconds:=0.800 \\
  pick_pregrasp_settle_seconds:={pregrasp_settle:.3f} \\
  pick_pregrasp_post_step_seconds:={pregrasp_post:.3f} \\
  pick_preclose_required:=false \\
  pick_preclose_fail_on_timeout:=false \\
  pick_preclose_center_x_ratio:={preclose_center:.4f} \\
  pick_preclose_target_area_ratio:={preclose_area:.4f} \\
  pick_preclose_center_tolerance_ratio:=0.0650 \\
  pick_preclose_area_tolerance_ratio:=0.0200 \\
  pick_preclose_stable_frames:=1 \\
  pick_retry_attempts:={pick_attempts} \\
  grasp_check_enabled:=true \\
  gripper_state_topic:=/controller_manager/servo_states \\
  gripper_servo_id:=10 \\
  gripper_empty_close_position:={empty_close} \\
  gripper_grasp_min_gap:={gripper_gap} \\
  gripper_check_delay:={gripper_delay:.3f} \\
  gripper_feedback_timeout:=2.0 \\
  angular_k:=0.80 \\
  angular_sign:=-1.0 \\
  max_linear_speed:={max_linear:.4f} \\
  max_angular_speed:={max_angular:.4f} \\
  search_angular_speed:=0.12 \\
  mpc_horizon:=8 \\
  mpc_dt:={visual_period:.3f} \\
  mpc_center_response:=1.05 \\
  mpc_area_response:=0.24 \\
  mpc_center_weight:=8.0 \\
  mpc_area_weight:=26.0 \\
  mpc_velocity_weight:=0.08 \\
  mpc_delta_weight:=0.16 \\
  mpc_terminal_weight:=2.2 \\
  mpc_center_gate_ratio:=0.10 > {REMOTE_PICK_LOG} 2>&1 &
echo "pick_pid=$!"
for i in 1 2 3 4 5 6; do
  PCOUNT=$(count_matching 'competition_node')
  [ "$PCOUNT" -gt 0 ] && break
  sleep 1
done
echo "pick_launch_processes=$(count_matching 'ros2 launch competition_pick_place')"
echo "competition_processes=$(count_matching 'competition_node')"
echo "yolo_processes=$(count_matching 'yolov11_node')"
echo "vision_runner_processes=$(count_matching 'python3 -u /tmp/ui_yolo_runner.py')"
echo "stream_processes=$(count_matching 'local_yolo_overlay_stream')"
'''
    rc, out, err = robot.docker_exec(script, timeout=45)
    require_ok(rc, out, err)
    return out.strip()


def stop_pick(robot: Robot) -> str:
    script = ros_prefix() + shell_kill_helpers() + '''
set +e
reset_ui_processes
sleep 1
echo "pick stopped"
echo "pick_launch_processes=$(count_matching 'ros2 launch competition_pick_place')"
echo "competition_processes=$(count_matching 'competition_node')"
echo "yolo_processes=$(count_matching 'yolov11_node')"
echo "vision_runner_processes=$(count_matching 'python3 -u /tmp/ui_yolo_runner.py')"
echo "stream_processes=$(count_matching 'local_yolo_overlay_stream')"
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
    if not tcp_reachable(robot.host, 22, timeout=0.8):
        return {
            'robot': False,
            'yolo': False,
            'pick': False,
            'log': 'Robot SSH is not reachable. Connect WiFi to HW-9E5ACFD8 before using robot controls.',
            'process_summary': 'robot offline',
            'process_count': 0,
            'ssid': current_ssid(),
        }
    script = ros_prefix() + shell_kill_helpers() + f'''
set +e
echo "__TOPICS__"
ros2 topic list 2>/dev/null | grep -E "ascamera|object_detect|yolo|controller/cmd_vel" || true
echo "__PROCS__"
for PATTERN in "yolov11_node" "python3 -u /tmp/ui_yolo_runner.py" "competition_node" "ros2 launch competition_pick_place" "local_yolo_overlay_stream"; do
  for PID in $(matching_pids "$PATTERN"); do
    ps -p "$PID" -o pid=,args= 2>/dev/null || true
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
    yolo = 'yolov11_node' in procs
    pick = 'competition_node' in procs
    combined_log = ''
    if yolo_log:
        combined_log += '== YOLO ==\n' + yolo_log[-2200:] + '\n'
    if log:
        combined_log += '== PICK ==\n' + log[-3000:]
    topic_lines = [line.strip() for line in topics.splitlines() if line.strip()]
    process_lines = [line.strip() for line in procs.splitlines() if line.strip()]
    summary_parts = []
    if topic_lines:
        summary_parts.append('== TOPICS ==\n' + '\n'.join(topic_lines))
    summary_parts.append(
        '== PROCESSES ==\n' + '\n'.join(process_lines)
        if process_lines
        else '== PROCESSES ==\nno robot-side UI/yolo/pick processes'
    )
    process_summary = '\n'.join(summary_parts)
    return {
        'robot': True,
        'yolo': yolo,
        'pick': pick,
        'log': combined_log[-5500:],
        'process_summary': process_summary,
        'process_count': len(process_lines),
        'ssid': current_ssid(),
    }


def make_stream_script(fps: int = 8, quality: int = 78) -> str:
    script = STREAM_SCRIPT.replace('__CAMERA_TOPIC__', CAMERA_TOPIC)
    script = script.replace('__DETECTION_TOPIC__', DETECTION_TOPIC)
    script = script.replace('__FPS__', str(fps))
    script = script.replace('__JPEG_QUALITY__', str(quality))
    script = script.replace('__STALE_SECONDS__', '0.8')
    return (
        ros_prefix()
        + shell_kill_helpers()
        + camera_helpers()
        + 'ensure_camera || exit 20\n'
        + 'python3 -u - <<\'PY\'\n'
        + script
        + '\nPY\n'
    )


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
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_error_json(self, exc: Exception, status: int = 500) -> None:
        self.send_json({'ok': False, 'error': str(exc)}, status)

    def add_local_status(self, data: dict) -> dict:
        data = dict(data)
        with self.server.stream_lock:
            active_streams = self.server.active_streams
        data['active_streams'] = active_streams
        summary = (data.get('process_summary') or '').strip()
        local_line = f'local_ui_streams={active_streams}'
        if local_line not in summary:
            data['process_summary'] = f'{summary}\n{local_line}' if summary else local_line
        return data

    def cached_status_payload(self) -> dict:
        with self.server.status_cache_lock:
            data = dict(self.server.status_cache)
            cache_time = self.server.status_cache_time
        data = self.add_local_status(data)
        data['status_age_seconds'] = round(max(0.0, time.time() - cache_time), 2) if cache_time else None
        return data

    def invalidate_status_cache(self) -> None:
        with self.server.status_cache_lock:
            self.server.status_cache_time = 0.0

    def handle_status(self, force: bool = False) -> None:
        now = time.time()
        with self.server.status_cache_lock:
            cache_time = self.server.status_cache_time
        if not force and cache_time and now - cache_time <= self.server.status_cache_ttl:
            data = self.cached_status_payload()
            data['status_cached'] = True
            data['status_refreshing'] = False
            self.send_json({'ok': True, **data})
            return

        acquired = self.server.status_lock.acquire(timeout=12.0) if force else self.server.status_lock.acquire(blocking=False)
        if not acquired:
            data = self.cached_status_payload()
            data['status_cached'] = True
            data['status_refreshing'] = True
            if not data.get('log'):
                data['log'] = 'Status refresh is already running; showing the last cached robot state.'
            self.send_json({'ok': True, **data})
            return

        try:
            try:
                fresh = get_status(self.robot)
            except Exception as exc:
                fresh = {
                    'robot': False,
                    'yolo': False,
                    'pick': False,
                    'log': f'Status check failed: {exc}',
                    'process_summary': 'status check failed',
                    'process_count': 0,
                    'ssid': current_ssid(),
                    'status_error': str(exc),
                }
            with self.server.status_cache_lock:
                self.server.status_cache = dict(fresh)
                self.server.status_cache_time = time.time()
            data = self.add_local_status(fresh)
            data['status_cached'] = False
            data['status_refreshing'] = False
            data['status_age_seconds'] = 0.0
            self.send_json({'ok': True, **data})
        finally:
            self.server.status_lock.release()

    def stop_active_stream(self) -> int:
        with self.server.stream_lock:
            old_client = self.server.current_stream_client
            was_active = self.server.active_streams
            self.server.current_stream_client = None
            self.server.stream_generation += 1
            self.server.active_streams = 0
        if old_client is not None:
            try:
                old_client.close()
            except Exception:
                pass
        return was_active

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/':
            body = INDEX_HTML.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == '/api/status':
            query = parse_qs(parsed.query)
            self.handle_status(force=query.get('force', ['0'])[0] in {'1', 'true', 'yes'})
            return
        if parsed.path == '/stream.mjpg':
            self.stream_mjpg(parsed)
            return
        self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/stop_stream':
            stopped = self.stop_active_stream()
            self.send_json({'ok': True, 'message': f'local stream stopped; previous_active_streams={stopped}'})
            return
        command_paths = {
            '/api/reset',
            '/api/start_vision',
            '/api/stop_vision',
            '/api/start_pick',
            '/api/stop_pick',
            '/api/drop',
        }
        if parsed.path not in command_paths:
            self.send_error(404)
            return
        if not self.server.command_lock.acquire(blocking=False):
            elapsed = time.time() - self.server.command_started_at if self.server.command_started_at else 0.0
            self.send_error_json(
                RuntimeError(f'Another robot command is still running: {self.server.current_command or "unknown"} ({elapsed:.1f}s). Click 重置通信 if it does not finish.'),
                HTTPStatus.CONFLICT,
            )
            return
        try:
            self.server.current_command = parsed.path
            self.server.command_started_at = time.time()
            payload = self.read_json()
            if parsed.path in {'/api/reset', '/api/start_pick', '/api/stop_pick', '/api/stop_vision'}:
                self.stop_active_stream()
            if parsed.path == '/api/reset':
                message = reset_robot(self.robot)
            elif parsed.path == '/api/start_vision':
                message = start_vision(self.robot, payload)
            elif parsed.path == '/api/stop_vision':
                message = stop_vision(self.robot)
            elif parsed.path == '/api/start_pick':
                message = start_pick(self.robot, payload)
            elif parsed.path == '/api/stop_pick':
                message = stop_pick(self.robot)
            elif parsed.path == '/api/drop':
                message = drop_block(self.robot)
            self.invalidate_status_cache()
            self.send_json({'ok': True, 'message': message})
        except Exception as exc:
            self.send_error_json(exc, HTTPStatus.BAD_GATEWAY)
        finally:
            self.server.current_command = ''
            self.server.command_started_at = 0.0
            self.server.command_lock.release()

    def write_multipart_frame(self, payload: bytes, mime: str) -> None:
        self.wfile.write(b'--frame\r\n')
        self.wfile.write(f'Content-Type: {mime}\r\n'.encode('ascii'))
        self.wfile.write(f'Content-Length: {len(payload)}\r\n\r\n'.encode('ascii'))
        self.wfile.write(payload)
        self.wfile.write(b'\r\n')
        self.wfile.flush()

    def svg_frame(self, message: str) -> bytes:
        safe = (
            message.replace('&', '&amp;')
            .replace('<', '&lt;')
            .replace('>', '&gt;')
            .replace('"', '&quot;')
        )
        lines = safe.splitlines()[:5] or ['waiting']
        tspans = []
        start_y = 210 - max(0, len(lines) - 1) * 18
        for index, line in enumerate(lines):
            tspans.append(f'<text x="480" y="{start_y + index * 42}" text-anchor="middle">{line}</text>')
        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="960" height="540" viewBox="0 0 960 540">
<rect width="960" height="540" fill="#111827"/>
<rect x="80" y="120" width="800" height="300" rx="16" fill="#182234" stroke="#334155"/>
<g fill="#d5e1f3" font-family="Microsoft YaHei, Segoe UI, sans-serif" font-size="28">{''.join(tspans)}</g>
<text x="480" y="380" text-anchor="middle" fill="#93a4b8" font-family="Consolas, monospace" font-size="18">HW-9E5ACFD8 / hiwonder</text>
</svg>'''
        return svg.encode('utf-8')

    def stream_mjpg(self, parsed) -> None:
        query = parse_qs(parsed.query)
        fps = int(query.get('fps', ['8'])[0])
        quality = int(query.get('quality', ['78'])[0])
        client = None
        old_client = None
        with self.server.stream_lock:
            old_client = self.server.current_stream_client
            self.server.current_stream_client = None
            self.server.stream_generation += 1
            generation = self.server.stream_generation
            self.server.active_streams = 1
        if old_client is not None:
            try:
                old_client.close()
            except Exception:
                pass
        try:
            client, channel = self.robot.docker_stream(make_stream_script(fps=fps, quality=quality))
            with self.server.stream_lock:
                if generation != self.server.stream_generation:
                    return
                self.server.current_stream_client = client
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.end_headers()
            buffer = b''
            err_buffer = b''
            last_frame = 0.0
            last_notice = 0.0
            self.write_multipart_frame(self.svg_frame('正在连接机器人摄像头和 YOLO 画面...'), 'image/svg+xml')
            while True:
                with self.server.stream_lock:
                    if generation != self.server.stream_generation:
                        break
                if channel.recv_ready():
                    chunk = channel.recv(65536)
                    if not chunk:
                        break
                    buffer += chunk
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        if not line.startswith(b'__JPEG__'):
                            continue
                        jpg = base64.b64decode(line[len(b'__JPEG__'):].strip())
                        self.write_multipart_frame(jpg, 'image/jpeg')
                        last_frame = time.time()
                if channel.recv_stderr_ready():
                    err_buffer = (err_buffer + channel.recv_stderr(8192))[-1200:]
                if channel.exit_status_ready():
                    if time.time() - last_frame > 1.0:
                        detail = err_buffer.decode('utf-8', errors='replace').strip()
                        self.write_multipart_frame(
                            self.svg_frame('机器人视频流已结束。\n请点“重置通信”后重新打开画面。' + (f'\n{detail}' if detail else '')),
                            'image/svg+xml',
                        )
                    break
                now = time.time()
                if now - last_frame > 4.0 and now - last_notice > 4.0:
                    detail = err_buffer.decode('utf-8', errors='replace').strip()
                    self.write_multipart_frame(
                        self.svg_frame('等待机器人摄像头/YOLO 画面...\n如果长期停在这里，点“重置通信”。' + (f'\n{detail}' if detail else '')),
                        'image/svg+xml',
                    )
                    last_notice = now
                time.sleep(0.04)
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
            with self.server.stream_lock:
                if generation == self.server.stream_generation:
                    self.server.current_stream_client = None
                    self.server.active_streams = 0


def existing_ui_running(port: int) -> bool:
    try:
        with urlopen(f'http://127.0.0.1:{port}/api/status', timeout=2) as response:
            data = json.loads(response.read().decode('utf-8', errors='replace'))
            return response.status == 200 and bool(data.get('ok'))
    except Exception:
        return False


def find_free_port(preferred: int) -> int | None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(('127.0.0.1', preferred))
            return preferred
        except OSError:
            return None


def main():
    args = parse_args()
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    port = find_free_port(args.port)
    if port is None:
        if existing_ui_running(args.port):
            print(f'Robot YOLO control UI is already running: http://127.0.0.1:{args.port}', flush=True)
            print('Use the existing page, or stop the old robot_yolo_control_ui.py process before restarting.', flush=True)
            return
        raise RuntimeError(f'Port {args.port} is already in use by another process. Free it before starting the UI.')
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.robot = Robot(args.host, args.user, args.password, args.container)
    server.active_streams = 0
    server.current_stream_client = None
    server.stream_generation = 0
    server.stream_lock = threading.Lock()
    server.command_lock = threading.Lock()
    server.current_command = ''
    server.command_started_at = 0.0
    server.status_lock = threading.Lock()
    server.status_cache_lock = threading.Lock()
    server.status_cache_ttl = 2.0
    server.status_cache_time = 0.0
    server.status_cache = {
        'robot': False,
        'yolo': False,
        'pick': False,
        'log': 'Status has not been checked yet.',
        'process_summary': 'status not checked yet',
        'process_count': 0,
        'ssid': current_ssid(),
    }
    print(f'Robot YOLO control UI: http://127.0.0.1:{port}', flush=True)
    print('WiFi switching is manual; connect to HW-9E5ACFD8 before using robot controls.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
