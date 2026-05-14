#!/usr/bin/env python3
import base64
import csv
import json
import os
import platform
import re
import shlex
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:
    import paramiko
except Exception:  # pragma: no cover
    paramiko = None


ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = ROOT / 'datasets' / 'color_block_capture'
RAW_ROOT = DATA_ROOT / 'raw'
CACHE_ROOT = DATA_ROOT / '.cache'
MANIFEST = DATA_ROOT / 'manifest.csv'

ROBOT_SSID = 'HW-9E5ACFD8'
RESTORE_SSID = 'TJ-WIFI'
ROBOT_HOST = '192.168.149.1'
ROBOT_USER = 'pi'
ROBOT_PASSWORD = 'raspberrypi'
CAMERA_TOPIC = '/ascamera/camera_publisher/rgb0/image'

CLASSES = [
    {'id': 'red', 'name': '红色方块', 'color': '#d83a34', 'hint': '红色目标块，单独采集时只保留红块入镜。'},
    {'id': 'green', 'name': '绿色方块', 'color': '#2f9b57', 'hint': '绿色目标块，避免叶片、绿色标记等背景抢占画面。'},
    {'id': 'blue', 'name': '蓝色方块', 'color': '#286fcb', 'hint': '蓝色目标块，注意暗光下不要拍成黑色或紫色。'},
]

SCENES = [
    {
        'id': 'single_front_mid',
        'name': '单块正面中距',
        'mode': 'single',
        'target_per_class': 50,
        'guide': '只放当前颜色方块；方块完整入镜，位于画面中部；占画面宽度约 15%-35%；背景尽量接近比赛原料区。',
    },
    {
        'id': 'single_distance',
        'name': '距离变化',
        'mode': 'single',
        'target_per_class': 50,
        'guide': '只放当前颜色方块；近、中、远距离都要拍；保持清晰，不要运动模糊；方块不能被裁切。',
    },
    {
        'id': 'single_angle',
        'name': '角度变化',
        'mode': 'single',
        'target_per_class': 50,
        'guide': '只放当前颜色方块；从左前、右前、斜侧方向采集；允许看到侧面；不要让方块贴边。',
    },
    {
        'id': 'lighting_background',
        'name': '光照和背景变化',
        'mode': 'single',
        'target_per_class': 40,
        'guide': '只放当前颜色方块；覆盖阴影、反光、偏暗、偏亮、不同地面纹理；避免严重过曝。',
    },
    {
        'id': 'partial_occlusion',
        'name': '轻微遮挡/夹爪干扰',
        'mode': 'single',
        'target_per_class': 30,
        'guide': '只放当前颜色方块；可让夹爪边缘、手刚离开后的轻微遮挡入镜；遮挡不要超过方块面积 20%。',
    },
    {
        'id': 'mixed_all_colors',
        'name': '红绿蓝三色同框',
        'mode': 'mixed',
        'target_global': 80,
        'guide': '三种颜色同时入镜；每块都完整可见并彼此分开；多换排列顺序和距离。这一张会同时计入红、绿、蓝三类。',
    },
]

MANIFEST_FIELDS = [
    'id',
    'timestamp',
    'relative_path',
    'mode',
    'primary_class',
    'classes_present',
    'scene_id',
    'scene_name',
    'note',
]

REMOTE_CAPTURE_SCRIPT = r'''
import base64
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = "__CAMERA_TOPIC__"
TIMEOUT = __TIMEOUT__
JPEG_QUALITY = __JPEG_QUALITY__

class OneShot(Node):
    def __init__(self):
        super().__init__('local_ui_one_shot_capture')
        self.bridge = CvBridge()
        self.jpg = None
        self.sub = self.create_subscription(Image, TOPIC, self.cb, 1)

    def cb(self, msg):
        if self.jpg is not None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            self.jpg = encoded.tobytes()

rclpy.init()
node = OneShot()
deadline = time.time() + TIMEOUT
while rclpy.ok() and node.jpg is None and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

if node.jpg is None:
    print('__CAPTURE_ERROR__ timeout waiting for camera frame', flush=True)
else:
    print('__FRAME_BEGIN__', flush=True)
    sys.stdout.write(base64.b64encode(node.jpg).decode('ascii'))
    print('\n__FRAME_END__', flush=True)

node.destroy_node()
rclpy.shutdown()
'''


class CaptureStore:
    lock = threading.Lock()

    def __init__(self):
        DATA_ROOT.mkdir(parents=True, exist_ok=True)
        RAW_ROOT.mkdir(parents=True, exist_ok=True)
        CACHE_ROOT.mkdir(parents=True, exist_ok=True)
        if not MANIFEST.exists():
            with MANIFEST.open('w', newline='', encoding='utf-8') as f:
                csv.DictWriter(f, fieldnames=MANIFEST_FIELDS).writeheader()

    def rows(self):
        if not MANIFEST.exists():
            return []
        with MANIFEST.open('r', newline='', encoding='utf-8') as f:
            return list(csv.DictReader(f))

    def counts(self):
        rows = self.rows()
        class_ids = [c['id'] for c in CLASSES]
        counts = {
            cls: {
                'total': 0,
                'scenes': {scene['id']: 0 for scene in SCENES},
            }
            for cls in class_ids
        }
        mixed_count = 0
        total_images = len(rows)
        for row in rows:
            present = [x for x in row.get('classes_present', '').split('|') if x]
            scene_id = row.get('scene_id', '')
            if row.get('mode') == 'mixed':
                mixed_count += 1
            for cls in present:
                if cls not in counts:
                    continue
                counts[cls]['total'] += 1
                if scene_id in counts[cls]['scenes']:
                    counts[cls]['scenes'][scene_id] += 1
        targets = self.targets()
        for cls in class_ids:
            counts[cls]['target'] = targets['per_class_total']
            counts[cls]['remaining'] = max(0, targets['per_class_total'] - counts[cls]['total'])
        return {
            'total_images': total_images,
            'mixed_count': mixed_count,
            'mixed_target': targets['mixed_target'],
            'classes': counts,
        }

    def targets(self):
        single_sum = sum(scene.get('target_per_class', 0) for scene in SCENES if scene['mode'] == 'single')
        mixed_target = sum(scene.get('target_global', 0) for scene in SCENES if scene['mode'] == 'mixed')
        return {'per_class_total': single_sum + mixed_target, 'mixed_target': mixed_target}

    def recommendation(self):
        counts = self.counts()
        class_ids = [c['id'] for c in CLASSES]
        best = None
        for scene in SCENES:
            if scene['mode'] == 'mixed':
                deficit = scene['target_global'] - counts['mixed_count']
                score = deficit / max(1, scene['target_global'])
                candidate = {
                    'scene_id': scene['id'],
                    'class_id': 'mixed',
                    'score': score,
                    'deficit': deficit,
                }
                if deficit > 0 and (best is None or candidate['score'] > best['score']):
                    best = candidate
                continue
            for cls in class_ids:
                have = counts['classes'][cls]['scenes'][scene['id']]
                target = scene['target_per_class']
                deficit = target - have
                score = deficit / max(1, target)
                candidate = {
                    'scene_id': scene['id'],
                    'class_id': cls,
                    'score': score,
                    'deficit': deficit,
                }
                if deficit > 0 and (best is None or candidate['score'] > best['score']):
                    best = candidate
        return best or {'scene_id': SCENES[-1]['id'], 'class_id': 'mixed', 'score': 0, 'deficit': 0}

    def save_image(self, image_bytes, class_id, scene_id, note=''):
        scene = scene_by_id(scene_id)
        if scene['mode'] == 'mixed':
            primary = 'mixed'
            classes_present = ['red', 'green', 'blue']
            folder = RAW_ROOT / 'mixed' / scene_id
        else:
            if class_id not in {c['id'] for c in CLASSES}:
                raise ValueError('invalid class_id')
            primary = class_id
            classes_present = [class_id]
            folder = RAW_ROOT / class_id / scene_id
        folder.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        filename = f'{timestamp}_{primary}_{scene_id}.jpg'
        path = folder / filename
        path.write_bytes(image_bytes)
        rel = path.relative_to(DATA_ROOT).as_posix()
        row = {
            'id': timestamp,
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'relative_path': rel,
            'mode': scene['mode'],
            'primary_class': primary,
            'classes_present': '|'.join(classes_present),
            'scene_id': scene_id,
            'scene_name': scene['name'],
            'note': note or '',
        }
        with self.lock:
            with MANIFEST.open('a', newline='', encoding='utf-8') as f:
                csv.DictWriter(f, fieldnames=MANIFEST_FIELDS).writerow(row)
        (CACHE_ROOT / 'last.jpg').write_bytes(image_bytes)
        return row


class RobotClient:
    def __init__(self):
        self.client = None
        self.lock = threading.Lock()
        self.last_error = ''

    def close(self):
        with self.lock:
            if self.client:
                self.client.close()
                self.client = None

    def connect(self):
        if paramiko is None:
            raise RuntimeError('paramiko is not installed; run: pip install paramiko')
        with self.lock:
            if self.client:
                try:
                    self.client.exec_command('true', timeout=5)
                    return self.client
                except Exception:
                    self.client.close()
                    self.client = None
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                ROBOT_HOST,
                username=ROBOT_USER,
                password=ROBOT_PASSWORD,
                timeout=8,
                banner_timeout=8,
                auth_timeout=8,
            )
            self.client = client
            return client

    def check(self):
        try:
            client = self.connect()
            command = (
                "docker exec -u ubuntu MentorPi bash -lc "
                + shlex.quote(
                    "source /opt/ros/humble/setup.bash; "
                    "source /home/ubuntu/ros2_ws/install/setup.bash || true; "
                    "ros2 topic list | grep -F " + shlex.quote(CAMERA_TOPIC)
                )
            )
            rc, out, err = self.exec(command, timeout=12)
            camera_ok = rc == 0 and CAMERA_TOPIC in out
            return {'ssh': True, 'camera': camera_ok, 'message': out.strip() or err.strip()}
        except Exception as exc:
            self.last_error = str(exc)
            return {'ssh': False, 'camera': False, 'message': str(exc)}

    def capture(self, timeout=8, jpeg_quality=95):
        self.connect()
        script = REMOTE_CAPTURE_SCRIPT.replace('__CAMERA_TOPIC__', CAMERA_TOPIC)
        script = script.replace('__TIMEOUT__', str(float(timeout)))
        script = script.replace('__JPEG_QUALITY__', str(int(jpeg_quality)))
        inner = (
            "source /opt/ros/humble/setup.bash; "
            "source /home/ubuntu/ros2_ws/install/setup.bash || true; "
            "python3 - <<'PY'\n" + script + "\nPY"
        )
        command = "docker exec -u ubuntu MentorPi bash -lc " + shlex.quote(inner)
        rc, out, err = self.exec(command, timeout=timeout + 12)
        if rc != 0:
            raise RuntimeError(err.strip() or out.strip() or f'capture command failed rc={rc}')
        match = re.search(r'__FRAME_BEGIN__\s*(.*?)\s*__FRAME_END__', out, re.S)
        if not match:
            raise RuntimeError((err + '\n' + out).strip() or 'capture output did not contain a frame')
        return base64.b64decode(match.group(1))

    def exec(self, command, timeout=20):
        client = self.connect()
        stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
        out = stdout.read().decode('utf-8', errors='replace')
        err = stderr.read().decode('utf-8', errors='replace')
        rc = stdout.channel.recv_exit_status()
        return rc, out, err


def scene_by_id(scene_id):
    for scene in SCENES:
        if scene['id'] == scene_id:
            return scene
    raise ValueError(f'unknown scene_id: {scene_id}')


def run_local(command, timeout=30):
    result = subprocess.run(
        command,
        shell=True,
        text=True,
        encoding='utf-8',
        errors='replace',
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    return result.stdout or ''


def current_ssid():
    if platform.system().lower() != 'windows':
        return ''
    output = run_local('netsh wlan show interfaces', timeout=10)
    for line in output.splitlines():
        text = line.strip()
        if text.startswith('SSID') and 'BSSID' not in text:
            return text.split(':', 1)[1].strip()
    return ''


def connect_wifi(ssid):
    if platform.system().lower() != 'windows':
        raise RuntimeError('WiFi switching is only implemented on Windows')
    if ssid == ROBOT_SSID:
        run_local(f'netsh wlan set profileparameter name="{RESTORE_SSID}" connectionmode=manual', timeout=10)
    else:
        run_local(f'netsh wlan set profileparameter name="{RESTORE_SSID}" connectionmode=auto', timeout=10)
    run_local('netsh wlan disconnect', timeout=10)
    time.sleep(2)
    run_local(f'netsh wlan connect name="{ssid}" ssid="{ssid}"', timeout=20)
    deadline = time.time() + 55
    while time.time() < deadline:
        if current_ssid() == ssid:
            return True
        time.sleep(2)
    return False


STORE = CaptureStore()
ROBOT = RobotClient()


class Handler(BaseHTTPRequestHandler):
    server_version = 'ColorBlockCaptureUI/1.0'

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/':
            return self.html(INDEX_HTML)
        if parsed.path == '/api/state':
            return self.json(state_payload())
        if parsed.path == '/api/last-image':
            path = CACHE_ROOT / 'last.jpg'
            if not path.exists():
                return self.not_found('no image yet')
            return self.bytes(path.read_bytes(), 'image/jpeg')
        if parsed.path == '/api/file':
            qs = parse_qs(parsed.query)
            rel = qs.get('path', [''])[0]
            path = (DATA_ROOT / rel).resolve()
            if DATA_ROOT.resolve() not in path.parents and path != DATA_ROOT.resolve():
                return self.error(400, 'invalid path')
            if not path.exists():
                return self.not_found('file not found')
            return self.bytes(path.read_bytes(), 'image/jpeg')
        return self.not_found('not found')

    def do_POST(self):
        parsed = urlparse(self.path)
        try:
            payload = self.read_json()
            if parsed.path == '/api/wifi/connect-robot':
                ok = connect_wifi(ROBOT_SSID)
                return self.json({'ok': ok, 'ssid': current_ssid()})
            if parsed.path == '/api/wifi/restore':
                ok = connect_wifi(RESTORE_SSID)
                ROBOT.close()
                return self.json({'ok': ok, 'ssid': current_ssid()})
            if parsed.path == '/api/robot/check':
                return self.json({'ok': True, 'robot': ROBOT.check(), 'ssid': current_ssid()})
            if parsed.path == '/api/preview':
                image = ROBOT.capture(timeout=8)
                (CACHE_ROOT / 'last.jpg').write_bytes(image)
                return self.json({'ok': True, 'image': '/api/last-image?ts=' + str(time.time())})
            if parsed.path == '/api/capture':
                scene_id = payload.get('scene_id') or STORE.recommendation()['scene_id']
                class_id = payload.get('class_id') or STORE.recommendation()['class_id']
                note = payload.get('note', '')
                image = ROBOT.capture(timeout=8)
                row = STORE.save_image(image, class_id, scene_id, note)
                return self.json({'ok': True, 'saved': row, 'state': state_payload(), 'image': '/api/last-image?ts=' + str(time.time())})
            return self.not_found('not found')
        except Exception as exc:
            return self.error(500, str(exc))

    def read_json(self):
        length = int(self.headers.get('Content-Length', '0'))
        if not length:
            return {}
        data = self.rfile.read(length).decode('utf-8')
        return json.loads(data)

    def json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def html(self, html, status=200):
        body = html.encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def bytes(self, body, content_type, status=200):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def not_found(self, message):
        return self.error(404, message)

    def error(self, status, message):
        return self.json({'ok': False, 'error': message}, status=status)

    def log_message(self, fmt, *args):
        sys.stdout.write('%s - %s\n' % (self.log_date_time_string(), fmt % args))


def state_payload():
    return {
        'classes': CLASSES,
        'scenes': SCENES,
        'counts': STORE.counts(),
        'recommendation': STORE.recommendation(),
        'data_root': str(DATA_ROOT),
        'manifest': str(MANIFEST),
        'ssid': current_ssid(),
        'robot_host': ROBOT_HOST,
        'camera_topic': CAMERA_TOPIC,
    }


INDEX_HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>红绿蓝方块数据采集</title>
  <style>
    :root {
      --bg: #f6f7f9;
      --panel: #ffffff;
      --text: #17202a;
      --muted: #677282;
      --line: #d9dee7;
      --accent: #245fbd;
      --danger: #c83c34;
      --good: #2f8e56;
      --shadow: 0 10px 26px rgba(20, 32, 50, 0.08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
      letter-spacing: 0;
    }
    button, input, select, textarea { font: inherit; }
    .app {
      min-height: 100vh;
      display: grid;
      grid-template-rows: auto 1fr;
    }
    header {
      background: #fff;
      border-bottom: 1px solid var(--line);
      padding: 14px 22px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      position: sticky;
      top: 0;
      z-index: 5;
    }
    h1 { font-size: 20px; margin: 0; }
    .top-actions { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
    .status-pill {
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 7px 10px;
      background: #fbfcfd;
      color: var(--muted);
      font-size: 13px;
      white-space: nowrap;
    }
    main {
      padding: 18px 22px 24px;
      display: grid;
      grid-template-columns: minmax(320px, 1.05fr) minmax(340px, 1fr) minmax(360px, 1.1fr);
      gap: 16px;
      align-items: start;
    }
    section, .card {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: var(--shadow);
    }
    section { padding: 16px; }
    h2 { font-size: 16px; margin: 0 0 12px; }
    h3 { font-size: 14px; margin: 0 0 8px; }
    .stack { display: grid; gap: 12px; }
    .class-card { padding: 12px; }
    .class-head { display: flex; justify-content: space-between; align-items: center; gap: 10px; }
    .class-name { display: flex; align-items: center; gap: 8px; font-weight: 700; }
    .swatch { width: 14px; height: 14px; border-radius: 3px; border: 1px solid rgba(0,0,0,.2); flex: 0 0 14px; }
    .count { color: var(--muted); font-size: 13px; }
    .bar { height: 10px; background: #edf0f5; border-radius: 999px; overflow: hidden; margin: 10px 0 8px; }
    .fill { height: 100%; width: 0%; background: var(--accent); transition: width .18s ease; }
    .scene-grid { display: grid; gap: 5px; font-size: 12px; color: var(--muted); }
    .scene-row { display: flex; justify-content: space-between; gap: 8px; }
    .selector { display: grid; gap: 10px; }
    .segmented { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
    .scene-list { display: grid; gap: 8px; }
    .choice {
      border: 1px solid var(--line);
      background: #fbfcfd;
      border-radius: 8px;
      padding: 10px;
      text-align: left;
      cursor: pointer;
    }
    .choice.active {
      border-color: var(--accent);
      outline: 2px solid rgba(36, 95, 189, .16);
      background: #f4f8ff;
    }
    .choice strong { display: block; font-size: 13px; margin-bottom: 3px; }
    .choice span { color: var(--muted); font-size: 12px; line-height: 1.5; }
    .guide {
      background: #f7f9fc;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      color: #344051;
      line-height: 1.65;
      min-height: 110px;
    }
    .capture-panel { display: grid; gap: 12px; }
    .preview {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #101820;
      aspect-ratio: 4 / 3;
      display: grid;
      place-items: center;
      overflow: hidden;
      color: #d5dce8;
      min-height: 260px;
    }
    .preview img { width: 100%; height: 100%; object-fit: contain; background: #111; }
    textarea {
      width: 100%;
      min-height: 76px;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 10px;
      resize: vertical;
      background: #fff;
    }
    button {
      border: 1px solid var(--line);
      background: #fff;
      border-radius: 8px;
      padding: 9px 12px;
      cursor: pointer;
      color: var(--text);
    }
    button:hover { background: #f4f6f9; }
    button.primary {
      background: var(--accent);
      color: #fff;
      border-color: var(--accent);
      font-weight: 700;
    }
    button.primary:hover { background: #1f55aa; }
    button.danger { border-color: #e5b9b4; color: var(--danger); }
    button:disabled { opacity: .55; cursor: wait; }
    .button-row { display: flex; gap: 8px; flex-wrap: wrap; }
    .log {
      min-height: 44px;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.5;
      border-top: 1px solid var(--line);
      padding-top: 10px;
    }
    .path {
      font-size: 12px;
      color: var(--muted);
      word-break: break-all;
      line-height: 1.5;
    }
    @media (max-width: 1120px) {
      main { grid-template-columns: 1fr; }
      header { align-items: flex-start; flex-direction: column; }
    }
  </style>
</head>
<body>
  <div class="app">
    <header>
      <div>
        <h1>红绿蓝方块数据采集工作台</h1>
        <div class="path" id="dataPath">数据目录加载中</div>
      </div>
      <div class="top-actions">
        <span class="status-pill" id="ssid">WiFi: -</span>
        <span class="status-pill" id="robot">机器人: 未检测</span>
        <button id="wifiRobot">连接机器人热点</button>
        <button id="wifiRestore">恢复 TJ-WIFI</button>
        <button id="checkRobot">检测相机</button>
      </div>
    </header>
    <main>
      <section>
        <h2>采集进度</h2>
        <div class="stack" id="classCards"></div>
      </section>
      <section>
        <h2>当前拍摄任务</h2>
        <div class="selector">
          <div>
            <h3>颜色</h3>
            <div class="segmented" id="classChoices"></div>
          </div>
          <div>
            <h3>图片类型</h3>
            <div class="scene-list" id="sceneChoices"></div>
          </div>
          <div>
            <h3>拍摄要求</h3>
            <div class="guide" id="guide"></div>
          </div>
        </div>
      </section>
      <section class="capture-panel">
        <h2>机器人相机</h2>
        <div class="preview" id="preview"><span>还没有预览图</span></div>
        <textarea id="note" placeholder="可选备注：光照、角度、摆放情况"></textarea>
        <div class="button-row">
          <button id="refreshPreview">刷新预览</button>
          <button class="primary" id="capture">拍照并保存</button>
          <button id="autoNext">使用推荐任务</button>
        </div>
        <div class="log" id="log">准备就绪。先连接机器人热点，再按提示摆放方块。</div>
      </section>
    </main>
  </div>
  <script>
    let state = null;
    let activeClass = 'red';
    let activeScene = 'single_front_mid';
    const $ = id => document.getElementById(id);

    function log(message) { $('log').textContent = message; }

    async function api(path, options = {}) {
      const res = await fetch(path, {
        headers: { 'Content-Type': 'application/json' },
        ...options
      });
      const data = await res.json();
      if (!res.ok || data.ok === false) throw new Error(data.error || '请求失败');
      return data;
    }

    async function loadState() {
      state = await api('/api/state');
      $('ssid').textContent = 'WiFi: ' + (state.ssid || '-');
      $('dataPath').textContent = '本地数据目录：' + state.data_root;
      render();
    }

    function classInfo(id) { return state.classes.find(c => c.id === id); }
    function sceneInfo(id) { return state.scenes.find(s => s.id === id); }

    function render() {
      renderProgress();
      renderChoices();
      updateGuide();
    }

    function renderProgress() {
      const cards = state.classes.map(cls => {
        const c = state.counts.classes[cls.id];
        const pct = Math.min(100, Math.round(c.total / c.target * 100));
        const rows = state.scenes.map(scene => {
          const target = scene.mode === 'mixed' ? scene.target_global : scene.target_per_class;
          const have = c.scenes[scene.id] || 0;
          return `<div class="scene-row"><span>${scene.name}</span><span>${have}/${target}</span></div>`;
        }).join('');
        return `<div class="card class-card">
          <div class="class-head">
            <div class="class-name"><span class="swatch" style="background:${cls.color}"></span>${cls.name}</div>
            <div class="count">${c.total}/${c.target}，还差 ${c.remaining}</div>
          </div>
          <div class="bar"><div class="fill" style="width:${pct}%; background:${cls.color}"></div></div>
          <div class="scene-grid">${rows}</div>
        </div>`;
      }).join('');
      $('classCards').innerHTML = cards;
    }

    function renderChoices() {
      $('classChoices').innerHTML = state.classes.map(cls => `
        <button class="choice ${activeClass === cls.id ? 'active' : ''}" data-class="${cls.id}">
          <strong><span class="swatch" style="background:${cls.color}"></span> ${cls.name}</strong>
          <span>${cls.hint}</span>
        </button>`).join('');
      document.querySelectorAll('[data-class]').forEach(btn => {
        btn.onclick = () => { activeClass = btn.dataset.class; renderChoices(); updateGuide(); };
      });

      $('sceneChoices').innerHTML = state.scenes.map(scene => `
        <button class="choice ${activeScene === scene.id ? 'active' : ''}" data-scene="${scene.id}">
          <strong>${scene.name}</strong>
          <span>${scene.mode === 'mixed' ? '三色同框，计入三类' : '单色采集'}：${scene.guide}</span>
        </button>`).join('');
      document.querySelectorAll('[data-scene]').forEach(btn => {
        btn.onclick = () => { activeScene = btn.dataset.scene; renderChoices(); updateGuide(); };
      });
    }

    function updateGuide() {
      const scene = sceneInfo(activeScene);
      const cls = classInfo(activeClass);
      if (!scene || !cls) return;
      if (scene.mode === 'mixed') {
        $('guide').innerHTML = `<strong>现在摆放：红、绿、蓝三块同时入镜</strong><br>${scene.guide}`;
      } else {
        $('guide').innerHTML = `<strong>现在摆放：${cls.name}</strong><br>${scene.guide}`;
      }
    }

    function applyRecommendation() {
      const rec = state.recommendation;
      activeScene = rec.scene_id;
      if (rec.class_id !== 'mixed') activeClass = rec.class_id;
      renderChoices();
      updateGuide();
      log('已切到推荐任务。');
    }

    async function busy(button, work) {
      const old = button.textContent;
      button.disabled = true;
      try { await work(); }
      finally { button.disabled = false; button.textContent = old; }
    }

    $('wifiRobot').onclick = () => busy($('wifiRobot'), async () => {
      $('wifiRobot').textContent = '连接中';
      log('正在切换到机器人热点，浏览器会继续保持本地页面。');
      const data = await api('/api/wifi/connect-robot', { method: 'POST', body: '{}' });
      log(data.ok ? '已连接机器人热点。' : '未能连接机器人热点，请确认机器人已开机。');
      await loadState();
    });

    $('wifiRestore').onclick = () => busy($('wifiRestore'), async () => {
      $('wifiRestore').textContent = '恢复中';
      const data = await api('/api/wifi/restore', { method: 'POST', body: '{}' });
      log(data.ok ? '已恢复 TJ-WIFI。' : '恢复失败，请手动切回校园网。');
      await loadState();
    });

    $('checkRobot').onclick = () => busy($('checkRobot'), async () => {
      $('checkRobot').textContent = '检测中';
      const data = await api('/api/robot/check', { method: 'POST', body: '{}' });
      $('robot').textContent = data.robot.camera ? '机器人: 相机可用' : (data.robot.ssh ? '机器人: SSH 可用，相机未见' : '机器人: 未连接');
      log(data.robot.message || (data.robot.camera ? '相机 topic 正常。' : '检测完成。'));
    });

    $('refreshPreview').onclick = () => busy($('refreshPreview'), async () => {
      $('refreshPreview').textContent = '取图中';
      log('正在从机器人相机抓取预览帧。');
      const data = await api('/api/preview', { method: 'POST', body: '{}' });
      $('preview').innerHTML = `<img src="${data.image}" alt="preview">`;
      log('预览已刷新。');
    });

    $('capture').onclick = () => busy($('capture'), async () => {
      $('capture').textContent = '拍照中';
      const scene = sceneInfo(activeScene);
      const classId = scene.mode === 'mixed' ? 'mixed' : activeClass;
      log('正在拍照并保存到本地数据集。');
      const data = await api('/api/capture', {
        method: 'POST',
        body: JSON.stringify({ class_id: classId, scene_id: activeScene, note: $('note').value })
      });
      $('preview').innerHTML = `<img src="${data.image}" alt="captured">`;
      await loadState();
      applyRecommendation();
      log('已保存：' + data.saved.relative_path);
    });

    $('autoNext').onclick = applyRecommendation;

    loadState().then(applyRecommendation).catch(err => log(err.message));
  </script>
</body>
</html>'''


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Local UI for robot camera color-block dataset capture.')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f'Color block capture UI: http://{args.host}:{args.port}')
    print(f'Data root: {DATA_ROOT}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        ROBOT.close()
        server.server_close()


if __name__ == '__main__':
    main()
