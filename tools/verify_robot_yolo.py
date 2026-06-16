#!/usr/bin/env python3
import argparse
import json
import shlex
import time
from datetime import datetime
from pathlib import Path

import paramiko


DEFAULT_HOST = '192.168.149.1'
DEFAULT_USER = 'pi'
DEFAULT_PASSWORD = 'raspberrypi'
DEFAULT_CONTAINER = 'MentorPi'


PROBE_PY = r'''
import argparse
import csv
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectsInfo
from rclpy.node import Node
from sensor_msgs.msg import Image


class YoloProbe(Node):
    def __init__(self, out_dir, duration, frame_period):
        super().__init__('headless_yolo_probe')
        self.out_dir = Path(out_dir)
        self.duration = float(duration)
        self.frame_period = float(frame_period)
        self.bridge = CvBridge()
        self.start = time.time()
        self.last_frame = 0.0
        self.frame_count = 0
        self.msg_count = 0
        self.object_count = 0
        self.class_counts = {}
        self.best = {}
        self.csv_file = open(self.out_dir / 'detections.csv', 'w', newline='', encoding='utf-8')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['t', 'msg_index', 'class', 'score', 'x1', 'y1', 'x2', 'y2', 'width', 'height'])
        self.create_subscription(ObjectsInfo, '/yolo_node/object_detect', self.detect_cb, 10)
        self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_cb, 10)

    def detect_cb(self, msg):
        now = time.time()
        self.msg_count += 1
        for obj in msg.objects:
            name = str(obj.class_name).strip()
            score = float(obj.score)
            box = list(obj.box[:4]) if len(obj.box) >= 4 else ['', '', '', '']
            width = int(obj.width or 0)
            height = int(obj.height or 0)
            self.object_count += 1
            self.class_counts[name] = self.class_counts.get(name, 0) + 1
            if name not in self.best or score > self.best[name]:
                self.best[name] = score
            self.csv.writerow([f'{now:.3f}', self.msg_count, name, f'{score:.4f}', *box, width, height])
        self.csv_file.flush()

    def image_cb(self, msg):
        now = time.time()
        if now - self.last_frame < self.frame_period:
            return
        self.last_frame = now
        self.frame_count += 1
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(str(self.out_dir / f'frame_{self.frame_count:04d}.jpg'), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])

    def done(self):
        return time.time() - self.start >= self.duration

    def close(self):
        self.csv_file.close()
        summary = {
            'messages': self.msg_count,
            'objects': self.object_count,
            'frames': self.frame_count,
            'class_counts': self.class_counts,
            'best_scores': self.best,
        }
        (self.out_dir / 'summary.json').write_text(str(summary), encoding='utf-8')
        print('probe_summary=' + str(summary), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--frame-period', type=float, default=0.5)
    args = parser.parse_args()
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = YoloProbe(args.out_dir, args.duration, args.frame_period)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
'''


def parse_args():
    parser = argparse.ArgumentParser(description='Verify robot YOLO output without running grasp control.')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--user', default=DEFAULT_USER)
    parser.add_argument('--password', default=DEFAULT_PASSWORD)
    parser.add_argument('--container', default=DEFAULT_CONTAINER)
    parser.add_argument('--model', default='tongji')
    parser.add_argument('--classes', default='gray,yellow,grass,blue')
    parser.add_argument('--conf', type=float, default=0.70)
    parser.add_argument('--duration', type=int, default=18)
    parser.add_argument('--out-dir', default='runs/yolo_verify')
    return parser.parse_args()


def q(value) -> str:
    return shlex.quote(str(value))


def remote_script(args, remote_dir: str, remote_tar: str) -> str:
    return f'''
set +e
source /home/ubuntu/ros2_ws/.robotrc

DEBUG_DIR={q(remote_dir)}
TAR_PATH={q(remote_tar)}
CAMERA_TOPIC=/ascamera/camera_publisher/rgb0/image
mkdir -p "$DEBUG_DIR" "$(dirname "$TAR_PATH")"
cat > "$DEBUG_DIR/params.json" <<'JSON'
{json.dumps(vars(args), ensure_ascii=False, indent=2)}
JSON
cat > "$DEBUG_DIR/probe.py" <<'PY'
{PROBE_PY}
PY

kill_matching() {{
  PATTERN="$1"
  for PID in $(pgrep -f "$PATTERN" 2>/dev/null || true); do
    if [ "$PID" != "$$" ] && [ "$PID" != "$BASHPID" ] && [ "$PID" != "$PPID" ]; then
      kill "$PID" 2>/dev/null || true
    fi
  done
}}

camera_ready() {{
  timeout 4s ros2 topic echo --once "$CAMERA_TOPIC" >/dev/null 2>&1
}}

ensure_camera() {{
  if camera_ready; then
    echo camera_ready=1 | tee -a "$DEBUG_DIR/session.log"
    return 0
  fi
  kill_matching "depth_camera.launch.py"
  kill_matching "camera_publisher"
  kill_matching "ascamera"
  nohup ros2 launch peripherals depth_camera.launch.py > "$DEBUG_DIR/camera.log" 2>&1 &
  echo camera_pid=$! | tee -a "$DEBUG_DIR/session.log"
  for i in $(seq 1 12); do
    if camera_ready; then
      echo camera_ready=1 | tee -a "$DEBUG_DIR/session.log"
      return 0
    fi
    sleep 1
  done
  echo camera_ready=0 | tee -a "$DEBUG_DIR/session.log"
  return 1
}}

echo "session_start=$(date '+%F %T')" | tee "$DEBUG_DIR/session.log"
kill_matching "competition_run.launch.py"
kill_matching "competition_node"
kill_matching "yolov11_node"
ensure_camera
CAMERA_RC=$?

python3 -u "$DEBUG_DIR/probe.py" --out-dir "$DEBUG_DIR" --duration {int(args.duration)} > "$DEBUG_DIR/probe.log" 2>&1 &
PROBE_PID=$!
echo probe_pid=$PROBE_PID | tee -a "$DEBUG_DIR/session.log"

ros2 launch competition_pick_place competition_run.launch.py \\
  dry_run:=true \\
  exit_on_done:=true \\
  stop_after_pick:=true \\
  start_camera:=false \\
  start_yolo:=true \\
  start_base:=false \\
  start_navigation:=false \\
  use_nav:=false \\
  use_arm:=false \\
  wait_for_detection_stream:=false \\
  yolo_model:={q(args.model)} \\
  yolo_classes:={q(args.classes)} \\
  yolo_conf:={args.conf:.3f} \\
  target_class:=grass > "$DEBUG_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
echo launch_pid=$LAUNCH_PID | tee -a "$DEBUG_DIR/session.log"

sleep {int(args.duration) + 5}
echo topic_publishers=$(ros2 topic info /yolo_node/object_detect 2>/dev/null | grep "Publisher count" | awk '{{print $3}}') | tee -a "$DEBUG_DIR/session.log"
tail -n 120 "$DEBUG_DIR/launch.log" > "$DEBUG_DIR/launch_tail.log" 2>/dev/null || true
kill "$LAUNCH_PID" "$PROBE_PID" 2>/dev/null || true
kill_matching "competition_run.launch.py"
kill_matching "competition_node"
kill_matching "yolov11_node"
sleep 1
echo detection_rows=$(($(wc -l < "$DEBUG_DIR/detections.csv" 2>/dev/null || echo 1)-1)) | tee -a "$DEBUG_DIR/session.log"
echo frame_count=$(ls "$DEBUG_DIR"/frame_*.jpg 2>/dev/null | wc -l) | tee -a "$DEBUG_DIR/session.log"
tar -czf "$TAR_PATH" -C "$(dirname "$DEBUG_DIR")" "$(basename "$DEBUG_DIR")"
echo tar_path="$TAR_PATH"
[ "$CAMERA_RC" = 0 ]
'''


def connect(args):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password, timeout=10, banner_timeout=10, auth_timeout=10)
    return client


def run_host(client, command_text: str, timeout: int = 45) -> tuple[int, str, str]:
    stdin, stdout, stderr = client.exec_command(command_text, timeout=timeout)
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    return stdout.channel.recv_exit_status(), out, err


def run_remote(args, remote_dir: str, remote_tar: str) -> int:
    command = f'docker exec -u ubuntu {q(args.container)} bash -lc {q(remote_script(args, remote_dir, remote_tar))}'
    client = connect(args)
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=args.duration + 90)
        channel = stdout.channel
        while not channel.exit_status_ready():
            while channel.recv_ready():
                print(channel.recv(4096).decode('utf-8', errors='replace'), end='', flush=True)
            while channel.recv_stderr_ready():
                print(channel.recv_stderr(4096).decode('utf-8', errors='replace'), end='', flush=True)
            time.sleep(0.2)
        while channel.recv_ready():
            print(channel.recv(4096).decode('utf-8', errors='replace'), end='', flush=True)
        while channel.recv_stderr_ready():
            print(channel.recv_stderr(4096).decode('utf-8', errors='replace'), end='', flush=True)
        rc = channel.recv_exit_status()
        local_base = Path(args.out_dir)
        local_base.mkdir(parents=True, exist_ok=True)
        local_tar = local_base / Path(remote_tar).name
        remote_tar_dir = remote_tar.rsplit('/', 1)[0] or '.'
        copy_rc, copy_out, copy_err = run_host(
            client,
            f'mkdir -p {q(remote_tar_dir)}; docker cp {q(args.container + ":" + remote_tar)} {q(remote_tar)}',
        )
        if copy_rc != 0:
            print(f'container_tar_copy_failed_rc={copy_rc}')
            if copy_out.strip():
                print(copy_out.rstrip())
            if copy_err.strip():
                print(copy_err.rstrip())
        sftp = client.open_sftp()
        try:
            sftp.get(remote_tar, str(local_tar))
        finally:
            sftp.close()
        print(f'local_result_tar={local_tar}')
        return rc
    finally:
        client.close()


def main():
    args = parse_args()
    session = datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{args.model}_conf{args.conf:.2f}'.replace('.', 'p')
    remote_dir = f'/tmp/yolo_verify/{session}'
    remote_tar = f'/tmp/yolo_verify/{session}.tar.gz'
    rc = run_remote(args, remote_dir, remote_tar)
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
