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


CALIBRATOR_PY = r'''
import argparse
import csv
import json
import math
import statistics
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import ObjectsInfo
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


COLORS = {
    "gray": (210, 210, 210),
    "grey": (210, 210, 210),
    "yellow": (0, 230, 255),
    "grass": (40, 220, 40),
    "blue": (255, 120, 20),
}


class GraspPoseCalibrator(Node):
    def __init__(self, args):
        super().__init__("grasp_pose_calibrator")
        self.args = args
        self.out_dir = Path(args.out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.started_at = time.time()
        self.last_frame = 0.0
        self.frame_index = 0
        self.depth = None
        self.depth_time = 0.0
        self.camera_k = None
        self.latest = []
        self.samples = []
        self.target_names = {part.strip() for part in args.target_names.split(",") if part.strip()}
        self.camera_tilt = math.radians(args.camera_tilt_deg)

        self.csv_file = open(self.out_dir / "samples.csv", "w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow([
            "t", "class", "score", "x1", "y1", "x2", "y2", "cx_ratio", "area_ratio",
            "depth_m", "camera_x", "camera_y", "camera_z", "robot_x", "robot_y", "robot_z",
        ])

        self.create_subscription(ObjectsInfo, "/yolo_node/object_detect", self.detect_cb, 10)
        self.create_subscription(CameraInfo, "/ascamera/camera_publisher/rgb0/camera_info", self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/ascamera/camera_publisher/depth0/image_raw", self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/ascamera/camera_publisher/rgb0/image", self.image_cb, qos_profile_sensor_data)

    def info_cb(self, msg):
        self.camera_k = list(msg.k)

    def depth_cb(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_time = time.time()
        except Exception as exc:
            self.get_logger().warn(f"depth conversion failed: {exc}")

    def detect_cb(self, msg):
        now = time.time()
        rows = []
        for obj in msg.objects:
            name = str(obj.class_name).strip()
            if name not in self.target_names or float(obj.score) < self.args.conf:
                continue
            if len(obj.box) < 4:
                continue
            x1, y1, x2, y2 = [int(v) for v in obj.box[:4]]
            width = int(obj.width or 640)
            height = int(obj.height or 480)
            cx = ((x1 + x2) * 0.5) / max(1, width)
            area = max(0, x2 - x1) * max(0, y2 - y1) / max(1, width * height)
            pose = self.estimate_pose((x1 + x2) * 0.5, (y1 + y2) * 0.5, width, height)
            if pose is None:
                continue
            row = (now, name, float(obj.score), x1, y1, x2, y2, cx, area, *pose)
            rows.append(row)
            self.samples.append(row)
            self.csv.writerow([
                f"{now:.3f}", name, f"{float(obj.score):.4f}", x1, y1, x2, y2,
                f"{cx:.5f}", f"{area:.6f}", *[f"{v:.5f}" for v in pose],
            ])
        self.csv_file.flush()
        if rows:
            self.latest = rows

    def estimate_pose(self, u_rgb, v_rgb, rgb_w, rgb_h):
        if self.depth is None or self.camera_k is None or len(self.camera_k) < 6:
            return None
        if time.time() - self.depth_time > 1.0:
            return None
        depth = self.depth
        h, w = depth.shape[:2]
        u = int(round(u_rgb * w / max(1, rgb_w)))
        v = int(round(v_rgb * h / max(1, rgb_h)))
        radius = max(1, int(self.args.depth_roi_pixels))
        x1 = max(0, u - radius)
        x2 = min(w - 1, u + radius)
        y1 = max(0, v - radius)
        y2 = min(h - 1, v + radius)
        roi = depth[y1:y2 + 1, x1:x2 + 1].astype(np.float32)
        if roi.size == 0:
            return None
        if roi.dtype == np.float32:
            values = roi[np.isfinite(roi)]
        else:
            values = roi
        values = values[(values > 0)]
        values = values * self.args.depth_unit_scale
        values = values[(values >= 0.08) & (values <= 1.5)]
        if values.size < self.args.depth_min_valid_samples:
            return None
        depth_m = float(np.mean(values))

        fx = float(self.camera_k[0]) * w / max(1, rgb_w)
        fy = float(self.camera_k[4]) * h / max(1, rgb_h)
        cx0 = float(self.camera_k[2]) * w / max(1, rgb_w)
        cy0 = float(self.camera_k[5]) * h / max(1, rgb_h)
        camera_x = (u - cx0) * depth_m / fx
        camera_y = (v - cy0) * depth_m / fy
        camera_z = depth_m
        robot_x = camera_z * math.cos(self.camera_tilt) - camera_y * math.sin(self.camera_tilt) + self.args.camera_offset_x
        robot_y = -camera_x
        robot_z = self.args.camera_height - (camera_y * math.cos(self.camera_tilt) + camera_z * math.sin(self.camera_tilt))
        return depth_m, camera_x, camera_y, camera_z, robot_x, robot_y, robot_z

    def image_cb(self, msg):
        now = time.time()
        if now - self.last_frame < self.args.frame_period:
            return
        self.last_frame = now
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h), (235, 235, 235), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (235, 235, 235), 1)
        for row in self.latest:
            _, name, score, x1, y1, x2, y2, _, _, _, _, _, _, robot_x, robot_y, robot_z = row
            color = COLORS.get(name, (0, 210, 255))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            depth_m = row[9]
            label = f"{name} {score:.2f} d={depth_m:.3f} rx={robot_x:.3f} ry={robot_y:.3f}"
            cv2.putText(frame, label, (x1 + 4, max(22, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2)
        lines = [
            datetime.fromtimestamp(now).strftime("time=%Y-%m-%d %H:%M:%S.%f")[:-3],
            f"target={','.join(sorted(self.target_names))} samples={len(self.samples)}",
            "place cube at real gripper capture point; robot is not moving",
        ]
        overlay = frame.copy()
        cv2.rectangle(overlay, (6, 6), (626, 86), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (10, 28 + i * 23), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 230, 255), 2)
        self.frame_index += 1
        cv2.imwrite(str(self.out_dir / f"frame_{self.frame_index:04d}.jpg"), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 84])

    def done(self):
        return time.time() - self.started_at >= self.args.duration

    def close(self):
        self.csv_file.close()
        target_rows = self.samples
        summary = {"sample_count": len(target_rows)}
        if target_rows:
            cols = {
                "depth_m": [r[9] for r in target_rows],
                "camera_x": [r[10] for r in target_rows],
                "camera_y": [r[11] for r in target_rows],
                "camera_z": [r[12] for r in target_rows],
                "robot_x": [r[13] for r in target_rows],
                "robot_y": [r[14] for r in target_rows],
                "robot_z": [r[15] for r in target_rows],
            }
            for key, values in cols.items():
                summary[key + "_median"] = statistics.median(values)
                summary[key + "_mean"] = statistics.mean(values)
                summary[key + "_stdev"] = statistics.pstdev(values) if len(values) > 1 else 0.0
            summary["recommended"] = {
                "pick_target_robot_x_m": round(summary["robot_x_median"], 4),
                "pick_target_robot_y_m": round(summary["robot_y_median"], 4),
                "pick_robot_x_tolerance_m": 0.025,
                "pick_robot_y_tolerance_m": 0.025,
            }
        (self.out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print("calibration_summary=" + json.dumps(summary, ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--target-names", default="gray,grey")
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--frame-period", type=float, default=0.4)
    parser.add_argument("--depth-roi-pixels", type=int, default=15)
    parser.add_argument("--depth-min-valid-samples", type=int, default=20)
    parser.add_argument("--depth-unit-scale", type=float, default=0.001)
    parser.add_argument("--camera-tilt-deg", type=float, default=45.0)
    parser.add_argument("--camera-height", type=float, default=0.22)
    parser.add_argument("--camera-offset-x", type=float, default=0.06)
    args = parser.parse_args()
    rclpy.init()
    node = GraspPoseCalibrator(args)
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


if __name__ == "__main__":
    main()
'''


def parse_args():
    parser = argparse.ArgumentParser(description='Calibrate RGB-D robot-frame grasp target, then optionally try one grasp.')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--user', default=DEFAULT_USER)
    parser.add_argument('--password', default=DEFAULT_PASSWORD)
    parser.add_argument('--container', default=DEFAULT_CONTAINER)
    parser.add_argument('--target-class', default='gray')
    parser.add_argument('--target-names', default='')
    parser.add_argument('--model', default='tongji')
    parser.add_argument('--classes', default='gray,yellow,grass,blue')
    parser.add_argument('--conf', type=float, default=0.20)
    parser.add_argument('--duration', type=int, default=20)
    parser.add_argument('--skip-grasp', action='store_true', help='Only collect calibration samples; do not execute the trial grasp.')
    parser.add_argument('--grasp-timeout', type=int, default=55)
    parser.add_argument('--robot-x-tolerance', type=float, default=0.025)
    parser.add_argument('--robot-y-tolerance', type=float, default=0.025)
    parser.add_argument('--max-linear-speed', type=float, default=0.04)
    parser.add_argument('--max-angular-speed', type=float, default=0.12)
    parser.add_argument('--cmd-pulse', type=float, default=0.03)
    parser.add_argument('--pick-attempts', type=int, default=1)
    parser.add_argument('--out-dir', default='runs/grasp_calibration')
    parser.add_argument('--depth-roi-pixels', type=int, default=15)
    parser.add_argument('--camera-tilt-deg', type=float, default=45.0)
    parser.add_argument('--camera-height', type=float, default=0.22)
    parser.add_argument('--camera-offset-x', type=float, default=0.06)
    return parser.parse_args()


def q(value) -> str:
    return shlex.quote(str(value))


def target_names(args) -> str:
    if args.target_names.strip():
        return args.target_names
    aliases = {
        'gray': 'gray,grey',
        'grey': 'gray,grey',
        'grass': 'grass,green',
        'yellow': 'yellow',
        'blue': 'blue',
    }
    return aliases.get(args.target_class, args.target_class)


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
cat > "$DEBUG_DIR/calibrator.py" <<'PY'
{CALIBRATOR_PY}
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

echo "session_start=$(date '+%F %T')" | tee "$DEBUG_DIR/session.log"
kill_matching "competition_run.launch.py"
kill_matching "competition_node"
kill_matching "yolov11_node"

if camera_ready; then
  echo camera_ready=1 | tee -a "$DEBUG_DIR/session.log"
else
  echo camera_ready=0 | tee -a "$DEBUG_DIR/session.log"
fi

python3 -u "$DEBUG_DIR/calibrator.py" \\
  --out-dir "$DEBUG_DIR" \\
  --target-names {q(target_names(args))} \\
  --conf {args.conf:.3f} \\
  --duration {int(args.duration)} \\
  --depth-roi-pixels {int(args.depth_roi_pixels)} \\
  --camera-tilt-deg {args.camera_tilt_deg:.3f} \\
  --camera-height {args.camera_height:.3f} \\
  --camera-offset-x {args.camera_offset_x:.3f} > "$DEBUG_DIR/calibrator.log" 2>&1 &
CAL_PID=$!
echo calibrator_pid=$CAL_PID | tee -a "$DEBUG_DIR/session.log"

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
  min_score:={args.conf:.3f} \\
  target_class:={q(args.target_class)} > "$DEBUG_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
echo launch_pid=$LAUNCH_PID | tee -a "$DEBUG_DIR/session.log"

wait "$CAL_PID"
RC=$?
tail -n 160 "$DEBUG_DIR/launch.log" > "$DEBUG_DIR/launch_tail.log" 2>/dev/null || true
kill "$LAUNCH_PID" 2>/dev/null || true
kill_matching "competition_run.launch.py"
kill_matching "competition_node"
kill_matching "yolov11_node"

RUN_GRASP={'false' if args.skip_grasp else 'true'}
TARGET_LINE=$(python3 -c "import json; s=json.load(open('$DEBUG_DIR/summary.json', encoding='utf-8')); r=s.get('recommended') or {{}}; print(str(r.get('pick_target_robot_x_m','')) + ' ' + str(r.get('pick_target_robot_y_m','')))" 2>/dev/null || true)
TARGET_ROBOT_X=$(echo "$TARGET_LINE" | awk '{{print $1}}')
TARGET_ROBOT_Y=$(echo "$TARGET_LINE" | awk '{{print $2}}')
echo recommended_robot_x="$TARGET_ROBOT_X" | tee -a "$DEBUG_DIR/session.log"
echo recommended_robot_y="$TARGET_ROBOT_Y" | tee -a "$DEBUG_DIR/session.log"

if [ "$RUN_GRASP" = "true" ] && [ -n "$TARGET_ROBOT_X" ] && [ -n "$TARGET_ROBOT_Y" ]; then
  echo grasp_trial=1 | tee -a "$DEBUG_DIR/session.log"
  ros2 launch competition_pick_place competition_run.launch.py \\
    target_class:={q(args.target_class)} \\
    dry_run:=false \\
    stop_after_pick:=true \\
    exit_on_done:=true \\
    start_navigation:=false \\
    start_base:=false \\
    start_camera:=false \\
    start_yolo:=true \\
    use_nav:=false \\
    use_arm:=true \\
    yolo_model:={q(args.model)} \\
    yolo_classes:={q(args.classes)} \\
    yolo_conf:={args.conf:.3f} \\
    min_score:={args.conf:.3f} \\
    init_action:=navigation_pick_init_ai \\
    pick_action:=navigation_pick_ai \\
    search_timeout:=12.0 \\
    align_timeout:=45.0 \\
    wait_for_detection_stream:=true \\
    detection_stream_timeout:=20.0 \\
    detection_ready_min_messages:=1 \\
    wait_for_target_before_search:=true \\
    allow_search_rotation:=false \\
    use_depth_distance:=true \\
    depth_topic:=/ascamera/camera_publisher/depth0/image_raw \\
    camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info \\
    use_robot_frame_distance:=true \\
    camera_tilt_deg:={args.camera_tilt_deg:.3f} \\
    camera_height_m:={args.camera_height:.3f} \\
    camera_offset_x_m:={args.camera_offset_x:.3f} \\
    depth_roi_pixels:={int(args.depth_roi_pixels)} \\
    depth_stale_seconds:=0.800 \\
    depth_unit_scale:=0.001 \\
    depth_min_valid_samples:=20 \\
    depth_min_m:=0.080 \\
    depth_max_m:=1.500 \\
    pick_target_depth_m:="$TARGET_ROBOT_X" \\
    pick_target_robot_x_m:="$TARGET_ROBOT_X" \\
    pick_target_robot_y_m:="$TARGET_ROBOT_Y" \\
    pick_robot_x_tolerance_m:={args.robot_x_tolerance:.3f} \\
    pick_robot_y_tolerance_m:={args.robot_y_tolerance:.3f} \\
    pick_depth_tolerance_m:={args.robot_x_tolerance:.3f} \\
    desired_center_x_ratio:=0.5000 \\
    center_tolerance_ratio:=0.0280 \\
    pick_target_area_ratio:=0.0430 \\
    area_tolerance_ratio:=0.0100 \\
    stable_frames:=1 \\
    control_mode:=mpc \\
    closed_loop_pick:=true \\
    pick_visual_servo_timeout:=5.0 \\
    visual_servo_period:=0.100 \\
    visual_servo_command_seconds:={args.cmd_pulse:.3f} \\
    adaptive_servo_timing:=true \\
    visual_servo_min_period:=0.035 \\
    visual_servo_max_period:=0.160 \\
    visual_servo_period_scale:=1.05 \\
    require_fresh_detection_for_control:=true \\
    pick_pregrasp_visual_servo:=false \\
    pick_pregrasp_time_scale:=2.400 \\
    pick_pregrasp_min_step_seconds:=0.800 \\
    pick_pregrasp_settle_seconds:=0.700 \\
    pick_pregrasp_post_step_seconds:=0.600 \\
    pick_preclose_required:=false \\
    pick_retry_attempts:={int(args.pick_attempts)} \\
    grasp_check_enabled:=true \\
    gripper_state_topic:=/controller_manager/servo_states \\
    gripper_servo_id:=10 \\
    gripper_empty_close_position:=500 \\
    gripper_grasp_min_gap:=30 \\
    gripper_check_delay:=0.350 \\
    gripper_feedback_timeout:=2.0 \\
    angular_k:=0.80 \\
    angular_sign:=-1.0 \\
    max_linear_speed:={args.max_linear_speed:.4f} \\
    max_angular_speed:={args.max_angular_speed:.4f} \\
    search_angular_speed:=0.12 \\
    mpc_horizon:=8 \\
    mpc_dt:=0.100 \\
    mpc_center_response:=1.05 \\
    mpc_area_response:=0.24 \\
    mpc_center_weight:=8.0 \\
    mpc_area_weight:=26.0 \\
    mpc_velocity_weight:=0.08 \\
    mpc_delta_weight:=0.16 \\
    mpc_terminal_weight:=2.2 \\
    mpc_center_gate_ratio:=0.10 > "$DEBUG_DIR/grasp.log" 2>&1 &
  GRASP_PID=$!
  echo grasp_pid=$GRASP_PID | tee -a "$DEBUG_DIR/session.log"
  GRASP_DEADLINE=$(( $(date +%s) + {int(args.grasp_timeout)} ))
  while kill -0 "$GRASP_PID" 2>/dev/null; do
    if [ "$(date +%s)" -ge "$GRASP_DEADLINE" ]; then
      echo grasp_status=timeout | tee -a "$DEBUG_DIR/session.log"
      kill "$GRASP_PID" 2>/dev/null || true
      break
    fi
    sleep 1
  done
  wait "$GRASP_PID" 2>/dev/null
  GRASP_RC=$?
  echo grasp_exit_code="$GRASP_RC" | tee -a "$DEBUG_DIR/session.log"
  tail -n 200 "$DEBUG_DIR/grasp.log" > "$DEBUG_DIR/grasp_tail.log" 2>/dev/null || true
  kill_matching "competition_run.launch.py"
  kill_matching "competition_node"
  kill_matching "yolov11_node"
else
  echo grasp_trial=0 | tee -a "$DEBUG_DIR/session.log"
fi

echo sample_rows=$(($(wc -l < "$DEBUG_DIR/samples.csv" 2>/dev/null || echo 1)-1)) | tee -a "$DEBUG_DIR/session.log"
echo frame_count=$(ls "$DEBUG_DIR"/frame_*.jpg 2>/dev/null | wc -l) | tee -a "$DEBUG_DIR/session.log"
tar -czf "$TAR_PATH" -C "$(dirname "$DEBUG_DIR")" "$(basename "$DEBUG_DIR")"
echo tar_path="$TAR_PATH"
exit "$RC"
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
        stdin, stdout, stderr = client.exec_command(command, timeout=args.duration + args.grasp_timeout + 120)
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
        run_host(client, f'mkdir -p {q(remote_tar_dir)}; docker cp {q(args.container + ":" + remote_tar)} {q(remote_tar)}')
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
    session = datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{args.target_class}'
    remote_dir = f'/tmp/grasp_calibration/{session}'
    remote_tar = f'/tmp/grasp_calibration/{session}.tar.gz'
    rc = run_remote(args, remote_dir, remote_tar)
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
