#!/usr/bin/env python3
import argparse
import json
import shlex
import time
from datetime import datetime
from pathlib import Path

import paramiko


def log_progress(message: str) -> None:
    print(f'[{datetime.now().strftime("%H:%M:%S")}] {message}', flush=True)


def normalize_place_class(value: str) -> str:
    target = str(value or '').strip().lower()
    if target in {'glass', 'grass', 'green'}:
        return 'blue'
    return target


def parse_args():
    parser = argparse.ArgumentParser(description='Run a headless grasp trial and pull debug frames/logs locally.')
    parser.add_argument('--host', default='192.168.149.1')
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--container', default='MentorPi')
    parser.add_argument('--target-class', default='grass', choices=['gray', 'grey', 'yellow', 'glass', 'grass', 'blue'])
    parser.add_argument('--place-class', default='', choices=['', 'gray', 'grey', 'yellow', 'glass', 'grass', 'blue'])
    parser.add_argument('--out-dir', default='runs/grasp_headless')
    parser.add_argument('--timeout', type=int, default=90)
    parser.add_argument('--yolo-model', default='tongji')
    parser.add_argument('--yolo-classes', default='gray,yellow,grass,blue')
    parser.add_argument('--yolo-conf', type=float, default=0.70)
    parser.add_argument('--center', type=float, default=0.50)
    parser.add_argument('--center-tolerance', type=float, default=0.028)
    parser.add_argument('--target-depth', type=float, default=0.32, help='Legacy alias for --target-robot-x.')
    parser.add_argument('--target-robot-x', type=float, default=None)
    parser.add_argument('--target-robot-y', type=float, default=0.0)
    parser.add_argument('--robot-x-tolerance', type=float, default=0.025)
    parser.add_argument('--robot-y-tolerance', type=float, default=0.025)
    parser.add_argument('--place-target-robot-x', type=float, default=None)
    parser.add_argument('--place-target-robot-y', type=float, default=0.0)
    parser.add_argument('--place-robot-x-tolerance', type=float, default=0.015)
    parser.add_argument('--place-robot-y-tolerance', type=float, default=0.015)
    parser.add_argument('--place-steps', default='')
    parser.add_argument('--hold-after-place', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--hold-place-steps', default='1,2')
    parser.add_argument('--grasp-check-enabled', action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument('--open-gripper-before-approach', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--gripper-open-position', type=int, default=200)
    parser.add_argument('--gripper-open-duration', type=float, default=0.30)
    parser.add_argument('--camera-tilt-deg', type=float, default=45.0)
    parser.add_argument('--camera-height', type=float, default=0.22)
    parser.add_argument('--camera-offset-x', type=float, default=0.06)
    parser.add_argument('--depth-roi-pixels', type=int, default=15)
    parser.add_argument('--depth-tolerance', type=float, default=0.025)
    parser.add_argument('--depth-roi-scale', type=float, default=0.45)
    parser.add_argument('--max-linear-speed', type=float, default=0.09)
    parser.add_argument('--max-angular-speed', type=float, default=0.35)
    parser.add_argument('--visual-period', type=float, default=0.10)
    parser.add_argument('--cmd-pulse', type=float, default=0.04)
    parser.add_argument('--pregrasp-scale', type=float, default=2.4)
    parser.add_argument('--settle-before', type=float, default=0.70)
    parser.add_argument('--settle-after', type=float, default=0.60)
    parser.add_argument('--pick-attempts', type=int, default=3)
    parser.add_argument('--snapshot-period', type=float, default=0.35)
    args = parser.parse_args()
    args.place_class = normalize_place_class(args.place_class)
    if args.target_robot_x is None:
        args.target_robot_x = args.target_depth
    if args.place_target_robot_x is None:
        args.place_target_robot_x = args.target_robot_x
    if args.grasp_check_enabled is None:
        args.grasp_check_enabled = not bool(args.place_class)
    return args


RECORDER_PY = r'''
import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

COLORS = {
    "grass": (70, 190, 95),
    "green": (70, 190, 95),
    "gray": (255, 0, 255),
    "grey": (255, 0, 255),
    "glass": (255, 255, 255),
    "yellow": (30, 215, 235),
    "blue": (230, 115, 60),
}


class Recorder(Node):
    def __init__(self, out_dir, target, period, camera_tilt, camera_height, camera_offset_x, roi_pixels):
        super().__init__("headless_grasp_recorder")
        self.out_dir = Path(out_dir)
        self.target = target
        self.period = max(0.05, float(period))
        self.camera_tilt = math.radians(float(camera_tilt))
        self.camera_height = float(camera_height)
        self.camera_offset_x = float(camera_offset_x)
        self.roi_pixels = max(1, int(roi_pixels))
        self.bridge = CvBridge()
        self.latest = []
        self.latest_depth = None
        self.latest_depth_time = 0.0
        self.camera_k = None
        self.latest_time = 0.0
        self.latest_cmd = (0.0, 0.0)
        self.latest_cmd_time = 0.0
        self.last_nonzero_cmd = (0.0, 0.0)
        self.last_nonzero_cmd_time = 0.0
        self.last_frame = 0.0
        self.index = 0
        self.csv_file = open(self.out_dir / "detections.csv", "w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow([
            "t", "class", "score", "x1", "y1", "x2", "y2", "cx", "area",
            "depth_m", "camera_x", "camera_y", "camera_z", "robot_x", "robot_y", "robot_z",
        ])
        self.cmd_file = open(self.out_dir / "cmd_vel.csv", "w", newline="", encoding="utf-8")
        self.cmd_csv = csv.writer(self.cmd_file)
        self.cmd_csv.writerow(["t", "linear_x", "angular_z", "linear_dir", "angular_dir"])
        self.create_subscription(ObjectsInfo, "/yolo_node/object_detect", self.detect_cb, 10)
        self.create_subscription(Twist, "/controller/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(CameraInfo, "/ascamera/camera_publisher/rgb0/camera_info", self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/ascamera/camera_publisher/depth0/image_raw", self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(Image, "/ascamera/camera_publisher/rgb0/image", self.image_cb, qos_profile_sensor_data)

    def info_cb(self, msg):
        self.camera_k = list(msg.k)

    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth_time = time.time()
        except Exception as exc:
            self.get_logger().warn(f"depth conversion failed: {exc}")

    def cmd_cb(self, msg):
        now = time.time()
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        self.latest_cmd = (linear_x, angular_z)
        self.latest_cmd_time = now
        if abs(linear_x) > 1e-4 or abs(angular_z) > 1e-4:
            self.last_nonzero_cmd = (linear_x, angular_z)
            self.last_nonzero_cmd_time = now
        self.cmd_csv.writerow([
            f"{now:.3f}",
            f"{linear_x:.5f}",
            f"{angular_z:.5f}",
            self.linear_dir(linear_x),
            self.angular_dir(angular_z),
        ])
        self.cmd_file.flush()

    def detect_cb(self, msg):
        now = time.time()
        rows = []
        for obj in msg.objects:
            if len(obj.box) < 4:
                continue
            x1, y1, x2, y2 = [int(v) for v in obj.box[:4]]
            w = int(obj.width or 640)
            h = int(obj.height or 480)
            cx = ((x1 + x2) * 0.5) / max(1, w)
            area = max(0, x2 - x1) * max(0, y2 - y1) / max(1, w * h)
            pose = self.estimate_pose((x1 + x2) * 0.5, (y1 + y2) * 0.5, w, h)
            row = (obj.class_name.strip(), float(obj.score), x1, y1, x2, y2, cx, area, *pose)
            rows.append(row)
            self.csv.writerow([f"{now:.3f}", *row])
        self.csv_file.flush()
        self.latest = rows
        self.latest_time = now

    def image_cb(self, msg):
        now = time.time()
        if now - self.last_frame < self.period:
            return
        self.last_frame = now
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        cv2.line(frame, (w // 2, 0), (w // 2, h), (235, 235, 235), 1)
        cv2.line(frame, (0, h // 2), (w, h // 2), (235, 235, 235), 1)
        status = f"target={self.target} waiting_yolo"
        det_age = now - self.latest_time if self.latest_time > 0.0 else None
        if now - self.latest_time < 1.0:
            status = f"target={self.target} dets={len(self.latest)}"
            for name, score, x1, y1, x2, y2, cx, area, depth_m, camera_x, camera_y, camera_z, robot_x, robot_y, robot_z in self.latest:
                color = COLORS.get(name, (0, 210, 255))
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                if robot_x != "" and robot_y != "":
                    label = f"{name} {score:.2f} x={float(robot_x):.2f} y={float(robot_y):.2f}"
                else:
                    label = f"{name} {score:.2f} cx={cx:.2f}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.56, 2)
                label_y0 = max(0, y1 - th - 10)
                cv2.rectangle(frame, (x1, label_y0), (min(w - 1, x1 + tw + 10), max(y1, label_y0 + th + 10)), color, -1)
                cv2.putText(frame, label, (x1 + 5, max(th + 3, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 2, cv2.LINE_AA)
        self.draw_overlay(frame, now, status, det_age)
        self.index += 1
        cv2.imwrite(str(self.out_dir / f"frame_{self.index:04d}.jpg"), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])

    def draw_overlay(self, frame, now, status, det_age):
        current_cmd_age = now - self.latest_cmd_time if self.latest_cmd_time > 0.0 else None
        last_cmd_age = now - self.last_nonzero_cmd_time if self.last_nonzero_cmd_time > 0.0 else None
        lx, az = self.latest_cmd
        nlx, naz = self.last_nonzero_cmd
        lines = [
            datetime.fromtimestamp(now).strftime("time=%Y-%m-%d %H:%M:%S.%f")[:-3],
            status + (f" det_age={det_age:.2f}s" if det_age is not None else " det_age=none"),
            f"cmd_now lin={lx:+.3f}({self.linear_dir(lx)}) ang={az:+.3f}({self.angular_dir(az)})"
            + (f" age={current_cmd_age:.2f}s" if current_cmd_age is not None else " age=none"),
            f"cmd_last_nonzero lin={nlx:+.3f}({self.linear_dir(nlx)}) ang={naz:+.3f}({self.angular_dir(naz)})"
            + (f" age={last_cmd_age:.2f}s" if last_cmd_age is not None else " age=none"),
        ]
        x, y0 = 10, 24
        line_h = 23
        box_w = min(frame.shape[1] - 16, 620)
        box_h = line_h * len(lines) + 10
        overlay = frame.copy()
        cv2.rectangle(overlay, (6, 6), (6 + box_w, 6 + box_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (x, y0 + i * line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 230, 255), 2)

    @staticmethod
    def linear_dir(value):
        if value > 1e-4:
            return "forward"
        if value < -1e-4:
            return "back"
        return "stop"

    @staticmethod
    def angular_dir(value):
        if value > 1e-4:
            return "z+"
        if value < -1e-4:
            return "z-"
        return "stop"

    def estimate_pose(self, u_rgb, v_rgb, rgb_w, rgb_h):
        if self.latest_depth is None or self.camera_k is None or len(self.camera_k) < 6:
            return ("", "", "", "", "", "", "")
        depth = self.latest_depth
        h, w = depth.shape[:2]
        u = u_rgb * w / max(1, rgb_w)
        v = v_rgb * h / max(1, rgb_h)
        if u < 0 or u >= w or v < 0 or v >= h:
            return ("", "", "", "", "", "", "")
        radius = self.roi_pixels
        x1 = max(0, int(round(u - radius)))
        x2 = min(w - 1, int(round(u + radius)))
        y1 = max(0, int(round(v - radius)))
        y2 = min(h - 1, int(round(v + radius)))
        raw_roi = np.asarray(depth[y1:y2 + 1, x1:x2 + 1])
        if raw_roi.size == 0:
            return ("", "", "", "", "", "", "")
        roi = raw_roi.astype(np.float32)
        if raw_roi.dtype.kind in {"u", "i"}:
            roi *= 0.001
        valid = roi[np.isfinite(roi) & (roi > 0.08) & (roi < 1.50)]
        if valid.size < 20:
            return ("", "", "", "", "", "", "")
        z_c = float(valid.mean())
        fx, fy = float(self.camera_k[0]), float(self.camera_k[4])
        cx_img, cy_img = float(self.camera_k[2]), float(self.camera_k[5])
        sx = w / max(1, rgb_w)
        sy = h / max(1, rgb_h)
        fx *= sx
        fy *= sy
        cx_img *= sx
        cy_img *= sy
        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            return ("", "", "", "", "", "", "")
        x_c = (u - cx_img) * z_c / fx
        y_c = (v - cy_img) * z_c / fy
        robot_x = z_c * math.cos(self.camera_tilt) - y_c * math.sin(self.camera_tilt) + self.camera_offset_x
        robot_y = -x_c
        robot_z = self.camera_height - (y_c * math.cos(self.camera_tilt) + z_c * math.sin(self.camera_tilt))
        return (
            f"{z_c:.4f}",
            f"{x_c:.4f}",
            f"{y_c:.4f}",
            f"{z_c:.4f}",
            f"{robot_x:.4f}",
            f"{robot_y:.4f}",
            f"{robot_z:.4f}",
        )

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        try:
            self.cmd_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--period", type=float, default=0.35)
    parser.add_argument("--camera-tilt", type=float, default=45.0)
    parser.add_argument("--camera-height", type=float, default=0.22)
    parser.add_argument("--camera-offset-x", type=float, default=0.06)
    parser.add_argument("--roi-pixels", type=int, default=15)
    args = parser.parse_args()
    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Recorder(args.out_dir, args.target, args.period, args.camera_tilt, args.camera_height, args.camera_offset_x, args.roi_pixels)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
'''


def q(value) -> str:
    return shlex.quote(str(value))


def remote_script(args, remote_dir: str, remote_tar: str) -> str:
    return f'''
set +e
source /home/ubuntu/ros2_ws/.robotrc

DEBUG_DIR={q(remote_dir)}
TAR_PATH={q(remote_tar)}
CAMERA_TOPIC=/ascamera/camera_publisher/rgb0/image
mkdir -p "$DEBUG_DIR"
mkdir -p "$(dirname "$TAR_PATH")"
cat > "$DEBUG_DIR/params.json" <<'JSON'
{json.dumps(vars(args), ensure_ascii=False, indent=2)}
JSON
cat > "$DEBUG_DIR/recorder.py" <<'PY'
{RECORDER_PY}
PY

matching_pids() {{
  PATTERN="$1"
  ps -eo pid=,comm=,args= 2>/dev/null | while read -r PID COMM ARGS; do
    [ -z "$PID" ] && continue
    [ "$PID" = "$$" ] && continue
    [ "$PID" = "$PPID" ] && continue
    case "$COMM" in
      bash|zsh|sh|dash|timeout|pgrep|grep|ps) continue ;;
    esac
    printf '%s\n' "$ARGS" | grep -F -- "$PATTERN" >/dev/null 2>&1 && printf '%s\n' "$PID"
  done
}}

kill_matching() {{
  PATTERN="$1"
  SIGNAL="${{2:-TERM}}"
  for PID in $(matching_pids "$PATTERN"); do
    kill "-$SIGNAL" "$PID" 2>/dev/null || true
  done
}}

force_kill_matching() {{
  PATTERN="$1"
  kill_matching "$PATTERN" TERM
  sleep 0.2
  kill_matching "$PATTERN" KILL
}}

stop_ours() {{
  timeout 2s ros2 service call /competition_pick_place/stop std_srvs/srv/Trigger "{{}}" >/dev/null 2>&1 || true
  force_kill_matching "competition_node"
  force_kill_matching "yolov11_node"
  force_kill_matching "ros2 launch competition_pick_place"
}}

camera_ready() {{
  timeout 4s ros2 topic echo --once "$CAMERA_TOPIC" >/dev/null 2>&1
}}

ensure_camera() {{
  if camera_ready; then
    echo camera_ready=1 | tee -a "$DEBUG_DIR/session.log"
    return 0
  fi
  for attempt in 1 2 3; do
    force_kill_matching "aurora930_node"
    force_kill_matching "depth_camera.launch.py"
    sleep 2
    nohup ros2 launch peripherals depth_camera.launch.py >> "$DEBUG_DIR/camera.log" 2>&1 &
    echo camera_pid=$! attempt=$attempt | tee -a "$DEBUG_DIR/session.log"
    for i in $(seq 1 35); do
      if camera_ready; then
        echo camera_ready=1 attempt=$attempt wait_seconds=$i | tee -a "$DEBUG_DIR/session.log"
        return 0
      fi
      sleep 1
    done
    echo camera_attempt_${{attempt}}_failed=1 | tee -a "$DEBUG_DIR/session.log"
  done
  echo camera_ready=0 | tee -a "$DEBUG_DIR/session.log"
  return 1
}}

echo "session_start=$(date '+%F %T')" | tee "$DEBUG_DIR/session.log"
stop_ours
sleep 1
ensure_camera
CAMERA_RC=$?
python3 -u "$DEBUG_DIR/recorder.py" --out-dir "$DEBUG_DIR" --target {q(args.target_class)} --period {args.snapshot_period:.3f} --camera-tilt {args.camera_tilt_deg:.3f} --camera-height {args.camera_height:.3f} --camera-offset-x {args.camera_offset_x:.3f} --roi-pixels {int(args.depth_roi_pixels)} > "$DEBUG_DIR/recorder.log" 2>&1 &
REC_PID=$!
echo recorder_pid=$REC_PID | tee -a "$DEBUG_DIR/session.log"

ros2 launch competition_pick_place competition_run.launch.py \\
  target_class:={q(args.target_class)} \\
  place_class:={q(args.place_class)} \\
  dry_run:=false \\
  stop_after_pick:={'false' if args.place_class else 'true'} \\
  exit_on_done:=true \\
  start_navigation:=false \\
  start_base:=false \\
  start_camera:=false \\
  start_yolo:=true \\
  use_nav:=false \\
  use_arm:=true \\
  yolo_model:={q(args.yolo_model)} \\
  yolo_classes:={q(args.yolo_classes)} \\
  yolo_conf:={args.yolo_conf:.3f} \\
  min_score:={args.yolo_conf:.3f} \\
  init_action:=navigation_pick_init_ai \\
  pick_action:=navigation_pick_ai \\
  place_action:=navigation_place_ai \\
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
  depth_roi_scale:={args.depth_roi_scale:.3f} \\
  depth_sample_grid:=5 \\
  depth_min_valid_samples:=20 \\
  depth_min_m:=0.080 \\
  depth_max_m:=1.500 \\
  pick_target_depth_m:={args.target_depth:.3f} \\
  pick_target_robot_x_m:={args.target_robot_x:.3f} \\
  pick_target_robot_y_m:={args.target_robot_y:.3f} \\
  pick_robot_x_tolerance_m:={args.robot_x_tolerance:.3f} \\
  pick_robot_y_tolerance_m:={args.robot_y_tolerance:.3f} \\
  place_target_robot_x_m:={args.place_target_robot_x:.3f} \\
  place_target_robot_y_m:={args.place_target_robot_y:.3f} \\
  place_robot_x_tolerance_m:={args.place_robot_x_tolerance:.3f} \\
  place_robot_y_tolerance_m:={args.place_robot_y_tolerance:.3f} \\
  pick_depth_tolerance_m:={args.depth_tolerance:.3f} \\
  pick_preclose_target_depth_m:=-1.0 \\
  desired_center_x_ratio:={args.center:.4f} \\
  center_tolerance_ratio:={args.center_tolerance:.4f} \\
  stable_frames:=1 \\
  control_mode:=mpc \\
  closed_loop_pick:=true \\
  pick_visual_servo_timeout:=5.0 \\
  visual_servo_period:={args.visual_period:.3f} \\
  visual_servo_command_seconds:={args.cmd_pulse:.3f} \\
  adaptive_servo_timing:=true \\
  visual_servo_min_period:=0.035 \\
  visual_servo_max_period:=0.160 \\
  visual_servo_period_scale:=1.05 \\
  require_fresh_detection_for_control:=false \\
  pick_pregrasp_visual_servo:=false \\
  open_gripper_before_approach:={'true' if args.open_gripper_before_approach else 'false'} \\
  gripper_open_position:={int(args.gripper_open_position)} \\
  gripper_open_duration:={args.gripper_open_duration:.3f} \\
  pick_pregrasp_time_scale:={args.pregrasp_scale:.3f} \\
  pick_pregrasp_min_step_seconds:=0.800 \\
  pick_pregrasp_settle_seconds:={args.settle_before:.3f} \\
  pick_pregrasp_post_step_seconds:={args.settle_after:.3f} \\
  pick_preclose_required:=false \\
  pick_retry_attempts:={int(args.pick_attempts)} \\
  place_steps:={q(args.place_steps)} \\
  hold_after_place:={'true' if args.hold_after_place else 'false'} \\
  hold_place_steps:={q(args.hold_place_steps)} \\
  grasp_check_enabled:={'true' if args.grasp_check_enabled else 'false'} \\
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
  mpc_dt:={args.visual_period:.3f} \\
  mpc_center_response:=1.05 \\
  mpc_area_response:=0.24 \\
  mpc_center_weight:=8.0 \\
  mpc_area_weight:=26.0 \\
  mpc_velocity_weight:=0.08 \\
  mpc_delta_weight:=0.16 \\
  mpc_terminal_weight:=2.2 \\
  mpc_center_gate_ratio:=0.10 > "$DEBUG_DIR/pick.log" 2>&1 &

LAUNCH_PID=$!
echo launch_pid=$LAUNCH_PID | tee -a "$DEBUG_DIR/session.log"
STATUS=timeout
DEADLINE=$((SECONDS + {int(args.timeout)}))
while [ "$SECONDS" -lt "$DEADLINE" ]; do
  if grep -q "DONE" "$DEBUG_DIR/pick.log"; then STATUS=done; break; fi
  if grep -q "FAILSAFE" "$DEBUG_DIR/pick.log"; then STATUS=failsafe; break; fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then STATUS=launch_exited; break; fi
  sleep 1
done
echo run_status=$STATUS | tee -a "$DEBUG_DIR/session.log"
tail -n 160 "$DEBUG_DIR/pick.log" > "$DEBUG_DIR/pick_tail.log" 2>/dev/null || true
kill "$LAUNCH_PID" 2>/dev/null || true
kill "$REC_PID" 2>/dev/null || true
stop_ours
sleep 1
echo frame_count=$(ls "$DEBUG_DIR"/frame_*.jpg 2>/dev/null | wc -l) | tee -a "$DEBUG_DIR/session.log"
echo detection_rows=$(($(wc -l < "$DEBUG_DIR/detections.csv" 2>/dev/null || echo 1)-1)) | tee -a "$DEBUG_DIR/session.log"
tar -czf "$TAR_PATH" -C "$(dirname "$DEBUG_DIR")" "$(basename "$DEBUG_DIR")"
echo tar_path="$TAR_PATH"
[ "$CAMERA_RC" = 0 ] && [ "$STATUS" = done ]
'''


def run_remote(args, remote_dir: str, remote_tar: str) -> int:
    command = f'docker exec -u ubuntu {q(args.container)} bash -lc {q(remote_script(args, remote_dir, remote_tar))}'
    log_progress(f'connecting ssh {args.user}@{args.host}')
    client = connect_ssh(args)

    def run_host(command_text: str, timeout: int = 30) -> tuple[int, str, str]:
        stdin, stdout, stderr = client.exec_command(command_text, timeout=timeout)
        out = stdout.read().decode('utf-8', errors='replace')
        err = stderr.read().decode('utf-8', errors='replace')
        return stdout.channel.recv_exit_status(), out, err

    try:
        log_progress(f'starting remote trial target={args.target_class} place={args.place_class or "none"} timeout={args.timeout}s')
        stdin, stdout, stderr = client.exec_command(command, timeout=args.timeout + 260)
        channel = stdout.channel
        last_heartbeat = time.time()
        while not channel.exit_status_ready():
            emitted = False
            while channel.recv_ready():
                print(channel.recv(4096).decode('utf-8', errors='replace'), end='', flush=True)
                emitted = True
            while channel.recv_stderr_ready():
                print(channel.recv_stderr(4096).decode('utf-8', errors='replace'), end='', flush=True)
                emitted = True
            if emitted:
                last_heartbeat = time.time()
            elif time.time() - last_heartbeat > 10:
                log_progress('remote trial still running...')
                last_heartbeat = time.time()
            time.sleep(0.2)
        while channel.recv_ready():
            print(channel.recv(4096).decode('utf-8', errors='replace'), end='', flush=True)
        while channel.recv_stderr_ready():
            print(channel.recv_stderr(4096).decode('utf-8', errors='replace'), end='', flush=True)
        rc = channel.recv_exit_status()
        log_progress(f'remote trial finished rc={rc}; preparing to pull result package')
        local_base = Path(args.out_dir)
        local_base.mkdir(parents=True, exist_ok=True)
        local_tar = local_base / (Path(remote_tar).name)
        remote_tar_dir = remote_tar.rsplit('/', 1)[0] or '.'
        container_tar = f'{args.container}:{remote_tar}'
        log_progress(f'copying result tar from container: {container_tar} -> {remote_tar}')
        copy_rc, copy_out, copy_err = run_host(
            f'mkdir -p {q(remote_tar_dir)}; docker cp {q(container_tar)} {q(remote_tar)}',
            timeout=45,
        )
        if copy_rc != 0:
            print(f'container_tar_copy_failed_rc={copy_rc}')
            if copy_out.strip():
                print(copy_out.rstrip())
            if copy_err.strip():
                print(copy_err.rstrip())
        sftp = client.open_sftp()
        try:
            try:
                last_fetch_exc = None
                for fetch_attempt in range(1, 7):
                    log_progress(f'fetching result tar attempt {fetch_attempt}/6: {remote_tar}')
                    try:
                        sftp.get(remote_tar, str(local_tar))
                        last_fetch_exc = None
                        break
                    except FileNotFoundError as exc:
                        last_fetch_exc = exc
                        log_progress(f'result tar not visible yet; waiting {fetch_attempt}s')
                        time.sleep(fetch_attempt)
                if last_fetch_exc is not None:
                    raise last_fetch_exc
            except FileNotFoundError as exc:
                partial_dir = local_base / (Path(remote_dir).name + '_partial')
                partial_dir.mkdir(parents=True, exist_ok=True)
                fetched = []
                host_partial_dir = f'{remote_dir}_hostcopy'
                container_debug_dir = f'{args.container}:{remote_dir}'
                log_progress('result tar missing; copying partial debug directory')
                dir_copy_rc, dir_copy_out, dir_copy_err = run_host(
                    f'rm -rf {q(host_partial_dir)}; docker cp {q(container_debug_dir)} {q(host_partial_dir)}',
                    timeout=45,
                )
                if dir_copy_rc != 0:
                    print(f'container_partial_copy_failed_rc={dir_copy_rc}')
                    if dir_copy_out.strip():
                        print(dir_copy_out.rstrip())
                    if dir_copy_err.strip():
                        print(dir_copy_err.rstrip())
                try:
                    for name in sftp.listdir(host_partial_dir):
                        if name.endswith(('.log', '.json', '.csv')) or name.startswith('frame_'):
                            remote_path = f'{host_partial_dir}/{name}'
                            local_path = partial_dir / name
                            if len(fetched) % 25 == 0:
                                log_progress(f'fetching partial files... count={len(fetched)}')
                            sftp.get(remote_path, str(local_path))
                            fetched.append(name)
                except Exception as fetch_exc:
                    print(f'failed_to_fetch_partial_logs={fetch_exc}')
                print(f'remote_exit_code={rc}')
                print(f'partial_result_dir={partial_dir}')
                if fetched:
                    print('partial_files=' + ','.join(fetched[:30]))
                raise RuntimeError(
                    f'remote result tar was not created: {remote_tar}. '
                    f'Partial logs were saved to {partial_dir}'
                ) from exc
        finally:
            sftp.close()
        log_progress(f'local_result_tar={local_tar}')
        return rc
    finally:
        client.close()


def connect_ssh(args):
    last_exc = None
    for attempt in range(1, 7):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                args.host,
                username=args.user,
                password=args.password,
                timeout=15,
                banner_timeout=20,
                auth_timeout=15,
            )
            if attempt > 1:
                log_progress(f'ssh connected attempt={attempt}')
            return client
        except Exception as exc:
            last_exc = exc
            client.close()
            wait_s = min(12, 2 * attempt)
            log_progress(f'ssh connect attempt={attempt}/6 failed={exc}; retrying in {wait_s}s')
            time.sleep(wait_s)
    raise last_exc


def main():
    args = parse_args()
    session = datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{args.target_class}'
    remote_dir = f'/tmp/grasp_headless/{session}'
    remote_tar = f'/tmp/grasp_headless/{session}.tar.gz'
    rc = run_remote(args, remote_dir, remote_tar)
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
