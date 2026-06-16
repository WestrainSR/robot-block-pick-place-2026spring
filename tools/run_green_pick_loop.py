#!/usr/bin/env python3
import argparse
import shlex
import time

import paramiko


def parse_args():
    parser = argparse.ArgumentParser(description='Run repeated competition block pick/drop attempts on the robot.')
    parser.add_argument('--host', default='192.168.149.1')
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--container', default='MentorPi')
    parser.add_argument('--target-class', default='grass', choices=['gray', 'grey', 'yellow', 'grass', 'blue'])
    parser.add_argument('--yolo-model', default='tongji')
    parser.add_argument('--yolo-classes', default='gray,yellow,grass,blue')
    parser.add_argument('--yolo-conf', type=float, default=0.70)
    parser.add_argument('--attempts', type=int, default=20)
    parser.add_argument('--interval', type=float, default=5.0)
    parser.add_argument('--attempt-timeout', type=int, default=90)
    parser.add_argument('--center', type=float, default=0.50)
    parser.add_argument('--center-tolerance', type=float, default=0.028)
    parser.add_argument('--target-area', type=float, default=0.043)
    parser.add_argument('--area-tolerance', type=float, default=0.010)
    parser.add_argument('--use-depth-distance', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--depth-topic', default='/ascamera/camera_publisher/depth0/image_raw')
    parser.add_argument('--target-depth', type=float, default=0.32)
    parser.add_argument('--depth-tolerance', type=float, default=0.025)
    parser.add_argument('--depth-roi-scale', type=float, default=0.45)
    parser.add_argument('--stable-frames', type=int, default=4)
    parser.add_argument('--detection-stream-timeout', type=float, default=20.0)
    parser.add_argument('--control-mode', default='mpc', choices=['p', 'mpc'])
    parser.add_argument('--closed-loop-pick', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--init-action', default='navigation_pick_init_ai')
    parser.add_argument('--pick-action', default='navigation_pick_ai')
    parser.add_argument('--place-action', default='navigation_place')
    parser.add_argument('--pregrasp-visual-servo', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--visual-servo-period', type=float, default=0.10)
    parser.add_argument('--visual-servo-command-seconds', type=float, default=0.04)
    parser.add_argument('--adaptive-servo-timing', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--visual-servo-min-period', type=float, default=0.035)
    parser.add_argument('--visual-servo-max-period', type=float, default=0.16)
    parser.add_argument('--visual-servo-period-scale', type=float, default=1.05)
    parser.add_argument('--pregrasp-time-scale', type=float, default=2.4)
    parser.add_argument('--pregrasp-min-step-seconds', type=float, default=0.80)
    parser.add_argument('--pregrasp-settle-seconds', type=float, default=0.70)
    parser.add_argument('--pregrasp-post-step-seconds', type=float, default=0.60)
    parser.add_argument('--preclose-required', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--preclose-fail-on-timeout', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--preclose-center', type=float, default=0.90)
    parser.add_argument('--preclose-target-area', type=float, default=0.095)
    parser.add_argument('--preclose-center-tolerance', type=float, default=0.065)
    parser.add_argument('--preclose-area-tolerance', type=float, default=0.020)
    parser.add_argument('--preclose-stable-frames', type=int, default=1)
    parser.add_argument('--pick-retry-attempts', type=int, default=3)
    parser.add_argument('--gripper-empty-close-position', type=int, default=500)
    parser.add_argument('--gripper-grasp-min-gap', type=int, default=30)
    parser.add_argument('--gripper-check-delay', type=float, default=0.35)
    parser.add_argument('--max-linear-speed', type=float, default=0.035)
    parser.add_argument('--max-angular-speed', type=float, default=0.14)
    return parser.parse_args()


def bool_text(value: bool) -> str:
    return str(bool(value)).lower()


def build_remote_script(args) -> str:
    target = shlex.quote(args.target_class)
    yolo_model = shlex.quote(args.yolo_model)
    yolo_classes = shlex.quote(args.yolo_classes)
    init_action = shlex.quote(args.init_action)
    pick_action = shlex.quote(args.pick_action)
    place_action = shlex.quote(args.place_action)
    control_mode = shlex.quote(args.control_mode)
    depth_topic = shlex.quote(args.depth_topic)
    return f'''
set +e
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export need_compile=True

LOG_DIR=/tmp/green_pick_loop
mkdir -p "$LOG_DIR"
SUMMARY="$LOG_DIR/summary.csv"
rm -f "$SUMMARY"
echo "attempt,status,elapsed_seconds,log" > "$SUMMARY"

cleanup_nodes() {{
  timeout 2s ros2 service call /competition_pick_place/stop std_srvs/srv/Trigger "{{}}" >/dev/null 2>&1 || true
  pkill -f "[c]ompetition_node" 2>/dev/null || true
  pkill -f "[y]olov11_node" 2>/dev/null || true
  for PID in $(pgrep -f "ros2 launch competition_pick_place" 2>/dev/null || true); do
    if [ "$PID" != "$$" ] && [ "$PID" != "$PPID" ]; then
      kill "$PID" 2>/dev/null || true
    fi
  done
  sleep 1
}}

run_place_action() {{
  ACTION_NAME="$1" python3 - <<'PY'
import os
import time

import rclpy
from rclpy.node import Node
from servo_controller.action_group_controller import ActionGroupController
from servo_controller_msgs.msg import ServosPosition
from std_srvs.srv import Trigger

action_name = os.environ['ACTION_NAME']
rclpy.init()
node = Node('loop_action_group_runner')
pub = node.create_publisher(ServosPosition, 'servo_controller', 1)
client = node.create_client(Trigger, '/controller_manager/init_finish')
client.wait_for_service(timeout_sec=8.0)
for _ in range(8):
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
controller = ActionGroupController(pub, '/home/ubuntu/software/arm_pc/ActionGroups')
controller.run_action(action_name)
time.sleep(0.3)
node.destroy_node()
rclpy.shutdown()
PY
}}

run_attempt() {{
  ATTEMPT="$1"
  LOG="$LOG_DIR/attempt_${{ATTEMPT}}.log"
  rm -f "$LOG"

  echo "== attempt $ATTEMPT/{int(args.attempts)} start $(date '+%F %T') =="
  cleanup_nodes
  START_SECONDS=$SECONDS

  ros2 launch competition_pick_place competition_run.launch.py \\
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
    yolo_classes:={yolo_classes} \\
    yolo_conf:={args.yolo_conf:.3f} \\
    init_action:={init_action} \\
    pick_action:={pick_action} \\
    search_timeout:=12.0 \\
    align_timeout:=45.0 \\
    wait_for_detection_stream:=true \\
    detection_stream_timeout:={args.detection_stream_timeout:.1f} \\
    detection_ready_min_messages:=1 \\
    wait_for_target_before_search:=true \\
    use_depth_distance:={bool_text(args.use_depth_distance)} \\
    depth_topic:={depth_topic} \\
    camera_info_topic:=/ascamera/camera_publisher/rgb0/camera_info \\
    use_robot_frame_distance:=true \\
    camera_tilt_deg:=45.0 \\
    camera_height_m:=0.22 \\
    camera_offset_x_m:=0.06 \\
    depth_roi_pixels:=15 \\
    depth_stale_seconds:=0.800 \\
    depth_unit_scale:=0.001 \\
    depth_roi_scale:={args.depth_roi_scale:.3f} \\
    depth_sample_grid:=5 \\
    depth_min_valid_samples:=20 \\
    depth_min_m:=0.080 \\
    depth_max_m:=1.500 \\
    pick_target_depth_m:={args.target_depth:.3f} \\
    pick_target_robot_x_m:={args.target_depth:.3f} \\
    pick_target_robot_y_m:=0.0 \\
    pick_robot_x_tolerance_m:={args.depth_tolerance:.3f} \\
    pick_robot_y_tolerance_m:=0.025 \\
    pick_depth_tolerance_m:={args.depth_tolerance:.3f} \\
    pick_preclose_target_depth_m:=-1.0 \\
    desired_center_x_ratio:={args.center:.4f} \\
    center_tolerance_ratio:={args.center_tolerance:.4f} \\
    pick_target_area_ratio:={args.target_area:.4f} \\
    area_tolerance_ratio:={args.area_tolerance:.4f} \\
    stable_frames:={int(args.stable_frames)} \\
    control_mode:={control_mode} \\
    closed_loop_pick:={bool_text(args.closed_loop_pick)} \\
    visual_servo_period:={args.visual_servo_period:.3f} \\
    visual_servo_command_seconds:={args.visual_servo_command_seconds:.3f} \\
    adaptive_servo_timing:={bool_text(args.adaptive_servo_timing)} \\
    visual_servo_min_period:={args.visual_servo_min_period:.3f} \\
    visual_servo_max_period:={args.visual_servo_max_period:.3f} \\
    visual_servo_period_scale:={args.visual_servo_period_scale:.3f} \\
    require_fresh_detection_for_control:=true \\
    pick_pregrasp_visual_servo:={bool_text(args.pregrasp_visual_servo)} \\
    pick_pregrasp_time_scale:={args.pregrasp_time_scale:.3f} \\
    pick_pregrasp_min_step_seconds:={args.pregrasp_min_step_seconds:.3f} \\
    pick_pregrasp_settle_seconds:={args.pregrasp_settle_seconds:.3f} \\
    pick_pregrasp_post_step_seconds:={args.pregrasp_post_step_seconds:.3f} \\
    pick_preclose_required:={bool_text(args.preclose_required)} \\
    pick_preclose_fail_on_timeout:={bool_text(args.preclose_fail_on_timeout)} \\
    pick_preclose_center_x_ratio:={args.preclose_center:.4f} \\
    pick_preclose_target_area_ratio:={args.preclose_target_area:.4f} \\
    pick_preclose_center_tolerance_ratio:={args.preclose_center_tolerance:.4f} \\
    pick_preclose_area_tolerance_ratio:={args.preclose_area_tolerance:.4f} \\
    pick_preclose_stable_frames:={int(args.preclose_stable_frames)} \\
    pick_retry_attempts:={int(args.pick_retry_attempts)} \\
    grasp_check_enabled:=true \\
    gripper_state_topic:=/controller_manager/servo_states \\
    gripper_servo_id:=10 \\
    gripper_empty_close_position:={int(args.gripper_empty_close_position)} \\
    gripper_grasp_min_gap:={int(args.gripper_grasp_min_gap)} \\
    gripper_check_delay:={args.gripper_check_delay:.3f} \\
    gripper_feedback_timeout:=2.0 \\
    angular_k:=0.80 \\
    max_linear_speed:={args.max_linear_speed:.4f} \\
    max_angular_speed:={args.max_angular_speed:.4f} \\
    search_angular_speed:=0.12 \\
    mpc_horizon:=8 \\
    mpc_dt:={args.visual_servo_period:.3f} \\
    mpc_center_response:=1.05 \\
    mpc_area_response:=0.24 \\
    mpc_center_weight:=8.0 \\
    mpc_area_weight:=26.0 \\
    mpc_velocity_weight:=0.08 \\
    mpc_delta_weight:=0.16 \\
    mpc_terminal_weight:=2.2 \\
    mpc_center_gate_ratio:=0.10 > "$LOG" 2>&1 &

  LAUNCH_PID=$!
  DEADLINE=$((SECONDS + {int(args.attempt_timeout)}))
  STATUS=timeout
  while [ "$SECONDS" -lt "$DEADLINE" ]; do
    if grep -q "DONE" "$LOG"; then
      STATUS=done
      break
    fi
    if grep -q "FAILSAFE" "$LOG"; then
      STATUS=failsafe
      break
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      STATUS=launch_exited
      break
    fi
    sleep 1
  done

  ELAPSED=$((SECONDS - START_SECONDS))
  echo "attempt=$ATTEMPT status=$STATUS elapsed=${{ELAPSED}}s log=$LOG"
  tail -n 45 "$LOG" | sed "s/^/[attempt $ATTEMPT] /" || true

  kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -P "$LAUNCH_PID" 2>/dev/null || true
  cleanup_nodes

  echo "$ATTEMPT,$STATUS,$ELAPSED,$LOG" >> "$SUMMARY"
  [ "$STATUS" = "done" ]
}}

SUCCESS=0
FAILED=0
for ATTEMPT in $(seq 1 {int(args.attempts)}); do
  if run_attempt "$ATTEMPT"; then
    SUCCESS=$((SUCCESS + 1))
    echo "attempt $ATTEMPT software_success: running drop action {args.place_action}"
    run_place_action {place_action}
  else
    FAILED=$((FAILED + 1))
    echo "attempt $ATTEMPT failed: skip drop action"
  fi

  if [ "$ATTEMPT" -lt {int(args.attempts)} ]; then
    echo "sleep {float(args.interval):.1f}s before next attempt"
    sleep {float(args.interval):.1f}
  fi
done

cleanup_nodes
TOTAL=$((SUCCESS + FAILED))
if [ "$TOTAL" -gt 0 ]; then
  RATE=$(python3 - <<PY
success=$SUCCESS
total=$TOTAL
print(f"{{success / total:.3f}}")
PY
)
else
  RATE=0.000
fi

echo "== loop summary =="
cat "$SUMMARY"
echo "software_success=$SUCCESS"
echo "failed=$FAILED"
echo "total=$TOTAL"
echo "software_success_rate=$RATE"

[ "$SUCCESS" -gt 0 ]
'''


def main():
    args = parse_args()
    if args.attempts <= 0:
        raise SystemExit('--attempts must be positive')
    if args.interval < 0:
        raise SystemExit('--interval must be non-negative')

    script = build_remote_script(args)
    command = f'docker exec -u ubuntu {shlex.quote(args.container)} bash -lc {shlex.quote(script)}'
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password, timeout=10, banner_timeout=10, auth_timeout=10)
    try:
        stdin, stdout, stderr = client.exec_command(
            command,
            timeout=args.attempts * (args.attempt_timeout + int(args.interval) + 30) + 180,
        )
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
    finally:
        client.close()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
