#!/usr/bin/env python3
import argparse
import shlex
import time

import paramiko


def parse_args():
    parser = argparse.ArgumentParser(description='Run repeated green-block pick/drop attempts on the robot.')
    parser.add_argument('--host', default='192.168.149.1')
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--container', default='MentorPi')
    parser.add_argument('--target-class', default='green', choices=['red', 'green', 'blue'])
    parser.add_argument('--attempts', type=int, default=20)
    parser.add_argument('--interval', type=float, default=5.0)
    parser.add_argument('--attempt-timeout', type=int, default=90)
    parser.add_argument('--center', type=float, default=0.50)
    parser.add_argument('--center-tolerance', type=float, default=0.028)
    parser.add_argument('--target-area', type=float, default=0.060)
    parser.add_argument('--area-tolerance', type=float, default=0.010)
    parser.add_argument('--stable-frames', type=int, default=4)
    parser.add_argument('--control-mode', default='mpc', choices=['p', 'mpc'])
    parser.add_argument('--closed-loop-pick', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--init-action', default='navigation_pick_init_ai')
    parser.add_argument('--pick-action', default='navigation_pick_ai')
    parser.add_argument('--place-action', default='navigation_place')
    parser.add_argument('--pregrasp-visual-servo', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--preclose-required', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--preclose-center', type=float, default=0.90)
    parser.add_argument('--preclose-target-area', type=float, default=0.095)
    parser.add_argument('--preclose-center-tolerance', type=float, default=0.065)
    parser.add_argument('--preclose-area-tolerance', type=float, default=0.020)
    parser.add_argument('--preclose-stable-frames', type=int, default=1)
    return parser.parse_args()


def bool_text(value: bool) -> str:
    return str(bool(value)).lower()


def build_remote_script(args) -> str:
    target = shlex.quote(args.target_class)
    init_action = shlex.quote(args.init_action)
    pick_action = shlex.quote(args.pick_action)
    place_action = shlex.quote(args.place_action)
    control_mode = shlex.quote(args.control_mode)
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
    yolo_model:=competition_blocks \\
    yolo_conf:=0.70 \\
    init_action:={init_action} \\
    pick_action:={pick_action} \\
    search_timeout:=12.0 \\
    align_timeout:=45.0 \\
    desired_center_x_ratio:={args.center:.4f} \\
    center_tolerance_ratio:={args.center_tolerance:.4f} \\
    pick_target_area_ratio:={args.target_area:.4f} \\
    area_tolerance_ratio:={args.area_tolerance:.4f} \\
    stable_frames:={int(args.stable_frames)} \\
    control_mode:={control_mode} \\
    closed_loop_pick:={bool_text(args.closed_loop_pick)} \\
    pick_pregrasp_visual_servo:={bool_text(args.pregrasp_visual_servo)} \\
    pick_preclose_required:={bool_text(args.preclose_required)} \\
    pick_preclose_center_x_ratio:={args.preclose_center:.4f} \\
    pick_preclose_target_area_ratio:={args.preclose_target_area:.4f} \\
    pick_preclose_center_tolerance_ratio:={args.preclose_center_tolerance:.4f} \\
    pick_preclose_area_tolerance_ratio:={args.preclose_area_tolerance:.4f} \\
    pick_preclose_stable_frames:={int(args.preclose_stable_frames)} \\
    angular_k:=0.80 \\
    max_linear_speed:=0.06 \\
    max_angular_speed:=0.20 \\
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
