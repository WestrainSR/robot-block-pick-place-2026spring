#!/usr/bin/env python3
import argparse
import shlex

import paramiko


def parse_args():
    parser = argparse.ArgumentParser(description='Run a real green-block pick attempt on the robot.')
    parser.add_argument('--host', default='192.168.149.1')
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--container', default='MentorPi')
    parser.add_argument('--target-class', default='green', choices=['red', 'green', 'blue'])
    parser.add_argument('--timeout', type=int, default=90)
    parser.add_argument('--center', type=float, default=0.50)
    parser.add_argument('--center-tolerance', type=float, default=0.028)
    parser.add_argument('--target-area', type=float, default=0.042)
    parser.add_argument('--area-tolerance', type=float, default=0.012)
    parser.add_argument('--stable-frames', type=int, default=4)
    parser.add_argument('--control-mode', default='mpc', choices=['p', 'mpc'])
    parser.add_argument('--closed-loop-pick', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--pick-visual-servo-timeout', type=float, default=12.0)
    parser.add_argument('--init-action', default='navigation_pick_init_ai')
    parser.add_argument('--pick-action', default='navigation_pick_ai')
    parser.add_argument('--pregrasp-visual-servo', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--preclose-required', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--preclose-center', type=float, default=0.90)
    parser.add_argument('--preclose-target-area', type=float, default=0.073)
    parser.add_argument('--preclose-center-tolerance', type=float, default=0.065)
    parser.add_argument('--preclose-area-tolerance', type=float, default=0.020)
    parser.add_argument('--preclose-stable-frames', type=int, default=1)
    return parser.parse_args()


def main():
    args = parse_args()
    script = f'''
set +e
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export need_compile=True

LOG=/tmp/green_pick_real_run.log
LAUNCH_BASE=competition_run
LAUNCH_FILE="${{LAUNCH_BASE}}.launch.py"
rm -f "$LOG"

echo "== stopping previous competition/yolo nodes =="
timeout 3s ros2 service call /competition_pick_place/stop std_srvs/srv/Trigger "{{}}" 2>/dev/null || true
pkill -f "[c]ompetition_node" 2>/dev/null || true
pkill -f "[y]olov11_node" 2>/dev/null || true
pkill -f "[c]ompetition_run.launch.py" 2>/dev/null || true
sleep 2

echo "== pre-run detection snapshot =="
timeout 8s ros2 topic echo /yolo_node/object_detect --once 2>/dev/null || true

echo "== launching real pick for {args.target_class} =="
ros2 launch competition_pick_place "$LAUNCH_FILE" \\
  target_class:={shlex.quote(args.target_class)} \\
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
  init_action:={shlex.quote(args.init_action)} \\
  pick_action:={shlex.quote(args.pick_action)} \\
  search_timeout:=12.0 \\
  align_timeout:=45.0 \\
  desired_center_x_ratio:={args.center:.4f} \\
  center_tolerance_ratio:={args.center_tolerance:.4f} \\
  pick_target_area_ratio:={args.target_area:.4f} \\
  area_tolerance_ratio:={args.area_tolerance:.4f} \\
  stable_frames:={int(args.stable_frames)} \\
  control_mode:={shlex.quote(args.control_mode)} \\
  closed_loop_pick:={str(bool(args.closed_loop_pick)).lower()} \\
  pick_visual_servo_timeout:={args.pick_visual_servo_timeout:.1f} \\
  pick_pregrasp_visual_servo:={str(bool(args.pregrasp_visual_servo)).lower()} \\
  pick_preclose_required:={str(bool(args.preclose_required)).lower()} \\
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
echo "launch_pid=$LAUNCH_PID"

DEADLINE=$((SECONDS + {int(args.timeout)}))
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

echo "run_status=$STATUS"
echo "== run log tail =="
tail -n 220 "$LOG" || true

echo "== stopping launched nodes =="
kill "$LAUNCH_PID" 2>/dev/null || true
pkill -P "$LAUNCH_PID" 2>/dev/null || true
pkill -f "[c]ompetition_node" 2>/dev/null || true
pkill -f "[y]olov11_node" 2>/dev/null || true
pkill -f "[c]ompetition_run.launch.py" 2>/dev/null || true
sleep 2

if [ "$STATUS" = "done" ]; then
  exit 0
fi
exit 2
'''
    command = f'docker exec -u ubuntu {shlex.quote(args.container)} bash -lc {shlex.quote(script)}'
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password, timeout=10, banner_timeout=10, auth_timeout=10)
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=args.timeout + 90)
        out = stdout.read().decode('utf-8', errors='replace')
        err = stderr.read().decode('utf-8', errors='replace')
        rc = stdout.channel.recv_exit_status()
    finally:
        client.close()
    print(out, end='')
    if err:
        print(err, end='')
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
