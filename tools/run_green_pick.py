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
    parser.add_argument('--target-area', type=float, default=0.043)
    parser.add_argument('--area-tolerance', type=float, default=0.010)
    parser.add_argument('--stable-frames', type=int, default=4)
    parser.add_argument('--detection-stream-timeout', type=float, default=20.0)
    parser.add_argument('--control-mode', default='mpc', choices=['p', 'mpc'])
    parser.add_argument('--closed-loop-pick', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--pick-visual-servo-timeout', type=float, default=5.0)
    parser.add_argument('--visual-servo-period', type=float, default=0.10)
    parser.add_argument('--init-action', default='navigation_pick_init_ai')
    parser.add_argument('--pick-action', default='navigation_pick_ai')
    parser.add_argument('--pregrasp-visual-servo', action=argparse.BooleanOptionalAction, default=True)
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
    parser.add_argument('--mpc-horizon', type=int, default=8)
    parser.add_argument('--mpc-dt', type=float, default=0.10)
    parser.add_argument('--mpc-center-response', type=float, default=1.05)
    parser.add_argument('--mpc-area-response', type=float, default=0.24)
    parser.add_argument('--mpc-center-weight', type=float, default=8.0)
    parser.add_argument('--mpc-area-weight', type=float, default=26.0)
    parser.add_argument('--mpc-velocity-weight', type=float, default=0.08)
    parser.add_argument('--mpc-delta-weight', type=float, default=0.16)
    parser.add_argument('--mpc-terminal-weight', type=float, default=2.2)
    parser.add_argument('--mpc-center-gate-ratio', type=float, default=0.10)
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
  wait_for_detection_stream:=true \\
  detection_stream_timeout:={args.detection_stream_timeout:.1f} \\
  detection_ready_min_messages:=1 \\
  desired_center_x_ratio:={args.center:.4f} \\
  center_tolerance_ratio:={args.center_tolerance:.4f} \\
  pick_target_area_ratio:={args.target_area:.4f} \\
  area_tolerance_ratio:={args.area_tolerance:.4f} \\
  stable_frames:={int(args.stable_frames)} \\
  control_mode:={shlex.quote(args.control_mode)} \\
  closed_loop_pick:={str(bool(args.closed_loop_pick)).lower()} \\
  pick_visual_servo_timeout:={args.pick_visual_servo_timeout:.1f} \\
  visual_servo_period:={args.visual_servo_period:.3f} \\
  pick_pregrasp_visual_servo:={str(bool(args.pregrasp_visual_servo)).lower()} \\
  pick_pregrasp_time_scale:={args.pregrasp_time_scale:.3f} \\
  pick_pregrasp_min_step_seconds:={args.pregrasp_min_step_seconds:.3f} \\
  pick_pregrasp_settle_seconds:={args.pregrasp_settle_seconds:.3f} \\
  pick_pregrasp_post_step_seconds:={args.pregrasp_post_step_seconds:.3f} \\
  pick_preclose_required:={str(bool(args.preclose_required)).lower()} \\
  pick_preclose_fail_on_timeout:={str(bool(args.preclose_fail_on_timeout)).lower()} \\
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
  mpc_horizon:={int(args.mpc_horizon)} \\
  mpc_dt:={args.mpc_dt:.3f} \\
  mpc_center_response:={args.mpc_center_response:.3f} \\
  mpc_area_response:={args.mpc_area_response:.3f} \\
  mpc_center_weight:={args.mpc_center_weight:.3f} \\
  mpc_area_weight:={args.mpc_area_weight:.3f} \\
  mpc_velocity_weight:={args.mpc_velocity_weight:.3f} \\
  mpc_delta_weight:={args.mpc_delta_weight:.3f} \\
  mpc_terminal_weight:={args.mpc_terminal_weight:.3f} \\
  mpc_center_gate_ratio:={args.mpc_center_gate_ratio:.3f} > "$LOG" 2>&1 &
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
