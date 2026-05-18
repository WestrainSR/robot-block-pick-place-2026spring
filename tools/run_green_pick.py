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
  search_timeout:=12.0 \\
  align_timeout:=45.0 \\
  pick_target_area_ratio:=0.052 \\
  area_tolerance_ratio:=0.014 \\
  max_linear_speed:=0.08 > "$LOG" 2>&1 &
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
