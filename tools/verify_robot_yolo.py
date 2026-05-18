#!/usr/bin/env python3
import argparse
import shlex

import paramiko


DEFAULT_HOST = '192.168.149.1'
DEFAULT_USER = 'pi'
DEFAULT_PASSWORD = 'raspberrypi'
DEFAULT_CONTAINER = 'MentorPi'


def parse_args():
    parser = argparse.ArgumentParser(description='Start robot camera+YOLO in Docker and verify object_detect output.')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--user', default=DEFAULT_USER)
    parser.add_argument('--password', default=DEFAULT_PASSWORD)
    parser.add_argument('--container', default=DEFAULT_CONTAINER)
    parser.add_argument('--target-class', default='red', choices=['red', 'green', 'blue'])
    parser.add_argument('--model', default='competition_blocks')
    parser.add_argument('--conf', default='0.70')
    parser.add_argument('--startup-seconds', type=int, default=25)
    parser.add_argument('--echo-timeout', type=int, default=15)
    return parser.parse_args()


def connect(args):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        args.host,
        username=args.user,
        password=args.password,
        timeout=10,
        banner_timeout=10,
        auth_timeout=10,
    )
    return client


def run(client, command, timeout=90):
    stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    rc = stdout.channel.recv_exit_status()
    return rc, out, err


def main():
    args = parse_args()
    inner = f"""
set +e
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export need_compile=True

LOG=/tmp/competition_yolo_verify.log
LAUNCH_BASE=competition_run
LAUNCH_FILE="${{LAUNCH_BASE}}.launch.py"
rm -f "$LOG"
pkill -f "[c]ompetition_run.launch.py" 2>/dev/null || true

echo "== model files =="
ls -lh /home/ubuntu/ros2_ws/src/yolov11_detect/models/competition_blocks.* || true

echo "== package check =="
ros2 pkg prefix competition_pick_place
PKG_RC=$?
ros2 pkg prefix yolov11_detect
YOLO_PKG_RC=$?
if [ "$PKG_RC" -ne 0 ] || [ "$YOLO_PKG_RC" -ne 0 ]; then
  echo "required ROS packages are missing"
  exit 10
fi

echo "== starting launch =="
ros2 launch competition_pick_place "$LAUNCH_FILE" \\
  dry_run:=true \\
  start_camera:=false \\
  start_yolo:=true \\
  start_base:=false \\
  start_navigation:=false \\
  use_nav:=false \\
  use_arm:=false \\
  yolo_model:={shlex.quote(args.model)} \\
  yolo_conf:={shlex.quote(args.conf)} \\
  target_class:={shlex.quote(args.target_class)} > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "launch_pid=$LAUNCH_PID"

sleep {int(args.startup_seconds)}

echo "== topics =="
ros2 topic list | sort | grep -E 'ascamera|yolo|object_detect|camera' || true

echo "== object_detect once =="
timeout {int(args.echo_timeout)}s ros2 topic echo /yolo_node/object_detect --once
ECHO_RC=$?
echo "object_detect_echo_rc=$ECHO_RC"

echo "== launch log tail =="
tail -n 120 "$LOG" || true

kill "$LAUNCH_PID" 2>/dev/null || true
pkill -P "$LAUNCH_PID" 2>/dev/null || true
pkill -f "[c]ompetition_run.launch.py" 2>/dev/null || true
sleep 2

if ros2 topic list | grep -F /yolo_node/object_detect >/dev/null; then
  exit 0
fi
exit "$ECHO_RC"
"""
    command = f'docker exec -u ubuntu {shlex.quote(args.container)} bash -lc {shlex.quote(inner)}'
    client = connect(args)
    try:
        rc, out, err = run(client, command, timeout=args.startup_seconds + args.echo_timeout + 90)
    finally:
        client.close()
    print(out, end='')
    if err:
        print(err, end='')
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
