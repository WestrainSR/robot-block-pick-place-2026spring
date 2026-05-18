#!/usr/bin/env python3
import argparse
import shlex

import paramiko


def parse_args():
    parser = argparse.ArgumentParser(description='Check robot readiness for a real pick attempt.')
    parser.add_argument('--host', default='192.168.149.1')
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--container', default='MentorPi')
    return parser.parse_args()


def main():
    args = parse_args()
    script = r'''
set +e
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export need_compile=True

echo "== action groups =="
ls -lh /home/ubuntu/software/arm_pc/ActionGroups/navigation_pick_init* \
       /home/ubuntu/software/arm_pc/ActionGroups/navigation_pick* \
       /home/ubuntu/software/arm_pc/ActionGroups/navigation_place* 2>&1

echo "== key services =="
ros2 service list | grep -E '/controller_manager/init_finish|/competition_pick_place/stop' || true

echo "== key topics =="
ros2 topic list | sort | grep -E '/controller/cmd_vel|servo_controller|ascamera|yolo|object_detect' || true

echo "== key nodes =="
ros2 node list | sort | grep -E 'controller|servo|ascamera|yolo|competition' || true

echo "== model files =="
ls -lh /home/ubuntu/ros2_ws/src/yolov11_detect/models/competition_blocks.* 2>&1
'''
    command = f'docker exec -u ubuntu {shlex.quote(args.container)} bash -lc {shlex.quote(script)}'
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password, timeout=10, banner_timeout=10, auth_timeout=10)
    try:
        stdin, stdout, stderr = client.exec_command(command, timeout=60)
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
