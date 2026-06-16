#!/usr/bin/env python3
import argparse
import io
import shlex
import tarfile
import time
from pathlib import Path

import paramiko


DEFAULT_HOST = '192.168.149.1'
DEFAULT_USER = 'pi'
DEFAULT_PASSWORD = 'raspberrypi'
DEFAULT_CONTAINER = 'MentorPi'
DEFAULT_PACKAGE_DIR = 'competition_pick_place'
DEFAULT_REMOTE_TAR = '/home/pi/competition_pick_place.tar.gz'
DEFAULT_CONTAINER_TAR = '/tmp/competition_pick_place.tar.gz'
DEFAULT_WS = '/home/ubuntu/ros2_ws'


def parse_args():
    parser = argparse.ArgumentParser(description='Deploy and build the competition_pick_place ROS2 package on the robot.')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--user', default=DEFAULT_USER)
    parser.add_argument('--password', default=DEFAULT_PASSWORD)
    parser.add_argument('--container', default=DEFAULT_CONTAINER)
    parser.add_argument('--package-dir', default=DEFAULT_PACKAGE_DIR)
    parser.add_argument('--remote-tar', default=DEFAULT_REMOTE_TAR)
    parser.add_argument('--container-tar', default=DEFAULT_CONTAINER_TAR)
    parser.add_argument('--workspace', default=DEFAULT_WS)
    return parser.parse_args()


def make_tar_bytes(package_dir: Path) -> bytes:
    data = io.BytesIO()
    with tarfile.open(fileobj=data, mode='w:gz') as tar:
        for path in sorted(package_dir.rglob('*')):
            if any(part in {'__pycache__', '.pytest_cache'} for part in path.parts):
                continue
            if path.suffix in {'.pyc', '.pyo'}:
                continue
            arcname = Path(package_dir.name) / path.relative_to(package_dir)
            tar.add(path, arcname=str(arcname), recursive=False)
    return data.getvalue()


def connect(args):
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
                print(f'ssh connected on attempt {attempt}')
            return client
        except Exception as exc:
            last_exc = exc
            client.close()
            wait_s = min(12, 2 * attempt)
            print(f'ssh connect attempt {attempt}/6 failed: {exc}; retrying in {wait_s}s')
            time.sleep(wait_s)
    raise last_exc


def run(client, command, timeout=180):
    print(f'$ {command}')
    stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    rc = stdout.channel.recv_exit_status()
    if out.strip():
        print(out.rstrip())
    if err.strip():
        print(err.rstrip())
    if rc != 0:
        raise RuntimeError(f'command failed rc={rc}: {command}')


def put_bytes(client, data: bytes, remote_path: str):
    print(f'put {len(data)} bytes -> {remote_path}')
    sftp = client.open_sftp()
    try:
        with sftp.file(remote_path, 'wb') as f:
            f.write(data)
    finally:
        sftp.close()


def main():
    args = parse_args()
    package_dir = Path(args.package_dir)
    if not (package_dir / 'package.xml').exists():
        raise SystemExit(f'not a ROS2 package directory: {package_dir}')

    tar_data = make_tar_bytes(package_dir)
    client = connect(args)
    try:
        put_bytes(client, tar_data, args.remote_tar)
        run(client, f'docker ps --format "{{{{.Names}}}}" | grep -Fx {shlex.quote(args.container)}')
        run(client, f'docker cp {shlex.quote(args.remote_tar)} {shlex.quote(args.container)}:{shlex.quote(args.container_tar)}')
        inner = (
            f'set -e; '
            f'rm -rf {shlex.quote(args.workspace)}/src/competition_pick_place; '
            f'rm -rf {shlex.quote(args.workspace)}/build/competition_pick_place; '
            f'rm -rf {shlex.quote(args.workspace)}/install/competition_pick_place; '
            f'mkdir -p {shlex.quote(args.workspace)}/src; '
            f'tar -xzf {shlex.quote(args.container_tar)} -C {shlex.quote(args.workspace)}/src; '
            f'cd {shlex.quote(args.workspace)}; '
            f'source /opt/ros/humble/setup.bash; '
            f'colcon build --symlink-install --packages-select competition_pick_place; '
            f'source install/setup.bash; '
            f'ros2 pkg prefix competition_pick_place'
        )
        run(client, f'docker exec -u ubuntu {shlex.quote(args.container)} bash -lc {shlex.quote(inner)}', timeout=300)
        print('package deploy complete')
    finally:
        client.close()


if __name__ == '__main__':
    main()
