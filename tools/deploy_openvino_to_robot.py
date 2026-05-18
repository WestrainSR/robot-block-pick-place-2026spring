#!/usr/bin/env python3
import argparse
import shlex
from pathlib import Path

import paramiko


DEFAULT_HOST = '192.168.149.1'
DEFAULT_USER = 'pi'
DEFAULT_PASSWORD = 'raspberrypi'
DEFAULT_CONTAINER = 'MentorPi'
DEFAULT_LOCAL_DIR = 'deployment/competition_blocks_openvino'
DEFAULT_REMOTE_STAGE = '/home/pi/competition_blocks_openvino'
DEFAULT_CONTAINER_MODELS = '/home/ubuntu/ros2_ws/src/yolov11_detect/models'


def parse_args():
    parser = argparse.ArgumentParser(description='Deploy competition OpenVINO model files to the robot Docker container.')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--user', default=DEFAULT_USER)
    parser.add_argument('--password', default=DEFAULT_PASSWORD)
    parser.add_argument('--container', default=DEFAULT_CONTAINER)
    parser.add_argument('--local-dir', default=DEFAULT_LOCAL_DIR)
    parser.add_argument('--remote-stage', default=DEFAULT_REMOTE_STAGE)
    parser.add_argument('--container-models', default=DEFAULT_CONTAINER_MODELS)
    parser.add_argument('--skip-container-copy', action='store_true')
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


def run(client, command, timeout=60):
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
    return out


def sftp_put(client, local_path, remote_path):
    print(f'put {local_path} -> {remote_path}')
    sftp = client.open_sftp()
    try:
        sftp.put(str(local_path), remote_path)
    finally:
        sftp.close()


def main():
    args = parse_args()
    local_dir = Path(args.local_dir)
    xml = local_dir / 'competition_blocks.xml'
    bin_file = local_dir / 'competition_blocks.bin'
    if not xml.exists() or not bin_file.exists():
        raise SystemExit(f'missing OpenVINO model files in {local_dir}')

    client = connect(args)
    try:
        run(client, f'mkdir -p {shlex.quote(args.remote_stage)}')
        sftp_put(client, xml, f'{args.remote_stage}/competition_blocks.xml')
        sftp_put(client, bin_file, f'{args.remote_stage}/competition_blocks.bin')

        if not args.skip_container_copy:
            run(client, f'docker ps --format "{{{{.Names}}}}" | grep -Fx {shlex.quote(args.container)}')
            run(client, f'docker exec -u ubuntu {shlex.quote(args.container)} mkdir -p {shlex.quote(args.container_models)}')
            run(
                client,
                f'docker cp {shlex.quote(args.remote_stage)}/competition_blocks.xml '
                f'{shlex.quote(args.container)}:{shlex.quote(args.container_models)}/competition_blocks.xml',
            )
            run(
                client,
                f'docker cp {shlex.quote(args.remote_stage)}/competition_blocks.bin '
                f'{shlex.quote(args.container)}:{shlex.quote(args.container_models)}/competition_blocks.bin',
            )
            run(
                client,
                f'docker exec -u ubuntu {shlex.quote(args.container)} bash -lc '
                + shlex.quote(
                    f'ls -lh {shlex.quote(args.container_models)}/competition_blocks.* '
                    f'&& python3 - <<PY\n'
                    f'from pathlib import Path\n'
                    f'p=Path("{args.container_models}")\n'
                    f'print((p/"competition_blocks.xml").exists(), (p/"competition_blocks.bin").exists())\n'
                    f'PY'
                ),
            )
        print('deploy complete')
    finally:
        client.close()


if __name__ == '__main__':
    main()
