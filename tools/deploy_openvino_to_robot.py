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
    parser.add_argument('--model-name', default='competition_blocks')
    parser.add_argument('--xml', default='', help='Optional explicit local .xml path.')
    parser.add_argument('--bin', default='', help='Optional explicit local .bin path.')
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
    model_name = args.model_name.strip() or 'competition_blocks'
    xml = Path(args.xml) if args.xml else local_dir / f'{model_name}.xml'
    bin_file = Path(args.bin) if args.bin else local_dir / f'{model_name}.bin'
    if not xml.exists() or not bin_file.exists():
        raise SystemExit(f'missing OpenVINO model files: {xml} / {bin_file}')

    client = connect(args)
    try:
        run(client, f'mkdir -p {shlex.quote(args.remote_stage)}')
        remote_xml = f'{args.remote_stage}/{model_name}.xml'
        remote_bin = f'{args.remote_stage}/{model_name}.bin'
        sftp_put(client, xml, remote_xml)
        sftp_put(client, bin_file, remote_bin)

        if not args.skip_container_copy:
            run(client, f'docker ps --format "{{{{.Names}}}}" | grep -Fx {shlex.quote(args.container)}')
            run(client, f'docker exec -u ubuntu {shlex.quote(args.container)} mkdir -p {shlex.quote(args.container_models)}')
            run(
                client,
                f'docker cp {shlex.quote(remote_xml)} '
                f'{shlex.quote(args.container)}:{shlex.quote(args.container_models)}/{model_name}.xml',
            )
            run(
                client,
                f'docker cp {shlex.quote(remote_bin)} '
                f'{shlex.quote(args.container)}:{shlex.quote(args.container_models)}/{model_name}.bin',
            )
            run(
                client,
                f'docker exec -u ubuntu {shlex.quote(args.container)} bash -lc '
                + shlex.quote(
                    f'ls -lh {shlex.quote(args.container_models)}/{model_name}.* '
                    f'&& python3 - <<PY\n'
                    f'from pathlib import Path\n'
                    f'p=Path("{args.container_models}")\n'
                    f'print((p/"{model_name}.xml").exists(), (p/"{model_name}.bin").exists())\n'
                    f'PY'
                ),
            )
        print('deploy complete')
    finally:
        client.close()


if __name__ == '__main__':
    main()
