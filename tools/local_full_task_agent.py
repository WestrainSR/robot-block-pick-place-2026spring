#!/usr/bin/env python3
import argparse
import shlex
import sys
import time
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import paramiko


DEFAULT_HOST = '192.168.149.1'
DEFAULT_USER = 'pi'
DEFAULT_PASSWORD = 'raspberrypi'
DEFAULT_CONTAINER = 'MentorPi'
DEFAULT_ROBOT_ENV_FILE = '/home/ubuntu/ros2_ws/.typerc'


def import_delivery_agent():
    repo_root = Path(__file__).resolve().parents[1]
    package_root = repo_root / 'competition_pick_place'
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    from competition_pick_place import delivery_agent

    return delivery_agent


def parse_args(argv: Sequence[str] | None = None) -> Tuple[argparse.Namespace, List[str]]:
    parser = argparse.ArgumentParser(
        description='Local SSH agent for the full navigation, L-shape stacking, delivery, and return flow.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('action', choices=['run', 'stop', 'status', 'command'], nargs='?', default='run')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--user', default=DEFAULT_USER)
    parser.add_argument('--password', default=DEFAULT_PASSWORD)
    parser.add_argument('--container', default=DEFAULT_CONTAINER)
    parser.add_argument('--robot-env-file', default=DEFAULT_ROBOT_ENV_FILE)
    parser.add_argument('--timeout', type=float, default=0.0, help='Maximum seconds for run; 0 means wait until launch exits.')
    parser.add_argument('--stop-existing', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--stop-yolo', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--dry-command', action='store_true', help='Print parsed launch command and exit.')
    args, task_args = parser.parse_known_args(argv)
    return args, task_args


def connect(args: argparse.Namespace) -> paramiko.SSHClient:
    last_exc = None
    for attempt in range(1, 7):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            print(f'ssh_connect attempt={attempt} host={args.host} user={args.user}', flush=True)
            client.connect(
                args.host,
                username=args.user,
                password=args.password,
                timeout=15,
                banner_timeout=20,
                auth_timeout=15,
            )
            return client
        except Exception as exc:
            last_exc = exc
            client.close()
            wait_s = min(12, attempt * 2)
            print(f'ssh_connect_failed attempt={attempt} error={exc}; retry_in={wait_s}s', flush=True)
            time.sleep(wait_s)
    raise RuntimeError(f'failed to connect to {args.host}') from last_exc


def quote_list(values: Iterable[str]) -> str:
    return shlex.join([str(value) for value in values])


def source_env_script(args: argparse.Namespace, inner_command: str) -> str:
    env_file = shlex.quote(args.robot_env_file)
    return (
        'set -e; '
        'export need_compile=True; '
        f'if [ -f {env_file} ]; then source {env_file}; else echo robot_env_missing={env_file}; fi; '
        'for optional_setup in '
        '/home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash '
        '/home/ubuntu/third_party_ros2/third_party_ws/install/local_setup.bash '
        '/home/ubuntu/deptrum_ws/install/setup.bash '
        '/home/ubuntu/deptrum_ws/install/local_setup.bash '
        '/home/ubuntu/aurora930_ws/install/setup.bash '
        '/home/ubuntu/aurora930_ws/install/local_setup.bash '
        '/home/ubuntu/ros2_ws/install/setup.bash; do '
        'if [ -f "$optional_setup" ]; then source "$optional_setup"; echo sourced_setup="$optional_setup"; fi; '
        'done; '
        f'{inner_command}'
    )


def docker_exec(args: argparse.Namespace, inner_script: str) -> str:
    return f'docker exec -u ubuntu {shlex.quote(args.container)} bash -lc {shlex.quote(inner_script)}'


def run_capture(client: paramiko.SSHClient, command: str, timeout: float = 60.0) -> Tuple[int, str, str]:
    stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    rc = stdout.channel.recv_exit_status()
    return rc, out, err


def stream_command(client: paramiko.SSHClient, command: str, timeout: float = 0.0) -> int:
    channel = client.get_transport().open_session()
    channel.get_pty(width=180, height=48)
    channel.exec_command(command)
    started = time.monotonic()
    last_progress = started
    try:
        while not channel.exit_status_ready():
            while channel.recv_ready():
                sys.stdout.write(channel.recv(8192).decode('utf-8', errors='replace'))
                sys.stdout.flush()
            now = time.monotonic()
            if timeout > 0 and now - started > timeout:
                print(f'\nrun_timeout={timeout:.1f}s; requesting remote stop', flush=True)
                channel.close()
                return 124
            if now - last_progress > 10:
                print(f'[{time.strftime("%H:%M:%S")}] remote launch still running...', flush=True)
                last_progress = now
            time.sleep(0.1)
        while channel.recv_ready():
            sys.stdout.write(channel.recv(8192).decode('utf-8', errors='replace'))
            sys.stdout.flush()
        return channel.recv_exit_status()
    finally:
        if not channel.closed:
            channel.close()


def build_delivery_command(task_args: List[str]) -> List[str]:
    delivery_agent = import_delivery_agent()
    parsed = delivery_agent.parse_args(task_args)
    return delivery_agent.launch_command(parsed)


def normalize_task_args(task_args: List[str]) -> List[str]:
    # Keep the user's command shape, but make the common "glass" wording safe for the blue YOLO class.
    boolean_options = {
        'dry-run',
        'start-navigation',
        'start-base',
        'start-camera',
        'start-yolo',
        'use-nav',
        'use-arm',
        'closed-loop-pick',
        'require-fresh-detection-for-control',
        'grasp-check-enabled',
        'l-shape-push-enabled',
        'l-shape-push-release-before',
        'l-shape-push-close-after',
        'exit-on-done',
    }
    normalized = []
    for token in task_args:
        if token.startswith('--') and '=' in token:
            name, value = token[2:].split('=', 1)
            if name in boolean_options:
                text = value.strip().lower()
                if text in {'true', '1', 'yes', 'y', 'on'}:
                    normalized.append(f'--{name}')
                    continue
                if text in {'false', '0', 'no', 'n', 'off'}:
                    normalized.append(f'--no-{name}')
                    continue
        normalized.append(token)
    for idx, token in enumerate(normalized[:-1]):
        if token == '--place-class' and normalized[idx + 1].strip().lower() in {'glass', 'grass', 'green'}:
            normalized[idx + 1] = 'blue'
    return normalized


def stop_remote_task(client: paramiko.SSHClient, args: argparse.Namespace) -> int:
    stop_yolo = 'true' if args.stop_yolo else 'false'
    inner = source_env_script(
        args,
        (
            'set +e; '
            'echo stop_service=/competition_pick_place/stop; '
            'timeout 5s ros2 service call /competition_pick_place/stop std_srvs/srv/Trigger "{}" 2>&1 || true; '
            'echo kill_competition_processes; '
            'pkill -TERM -f "[r]os2 launch competition_pick_place competition_run.launch.py"; '
            'pkill -TERM -f "[c]ompetition_pick_place.competition_node"; '
            'pkill -TERM -f "[c]ompetition_node"; '
            'pkill -TERM -f "[d]elivery_agent"; '
            f'if [ {stop_yolo!r} = "true" ]; then '
            'echo kill_yolo_processes; '
            'pkill -TERM -f "[r]os2 launch yolov11_detect"; '
            'pkill -TERM -f "[y]olov11"; '
            'pkill -TERM -f "[y]olo_node"; '
            'fi; '
            'sleep 0.5; '
            'true'
        ),
    )
    command = docker_exec(args, inner)
    rc, out, err = run_capture(client, command, timeout=15)
    if out.strip():
        print(out.rstrip())
    if err.strip():
        print('preflight_stop_stderr:')
        print(err.rstrip())
    if rc in {137, 143}:
        print(f'preflight_stop_note=benign_signal_exit rc={rc}')
        return 0
    return rc


def print_status(client: paramiko.SSHClient, args: argparse.Namespace) -> int:
    inner = source_env_script(
        args,
        (
            'set +e; '
            'echo "== processes =="; '
            'pgrep -af "[c]ompetition_pick_place|[c]ompetition_node|[d]elivery_agent|[y]olov11|[y]olo_node" || true; '
            'echo "== ros nodes =="; '
            'timeout 5s ros2 node list 2>/dev/null | sort || true; '
            'echo "== key topics =="; '
            'timeout 5s ros2 topic list 2>/dev/null | grep -E "camera_publisher|yolo_node|cmd_vel|competition_pick_place" || true'
        ),
    )
    rc, out, err = run_capture(client, docker_exec(args, inner), timeout=20)
    if out.strip():
        print(out.rstrip())
    if err.strip():
        print(err.rstrip(), file=sys.stderr)
    return rc


def print_command(args: argparse.Namespace, task_args: List[str]) -> int:
    normalized = normalize_task_args(task_args)
    launch = build_delivery_command(normalized)
    inner = source_env_script(args, f'exec {quote_list(launch)}')
    print('task_args:')
    print('  ' + quote_list(normalized))
    print('launch_command:')
    print('  ' + quote_list(launch))
    print('remote_command:')
    print('  ' + docker_exec(args, inner))
    return 0


def run_task(client: paramiko.SSHClient, args: argparse.Namespace, task_args: List[str]) -> int:
    normalized = normalize_task_args(task_args)
    launch = build_delivery_command(normalized)
    print('remote_full_task_launch:')
    print('  ' + quote_list(launch), flush=True)
    if args.stop_existing:
        print('preflight_stop_existing=true', flush=True)
        stop_remote_task(client, args)
    inner = source_env_script(args, f'exec {quote_list(launch)}')
    command = docker_exec(args, inner)
    try:
        rc = stream_command(client, command, timeout=args.timeout)
    except KeyboardInterrupt:
        print('\nkeyboard_interrupt: stopping remote task', flush=True)
        stop_remote_task(client, args)
        return 130
    if rc == 124:
        stop_remote_task(client, args)
    print(f'remote_full_task_exit_code={rc}', flush=True)
    return rc


def main(argv: Sequence[str] | None = None) -> int:
    args, task_args = parse_args(argv)
    if args.dry_command or args.action == 'command':
        return print_command(args, task_args)
    client = connect(args)
    try:
        if args.action == 'stop':
            return stop_remote_task(client, args)
        if args.action == 'status':
            return print_status(client, args)
        return run_task(client, args, task_args)
    finally:
        client.close()


if __name__ == '__main__':
    raise SystemExit(main())
