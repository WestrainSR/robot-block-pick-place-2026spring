#!/usr/bin/env python3
import argparse
import os
import shlex
import subprocess
import sys
from typing import List, Optional


START_COMMANDS = {'开始', '启动', 'start', 'run'}
STOP_COMMANDS = {'停止', '急停', 'stop', 'halt'}
EXIT_COMMANDS = {'退出', 'exit', 'quit'}
STATUS_COMMANDS = {'状态', 'status'}


def bool_text(value: bool) -> str:
    return str(bool(value)).lower()


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description='Minimal start-only agent for the full pick-feed-return flow.'
    )
    parser.add_argument('--target-class', default='gray')
    parser.add_argument('--target-sequence', default='')
    parser.add_argument('--place-class', default='glass')
    parser.add_argument('--map-name', default='map_02')
    parser.add_argument('--dry-run', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--start-navigation', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--start-base', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--start-camera', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--start-yolo', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--use-nav', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--use-arm', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--nav-mode', choices=['odom', 'nav2'], default='odom')
    parser.add_argument('--odom-material-x', default='1.03')
    parser.add_argument('--odom-material-y', default='-1.03')
    parser.add_argument('--odom-material-yaw', default='-0.7853981633974483')
    parser.add_argument('--odom-feed-x', default='0.15')
    parser.add_argument('--odom-feed-y', default='-1.07')
    parser.add_argument('--odom-feed-yaw', default='3.141592653589793')
    parser.add_argument('--odom-return-x', default='1.03')
    parser.add_argument('--odom-return-y', default='-1.03')
    parser.add_argument('--odom-return-yaw', default='-0.7853981633974483')
    parser.add_argument('--odom-goal-tolerance-m', default='0.045')
    parser.add_argument('--odom-yaw-tolerance-rad', default='0.10')
    parser.add_argument('--odom-max-linear-speed', default='0.16')
    parser.add_argument('--odom-max-angular-speed', default='0.45')
    parser.add_argument('--yolo-model', default='tongji')
    parser.add_argument('--yolo-classes', default='gray,yellow,grass,blue')
    parser.add_argument('--yolo-conf', default='0.20')
    parser.add_argument('--min-score', default='0.20')
    parser.add_argument('--control-mode', default='mpc')
    parser.add_argument('--closed-loop-pick', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--center-tolerance-ratio', default='0.028')
    parser.add_argument('--area-tolerance-ratio', default='0.010')
    parser.add_argument('--stable-frames', default='4')
    parser.add_argument('--visual-servo-period', default='0.10')
    parser.add_argument('--visual-servo-command-seconds', default='0.06')
    parser.add_argument('--require-fresh-detection-for-control', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--max-linear-speed', default='0.08')
    parser.add_argument('--max-angular-speed', default='0.25')
    parser.add_argument('--search-angular-speed', default='0.12')
    parser.add_argument('--pick-target-robot-x', default='0.1422')
    parser.add_argument('--pick-target-robot-y', default='-0.01')
    parser.add_argument('--pick-robot-x-tolerance', default='0.005')
    parser.add_argument('--pick-robot-y-tolerance', default='0.002')
    parser.add_argument('--place-target-robot-x', default='0.175')
    parser.add_argument('--place-target-robot-y', default='0.01')
    parser.add_argument('--place-robot-x-tolerance', default='0.005')
    parser.add_argument('--place-robot-y-tolerance', default='0.002')
    parser.add_argument('--grasp-check-enabled', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--pick-pregrasp-time-scale', default='2.4')
    parser.add_argument('--pick-pregrasp-min-step-seconds', default='0.80')
    parser.add_argument('--pick-pregrasp-settle-seconds', default='0.70')
    parser.add_argument('--pick-pregrasp-post-step-seconds', default='0.60')
    parser.add_argument('--pick-preclose-center-x-ratio', default='0.90')
    parser.add_argument('--pick-preclose-target-area-ratio', default='0.095')
    parser.add_argument('--pick-preclose-center-tolerance-ratio', default='0.065')
    parser.add_argument('--pick-preclose-area-tolerance-ratio', default='0.020')
    parser.add_argument('--pick-preclose-stable-frames', default='1')
    parser.add_argument('--feed-waypoint', default='feed_pose')
    parser.add_argument('--post-feed-return-waypoint', default='material_standoff_pose')
    parser.add_argument('--post-pick-advance-m', default='0.20')
    parser.add_argument('--post-pick-advance-speed', default='0.08')
    parser.add_argument('--init-action', default='navigation_pick_init_ai')
    parser.add_argument('--pick-action', default='navigation_pick_ai')
    parser.add_argument('--place-action', default='navigation_place')
    parser.add_argument('--release-action', default='')
    parser.add_argument('--l-shape-push-enabled', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--l-shape-push-pose', default='518,196,176,597,500,335')
    parser.add_argument('--l-shape-push-pose-action', default='horizontal')
    parser.add_argument('--l-shape-push-pose-step', default='1')
    parser.add_argument('--l-shape-push-pose-duration', default='1.0')
    parser.add_argument('--l-shape-push-wrist-servo-index', default='4')
    parser.add_argument('--l-shape-push-wrist-position', default='108')
    parser.add_argument('--l-shape-push-gripper-position', default='-1')
    parser.add_argument('--l-shape-push-distance', default='0.05')
    parser.add_argument('--l-shape-push-speed', default='0.04')
    parser.add_argument('--l-shape-push-max-seconds', default='2.0')
    parser.add_argument('--l-shape-push-release-before', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--l-shape-push-close-after', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--l-shape-push-close-position', default='500')
    parser.add_argument('--l-shape-push-close-duration', default='0.35')
    parser.add_argument('--l-shape-push-lift-action', default='')
    parser.add_argument('--l-shape-push-lift-steps', default='5,6')
    parser.add_argument('--exit-on-done', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--robot-env-file', default='/home/ubuntu/ros2_ws/.typerc')
    return parser.parse_args(args)


def launch_command(args) -> List[str]:
    effective = effective_runtime_flags(args)
    launch_args = {
        'target_class': args.target_class,
        'target_sequence': args.target_sequence,
        'place_class': args.place_class,
        'dry_run': bool_text(args.dry_run),
        'exit_on_done': bool_text(args.exit_on_done),
        'stop_after_pick': 'false',
        'use_nav': bool_text(effective['use_nav']),
        'use_arm': bool_text(effective['use_arm']),
        'nav_mode': args.nav_mode,
        'map_name': args.map_name,
        'start_navigation': bool_text(effective['start_navigation']),
        'start_base': bool_text(effective['start_base']),
        'start_camera': bool_text(effective['start_camera']),
        'start_yolo': bool_text(effective['start_yolo']),
        'odom_material_x': args.odom_material_x,
        'odom_material_y': args.odom_material_y,
        'odom_material_yaw': args.odom_material_yaw,
        'odom_feed_x': args.odom_feed_x,
        'odom_feed_y': args.odom_feed_y,
        'odom_feed_yaw': args.odom_feed_yaw,
        'odom_return_x': args.odom_return_x,
        'odom_return_y': args.odom_return_y,
        'odom_return_yaw': args.odom_return_yaw,
        'odom_goal_tolerance_m': args.odom_goal_tolerance_m,
        'odom_yaw_tolerance_rad': args.odom_yaw_tolerance_rad,
        'odom_max_linear_speed': args.odom_max_linear_speed,
        'odom_max_angular_speed': args.odom_max_angular_speed,
        'yolo_model': args.yolo_model,
        'yolo_classes': args.yolo_classes,
        'yolo_conf': args.yolo_conf,
        'min_score': args.min_score,
        'control_mode': args.control_mode,
        'closed_loop_pick': bool_text(args.closed_loop_pick),
        'center_tolerance_ratio': args.center_tolerance_ratio,
        'area_tolerance_ratio': args.area_tolerance_ratio,
        'stable_frames': args.stable_frames,
        'visual_servo_period': args.visual_servo_period,
        'visual_servo_command_seconds': args.visual_servo_command_seconds,
        'require_fresh_detection_for_control': bool_text(args.require_fresh_detection_for_control),
        'max_linear_speed': args.max_linear_speed,
        'max_angular_speed': args.max_angular_speed,
        'search_angular_speed': args.search_angular_speed,
        'pick_target_robot_x_m': args.pick_target_robot_x,
        'pick_target_robot_y_m': args.pick_target_robot_y,
        'pick_robot_x_tolerance_m': args.pick_robot_x_tolerance,
        'pick_robot_y_tolerance_m': args.pick_robot_y_tolerance,
        'place_target_robot_x_m': args.place_target_robot_x,
        'place_target_robot_y_m': args.place_target_robot_y,
        'place_robot_x_tolerance_m': args.place_robot_x_tolerance,
        'place_robot_y_tolerance_m': args.place_robot_y_tolerance,
        'grasp_check_enabled': bool_text(args.grasp_check_enabled),
        'pick_pregrasp_time_scale': args.pick_pregrasp_time_scale,
        'pick_pregrasp_min_step_seconds': args.pick_pregrasp_min_step_seconds,
        'pick_pregrasp_settle_seconds': args.pick_pregrasp_settle_seconds,
        'pick_pregrasp_post_step_seconds': args.pick_pregrasp_post_step_seconds,
        'pick_preclose_center_x_ratio': args.pick_preclose_center_x_ratio,
        'pick_preclose_target_area_ratio': args.pick_preclose_target_area_ratio,
        'pick_preclose_center_tolerance_ratio': args.pick_preclose_center_tolerance_ratio,
        'pick_preclose_area_tolerance_ratio': args.pick_preclose_area_tolerance_ratio,
        'pick_preclose_stable_frames': args.pick_preclose_stable_frames,
        'feed_waypoint': args.feed_waypoint,
        'post_feed_return_waypoint': args.post_feed_return_waypoint,
        'post_pick_advance_m': args.post_pick_advance_m,
        'post_pick_advance_speed': args.post_pick_advance_speed,
        'init_action': args.init_action,
        'pick_action': args.pick_action,
        'place_action': args.place_action,
        'release_action': args.release_action,
        'l_shape_push_enabled': bool_text(args.l_shape_push_enabled),
        'l_shape_push_pose': args.l_shape_push_pose,
        'l_shape_push_pose_action': args.l_shape_push_pose_action,
        'l_shape_push_pose_step': args.l_shape_push_pose_step,
        'l_shape_push_pose_duration': args.l_shape_push_pose_duration,
        'l_shape_push_wrist_servo_index': args.l_shape_push_wrist_servo_index,
        'l_shape_push_wrist_position': args.l_shape_push_wrist_position,
        'l_shape_push_gripper_position': args.l_shape_push_gripper_position,
        'l_shape_push_distance_m': args.l_shape_push_distance,
        'l_shape_push_speed_mps': args.l_shape_push_speed,
        'l_shape_push_max_seconds': args.l_shape_push_max_seconds,
        'l_shape_push_release_before': bool_text(args.l_shape_push_release_before),
        'l_shape_push_close_after': bool_text(args.l_shape_push_close_after),
        'l_shape_push_close_position': args.l_shape_push_close_position,
        'l_shape_push_close_duration': args.l_shape_push_close_duration,
        'l_shape_push_lift_action': args.l_shape_push_lift_action or args.pick_action,
        'l_shape_push_lift_steps': args.l_shape_push_lift_steps,
    }
    return (
        ['ros2', 'launch', 'competition_pick_place', 'competition_run.launch.py']
        + [f'{key}:={value}' for key, value in launch_args.items() if str(value) != '']
    )


def effective_runtime_flags(args) -> dict:
    return {
        'start_navigation': args.start_navigation and not args.dry_run,
        'start_base': args.start_base and not args.dry_run,
        'start_camera': args.start_camera and not args.dry_run,
        'start_yolo': args.start_yolo and not args.dry_run,
        'use_nav': args.use_nav and not args.dry_run,
        'use_arm': args.use_arm and not args.dry_run,
    }


def request_stop() -> int:
    command = [
        'ros2',
        'service',
        'call',
        '/competition_pick_place/stop',
        'std_srvs/srv/Trigger',
        '{}',
    ]
    return subprocess.run(command, timeout=8, check=False).returncode


class DeliveryAgent:
    def __init__(self, args) -> None:
        self.args = args
        self.process: Optional[subprocess.Popen] = None

    def print_status(self) -> None:
        running = self.process is not None and self.process.poll() is None
        effective = effective_runtime_flags(self.args)
        print('delivery_agent status')
        print(f'  running={running}')
        print(f'  target_class={self.args.target_class}')
        print(f'  target_sequence={self.args.target_sequence or "(single target)"}')
        print(f'  place_class={self.args.place_class}')
        print(f'  map_name={self.args.map_name}')
        print(f'  nav_mode={self.args.nav_mode}')
        print(f'  dry_run={self.args.dry_run}')
        print(f'  start_navigation={effective["start_navigation"]} requested={self.args.start_navigation}')
        print(f'  start_base={effective["start_base"]} requested={self.args.start_base}')
        print(f'  start_camera={effective["start_camera"]} requested={self.args.start_camera}')
        print(f'  start_yolo={effective["start_yolo"]} requested={self.args.start_yolo}')
        print(f'  use_nav={effective["use_nav"]} requested={self.args.use_nav}')
        print(f'  use_arm={effective["use_arm"]} requested={self.args.use_arm}')
        print(
            '  odom_material='
            f'({self.args.odom_material_x},{self.args.odom_material_y},{self.args.odom_material_yaw})'
        )
        print(f'  odom_feed=({self.args.odom_feed_x},{self.args.odom_feed_y},{self.args.odom_feed_yaw})')
        print(f'  odom_return=({self.args.odom_return_x},{self.args.odom_return_y},{self.args.odom_return_yaw})')
        print(f'  yolo_model={self.args.yolo_model}')
        print(f'  yolo_classes={self.args.yolo_classes}')
        print(f'  yolo_conf={self.args.yolo_conf}')
        print(f'  control_mode={self.args.control_mode}')
        print(f'  closed_loop_pick={self.args.closed_loop_pick}')
        print(
            '  pick_target='
            f'x={self.args.pick_target_robot_x}, y={self.args.pick_target_robot_y}, '
            f'tol=({self.args.pick_robot_x_tolerance},{self.args.pick_robot_y_tolerance}), '
            f'grasp_check={self.args.grasp_check_enabled}'
        )
        print(
            '  place_target='
            f'x={self.args.place_target_robot_x}, y={self.args.place_target_robot_y}, '
            f'tol=({self.args.place_robot_x_tolerance},{self.args.place_robot_y_tolerance})'
        )
        print(f'  init_action={self.args.init_action}')
        print(f'  pick_action={self.args.pick_action}')
        print(f'  place_action={self.args.place_action}')
        print(f'  feed_waypoint={self.args.feed_waypoint}')
        print(f'  post_feed_return_waypoint={self.args.post_feed_return_waypoint}')
        l_shape_pose = self.args.l_shape_push_pose or (
            self.args.l_shape_push_pose_action + ':step' + self.args.l_shape_push_pose_step
        )
        print(
            '  l_shape_push='
            f'enabled={self.args.l_shape_push_enabled}, '
            f'pose={l_shape_pose}, '
            f'wrist{self.args.l_shape_push_wrist_servo_index}={self.args.l_shape_push_wrist_position}, '
            f'gripper={self.args.l_shape_push_gripper_position}, '
            f'distance={self.args.l_shape_push_distance}, speed={self.args.l_shape_push_speed}, '
            f'release_before={self.args.l_shape_push_release_before}, '
            f'close_after={self.args.l_shape_push_close_after}'
        )

    def start(self) -> None:
        if self.process is not None and self.process.poll() is None:
            print('完整流程已经在运行。')
            return
        command = launch_command(self.args)
        print('启动完整流程：')
        print('need_compile=True')
        if self.args.robot_env_file:
            print(f'robot_env_file={self.args.robot_env_file}')
        print(' '.join(command))
        env = os.environ.copy()
        env['need_compile'] = 'True'
        if self.args.robot_env_file:
            env_file = shlex.quote(self.args.robot_env_file)
            effective = effective_runtime_flags(self.args)
            start_navigation = 'true' if effective['start_navigation'] else 'false'
            script = (
                f'set -e; '
                f'if [ ! -f {env_file} ]; then echo robot_env_missing={env_file}; exit 30; fi; '
                f'source {env_file}; '
                f'for optional_setup in '
                f'/home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash '
                f'/home/ubuntu/third_party_ros2/third_party_ws/install/local_setup.bash '
                f'/home/ubuntu/deptrum_ws/install/setup.bash '
                f'/home/ubuntu/deptrum_ws/install/local_setup.bash '
                f'/home/ubuntu/aurora930_ws/install/setup.bash '
                f'/home/ubuntu/aurora930_ws/install/local_setup.bash '
                f'/home/ubuntu/ros2_ws/install/setup.bash; do '
                f'if [ -f "$optional_setup" ]; then source "$optional_setup"; echo sourced_setup="$optional_setup"; fi; '
                f'done; '
                f'if [ {start_navigation!r} = "true" ] '
                f'&& [ "${{DEPTH_CAMERA_TYPE:-}}" = "aurora" ] '
                f'&& ! ros2 pkg prefix deptrum-ros-driver-aurora930 >/dev/null 2>&1; then '
                f'echo aurora_driver_package_missing=deptrum-ros-driver-aurora930; '
                f'echo checked_optional_setups=/home/ubuntu/third_party_ros2/third_party_ws/install,/home/ubuntu/deptrum_ws/install,/home/ubuntu/aurora930_ws/install; '
                f'fi; '
                f'exec {shlex.join(command)}'
            )
            self.process = subprocess.Popen(['bash', '-lc', script], env=env)
        else:
            self.process = subprocess.Popen(command, env=env)

    def stop(self) -> None:
        rc = request_stop()
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        print(f'已请求停止，stop service rc={rc}')

    def reap(self) -> None:
        if self.process is not None and self.process.poll() is not None:
            print(f'完整流程已退出，rc={self.process.returncode}')
            self.process = None

    def run(self) -> int:
        print('delivery_agent ready. 输入“开始”执行完整流程；输入“停止”急停；输入“状态”查看配置；输入“退出”结束。')
        self.print_status()
        while True:
            self.reap()
            try:
                text = input('agent> ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if not text:
                continue
            normalized = text.lower()
            if text in START_COMMANDS or normalized in START_COMMANDS:
                self.start()
            elif text in STOP_COMMANDS or normalized in STOP_COMMANDS:
                self.stop()
            elif text in STATUS_COMMANDS or normalized in STATUS_COMMANDS:
                self.print_status()
            elif text in EXIT_COMMANDS or normalized in EXIT_COMMANDS:
                return 0
            else:
                print('不解析该输入。请输入“开始”“停止”“状态”或“退出”。')


def main(args=None) -> None:
    parsed = parse_args(args)
    raise SystemExit(DeliveryAgent(parsed).run())


if __name__ == '__main__':
    main()
