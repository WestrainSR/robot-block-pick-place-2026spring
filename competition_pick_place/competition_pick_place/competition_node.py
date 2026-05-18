#!/usr/bin/env python3
import math
import os
import sqlite3
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from interfaces.msg import ObjectsInfo
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except Exception:  # pragma: no cover - only used on the robot image.
    BasicNavigator = None
    TaskResult = None

try:
    from servo_controller.action_group_controller import ActionGroupController
    from servo_controller_msgs.msg import ServosPosition
except Exception:  # pragma: no cover - dry-run can still start without arm libs.
    ActionGroupController = None
    ServosPosition = None


class Phase(str, Enum):
    INIT = 'INIT'
    NAV_TO_MATERIAL = 'NAV_TO_MATERIAL'
    SEARCH_TARGET = 'SEARCH_TARGET'
    ALIGN_TARGET = 'ALIGN_TARGET'
    PICK = 'PICK'
    NAV_TO_PLACE = 'NAV_TO_PLACE'
    ALIGN_PLACE = 'ALIGN_PLACE'
    PLACE = 'PLACE'
    NAV_HOME = 'NAV_HOME'
    DONE = 'DONE'
    FAILSAFE = 'FAILSAFE'


@dataclass
class Detection:
    class_name: str
    score: float
    box: List[int]
    width: int
    height: int
    stamp: float

    @property
    def cx_ratio(self) -> float:
        return ((self.box[0] + self.box[2]) * 0.5) / max(1, self.width)

    @property
    def cy_ratio(self) -> float:
        return ((self.box[1] + self.box[3]) * 0.5) / max(1, self.height)

    @property
    def area_ratio(self) -> float:
        box_w = max(0, self.box[2] - self.box[0])
        box_h = max(0, self.box[3] - self.box[1])
        return (box_w * box_h) / max(1, self.width * self.height)


class CompetitionPickPlace(Node):
    VALID_TARGETS = {'red', 'green', 'blue'}

    def __init__(self) -> None:
        super().__init__(
            'competition_pick_place',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=False,
        )
        self.started_at = time.monotonic()
        self.phase = Phase.INIT
        self.shutdown_requested = False
        self.detection_lock = threading.Lock()
        self.latest_detections: Dict[str, Detection] = {}
        self.last_msg_time = 0.0

        share_dir = get_package_share_directory('competition_pick_place')
        self.declare_parameter('target_class', 'red')
        self.declare_parameter('target_sequence', '')
        self.declare_parameter('target_aliases', '')
        self.declare_parameter('place_class', '')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('exit_on_done', False)
        self.declare_parameter('stop_after_pick', False)
        self.declare_parameter('use_nav', True)
        self.declare_parameter('use_arm', True)
        self.declare_parameter('waypoints_yaml', os.path.join(share_dir, 'config', 'competition_waypoints.yaml'))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('detection_topic', '/yolo_node/object_detect')
        self.declare_parameter('cmd_vel_topic', '/controller/cmd_vel')
        self.declare_parameter('min_score', 0.70)
        self.declare_parameter('search_timeout', 18.0)
        self.declare_parameter('align_timeout', 24.0)
        self.declare_parameter('place_align_timeout', 18.0)
        self.declare_parameter('nav_timeout', 180.0)
        self.declare_parameter('detection_stale_seconds', 0.8)
        self.declare_parameter('desired_center_x_ratio', 0.50)
        self.declare_parameter('center_tolerance_ratio', 0.055)
        self.declare_parameter('pick_target_area_ratio', 0.095)
        self.declare_parameter('place_target_area_ratio', 0.080)
        self.declare_parameter('area_tolerance_ratio', 0.018)
        self.declare_parameter('stable_frames', 5)
        self.declare_parameter('control_mode', 'p')
        self.declare_parameter('closed_loop_pick', False)
        self.declare_parameter('pick_visual_servo_timeout', 10.0)
        self.declare_parameter('pick_pregrasp_visual_servo', True)
        self.declare_parameter('pick_preclose_required', True)
        self.declare_parameter('pick_preclose_center_x_ratio', 0.50)
        self.declare_parameter('pick_preclose_target_area_ratio', -1.0)
        self.declare_parameter('pick_preclose_center_tolerance_ratio', -1.0)
        self.declare_parameter('pick_preclose_area_tolerance_ratio', -1.0)
        self.declare_parameter('pick_preclose_stable_frames', 2)
        self.declare_parameter('pick_pregrasp_steps', '1,2')
        self.declare_parameter('pick_close_steps', '3,4')
        self.declare_parameter('pick_lift_steps', '5,6')
        self.declare_parameter('linear_k', 0.42)
        self.declare_parameter('angular_k', 1.35)
        self.declare_parameter('max_linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.45)
        self.declare_parameter('search_angular_speed', 0.22)
        self.declare_parameter('angular_sign', -1.0)
        self.declare_parameter('linear_sign', 1.0)
        self.declare_parameter('mpc_horizon', 6)
        self.declare_parameter('mpc_dt', 0.12)
        self.declare_parameter('mpc_center_response', 1.05)
        self.declare_parameter('mpc_area_response', 0.24)
        self.declare_parameter('mpc_center_weight', 8.0)
        self.declare_parameter('mpc_area_weight', 26.0)
        self.declare_parameter('mpc_velocity_weight', 0.08)
        self.declare_parameter('mpc_delta_weight', 0.16)
        self.declare_parameter('mpc_terminal_weight', 2.2)
        self.declare_parameter('mpc_center_gate_ratio', 0.12)
        self.declare_parameter('pick_action', 'navigation_pick')
        self.declare_parameter('place_action', 'navigation_place')
        self.declare_parameter('init_action', 'navigation_pick_init')
        self.declare_parameter('action_group_path', '/home/ubuntu/software/arm_pc/ActionGroups')

        self.target_class = str(self.get_parameter('target_class').value).strip()
        self.target_sequence = self.parse_target_sequence(
            self.get_parameter('target_sequence').value,
            self.target_class,
        )
        self.place_class = str(self.get_parameter('place_class').value).strip()
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.stop_after_pick = bool(self.get_parameter('stop_after_pick').value)
        self.use_nav = bool(self.get_parameter('use_nav').value)
        self.use_arm = bool(self.get_parameter('use_arm').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.stale_seconds = float(self.get_parameter('detection_stale_seconds').value)
        self.stable_frames = int(self.get_parameter('stable_frames').value)
        self.control_mode = str(self.get_parameter('control_mode').value).strip().lower()
        self.closed_loop_pick = bool(self.get_parameter('closed_loop_pick').value)
        self.waypoints = self.load_waypoints(str(self.get_parameter('waypoints_yaml').value))
        self.target_aliases = set(self.parse_aliases(self.get_parameter('target_aliases').value))
        self.last_twist = Twist()

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 1)
        self.create_subscription(
            ObjectsInfo,
            str(self.get_parameter('detection_topic').value),
            self.detection_callback,
            10,
        )
        self.create_service(Trigger, '~/stop', self.stop_callback)
        self.create_service(Trigger, '~/init_finish', self.init_finish_callback)

        self.navigator = None
        self.arm_controller = None
        self.arm_ready_client = None
        if self.use_nav and not self.dry_run:
            if BasicNavigator is None:
                raise RuntimeError('nav2_simple_commander is not available')
            self.navigator = BasicNavigator()
        if self.use_arm and not self.dry_run:
            if ActionGroupController is None or ServosPosition is None:
                raise RuntimeError('servo_controller is not available')
            servo_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
            self.arm_controller = ActionGroupController(
                servo_pub,
                str(self.get_parameter('action_group_path').value),
            )
            self.arm_ready_client = self.create_client(Trigger, '/controller_manager/init_finish')

        self.task_thread = threading.Thread(target=self.run_task, daemon=True)
        self.task_thread.start()

    def parse_aliases(self, value) -> Iterable[str]:
        if value is None:
            return []
        if isinstance(value, str):
            return [part.strip() for part in value.split(',') if part.strip()]
        return [str(part).strip() for part in value if str(part).strip()]

    def parse_target_sequence(self, value, fallback: str) -> List[str]:
        if value is None:
            return [fallback]
        if isinstance(value, str):
            targets = [part.strip() for part in value.split(',') if part.strip()]
        else:
            targets = [str(part).strip() for part in value if str(part).strip()]
        return targets or [fallback]

    def target_names_for(self, target: str) -> set:
        names = {target}
        if len(self.target_sequence) == 1:
            names.update(self.target_aliases)
        return names

    def load_waypoints(self, path: str) -> dict:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return data

    def detection_callback(self, msg: ObjectsInfo) -> None:
        now = time.monotonic()
        fresh: Dict[str, Detection] = {}
        for obj in msg.objects:
            if obj.score < self.min_score or len(obj.box) < 4:
                continue
            width = obj.width if obj.width > 0 else 640
            height = obj.height if obj.height > 0 else 480
            det = Detection(
                class_name=obj.class_name.strip(),
                score=float(obj.score),
                box=list(obj.box[:4]),
                width=int(width),
                height=int(height),
                stamp=now,
            )
            old = fresh.get(det.class_name)
            if old is None or self.detection_rank(det) > self.detection_rank(old):
                fresh[det.class_name] = det
        with self.detection_lock:
            self.latest_detections = fresh
            self.last_msg_time = now

    def detection_rank(self, det: Detection) -> float:
        center_error = abs(det.cx_ratio - float(self.get_parameter('desired_center_x_ratio').value))
        return det.score + 0.30 * det.area_ratio - 0.12 * center_error

    def stop_callback(self, request, response):
        self.shutdown_requested = True
        self.stop_robot()
        response.success = True
        response.message = 'stop requested'
        return response

    def init_finish_callback(self, request, response):
        response.success = True
        response.message = self.phase.value
        return response

    def run_task(self) -> None:
        try:
            self.set_phase(Phase.INIT, f'targets={self.target_sequence}, dry_run={self.dry_run}')
            self.validate_targets()
            self.wait_for_systems()
            self.run_action_group(str(self.get_parameter('init_action').value), 'arm init')

            total = len(self.target_sequence)
            for index, target in enumerate(self.target_sequence, 1):
                self.target_class = target
                target_names = self.target_names_for(target)
                prefix = f'[{index}/{total}] {target}'

                self.set_phase(Phase.NAV_TO_MATERIAL, f'{prefix}: going to material standoff')
                self.navigate_to('material_standoff_pose')

                self.set_phase(Phase.SEARCH_TARGET, f'{prefix}: searching {sorted(target_names)}')
                self.visual_servo_to_classes(
                    names=target_names,
                    timeout=float(self.get_parameter('align_timeout').value),
                    target_area=float(self.get_parameter('pick_target_area_ratio').value),
                    label=f'{target} pick target',
                )

                self.set_phase(Phase.PICK, f'{prefix}: running pick controller')
                self.run_pick_controller(target_names, f'{target} pick')
                if self.stop_after_pick:
                    self.stop_robot()
                    self.set_phase(Phase.DONE, f'{prefix}: stop_after_pick=true')
                    self.shutdown_if_requested()
                    return

                self.set_phase(Phase.NAV_TO_PLACE, f'{prefix}: going to place standoff')
                self.navigate_to('place_standoff_pose')

                if self.place_class:
                    self.set_phase(Phase.ALIGN_PLACE, f'{prefix}: aligning place marker {self.place_class}')
                    self.visual_servo_to_classes(
                        names={self.place_class},
                        timeout=float(self.get_parameter('place_align_timeout').value),
                        target_area=float(self.get_parameter('place_target_area_ratio').value),
                        label='place marker',
                    )
                else:
                    self.get_logger().warn('place_class is empty; placement uses Nav2 standoff plus calibrated action group')

                self.set_phase(Phase.PLACE, f'{prefix}: running place action group')
                self.run_action_group(str(self.get_parameter('place_action').value), f'{target} place')

            self.set_phase(Phase.NAV_HOME, 'returning home')
            self.navigate_to('return_pose')

            self.stop_robot()
            self.set_phase(Phase.DONE, 'task completed')
            self.shutdown_if_requested()
        except Exception as exc:
            self.stop_robot()
            self.set_phase(Phase.FAILSAFE, str(exc))
            self.shutdown_if_requested()

    def validate_targets(self) -> None:
        invalid = [target for target in self.target_sequence if target not in self.VALID_TARGETS]
        if invalid:
            raise RuntimeError(f'targets must be in {sorted(self.VALID_TARGETS)}, got {invalid!r}')
        if not self.dry_run and self.use_nav:
            for name in ('material_standoff_pose', 'place_standoff_pose', 'return_pose'):
                pose = self.waypoints.get(name)
                if not pose:
                    raise RuntimeError(f'missing waypoint: {name}')
                if abs(float(pose.get('x', 0.0))) < 1e-6 and abs(float(pose.get('y', 0.0))) < 1e-6 and name != 'return_pose':
                    raise RuntimeError(f'{name} is still [0, 0]; calibrate config/competition_waypoints.yaml first')

    def wait_for_systems(self) -> None:
        if self.dry_run:
            self.get_logger().info('dry_run=true: hardware movement and blocking waits are skipped')
            return
        if self.navigator is not None:
            self.get_logger().info('waiting for Nav2 active')
            self.navigator.waitUntilNav2Active()
        if self.arm_ready_client is not None:
            self.get_logger().info('waiting for controller_manager/init_finish')
            if not self.arm_ready_client.wait_for_service(timeout_sec=12.0):
                raise RuntimeError('/controller_manager/init_finish service is not available')

    def navigate_to(self, waypoint_name: str) -> None:
        if not self.use_nav:
            self.get_logger().warn(f'use_nav=false: skip waypoint {waypoint_name}')
            return
        pose_data = self.waypoints.get(waypoint_name)
        if pose_data is None:
            raise RuntimeError(f'missing waypoint {waypoint_name}')
        if self.dry_run:
            self.get_logger().info(f'dry-run navigation to {waypoint_name}: {pose_data}')
            return
        if self.navigator is None:
            raise RuntimeError('navigator is not initialized')

        pose = self.make_pose(float(pose_data['x']), float(pose_data['y']), float(pose_data.get('yaw', 0.0)))
        self.navigator.goToPose(pose)
        deadline = time.monotonic() + float(self.get_parameter('nav_timeout').value)
        while rclpy.ok() and not self.navigator.isTaskComplete():
            if self.shutdown_requested:
                self.navigator.cancelTask()
                raise RuntimeError('stop requested during navigation')
            if time.monotonic() > deadline:
                self.navigator.cancelTask()
                raise RuntimeError(f'navigation timeout at {waypoint_name}')
            time.sleep(0.2)

        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            raise RuntimeError(f'navigation failed at {waypoint_name}: {result}')

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def visual_servo_to_classes(
        self,
        names: Iterable[str],
        timeout: float,
        target_area: float,
        label: str,
        desired_center: Optional[float] = None,
        center_tolerance: Optional[float] = None,
        area_tolerance: Optional[float] = None,
        stable_frames: Optional[int] = None,
    ) -> None:
        names = set(names)
        desired_center = (
            float(self.get_parameter('desired_center_x_ratio').value)
            if desired_center is None
            else float(desired_center)
        )
        center_tolerance = (
            float(self.get_parameter('center_tolerance_ratio').value)
            if center_tolerance is None
            else float(center_tolerance)
        )
        area_tolerance = (
            float(self.get_parameter('area_tolerance_ratio').value)
            if area_tolerance is None
            else float(area_tolerance)
        )
        stable_required = self.stable_frames if stable_frames is None else max(1, int(stable_frames))
        if self.dry_run:
            self.get_logger().info(
                f'dry-run visual servo {label}: names={sorted(names)}, target_area={target_area}, '
                f'desired_center={desired_center}, control_mode={self.control_mode}'
            )
            return

        found_deadline = time.monotonic() + float(self.get_parameter('search_timeout').value)
        align_deadline = time.monotonic() + timeout
        stable = 0
        last_log = 0.0
        found_once = False

        while rclpy.ok() and not self.shutdown_requested:
            det = self.best_detection(names)
            now = time.monotonic()
            if det is None:
                stable = 0
                if not found_once and now > found_deadline:
                    raise RuntimeError(f'{label} not found before search timeout')
                if found_once and now > align_deadline:
                    raise RuntimeError(f'{label} alignment timeout after temporary target loss')
                self.publish_search_twist()
                if now - last_log > 1.5:
                    self.get_logger().info(f'searching {label}; detections={self.visible_classes()}')
                    last_log = now
                time.sleep(0.08)
                continue

            found_once = True
            if now > align_deadline:
                raise RuntimeError(f'{label} alignment timeout')

            center_error = det.cx_ratio - desired_center
            area_error = target_area - det.area_ratio
            center_tol = center_tolerance
            area_tol = area_tolerance

            if abs(center_error) <= center_tol and abs(area_error) <= area_tol:
                stable += 1
                self.stop_robot()
                if stable >= stable_required:
                    self.get_logger().info(
                        f'{label} aligned: class={det.class_name}, score={det.score:.2f}, '
                        f'cx={det.cx_ratio:.3f}, area={det.area_ratio:.3f}, '
                        f'desired_cx={desired_center:.3f}, target_area={target_area:.3f}'
                    )
                    return
                time.sleep(0.08)
                continue

            stable = 0
            twist = self.compute_visual_servo_twist(center_error, area_error, center_tol)
            self.cmd_pub.publish(twist)
            self.last_twist = twist
            if now - last_log > 0.8:
                self.get_logger().info(
                    f'visual servo {label}: mode={self.control_mode} class={det.class_name} score={det.score:.2f} '
                    f'cx={det.cx_ratio:.3f} area={det.area_ratio:.3f} '
                    f'cmd=({twist.linear.x:.3f},{twist.angular.z:.3f})'
                )
                last_log = now
            time.sleep(0.08)

        raise RuntimeError('stop requested during alignment')

    def compute_visual_servo_twist(self, center_error: float, area_error: float, center_tol: float) -> Twist:
        if self.control_mode == 'mpc':
            return self.compute_mpc_twist(center_error, area_error, center_tol)
        return self.compute_p_twist(center_error, area_error, center_tol)

    def compute_p_twist(self, center_error: float, area_error: float, center_tol: float) -> Twist:
        twist = Twist()
        angular = (
            float(self.get_parameter('angular_sign').value)
            * float(self.get_parameter('angular_k').value)
            * center_error
        )
        max_angular = float(self.get_parameter('max_angular_speed').value)
        twist.angular.z = self.clamp(angular, -max_angular, max_angular)

        if abs(center_error) < center_tol * 2.5:
            linear = (
                float(self.get_parameter('linear_sign').value)
                * float(self.get_parameter('linear_k').value)
                * area_error
            )
            max_linear = float(self.get_parameter('max_linear_speed').value)
            twist.linear.x = self.clamp(linear, -max_linear * 0.45, max_linear)
        return twist

    def compute_mpc_twist(self, center_error: float, area_error: float, center_tol: float) -> Twist:
        max_linear = float(self.get_parameter('max_linear_speed').value)
        max_angular = float(self.get_parameter('max_angular_speed').value)
        center_gate = max(center_tol * 2.8, float(self.get_parameter('mpc_center_gate_ratio').value))
        linear_candidates = self.mpc_candidates(max_linear)
        angular_candidates = self.mpc_candidates(max_angular)

        best_cost = float('inf')
        best_linear = 0.0
        best_angular = 0.0
        for linear in linear_candidates:
            if abs(center_error) > center_gate and linear > 1e-6:
                continue
            for angular in angular_candidates:
                cost = self.predict_mpc_cost(center_error, area_error, linear, angular)
                if cost < best_cost:
                    best_cost = cost
                    best_linear = linear
                    best_angular = angular

        twist = Twist()
        twist.linear.x = best_linear
        twist.angular.z = best_angular
        return twist

    def mpc_candidates(self, max_abs: float) -> List[float]:
        if max_abs <= 1e-6:
            return [0.0]
        return [-max_abs, -0.5 * max_abs, 0.0, 0.5 * max_abs, max_abs]

    def predict_mpc_cost(self, center_error: float, area_error: float, linear: float, angular: float) -> float:
        horizon = max(1, int(self.get_parameter('mpc_horizon').value))
        dt = max(0.02, float(self.get_parameter('mpc_dt').value))
        center_response = float(self.get_parameter('mpc_center_response').value)
        area_response = float(self.get_parameter('mpc_area_response').value)
        center_weight = float(self.get_parameter('mpc_center_weight').value)
        area_weight = float(self.get_parameter('mpc_area_weight').value)
        velocity_weight = float(self.get_parameter('mpc_velocity_weight').value)
        delta_weight = float(self.get_parameter('mpc_delta_weight').value)
        terminal_weight = float(self.get_parameter('mpc_terminal_weight').value)
        angular_sign = float(self.get_parameter('angular_sign').value)
        linear_sign = float(self.get_parameter('linear_sign').value)

        predicted_center = center_error
        predicted_area = area_error
        cost = 0.0
        last_linear = float(getattr(self.last_twist, 'linear', Twist().linear).x)
        last_angular = float(getattr(self.last_twist, 'angular', Twist().angular).z)
        for _ in range(horizon):
            predicted_center -= center_response * angular_sign * angular * dt
            predicted_area -= area_response * linear_sign * linear * dt
            cost += center_weight * predicted_center * predicted_center
            cost += area_weight * predicted_area * predicted_area
            cost += velocity_weight * (linear * linear + 0.35 * angular * angular)
        cost += delta_weight * ((linear - last_linear) ** 2 + 0.4 * (angular - last_angular) ** 2)
        cost += terminal_weight * (
            center_weight * predicted_center * predicted_center
            + area_weight * predicted_area * predicted_area
        )
        return cost

    def best_detection(self, names: Iterable[str]) -> Optional[Detection]:
        now = time.monotonic()
        with self.detection_lock:
            candidates = [
                det for name, det in self.latest_detections.items()
                if name in names and now - det.stamp <= self.stale_seconds
            ]
        if not candidates:
            return None
        return max(candidates, key=self.detection_rank)

    def visible_classes(self) -> List[str]:
        now = time.monotonic()
        with self.detection_lock:
            return sorted(
                name for name, det in self.latest_detections.items()
                if now - det.stamp <= self.stale_seconds
            )

    def publish_search_twist(self) -> None:
        twist = Twist()
        twist.angular.z = float(self.get_parameter('search_angular_speed').value)
        self.cmd_pub.publish(twist)
        self.last_twist = twist

    def run_pick_controller(self, target_names: Iterable[str], label: str) -> None:
        if not self.closed_loop_pick:
            self.run_action_group(str(self.get_parameter('pick_action').value), label)
            return
        if self.dry_run:
            self.get_logger().info(f'dry-run closed-loop pick {label}')
            return

        pick_action = str(self.get_parameter('pick_action').value)
        pregrasp_steps = self.parse_step_indices(self.get_parameter('pick_pregrasp_steps').value)
        close_steps = self.parse_step_indices(self.get_parameter('pick_close_steps').value)
        lift_steps = self.parse_step_indices(self.get_parameter('pick_lift_steps').value)

        self.get_logger().info(
            f'closed-loop pick {label}: pregrasp={pregrasp_steps}, close={close_steps}, '
            f'lift={lift_steps}, mode={self.control_mode}'
        )
        preclose_area = float(self.get_parameter('pick_preclose_target_area_ratio').value)
        if preclose_area <= 0.0:
            preclose_area = float(self.get_parameter('pick_target_area_ratio').value)
        preclose_center_tol = float(self.get_parameter('pick_preclose_center_tolerance_ratio').value)
        if preclose_center_tol <= 0.0:
            preclose_center_tol = float(self.get_parameter('center_tolerance_ratio').value)
        preclose_area_tol = float(self.get_parameter('pick_preclose_area_tolerance_ratio').value)
        if preclose_area_tol <= 0.0:
            preclose_area_tol = float(self.get_parameter('area_tolerance_ratio').value)

        preclose_center = float(self.get_parameter('pick_preclose_center_x_ratio').value)
        if bool(self.get_parameter('pick_pregrasp_visual_servo').value):
            self.run_action_group_steps_with_visual_servo(
                pick_action,
                pregrasp_steps,
                target_names,
                f'{label} pregrasp',
                desired_center=preclose_center,
                target_area=preclose_area,
                center_tolerance=preclose_center_tol,
            )
        else:
            self.run_action_group_steps(pick_action, pregrasp_steps, f'{label} pregrasp')

        if bool(self.get_parameter('pick_preclose_required').value):
            self.visual_servo_to_classes(
                names=target_names,
                timeout=float(self.get_parameter('pick_visual_servo_timeout').value),
                target_area=preclose_area,
                label=f'{label} pre-close',
                desired_center=preclose_center,
                center_tolerance=preclose_center_tol,
                area_tolerance=preclose_area_tol,
                stable_frames=int(self.get_parameter('pick_preclose_stable_frames').value),
            )
        self.run_action_group_steps(pick_action, close_steps, f'{label} close')
        self.run_action_group_steps(pick_action, lift_steps, f'{label} lift')

    def parse_step_indices(self, value) -> List[int]:
        if value is None:
            return []
        if isinstance(value, str):
            parts = [part.strip() for part in value.split(',') if part.strip()]
        else:
            parts = [str(part).strip() for part in value if str(part).strip()]
        result: List[int] = []
        for part in parts:
            try:
                index = int(part)
            except ValueError as exc:
                raise RuntimeError(f'invalid action step index {part!r}') from exc
            if index <= 0:
                raise RuntimeError(f'action step index must be positive, got {index}')
            result.append(index)
        return result

    def run_action_group_steps(self, action_name: str, step_indices: Sequence[int], label: str) -> None:
        if not step_indices:
            self.get_logger().warn(f'no action steps configured for {label}')
            return
        if not self.use_arm:
            self.get_logger().warn(f'use_arm=false: skip {label} action steps {list(step_indices)}')
            return
        if self.dry_run:
            self.get_logger().info(f'dry-run action steps {label}: {action_name} {list(step_indices)}')
            return
        if ServosPosition is None:
            raise RuntimeError('servo_controller_msgs is not available')

        self.stop_robot()
        rows = self.load_action_group_rows(action_name)
        selected = [row for row in rows if int(row[0]) in set(step_indices)]
        if len(selected) != len(set(step_indices)):
            available = [int(row[0]) for row in rows]
            raise RuntimeError(f'{label}: requested steps {list(step_indices)} not in action group {available}')
        self.get_logger().info(f'run action group steps {action_name} {list(step_indices)} for {label}')
        for row in selected:
            if self.shutdown_requested:
                raise RuntimeError('stop requested during action steps')
            self.publish_servo_row(row)

    def run_action_group_steps_with_visual_servo(
        self,
        action_name: str,
        step_indices: Sequence[int],
        target_names: Iterable[str],
        label: str,
        desired_center: float,
        target_area: float,
        center_tolerance: float,
    ) -> None:
        if not step_indices:
            self.get_logger().warn(f'no action steps configured for {label}')
            return
        if not self.use_arm:
            self.get_logger().warn(f'use_arm=false: skip {label} visual-servo action steps {list(step_indices)}')
            return
        if self.dry_run:
            self.get_logger().info(f'dry-run visual-servo action steps {label}: {action_name} {list(step_indices)}')
            return

        self.stop_robot()
        rows = self.load_action_group_rows(action_name)
        selected = [row for row in rows if int(row[0]) in set(step_indices)]
        if len(selected) != len(set(step_indices)):
            available = [int(row[0]) for row in rows]
            raise RuntimeError(f'{label}: requested steps {list(step_indices)} not in action group {available}')

        names = set(target_names)
        self.get_logger().info(
            f'run visual-servo action steps {action_name} {list(step_indices)} for {label}: '
            f'desired_cx={desired_center:.3f}, target_area={target_area:.3f}'
        )
        for row in selected:
            if self.shutdown_requested:
                raise RuntimeError('stop requested during visual-servo action steps')
            duration = max(0.05, float(row[1]) / 1000.0)
            self.publish_servo_row(row, wait=False)
            self.track_target_for_duration(
                names=names,
                duration=duration,
                desired_center=desired_center,
                target_area=target_area,
                center_tolerance=center_tolerance,
                label=label,
            )
        self.stop_robot()

    def track_target_for_duration(
        self,
        names: Iterable[str],
        duration: float,
        desired_center: float,
        target_area: float,
        center_tolerance: float,
        label: str,
    ) -> None:
        deadline = time.monotonic() + duration
        last_log = 0.0
        while rclpy.ok() and not self.shutdown_requested and time.monotonic() < deadline:
            det = self.best_detection(names)
            now = time.monotonic()
            if det is None:
                self.stop_robot()
                if now - last_log > 0.5:
                    self.get_logger().info(f'visual-servo {label}: target temporarily hidden')
                    last_log = now
                time.sleep(0.06)
                continue

            center_error = det.cx_ratio - desired_center
            area_error = target_area - det.area_ratio
            twist = self.compute_visual_servo_twist(center_error, area_error, center_tolerance)
            self.cmd_pub.publish(twist)
            self.last_twist = twist
            if now - last_log > 0.45:
                self.get_logger().info(
                    f'visual-servo {label}: class={det.class_name} score={det.score:.2f} '
                    f'cx={det.cx_ratio:.3f} area={det.area_ratio:.3f} '
                    f'desired_cx={desired_center:.3f} target_area={target_area:.3f} '
                    f'cmd=({twist.linear.x:.3f},{twist.angular.z:.3f})'
                )
                last_log = now
            time.sleep(0.06)

    def load_action_group_rows(self, action_name: str) -> List[Tuple[int, int, int, int, int, int, int, int]]:
        path = os.path.join(str(self.get_parameter('action_group_path').value), action_name + '.d6a')
        if not os.path.exists(path):
            raise RuntimeError(f'action group file not found: {path}')
        con = sqlite3.connect(path)
        try:
            rows = con.execute('select * from ActionGroup order by [Index]').fetchall()
        finally:
            con.close()
        if not rows:
            raise RuntimeError(f'action group has no steps: {path}')
        return rows

    def publish_servo_row(self, row: Sequence[int], wait: bool = True) -> None:
        if len(row) < 8:
            raise RuntimeError(f'invalid action row: {row!r}')
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = float(row[1]) / 1000.0
        positions = []
        for offset, value in enumerate(row[2:8], start=1):
            servo = self.make_servo_position(10 if offset == 6 else offset, float(value))
            positions.append(servo)
        msg.position = positions
        if self.arm_controller is None:
            raise RuntimeError('arm controller is not initialized')
        self.arm_controller.servo_controller_pub.publish(msg)
        if wait:
            time.sleep(msg.duration)

    def make_servo_position(self, servo_id: int, position: float):
        try:
            from servo_controller_msgs.msg import ServoPosition
        except Exception as exc:
            raise RuntimeError('ServoPosition message is not available') from exc
        servo = ServoPosition()
        servo.id = int(servo_id)
        servo.position = float(position)
        return servo

    def run_action_group(self, action_name: str, label: str) -> None:
        if not self.use_arm:
            self.get_logger().warn(f'use_arm=false: skip {label} action {action_name}')
            return
        if self.dry_run:
            self.get_logger().info(f'dry-run action {label}: {action_name}')
            return
        if self.arm_controller is None:
            raise RuntimeError('arm controller is not initialized')
        self.stop_robot()
        self.get_logger().info(f'run action group {action_name}')
        self.arm_controller.run_action(action_name)
        time.sleep(0.3)

    def stop_robot(self) -> None:
        if not rclpy.ok():
            return
        zero = Twist()
        try:
            for _ in range(3):
                self.cmd_pub.publish(zero)
                time.sleep(0.02)
            self.last_twist = zero
        except Exception as exc:
            self.get_logger().debug(f'ignored stop publish after shutdown: {exc}')

    def set_phase(self, phase: Phase, message: str = '') -> None:
        self.phase = phase
        elapsed = time.monotonic() - self.started_at
        suffix = f': {message}' if message else ''
        self.get_logger().info(f'[{elapsed:6.1f}s] {phase.value}{suffix}')

    def shutdown_if_requested(self) -> None:
        if bool(self.get_parameter('exit_on_done').value) and rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompetitionPickPlace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_requested = True
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
