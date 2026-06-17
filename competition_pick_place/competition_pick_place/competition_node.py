#!/usr/bin/env python3
import math
import os
import sqlite3
import struct
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
from rclpy._rclpy_pybind11 import RCLError
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except Exception:  # pragma: no cover - only used on the robot image.
    BasicNavigator = None
    TaskResult = None

try:
    from servo_controller.action_group_controller import ActionGroupController
    from servo_controller_msgs.msg import ServoStateList, ServosPosition
except Exception:  # pragma: no cover - dry-run can still start without arm libs.
    ActionGroupController = None
    ServoStateList = None
    ServosPosition = None


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Phase(str, Enum):
    INIT = 'INIT'
    NAV_TO_MATERIAL = 'NAV_TO_MATERIAL'
    SEARCH_TARGET = 'SEARCH_TARGET'
    ALIGN_TARGET = 'ALIGN_TARGET'
    PICK = 'PICK'
    ADVANCE_AFTER_PICK = 'ADVANCE_AFTER_PICK'
    NAV_TO_FEED = 'NAV_TO_FEED'
    NAV_TO_PLACE = 'NAV_TO_PLACE'
    ALIGN_PLACE = 'ALIGN_PLACE'
    PLACE = 'PLACE'
    RELEASE = 'RELEASE'
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
    seq: int

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


@dataclass
class DistanceEstimate:
    error: float
    aligned: bool
    source: str
    value: Optional[float]
    target: float
    tolerance: float
    lateral_error: Optional[float] = None
    lateral_target: Optional[float] = None
    lateral_tolerance: Optional[float] = None
    pose: Optional['PoseEstimate'] = None


@dataclass
class PoseEstimate:
    u: float
    v: float
    depth_m: float
    camera_x: float
    camera_y: float
    camera_z: float
    robot_x: float
    robot_y: float
    robot_z: float


@dataclass
class OdomPose:
    x: float
    y: float
    yaw: float
    stamp: float


class CompetitionPickPlace(Node):
    VALID_TARGETS = {'gray', 'grey', 'yellow', 'glass', 'grass', 'blue'}
    DEFAULT_TARGET_ALIASES = {
        'gray': {'gray', 'grey'},
        'grey': {'gray', 'grey'},
        'glass': {'glass', 'blue'},
        'grass': {'grass', 'green'},
        'yellow': {'yellow'},
        'blue': {'blue'},
    }

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
        self.depth_lock = threading.Lock()
        self.camera_info_lock = threading.Lock()
        self.servo_state_lock = threading.Lock()
        self.odom_lock = threading.Lock()
        self.latest_detections: Dict[str, Detection] = {}
        self.latest_depth_msg: Optional[Image] = None
        self.latest_camera_k: Optional[List[float]] = None
        self.latest_odom_pose: Optional[OdomPose] = None
        self.last_msg_time = 0.0
        self.last_depth_time = 0.0
        self.last_camera_info_time = 0.0
        self.detection_message_count = 0
        self.depth_message_count = 0
        self.camera_info_message_count = 0
        self.detection_period_ema = 0.0
        self.latest_servo_positions: Dict[int, int] = {}
        self.last_servo_state_time = 0.0

        share_dir = get_package_share_directory('competition_pick_place')
        self.declare_parameter('target_class', 'gray')
        self.declare_parameter('target_sequence', '')
        self.declare_parameter('target_aliases', '')
        self.declare_parameter('place_class', 'glass')
        self.declare_parameter('feed_waypoint', 'feed_pose')
        self.declare_parameter('post_feed_return_waypoint', 'material_standoff_pose')
        self.declare_parameter('post_pick_advance_m', 0.20)
        self.declare_parameter('post_pick_advance_speed', 0.08)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('exit_on_done', False)
        self.declare_parameter('stop_after_pick', False)
        self.declare_parameter('use_nav', True)
        self.declare_parameter('use_arm', True)
        self.declare_parameter('nav_mode', 'odom')
        self.declare_parameter('waypoints_yaml', os.path.join(share_dir, 'config', 'competition_waypoints.yaml'))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_stale_seconds', 0.6)
        self.declare_parameter('odom_wait_timeout', 10.0)
        self.declare_parameter('odom_goal_tolerance_m', 0.045)
        self.declare_parameter('odom_yaw_tolerance_rad', 0.10)
        self.declare_parameter('odom_control_period', 0.05)
        self.declare_parameter('odom_linear_k', 0.75)
        self.declare_parameter('odom_angular_k', 1.60)
        self.declare_parameter('odom_max_linear_speed', 0.16)
        self.declare_parameter('odom_max_angular_speed', 0.45)
        self.declare_parameter('post_pick_advance_use_odom', True)
        self.declare_parameter('odom_material_x', 1.03)
        self.declare_parameter('odom_material_y', -1.03)
        self.declare_parameter('odom_material_yaw', -0.7853981633974483)
        self.declare_parameter('odom_feed_x', 0.15)
        self.declare_parameter('odom_feed_y', -1.07)
        self.declare_parameter('odom_feed_yaw', 3.141592653589793)
        self.declare_parameter('odom_return_x', 1.03)
        self.declare_parameter('odom_return_y', -1.03)
        self.declare_parameter('odom_return_yaw', -0.7853981633974483)
        self.declare_parameter('detection_topic', '/yolo_node/object_detect')
        self.declare_parameter('use_depth_distance', True)
        self.declare_parameter('depth_topic', '/ascamera/camera_publisher/depth0/image_raw')
        self.declare_parameter('camera_info_topic', '/ascamera/camera_publisher/rgb0/camera_info')
        self.declare_parameter('use_robot_frame_distance', True)
        self.declare_parameter('camera_tilt_deg', 45.0)
        self.declare_parameter('camera_height_m', 0.22)
        self.declare_parameter('camera_offset_x_m', 0.06)
        self.declare_parameter('depth_roi_pixels', 15)
        self.declare_parameter('depth_stale_seconds', 0.8)
        self.declare_parameter('depth_unit_scale', 0.001)
        self.declare_parameter('depth_roi_scale', 0.45)
        self.declare_parameter('depth_sample_grid', 5)
        self.declare_parameter('depth_min_valid_samples', 20)
        self.declare_parameter('depth_min_m', 0.08)
        self.declare_parameter('depth_max_m', 1.50)
        self.declare_parameter('pick_target_depth_m', 0.32)
        self.declare_parameter('pick_target_robot_x_m', 0.32)
        self.declare_parameter('pick_target_robot_y_m', 0.0)
        self.declare_parameter('pick_robot_x_tolerance_m', 0.025)
        self.declare_parameter('pick_robot_y_tolerance_m', 0.025)
        self.declare_parameter('place_target_robot_x_m', 0.145)
        self.declare_parameter('place_target_robot_y_m', 0.0)
        self.declare_parameter('place_robot_x_tolerance_m', 0.015)
        self.declare_parameter('place_robot_y_tolerance_m', 0.015)
        self.declare_parameter('pick_depth_tolerance_m', 0.025)
        self.declare_parameter('pick_preclose_target_depth_m', -1.0)
        self.declare_parameter('cmd_vel_topic', '/controller/cmd_vel')
        self.declare_parameter('min_score', 0.20)
        self.declare_parameter('search_timeout', 18.0)
        self.declare_parameter('align_timeout', 24.0)
        self.declare_parameter('place_align_timeout', 18.0)
        self.declare_parameter('nav_timeout', 180.0)
        self.declare_parameter('detection_stale_seconds', 0.8)
        self.declare_parameter('wait_for_detection_stream', True)
        self.declare_parameter('detection_stream_timeout', 20.0)
        self.declare_parameter('detection_ready_min_messages', 1)
        self.declare_parameter('wait_for_target_before_search', True)
        self.declare_parameter('desired_center_x_ratio', 0.50)
        self.declare_parameter('center_tolerance_ratio', 0.055)
        self.declare_parameter('pick_target_area_ratio', 0.043)
        self.declare_parameter('place_target_area_ratio', 0.080)
        self.declare_parameter('area_tolerance_ratio', 0.018)
        self.declare_parameter('stable_frames', 5)
        self.declare_parameter('control_mode', 'p')
        self.declare_parameter('closed_loop_pick', False)
        self.declare_parameter('pick_visual_servo_timeout', 10.0)
        self.declare_parameter('visual_servo_period', 0.06)
        self.declare_parameter('visual_servo_command_seconds', 0.05)
        self.declare_parameter('adaptive_servo_timing', True)
        self.declare_parameter('visual_servo_min_period', 0.035)
        self.declare_parameter('visual_servo_max_period', 0.16)
        self.declare_parameter('visual_servo_period_scale', 1.05)
        self.declare_parameter('require_fresh_detection_for_control', False)
        self.declare_parameter('pick_pregrasp_visual_servo', True)
        self.declare_parameter('open_gripper_before_approach', True)
        self.declare_parameter('gripper_open_position', 200)
        self.declare_parameter('gripper_open_duration', 0.30)
        self.declare_parameter('pick_pregrasp_time_scale', 1.0)
        self.declare_parameter('pick_pregrasp_min_step_seconds', 0.0)
        self.declare_parameter('pick_pregrasp_settle_seconds', 0.0)
        self.declare_parameter('pick_pregrasp_post_step_seconds', 0.0)
        self.declare_parameter('pick_preclose_required', False)
        self.declare_parameter('pick_preclose_fail_on_timeout', False)
        self.declare_parameter('pick_preclose_center_x_ratio', 0.50)
        self.declare_parameter('pick_preclose_target_area_ratio', -1.0)
        self.declare_parameter('pick_preclose_center_tolerance_ratio', -1.0)
        self.declare_parameter('pick_preclose_area_tolerance_ratio', -1.0)
        self.declare_parameter('pick_preclose_stable_frames', 2)
        self.declare_parameter('pick_pregrasp_steps', '1,2')
        self.declare_parameter('pick_close_steps', '3,4')
        self.declare_parameter('pick_lift_steps', '5,6')
        self.declare_parameter('place_steps', '')
        self.declare_parameter('hold_after_place', True)
        self.declare_parameter('hold_place_steps', '1,2')
        self.declare_parameter('l_shape_push_enabled', False)
        self.declare_parameter('l_shape_push_pose', '518,196,176,597,500,335')
        self.declare_parameter('l_shape_push_pose_action', 'horizontal')
        self.declare_parameter('l_shape_push_pose_step', 1)
        self.declare_parameter('l_shape_push_pose_duration', 1.0)
        self.declare_parameter('l_shape_push_servo_order', '5,4,3,2,1')
        self.declare_parameter('l_shape_push_wrist_servo_index', 4)
        self.declare_parameter('l_shape_push_wrist_position', 108)
        self.declare_parameter('l_shape_push_gripper_position', -1)
        self.declare_parameter('l_shape_push_distance_m', 0.05)
        self.declare_parameter('l_shape_push_speed_mps', 0.04)
        self.declare_parameter('l_shape_push_max_seconds', 2.0)
        self.declare_parameter('l_shape_push_release_before', False)
        self.declare_parameter('l_shape_push_close_after', True)
        self.declare_parameter('l_shape_push_close_position', 500)
        self.declare_parameter('l_shape_push_close_duration', 0.35)
        self.declare_parameter('l_shape_push_lift_action', 'navigation_pick')
        self.declare_parameter('l_shape_push_lift_steps', '5,6')
        self.declare_parameter('release_action', '')
        self.declare_parameter('release_gripper_position', 200)
        self.declare_parameter('release_gripper_duration', 0.35)
        self.declare_parameter('release_settle_seconds', 0.30)
        self.declare_parameter('pick_retry_attempts', 3)
        self.declare_parameter('grasp_check_enabled', True)
        self.declare_parameter('gripper_state_topic', '/controller_manager/servo_states')
        self.declare_parameter('gripper_servo_id', 10)
        self.declare_parameter('gripper_empty_close_position', 500)
        self.declare_parameter('gripper_grasp_min_gap', 30)
        self.declare_parameter('gripper_check_delay', 0.35)
        self.declare_parameter('gripper_feedback_timeout', 2.0)
        self.declare_parameter('linear_k', 0.42)
        self.declare_parameter('angular_k', 1.35)
        self.declare_parameter('max_linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.45)
        self.declare_parameter('search_angular_speed', 0.22)
        self.declare_parameter('angular_sign', -1.0)
        self.declare_parameter('linear_sign', 1.0)
        self.declare_parameter('mpc_horizon', 10)
        self.declare_parameter('mpc_dt', 0.06)
        self.declare_parameter('mpc_center_response', 1.05)
        self.declare_parameter('mpc_area_response', 0.24)
        self.declare_parameter('mpc_center_weight', 8.0)
        self.declare_parameter('mpc_area_weight', 26.0)
        self.declare_parameter('mpc_velocity_weight', 0.08)
        self.declare_parameter('mpc_delta_weight', 0.16)
        self.declare_parameter('mpc_terminal_weight', 2.2)
        self.declare_parameter('mpc_center_gate_ratio', 0.12)
        self.declare_parameter('mpc_forward_lateral_gate', 0.10)
        self.declare_parameter('pick_action', 'navigation_pick')
        self.declare_parameter('place_action', 'navigation_place')
        self.declare_parameter('init_action', 'navigation_pick_init')
        self.declare_parameter('action_group_path', '/home/ubuntu/software/arm_pc/ActionGroups')

        self.target_class = str(self.get_parameter('target_class').value).strip()
        self.target_sequence = self.parse_target_sequence(
            self.get_parameter('target_sequence').value,
            self.target_class,
        )
        self.place_class = self.normalize_place_class(self.get_parameter('place_class').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.stop_after_pick = bool(self.get_parameter('stop_after_pick').value)
        self.use_nav = bool(self.get_parameter('use_nav').value)
        self.use_arm = bool(self.get_parameter('use_arm').value)
        self.nav_mode = str(self.get_parameter('nav_mode').value).strip().lower()
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
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ObjectsInfo,
            str(self.get_parameter('detection_topic').value),
            self.detection_callback,
            10,
        )
        if bool(self.get_parameter('use_depth_distance').value):
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter('camera_info_topic').value),
                self.camera_info_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter('depth_topic').value),
                self.depth_callback,
                qos_profile_sensor_data,
            )
        if ServoStateList is not None:
            self.create_subscription(
                ServoStateList,
                str(self.get_parameter('gripper_state_topic').value),
                self.servo_state_callback,
                10,
            )
        self.create_service(Trigger, '~/stop', self.stop_callback)
        self.create_service(Trigger, '~/init_finish', self.init_finish_callback)

        self.navigator = None
        self.arm_controller = None
        self.arm_ready_client = None
        if self.use_nav and not self.dry_run and self.nav_mode == 'nav2':
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

    def normalize_place_class(self, value: str) -> str:
        target = str(value or '').strip().lower()
        if target in {'glass', 'grass', 'green'}:
            self.get_logger().warn(f'normalize legacy place_class {target!r} -> "blue"')
            return 'blue'
        return target

    def target_names_for(self, target: str) -> set:
        names = set(self.DEFAULT_TARGET_ALIASES.get(target, {target}))
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
        seq = self.detection_message_count + 1
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
                seq=seq,
            )
            old = fresh.get(det.class_name)
            if old is None or self.detection_rank(det) > self.detection_rank(old):
                fresh[det.class_name] = det
        with self.detection_lock:
            if self.last_msg_time > 0.0:
                interval = max(0.001, now - self.last_msg_time)
                if self.detection_period_ema <= 0.0:
                    self.detection_period_ema = interval
                else:
                    self.detection_period_ema = 0.75 * self.detection_period_ema + 0.25 * interval
            for name, det in fresh.items():
                self.latest_detections[name] = det
            self.latest_detections = {
                name: det
                for name, det in self.latest_detections.items()
                if now - det.stamp <= self.stale_seconds
            }
            self.last_msg_time = now
            self.detection_message_count += 1

    def depth_callback(self, msg: Image) -> None:
        with self.depth_lock:
            self.latest_depth_msg = msg
            self.last_depth_time = time.monotonic()
            self.depth_message_count += 1

    def camera_info_callback(self, msg: CameraInfo) -> None:
        with self.camera_info_lock:
            self.latest_camera_k = list(msg.k)
            self.last_camera_info_time = time.monotonic()
            self.camera_info_message_count += 1

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = self.yaw_from_quaternion(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        with self.odom_lock:
            self.latest_odom_pose = OdomPose(
                x=float(position.x),
                y=float(position.y),
                yaw=yaw,
                stamp=time.monotonic(),
            )

    def servo_state_callback(self, msg) -> None:
        now = time.monotonic()
        positions: Dict[int, int] = {}
        for state in msg.servo_state:
            positions[int(state.id)] = int(state.position)
        with self.servo_state_lock:
            self.latest_servo_positions = positions
            self.last_servo_state_time = now

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
                if bool(self.get_parameter('open_gripper_before_approach').value):
                    self.open_gripper_for_approach(f'{target} approach')

                self.set_phase(Phase.SEARCH_TARGET, f'{prefix}: searching {sorted(target_names)}')
                self.visual_servo_to_classes(
                    names=target_names,
                    timeout=float(self.get_parameter('align_timeout').value),
                    target_area=float(self.get_parameter('pick_target_area_ratio').value),
                    label=f'{target} pick target',
                    target_depth=float(self.get_parameter('pick_target_depth_m').value),
                    depth_tolerance=float(self.get_parameter('pick_depth_tolerance_m').value),
                )

                self.set_phase(Phase.PICK, f'{prefix}: running sr pick controller')
                self.run_pick_controller(target_names, f'{target} pick')
                if self.stop_after_pick:
                    self.stop_robot()
                    self.set_phase(Phase.DONE, f'{prefix}: stop_after_pick=true')
                    self.shutdown_if_requested()
                    return

                if self.place_class:
                    place_names = self.target_names_for(self.place_class)
                    self.set_phase(Phase.ALIGN_PLACE, f'{prefix}: aligning hold-place marker {sorted(place_names)}')
                    self.visual_servo_to_classes(
                        names=place_names,
                        timeout=float(self.get_parameter('place_align_timeout').value),
                        target_area=float(self.get_parameter('place_target_area_ratio').value),
                        label=f'{target} hold-place marker',
                        target_robot_x=float(self.get_parameter('place_target_robot_x_m').value),
                        target_robot_y=float(self.get_parameter('place_target_robot_y_m').value),
                        robot_x_tolerance=float(self.get_parameter('place_robot_x_tolerance_m').value),
                        robot_y_tolerance=float(self.get_parameter('place_robot_y_tolerance_m').value),
                    )
                else:
                    self.get_logger().warn('place_class is empty; hold-place uses calibrated action group only')

                self.set_phase(Phase.PLACE, f'{prefix}: running hold-place action without release')
                self.run_hold_place_action(str(self.get_parameter('place_action').value), f'{target} hold-place')
                self.run_l_shape_push(f'{target} l-shape')

                advance_m = float(self.get_parameter('post_pick_advance_m').value)
                self.set_phase(Phase.ADVANCE_AFTER_PICK, f'{prefix}: advancing {advance_m:.3f}m')
                self.drive_body_x(
                    distance_m=advance_m,
                    speed_mps=float(self.get_parameter('post_pick_advance_speed').value),
                    label=f'{target} post-pick advance',
                )

                feed_waypoint = self.feed_waypoint_name()
                self.set_phase(Phase.NAV_TO_FEED, f'{prefix}: going to feed waypoint {feed_waypoint}')
                self.navigate_to(feed_waypoint)

                self.set_phase(Phase.RELEASE, f'{prefix}: releasing gripper at feed waypoint')
                self.release_payload(f'{target} release')

                return_waypoint = self.post_feed_return_waypoint_name()
                self.set_phase(Phase.NAV_HOME, f'{prefix}: returning to {return_waypoint}')
                self.navigate_to(return_waypoint)

            self.stop_robot()
            self.set_phase(Phase.DONE, 'task completed')
            self.shutdown_if_requested()
        except Exception as exc:
            self.stop_robot()
            self.set_phase(Phase.FAILSAFE, str(exc))
            self.shutdown_if_requested()

    def validate_targets(self) -> None:
        if self.nav_mode not in {'odom', 'nav2'}:
            raise RuntimeError("nav_mode must be 'odom' or 'nav2'")
        invalid = [target for target in self.target_sequence if target not in self.VALID_TARGETS]
        if invalid:
            raise RuntimeError(f'targets must be in {sorted(self.VALID_TARGETS)}, got {invalid!r}')
        if not self.dry_run and self.use_nav:
            if self.nav_mode == 'odom':
                self.validate_odom_targets()
                return
            for name in self.required_navigation_waypoints():
                pose = self.waypoints.get(name)
                if not pose:
                    raise RuntimeError(f'missing waypoint: {name}')
                if abs(float(pose.get('x', 0.0))) < 1e-6 and abs(float(pose.get('y', 0.0))) < 1e-6:
                    raise RuntimeError(f'{name} is still [0, 0]; calibrate config/competition_waypoints.yaml first')

    def validate_odom_targets(self) -> None:
        for name in self.required_navigation_waypoints():
            x, y, yaw = self.odom_target_for_waypoint(name)
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise RuntimeError(f'odom target {name} contains non-finite values: {(x, y, yaw)!r}')

    def required_navigation_waypoints(self) -> List[str]:
        names = [
            'material_standoff_pose',
            self.feed_waypoint_name(),
            self.post_feed_return_waypoint_name(),
        ]
        return self.unique_waypoint_names(names)

    def feed_waypoint_name(self) -> str:
        name = str(self.get_parameter('feed_waypoint').value).strip()
        if not name:
            raise RuntimeError('feed_waypoint is empty')
        return name

    def post_feed_return_waypoint_name(self) -> str:
        name = str(self.get_parameter('post_feed_return_waypoint').value).strip()
        if not name:
            raise RuntimeError('post_feed_return_waypoint is empty')
        return name

    @staticmethod
    def unique_waypoint_names(names: Iterable[str]) -> List[str]:
        result: List[str] = []
        seen = set()
        for name in names:
            if name and name not in seen:
                seen.add(name)
                result.append(name)
        return result

    def wait_for_systems(self) -> None:
        if self.dry_run:
            self.get_logger().info('dry_run=true: hardware movement and blocking waits are skipped')
            return
        if self.navigator is not None:
            self.get_logger().info('waiting for Nav2 active')
            self.navigator.waitUntilNav2Active()
        if self.use_nav and self.nav_mode == 'odom':
            self.get_logger().info('waiting for fresh /odom')
            self.wait_for_fresh_odom('startup')
        if self.arm_ready_client is not None:
            self.get_logger().info('waiting for controller_manager/init_finish')
            if not self.arm_ready_client.wait_for_service(timeout_sec=12.0):
                raise RuntimeError('/controller_manager/init_finish service is not available')

    def navigate_to(self, waypoint_name: str) -> None:
        if not self.use_nav:
            self.get_logger().warn(f'use_nav=false: skip waypoint {waypoint_name}')
            return
        if self.nav_mode == 'odom':
            x, y, yaw = self.odom_target_for_waypoint(waypoint_name)
            if self.dry_run:
                self.get_logger().info(
                    f'dry-run odom navigation to {waypoint_name}: '
                    f'x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
                )
                return
            self.navigate_to_odom_pose(x, y, yaw, waypoint_name)
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

    def odom_target_for_waypoint(self, waypoint_name: str) -> Tuple[float, float, float]:
        if waypoint_name == 'material_standoff_pose':
            return self.odom_target_from_prefix('odom_material')
        if waypoint_name == self.feed_waypoint_name() or waypoint_name == 'feed_pose':
            return self.odom_target_from_prefix('odom_feed')
        if waypoint_name == 'return_pose':
            return self.odom_target_from_prefix('odom_return')
        if waypoint_name == self.post_feed_return_waypoint_name():
            return self.odom_target_from_prefix('odom_return')

        pose = self.waypoints.get(waypoint_name)
        if pose is not None:
            return (
                float(pose.get('x', 0.0)),
                float(pose.get('y', 0.0)),
                float(pose.get('yaw', 0.0)),
            )
        raise RuntimeError(f'missing odom target for waypoint {waypoint_name}')

    def odom_target_from_prefix(self, prefix: str) -> Tuple[float, float, float]:
        return (
            float(self.get_parameter(f'{prefix}_x').value),
            float(self.get_parameter(f'{prefix}_y').value),
            float(self.get_parameter(f'{prefix}_yaw').value),
        )

    def wait_for_fresh_odom(self, label: str) -> OdomPose:
        timeout = max(0.1, float(self.get_parameter('odom_wait_timeout').value))
        deadline = time.monotonic() + timeout
        last_error = 'no odom received'
        while rclpy.ok() and not self.shutdown_requested and time.monotonic() < deadline:
            try:
                return self.current_odom_pose()
            except RuntimeError as exc:
                last_error = str(exc)
                time.sleep(0.05)
        raise RuntimeError(f'{label}: fresh odom unavailable after {timeout:.1f}s ({last_error})')

    def current_odom_pose(self) -> OdomPose:
        with self.odom_lock:
            pose = self.latest_odom_pose
        if pose is None:
            raise RuntimeError('no odom received')
        age = time.monotonic() - pose.stamp
        max_age = max(0.05, float(self.get_parameter('odom_stale_seconds').value))
        if age > max_age:
            raise RuntimeError(f'odom is stale: age={age:.2f}s')
        return pose

    def navigate_to_odom_pose(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        label: str,
        max_linear_speed: Optional[float] = None,
    ) -> None:
        pos_tol = max(0.005, float(self.get_parameter('odom_goal_tolerance_m').value))
        yaw_tol = max(0.01, float(self.get_parameter('odom_yaw_tolerance_rad').value))
        linear_k = max(0.0, float(self.get_parameter('odom_linear_k').value))
        angular_k = max(0.0, float(self.get_parameter('odom_angular_k').value))
        max_linear = max(0.01, float(self.get_parameter('odom_max_linear_speed').value))
        if max_linear_speed is not None:
            max_linear = min(max_linear, max(0.01, abs(float(max_linear_speed))))
        max_angular = max(0.01, float(self.get_parameter('odom_max_angular_speed').value))
        period = self.clamp(float(self.get_parameter('odom_control_period').value), 0.02, 0.5)
        deadline = time.monotonic() + max(1.0, float(self.get_parameter('nav_timeout').value))

        self.get_logger().info(
            f'odom goal {label}: x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw:.3f}, '
            f'pos_tol={pos_tol:.3f}, yaw_tol={yaw_tol:.3f}'
        )
        try:
            while rclpy.ok() and not self.shutdown_requested:
                pose = self.current_odom_pose()
                dx = target_x - pose.x
                dy = target_y - pose.y
                distance = math.hypot(dx, dy)
                yaw_error = normalize_angle(target_yaw - pose.yaw)
                if distance <= pos_tol and abs(yaw_error) <= yaw_tol:
                    self.get_logger().info(
                        f'odom reached {label}: x={pose.x:.3f}, y={pose.y:.3f}, yaw={pose.yaw:.3f}, '
                        f'distance_error={distance:.3f}, yaw_error={yaw_error:.3f}'
                    )
                    return
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f'odom navigation timeout at {label}: '
                        f'distance_error={distance:.3f}, yaw_error={yaw_error:.3f}'
                    )

                twist = Twist()
                if distance > pos_tol:
                    cos_yaw = math.cos(pose.yaw)
                    sin_yaw = math.sin(pose.yaw)
                    body_x = cos_yaw * dx + sin_yaw * dy
                    body_y = -sin_yaw * dx + cos_yaw * dy
                    twist.linear.x = self.clamp(linear_k * body_x, -max_linear, max_linear)
                    twist.linear.y = self.clamp(linear_k * body_y, -max_linear, max_linear)
                    speed = math.hypot(twist.linear.x, twist.linear.y)
                    if speed > max_linear:
                        scale = max_linear / speed
                        twist.linear.x *= scale
                        twist.linear.y *= scale
                if abs(yaw_error) > yaw_tol:
                    twist.angular.z = self.clamp(angular_k * yaw_error, -max_angular, max_angular)

                self.cmd_pub.publish(twist)
                self.last_twist = twist
                time.sleep(period)
        finally:
            self.stop_robot()
        if self.shutdown_requested:
            raise RuntimeError(f'stop requested during odom navigation to {label}')

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

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
        target_depth: Optional[float] = None,
        depth_tolerance: Optional[float] = None,
        target_robot_x: Optional[float] = None,
        target_robot_y: Optional[float] = None,
        robot_x_tolerance: Optional[float] = None,
        robot_y_tolerance: Optional[float] = None,
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
        target_depth = self.valid_optional_depth(target_depth)
        depth_tolerance = (
            max(0.001, float(self.get_parameter('pick_depth_tolerance_m').value))
            if depth_tolerance is None
            else max(0.001, float(depth_tolerance))
        )
        stable_required = self.stable_frames if stable_frames is None else max(1, int(stable_frames))
        if self.dry_run:
            self.get_logger().info(
                f'dry-run visual servo {label}: names={sorted(names)}, target_area={target_area}, '
                f'target_depth={target_depth}, desired_center={desired_center}, control_mode={self.control_mode}'
            )
            return

        self.wait_until_detection_stream_ready(label, timeout, names=names)
        start = time.monotonic()
        found_deadline = start + min(float(self.get_parameter('search_timeout').value), timeout)
        align_deadline = start + timeout
        period = self.visual_servo_period_seconds()
        stable = 0
        last_log = 0.0
        found_once = False
        last_control_seq = -1
        last_seen_center_error: Optional[float] = None
        center_tol = center_tolerance
        area_tol = area_tolerance

        while rclpy.ok() and not self.shutdown_requested:
            det = self.best_detection(names)
            now = time.monotonic()
            if det is None:
                stable = 0
                if now > align_deadline:
                    if found_once:
                        raise RuntimeError(f'{label} alignment timeout after temporary target loss')
                    raise RuntimeError(f'{label} not found before alignment timeout')
                if not found_once and now > found_deadline:
                    raise RuntimeError(f'{label} not found before search timeout')
                self.stop_robot()
                if now - last_log > 1.5:
                    self.get_logger().info(
                        f'waiting {label}; target not visible, found_once={found_once}, '
                        f'detections={self.visible_classes()}'
                    )
                    last_log = now
                time.sleep(period)
                continue

            found_once = True
            if now > align_deadline:
                raise RuntimeError(f'{label} alignment timeout')

            if self.should_wait_for_fresh_detection(det, last_control_seq):
                self.stop_robot()
                if now - last_log > 0.8:
                    self.get_logger().info(
                        f'waiting fresh detection for {label}: '
                        f'class={det.class_name} seq={det.seq} last_control_seq={last_control_seq}'
                    )
                    last_log = now
                time.sleep(period)
                continue
            last_control_seq = det.seq
            distance = self.distance_estimate_for_detection(
                det,
                target_area=target_area,
                area_tolerance=area_tol,
                target_depth=target_depth,
                depth_tolerance=depth_tolerance,
                target_robot_x=target_robot_x,
                target_robot_y=target_robot_y,
                robot_x_tolerance=robot_x_tolerance,
                robot_y_tolerance=robot_y_tolerance,
            )
            if bool(self.get_parameter('use_robot_frame_distance').value) and distance.source != 'robot_frame':
                stable = 0
                self.stop_robot()
                if now - last_log > 0.8:
                    self.log_alignment_state(
                        label=label,
                        det=det,
                        distance=distance,
                        center_error=det.cx_ratio - desired_center,
                        center_tolerance=center_tol,
                        twist=None,
                        prefix='waiting robot-frame pose',
                    )
                    last_log = now
                time.sleep(period)
                continue
            center_error = (
                distance.lateral_error
                if distance.lateral_error is not None
                else det.cx_ratio - desired_center
            )
            last_seen_center_error = center_error
            active_center_tol = (
                distance.lateral_tolerance
                if distance.lateral_tolerance is not None
                else center_tol
            )

            if abs(center_error) <= active_center_tol and distance.aligned:
                stable += 1
                self.stop_robot()
                if stable >= stable_required:
                    self.log_alignment_state(
                        label=label,
                        det=det,
                        distance=distance,
                        center_error=center_error,
                        center_tolerance=active_center_tol,
                        twist=None,
                        prefix='aligned',
                    )
                    return
                time.sleep(period)
                continue

            stable = 0
            twist = self.compute_visual_servo_twist(distance, center_error, active_center_tol)
            if now - last_log > 0.8:
                self.log_alignment_state(
                    label=label,
                    det=det,
                    distance=distance,
                    center_error=center_error,
                    center_tolerance=active_center_tol,
                    twist=twist,
                    prefix='visual servo',
                )
                last_log = now
            self.publish_control_pulse(twist, period)

        raise RuntimeError('stop requested during alignment')

    def wait_until_detection_stream_ready(
        self,
        label: str,
        align_timeout: float,
        names: Optional[Iterable[str]] = None,
    ) -> None:
        if not bool(self.get_parameter('wait_for_detection_stream').value):
            return
        target_names = set(names or [])
        require_target = bool(self.get_parameter('wait_for_target_before_search').value) and bool(target_names)
        min_messages = max(1, int(self.get_parameter('detection_ready_min_messages').value))
        timeout = min(float(self.get_parameter('detection_stream_timeout').value), max(1.0, align_timeout))
        deadline = time.monotonic() + timeout
        period = self.visual_servo_period_seconds()
        last_log = 0.0
        while rclpy.ok() and not self.shutdown_requested:
            with self.detection_lock:
                count = self.detection_message_count
                last_msg_age = time.monotonic() - self.last_msg_time if self.last_msg_time > 0.0 else float('inf')
                visible = sorted(self.latest_detections)
                target_visible = any(name in self.latest_detections for name in target_names)
            stream_ready = count >= min_messages and last_msg_age <= self.stale_seconds
            if stream_ready and (target_visible or not require_target):
                self.get_logger().info(
                    f'YOLO detection stream ready for {label}: messages={count}, visible={visible}, '
                    f'target_visible={target_visible}'
                )
                return
            now = time.monotonic()
            if now > deadline:
                raise RuntimeError(
                    f'YOLO detection stream not ready before {timeout:.1f}s for {label}; '
                    f'messages={count}, visible={visible}, target_names={sorted(target_names)}, '
                    f'target_visible={target_visible}'
                )
            self.stop_robot()
            if now - last_log > 1.0:
                publisher_count = self.count_publishers(str(self.get_parameter('detection_topic').value))
                self.get_logger().info(
                    f'waiting YOLO detection stream for {label}: '
                    f'publishers={publisher_count}, messages={count}, visible={visible}, '
                    f'target_names={sorted(target_names)}, target_visible={target_visible}'
                )
                last_log = now
            time.sleep(period)

    def compute_visual_servo_twist(self, distance: DistanceEstimate, center_error: float, center_tol: float) -> Twist:
        if distance.source == 'robot_frame' and distance.lateral_error is not None:
            return self.compute_robot_frame_twist(distance)
        if self.control_mode == 'mpc':
            return self.compute_mpc_twist(center_error, distance.error, center_tol)
        return self.compute_p_twist(center_error, distance.error, center_tol)

    def compute_robot_frame_twist(self, distance: DistanceEstimate) -> Twist:
        twist = Twist()
        max_linear = float(self.get_parameter('max_linear_speed').value)
        max_angular = float(self.get_parameter('max_angular_speed').value)
        linear_sign = float(self.get_parameter('linear_sign').value)
        angular_sign = float(self.get_parameter('angular_sign').value)
        x_error = distance.error
        y_error = float(distance.lateral_error or 0.0)
        x_tol = max(0.001, distance.tolerance)
        y_tol = max(0.001, float(distance.lateral_tolerance or x_tol))

        if abs(x_error) > x_tol:
            linear_mag = self.clamp(abs(x_error) * 1.8, min(0.04, max_linear), max_linear)
            twist.linear.x = linear_sign * math.copysign(linear_mag, x_error)

        if abs(y_error) > y_tol:
            angular_mag = self.clamp(abs(y_error) * 4.0, min(0.12, max_angular), max_angular)
            twist.angular.z = angular_sign * math.copysign(angular_mag, y_error)

        return twist

    def should_wait_for_fresh_detection(self, det: Detection, last_control_seq: int) -> bool:
        return (
            bool(self.get_parameter('require_fresh_detection_for_control').value)
            and last_control_seq >= 0
            and det.seq <= last_control_seq
        )

    def valid_optional_depth(self, value: Optional[float]) -> Optional[float]:
        if value is None:
            return None
        depth = float(value)
        return depth if depth > 0.0 else None

    def distance_estimate_for_detection(
        self,
        det: Detection,
        target_area: float,
        area_tolerance: float,
        target_depth: Optional[float],
        depth_tolerance: Optional[float],
        target_robot_x: Optional[float] = None,
        target_robot_y: Optional[float] = None,
        robot_x_tolerance: Optional[float] = None,
        robot_y_tolerance: Optional[float] = None,
    ) -> DistanceEstimate:
        if bool(self.get_parameter('use_robot_frame_distance').value):
            target_x = float(self.get_parameter('pick_target_robot_x_m').value) if target_robot_x is None else float(target_robot_x)
            target_y = float(self.get_parameter('pick_target_robot_y_m').value) if target_robot_y is None else float(target_robot_y)
            tol_x = max(
                0.001,
                float(self.get_parameter('pick_robot_x_tolerance_m').value)
                if robot_x_tolerance is None
                else float(robot_x_tolerance),
            )
            tol_y = max(
                0.001,
                float(self.get_parameter('pick_robot_y_tolerance_m').value)
                if robot_y_tolerance is None
                else float(robot_y_tolerance),
            )
            pose = self.estimate_detection_pose(det)
            if pose is not None:
                forward_error = pose.robot_x - target_x
                lateral_error = target_y - pose.robot_y
                return DistanceEstimate(
                    error=forward_error,
                    aligned=abs(forward_error) <= tol_x and abs(lateral_error) <= tol_y,
                    source='robot_frame',
                    value=pose.robot_x,
                    target=target_x,
                    tolerance=tol_x,
                    lateral_error=lateral_error,
                    lateral_target=target_y,
                    lateral_tolerance=tol_y,
                    pose=pose,
                )
            return DistanceEstimate(
                error=float('inf'),
                aligned=False,
                source='robot_frame_wait',
                value=None,
                target=target_x,
                tolerance=tol_x,
                lateral_error=None,
                lateral_target=target_y,
                lateral_tolerance=tol_y,
            )

        depth_target = self.valid_optional_depth(target_depth)
        if bool(self.get_parameter('use_depth_distance').value) and depth_target is not None:
            depth_m = self.estimate_detection_depth_m(det)
            if depth_m is not None:
                tolerance = max(0.001, float(depth_tolerance or self.get_parameter('pick_depth_tolerance_m').value))
                error = depth_m - depth_target
                return DistanceEstimate(
                    error=error,
                    aligned=abs(error) <= tolerance,
                    source='depth',
                    value=depth_m,
                    target=depth_target,
                    tolerance=tolerance,
                )
            tolerance = max(0.001, float(depth_tolerance or self.get_parameter('pick_depth_tolerance_m').value))
            return DistanceEstimate(
                error=float('inf'),
                aligned=False,
                source='depth_wait',
                value=None,
                target=depth_target,
                tolerance=tolerance,
            )

        return DistanceEstimate(
            error=0.0,
            aligned=True,
            source='center_only',
            value=None,
            target=0.0,
            tolerance=0.0,
        )

    def estimate_detection_depth_m(self, det: Detection) -> Optional[float]:
        now = time.monotonic()
        with self.depth_lock:
            msg = self.latest_depth_msg
            stamp = self.last_depth_time
        if msg is None or now - stamp > float(self.get_parameter('depth_stale_seconds').value):
            return None
        return self.sample_depth_roi_m(msg, det)

    def estimate_detection_pose(self, det: Detection) -> Optional[PoseEstimate]:
        now = time.monotonic()
        with self.depth_lock:
            depth_msg = self.latest_depth_msg
            depth_stamp = self.last_depth_time
        with self.camera_info_lock:
            camera_k = list(self.latest_camera_k) if self.latest_camera_k is not None else None
        if depth_msg is None or now - depth_stamp > float(self.get_parameter('depth_stale_seconds').value):
            return None
        if camera_k is None or len(camera_k) < 6:
            return None

        center = self.detection_center_in_depth_pixels(depth_msg, det)
        if center is None:
            return None
        u, v = center
        depth_m = self.sample_depth_roi_m(depth_msg, det, center=center)
        if depth_m is None:
            return None

        fx = float(camera_k[0])
        fy = float(camera_k[4])
        cx_img = float(camera_k[2])
        cy_img = float(camera_k[5])
        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            return None

        scale_x = float(depth_msg.width) / max(1, det.width)
        scale_y = float(depth_msg.height) / max(1, det.height)
        if abs(scale_x - 1.0) > 1e-6 or abs(scale_y - 1.0) > 1e-6:
            cx_img *= scale_x
            cy_img *= scale_y
            fx *= scale_x
            fy *= scale_y

        camera_x = (u - cx_img) * depth_m / fx
        camera_y = (v - cy_img) * depth_m / fy
        camera_z = depth_m
        robot_x, robot_y, robot_z = self.transform_camera_to_robot(camera_x, camera_y, camera_z)
        return PoseEstimate(
            u=u,
            v=v,
            depth_m=depth_m,
            camera_x=camera_x,
            camera_y=camera_y,
            camera_z=camera_z,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_z=robot_z,
        )

    def transform_camera_to_robot(self, camera_x: float, camera_y: float, camera_z: float) -> Tuple[float, float, float]:
        theta = math.radians(float(self.get_parameter('camera_tilt_deg').value))
        height = float(self.get_parameter('camera_height_m').value)
        offset_x = float(self.get_parameter('camera_offset_x_m').value)
        robot_x = camera_z * math.cos(theta) - camera_y * math.sin(theta) + offset_x
        robot_y = -camera_x
        vertical_drop = camera_y * math.cos(theta) + camera_z * math.sin(theta)
        robot_z = height - vertical_drop
        return robot_x, robot_y, robot_z

    def detection_center_in_depth_pixels(self, msg: Image, det: Detection) -> Optional[Tuple[float, float]]:
        if msg.width <= 0 or msg.height <= 0 or det.width <= 0 or det.height <= 0:
            return None
        scale_x = float(msg.width) / max(1, det.width)
        scale_y = float(msg.height) / max(1, det.height)
        u = ((det.box[0] + det.box[2]) * 0.5) * scale_x
        v = ((det.box[1] + det.box[3]) * 0.5) * scale_y
        if u < 0 or u >= msg.width or v < 0 or v >= msg.height:
            return None
        return u, v

    def sample_depth_roi_m(
        self,
        msg: Image,
        det: Detection,
        center: Optional[Tuple[float, float]] = None,
    ) -> Optional[float]:
        encoding = str(msg.encoding or '').upper()
        if encoding not in {'16UC1', 'MONO16', '32FC1'}:
            return None
        if msg.width <= 0 or msg.height <= 0 or msg.step <= 0:
            return None

        if center is None:
            center = self.detection_center_in_depth_pixels(msg, det)
        if center is None:
            return None
        cx, cy = center
        radius = max(1, int(self.get_parameter('depth_roi_pixels').value))
        rx1 = int(self.clamp(cx - radius, 0, msg.width - 1))
        rx2 = int(self.clamp(cx + radius, 0, msg.width - 1))
        ry1 = int(self.clamp(cy - radius, 0, msg.height - 1))
        ry2 = int(self.clamp(cy + radius, 0, msg.height - 1))

        samples: List[float] = []
        for y in range(ry1, ry2 + 1):
            for x in range(rx1, rx2 + 1):
                depth = self.read_depth_pixel_m(msg, x, y, encoding)
                if depth is not None and self.depth_is_valid(depth):
                    samples.append(depth)

        min_samples = max(1, int(self.get_parameter('depth_min_valid_samples').value))
        if len(samples) < min_samples:
            return None
        return sum(samples) / len(samples)

    def read_depth_pixel_m(self, msg: Image, x: int, y: int, encoding: str) -> Optional[float]:
        byteorder = 'big' if bool(msg.is_bigendian) else 'little'
        try:
            if encoding in {'16UC1', 'MONO16'}:
                offset = y * msg.step + x * 2
                if offset + 2 > len(msg.data):
                    return None
                raw = int.from_bytes(bytes(msg.data[offset:offset + 2]), byteorder=byteorder, signed=False)
                if raw <= 0:
                    return None
                return raw * float(self.get_parameter('depth_unit_scale').value)
            if encoding == '32FC1':
                offset = y * msg.step + x * 4
                if offset + 4 > len(msg.data):
                    return None
                fmt = '>f' if bool(msg.is_bigendian) else '<f'
                value = float(struct.unpack(fmt, bytes(msg.data[offset:offset + 4]))[0])
                if not math.isfinite(value):
                    return None
                return value
        except Exception:
            return None
        return None

    def depth_is_valid(self, depth_m: float) -> bool:
        return (
            math.isfinite(depth_m)
            and float(self.get_parameter('depth_min_m').value) <= depth_m <= float(self.get_parameter('depth_max_m').value)
        )

    @staticmethod
    def format_optional(value: Optional[float]) -> str:
        return 'none' if value is None else f'{value:.3f}'

    def log_alignment_state(
        self,
        label: str,
        det: Detection,
        distance: DistanceEstimate,
        center_error: float,
        center_tolerance: float,
        twist: Optional[Twist],
        prefix: str,
    ) -> None:
        cmd = 'cmd=(0.000,0.000)' if twist is None else f'cmd=({twist.linear.x:.3f},{twist.angular.z:.3f})'
        pose_text = ''
        if distance.pose is not None:
            pose = distance.pose
            pose_text = (
                f' u={pose.u:.1f} v={pose.v:.1f} depth={pose.depth_m:.3f} '
                f'camera_xyz=({pose.camera_x:.3f},{pose.camera_y:.3f},{pose.camera_z:.3f}) '
                f'robot_xyz=({pose.robot_x:.3f},{pose.robot_y:.3f},{pose.robot_z:.3f}) '
                f'robot_y_target={self.format_optional(distance.lateral_target)} '
                f'robot_y_error={self.format_optional(distance.lateral_error)} '
                f'robot_y_tol={self.format_optional(distance.lateral_tolerance)}'
            )
        self.get_logger().info(
            f'{prefix} {label}: mode={self.control_mode} class={det.class_name} score={det.score:.2f} '
            f'cx={det.cx_ratio:.3f} area={det.area_ratio:.3f} '
            f'source={distance.source} value={self.format_optional(distance.value)} '
            f'target={distance.target:.3f} error={distance.error:.3f} tol={distance.tolerance:.3f} '
            f'lateral_error={center_error:.3f} lateral_tol={center_tolerance:.3f}{pose_text} {cmd}'
        )

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
        configured_gate = float(self.get_parameter('mpc_center_gate_ratio').value)
        forward_gate = max(center_tol, float(self.get_parameter('mpc_forward_lateral_gate').value))
        center_gate = self.clamp(configured_gate, center_tol, forward_gate)
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

    def publish_search_twist(
        self,
        period: Optional[float] = None,
        last_center_error: Optional[float] = None,
    ) -> None:
        twist = Twist()
        search_speed = float(self.get_parameter('search_angular_speed').value)
        if last_center_error is not None and abs(last_center_error) > 1e-6:
            target_side = 1.0 if last_center_error > 0.0 else -1.0
            twist.angular.z = float(self.get_parameter('angular_sign').value) * abs(search_speed) * target_side
        else:
            twist.angular.z = search_speed
        self.publish_control_pulse(twist, self.visual_servo_period_seconds() if period is None else period)

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
        retry_attempts = max(1, int(self.get_parameter('pick_retry_attempts').value))

        self.get_logger().info(
            f'closed-loop pick {label}: pregrasp={pregrasp_steps}, close={close_steps}, '
            f'lift={lift_steps}, attempts={retry_attempts}, mode={self.control_mode}'
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
        preclose_depth = float(self.get_parameter('pick_preclose_target_depth_m').value)
        if preclose_depth <= 0.0:
            preclose_depth = float(self.get_parameter('pick_target_depth_m').value)
        preclose_depth_tol = float(self.get_parameter('pick_depth_tolerance_m').value)

        preclose_center = float(self.get_parameter('pick_preclose_center_x_ratio').value)
        for attempt in range(1, retry_attempts + 1):
            attempt_label = f'{label} attempt {attempt}/{retry_attempts}'
            if attempt > 1:
                self.get_logger().info(f'{attempt_label}: realigning target before retry')
                self.visual_servo_to_classes(
                    names=target_names,
                    timeout=float(self.get_parameter('align_timeout').value),
                    target_area=float(self.get_parameter('pick_target_area_ratio').value),
                    label=f'{attempt_label} retry align',
                    target_depth=float(self.get_parameter('pick_target_depth_m').value),
                    depth_tolerance=float(self.get_parameter('pick_depth_tolerance_m').value),
                )

            if bool(self.get_parameter('pick_pregrasp_visual_servo').value):
                self.run_action_group_steps_with_visual_servo(
                    pick_action,
                    pregrasp_steps,
                    target_names,
                    f'{attempt_label} pregrasp',
                    desired_center=preclose_center,
                    target_area=preclose_area,
                    center_tolerance=preclose_center_tol,
                    area_tolerance=preclose_area_tol,
                    target_depth=preclose_depth,
                    depth_tolerance=preclose_depth_tol,
                )
            else:
                self.run_action_group_steps(pick_action, pregrasp_steps, f'{attempt_label} pregrasp')

            if bool(self.get_parameter('pick_preclose_required').value):
                try:
                    self.visual_servo_to_classes(
                        names=target_names,
                        timeout=float(self.get_parameter('pick_visual_servo_timeout').value),
                        target_area=preclose_area,
                        label=f'{attempt_label} pre-close',
                        desired_center=preclose_center,
                        center_tolerance=preclose_center_tol,
                        area_tolerance=preclose_area_tol,
                        stable_frames=int(self.get_parameter('pick_preclose_stable_frames').value),
                        target_depth=preclose_depth,
                        depth_tolerance=preclose_depth_tol,
                    )
                except RuntimeError as exc:
                    self.stop_robot()
                    if bool(self.get_parameter('pick_preclose_fail_on_timeout').value):
                        raise
                    self.get_logger().warn(f'best-effort {attempt_label} pre-close skipped: {exc}')

            self.run_action_group_steps(pick_action, close_steps, f'{attempt_label} close')
            if self.grasp_succeeded(attempt_label):
                self.run_action_group_steps(pick_action, lift_steps, f'{attempt_label} lift')
                return

            if attempt < retry_attempts:
                self.get_logger().warn(f'{attempt_label}: grasp check says empty; lifting clear and retrying')
                self.run_action_group_steps(pick_action, lift_steps, f'{attempt_label} failed lift-clear')
            else:
                self.run_action_group_steps(pick_action, lift_steps, f'{attempt_label} final lift-clear')
                raise RuntimeError(f'{label} failed after {retry_attempts} grasp attempts')

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

    def resolve_place_steps(self) -> List[int]:
        explicit_steps = self.parse_step_indices(self.get_parameter('place_steps').value)
        if explicit_steps:
            return explicit_steps

        if bool(self.get_parameter('hold_after_place').value):
            hold_steps = self.parse_step_indices(self.get_parameter('hold_place_steps').value)
            if hold_steps:
                self.get_logger().info(
                    f'hold_after_place=true: using partial place steps {list(hold_steps)} '
                    'to keep the gripper closed after placement'
                )
                return hold_steps

        return []

    def run_hold_place_action(self, action_name: str, label: str) -> None:
        hold_steps = self.resolve_place_steps()
        if hold_steps:
            self.run_action_group_steps(action_name, hold_steps, label)
            return
        self.get_logger().warn(
            f'{label}: no place_steps/hold_place_steps configured; skipping hold-place transition '
            'to avoid releasing the gripper early'
        )

    def run_l_shape_push(self, label: str) -> None:
        if not bool(self.get_parameter('l_shape_push_enabled').value):
            self.get_logger().info(f'skip l-shape push {label}: disabled')
            return

        distance = max(0.0, float(self.get_parameter('l_shape_push_distance_m').value))
        speed = max(0.0, float(self.get_parameter('l_shape_push_speed_mps').value))
        max_seconds = max(0.05, float(self.get_parameter('l_shape_push_max_seconds').value))
        duration = 0.0 if speed <= 1e-6 else min(max_seconds, distance / speed)
        release_before = bool(self.get_parameter('l_shape_push_release_before').value)
        close_after = bool(self.get_parameter('l_shape_push_close_after').value)
        lift_action = str(self.get_parameter('l_shape_push_lift_action').value)
        lift_steps = self.parse_step_indices(self.get_parameter('l_shape_push_lift_steps').value)

        if self.dry_run:
            self.get_logger().info(
                f'dry-run l-shape push {label}: pose={self.l_shape_push_pose_description()} '
                f'distance={distance:.3f}m speed={speed:.3f}m/s duration={duration:.3f}s '
                f'release_before={release_before} close_after={close_after} '
                f'lift_action={lift_action} lift_steps={list(lift_steps)}'
            )
            return
        if not self.use_arm:
            self.get_logger().warn(f'use_arm=false: skip {label} l-shape push')
            return
        if ServosPosition is None:
            raise RuntimeError('servo_controller_msgs is not available')

        self.stop_robot()
        if release_before:
            self.open_gripper_for_release(f'{label} release-before-push')
            self.stop_robot()
        push_row = self.resolve_l_shape_push_row(label)
        self.get_logger().info(
            f'run l-shape push {label}: pose={self.l_shape_push_pose_description()} '
            f'row={self.format_action_row(push_row)} '
            f'distance={distance:.3f}m speed={speed:.3f}m/s duration={duration:.3f}s '
            f'release_before={release_before} close_after={close_after}'
        )
        self.publish_l_shape_push_pose(push_row, label)
        if distance > 1e-6 and speed > 1e-6:
            self.run_blind_linear(f'{label} push', distance, speed, max_seconds)
        if close_after:
            self.publish_gripper_position(
                f'{label} close',
                float(self.get_parameter('l_shape_push_close_position').value),
                float(self.get_parameter('l_shape_push_close_duration').value),
            )
            if lift_steps:
                self.run_action_group_steps(lift_action, lift_steps, f'{label} lift')

    def resolve_l_shape_push_row(self, label: str) -> List[int]:
        pose_text = str(self.get_parameter('l_shape_push_pose').value).strip()
        duration_ms = int(round(max(0.05, float(self.get_parameter('l_shape_push_pose_duration').value)) * 1000.0))
        if pose_text:
            parts = [part.strip() for part in pose_text.split(',') if part.strip()]
            if len(parts) != 6:
                raise RuntimeError(f'{label}: l_shape_push_pose must contain 6 servo values, got {pose_text!r}')
            try:
                values = [int(round(float(part))) for part in parts]
            except ValueError as exc:
                raise RuntimeError(f'{label}: invalid l_shape_push_pose {pose_text!r}') from exc
            row = [0, duration_ms, *values]
        else:
            action = str(self.get_parameter('l_shape_push_pose_action').value)
            step = int(self.get_parameter('l_shape_push_pose_step').value)
            rows = self.load_action_group_rows(action)
            selected = [row for row in rows if int(row[0]) == step]
            if not selected:
                available = [int(row[0]) for row in rows]
                raise RuntimeError(f'{label}: l-shape pose step {step} not in {action}; available={available}')
            row = list(selected[0])
            row[1] = duration_ms

        wrist_position = float(self.get_parameter('l_shape_push_wrist_position').value)
        wrist_index = int(self.get_parameter('l_shape_push_wrist_servo_index').value)
        if wrist_position >= 0.0:
            if wrist_index < 1 or wrist_index > 5:
                raise RuntimeError(f'{label}: l_shape_push_wrist_servo_index must be 1..5, got {wrist_index}')
            row[1 + wrist_index] = int(round(wrist_position))

        gripper_override = float(self.get_parameter('l_shape_push_gripper_position').value)
        if gripper_override >= 0.0:
            row[7] = int(round(gripper_override))
        return row

    def parse_servo_order(self, value, label: str) -> List[int]:
        if value is None:
            return []
        if isinstance(value, str):
            parts = [part.strip() for part in value.split(',') if part.strip()]
        else:
            parts = [str(part).strip() for part in value if str(part).strip()]
        result: List[int] = []
        seen = set()
        for part in parts:
            try:
                servo_id = int(part)
            except ValueError as exc:
                raise RuntimeError(f'{label}: invalid servo id in l_shape_push_servo_order: {part!r}') from exc
            if servo_id not in {1, 2, 3, 4, 5, 10}:
                raise RuntimeError(f'{label}: l_shape_push_servo_order only supports 1..5 and 10, got {servo_id}')
            if servo_id in seen:
                continue
            seen.add(servo_id)
            result.append(servo_id)
        return result

    def publish_l_shape_push_pose(self, row: Sequence[int], label: str) -> None:
        order = self.parse_servo_order(self.get_parameter('l_shape_push_servo_order').value, label)
        if not order:
            self.publish_servo_row(row)
            return
        row_by_servo = {
            1: row[2],
            2: row[3],
            3: row[4],
            4: row[5],
            5: row[6],
            10: row[7],
        }
        duration = max(0.05, float(row[1]) / 1000.0)
        self.get_logger().info(
            f'{label}: l-shape pose servo order={order} duration_each={duration:.3f}s'
        )
        for servo_id in order:
            self.publish_single_servo_position(
                servo_id,
                float(row_by_servo[servo_id]),
                duration,
                f'{label} l-shape servo {servo_id}',
            )

    def l_shape_push_pose_description(self) -> str:
        pose_text = str(self.get_parameter('l_shape_push_pose').value).strip()
        if pose_text:
            return f'custom[{pose_text}]'
        return (
            f"{self.get_parameter('l_shape_push_pose_action').value}"
            f":step{int(self.get_parameter('l_shape_push_pose_step').value)}"
            f":wrist{int(self.get_parameter('l_shape_push_wrist_servo_index').value)}="
            f"{int(float(self.get_parameter('l_shape_push_wrist_position').value))}"
        )

    @staticmethod
    def format_action_row(row: Sequence[int]) -> str:
        if len(row) < 8:
            return repr(row)
        return (
            f't={int(row[1])}ms '
            f's1={int(row[2])} s2={int(row[3])} s3={int(row[4])} '
            f's4={int(row[5])} s5={int(row[6])} gripper={int(row[7])}'
        )

    def run_blind_linear(self, label: str, distance: float, speed: float, max_seconds: float) -> None:
        duration = min(max_seconds, distance / speed)
        signed_speed = float(self.get_parameter('linear_sign').value) * speed
        twist = Twist()
        twist.linear.x = math.copysign(abs(signed_speed), distance)
        self.stop_robot()
        self.get_logger().info(
            f'blind linear {label}: distance={distance:.3f}m '
            f'speed={twist.linear.x:.3f}m/s duration={duration:.3f}s'
        )
        deadline = time.monotonic() + duration
        try:
            while rclpy.ok() and not self.shutdown_requested and time.monotonic() < deadline:
                self.cmd_pub.publish(twist)
                self.last_twist = twist
                time.sleep(0.03)
        finally:
            self.stop_robot()
        if self.shutdown_requested:
            raise RuntimeError('stop requested during l-shape push')

    def publish_gripper_position(self, label: str, position: float, duration: float) -> None:
        if not self.use_arm:
            self.get_logger().warn(f'use_arm=false: skip gripper position for {label}')
            return
        if self.dry_run:
            self.get_logger().info(f'dry-run gripper {label}: position={position:.0f} duration={duration:.2f}s')
            return
        if ServosPosition is None:
            raise RuntimeError('servo_controller_msgs is not available')
        if self.arm_controller is None:
            raise RuntimeError('arm controller is not initialized')
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = max(0.05, float(duration))
        msg.position = [
            self.make_servo_position(int(self.get_parameter('gripper_servo_id').value), position)
        ]
        self.stop_robot()
        self.get_logger().info(f'gripper {label}: position={position:.0f} duration={msg.duration:.2f}s')
        self.arm_controller.servo_controller_pub.publish(msg)
        time.sleep(msg.duration)

    def grasp_succeeded(self, label: str) -> bool:
        if not bool(self.get_parameter('grasp_check_enabled').value):
            self.get_logger().info(f'{label}: grasp check disabled; assuming success')
            return True

        gripper_id = int(self.get_parameter('gripper_servo_id').value)
        empty_close = int(self.get_parameter('gripper_empty_close_position').value)
        min_gap = max(0, int(self.get_parameter('gripper_grasp_min_gap').value))
        delay = max(0.0, float(self.get_parameter('gripper_check_delay').value))
        timeout = max(0.1, float(self.get_parameter('gripper_feedback_timeout').value))
        if delay > 0.0:
            time.sleep(delay)

        sample_after = time.monotonic()
        deadline = sample_after + timeout
        last_log = 0.0
        while rclpy.ok() and not self.shutdown_requested and time.monotonic() < deadline:
            with self.servo_state_lock:
                position = self.latest_servo_positions.get(gripper_id)
                stamp = self.last_servo_state_time
            now = time.monotonic()
            if position is not None and stamp >= sample_after:
                gap = empty_close - position
                success = gap >= min_gap
                result = 'success' if success else 'empty'
                self.get_logger().info(
                    f'{label}: grasp check {result}: gripper_id={gripper_id}, '
                    f'position={position}, empty_close={empty_close}, gap={gap}, min_gap={min_gap}'
                )
                return success
            if now - last_log > 0.5:
                self.get_logger().info(
                    f'{label}: waiting gripper feedback id={gripper_id}; '
                    f'latest_position={position}, age={now - stamp if stamp > 0 else float("inf"):.2f}s'
                )
                last_log = now
            time.sleep(0.05)

        self.get_logger().warn(
            f'{label}: no fresh gripper feedback within {timeout:.1f}s; assuming success to avoid unsafe retry'
        )
        return True

    def open_gripper_for_approach(self, label: str) -> None:
        if not self.use_arm:
            self.get_logger().warn(f'use_arm=false: skip opening gripper for {label}')
            return
        if self.dry_run:
            self.get_logger().info(f'dry-run open gripper for {label}')
            return
        if ServosPosition is None:
            raise RuntimeError('servo_controller_msgs is not available')
        if self.arm_controller is None:
            raise RuntimeError('arm controller is not initialized')

        gripper_id = int(self.get_parameter('gripper_servo_id').value)
        open_position = float(self.get_parameter('gripper_open_position').value)
        duration = max(0.05, float(self.get_parameter('gripper_open_duration').value))
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = duration
        msg.position = [self.make_servo_position(gripper_id, open_position)]
        self.stop_robot()
        self.get_logger().info(
            f'open gripper before approach {label}: gripper_id={gripper_id}, '
            f'position={open_position:.0f}, duration={duration:.2f}s'
        )
        self.arm_controller.servo_controller_pub.publish(msg)
        time.sleep(duration)

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
        area_tolerance: Optional[float] = None,
        target_depth: Optional[float] = None,
        depth_tolerance: Optional[float] = None,
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
        time_scale = self.pregrasp_time_scale()
        min_step = self.pregrasp_min_step_seconds()
        settle_seconds = self.pregrasp_settle_seconds()
        post_step_seconds = self.pregrasp_post_step_seconds()
        self.get_logger().info(
            f'run visual-servo action steps {action_name} {list(step_indices)} for {label}: '
            f'desired_cx={desired_center:.3f}, target_area={target_area:.3f}, '
            f'target_depth={self.format_optional(self.valid_optional_depth(target_depth))}, '
            f'period={self.visual_servo_period_seconds():.3f}s, '
            f'time_scale={time_scale:.2f}, min_step={min_step:.2f}s, '
            f'settle={settle_seconds:.2f}s, post_step={post_step_seconds:.2f}s'
        )
        for row_index, row in enumerate(selected):
            if self.shutdown_requested:
                raise RuntimeError('stop requested during visual-servo action steps')
            step_label = f'{label} step {int(row[0])}'
            if settle_seconds > 1e-6:
                self.track_target_for_duration(
                    names=names,
                    duration=settle_seconds,
                    desired_center=desired_center,
                    target_area=target_area,
                    center_tolerance=center_tolerance,
                    area_tolerance=area_tolerance,
                    target_depth=target_depth,
                    depth_tolerance=depth_tolerance,
                    label=f'{step_label} settle-before',
                )
            duration = self.pregrasp_tracking_duration(row)
            self.publish_servo_row(row, wait=False, duration_scale=time_scale)
            self.track_target_for_duration(
                names=names,
                duration=duration,
                desired_center=desired_center,
                target_area=target_area,
                center_tolerance=center_tolerance,
                area_tolerance=area_tolerance,
                target_depth=target_depth,
                depth_tolerance=depth_tolerance,
                label=step_label,
            )
            if post_step_seconds > 1e-6 and row_index < len(selected) - 1:
                self.track_target_for_duration(
                    names=names,
                    duration=post_step_seconds,
                    desired_center=desired_center,
                    target_area=target_area,
                    center_tolerance=center_tolerance,
                    area_tolerance=area_tolerance,
                    target_depth=target_depth,
                    depth_tolerance=depth_tolerance,
                    label=f'{step_label} settle-after',
                )
        self.stop_robot()

    def pregrasp_tracking_duration(self, row: Sequence[int]) -> float:
        base_duration = max(0.05, float(row[1]) / 1000.0)
        return max(base_duration * self.pregrasp_time_scale(), self.pregrasp_min_step_seconds())

    def pregrasp_time_scale(self) -> float:
        return max(0.1, float(self.get_parameter('pick_pregrasp_time_scale').value))

    def pregrasp_min_step_seconds(self) -> float:
        return max(0.0, float(self.get_parameter('pick_pregrasp_min_step_seconds').value))

    def pregrasp_settle_seconds(self) -> float:
        return max(0.0, float(self.get_parameter('pick_pregrasp_settle_seconds').value))

    def pregrasp_post_step_seconds(self) -> float:
        return max(0.0, float(self.get_parameter('pick_pregrasp_post_step_seconds').value))

    def track_target_for_duration(
        self,
        names: Iterable[str],
        duration: float,
        desired_center: float,
        target_area: float,
        center_tolerance: float,
        label: str,
        area_tolerance: Optional[float] = None,
        target_depth: Optional[float] = None,
        depth_tolerance: Optional[float] = None,
    ) -> None:
        deadline = time.monotonic() + duration
        period = self.visual_servo_period_seconds()
        last_log = 0.0
        last_control_seq = -1
        while rclpy.ok() and not self.shutdown_requested and time.monotonic() < deadline:
            det = self.best_detection(names)
            now = time.monotonic()
            if det is None:
                self.stop_robot()
                if now - last_log > 0.5:
                    self.get_logger().info(f'visual-servo {label}: target temporarily hidden')
                    last_log = now
                time.sleep(period)
                continue

            if self.should_wait_for_fresh_detection(det, last_control_seq):
                self.stop_robot()
                time.sleep(period)
                continue
            last_control_seq = det.seq
            area_tol = 0.0 if area_tolerance is None else float(area_tolerance)
            distance = self.distance_estimate_for_detection(
                det,
                target_area=target_area,
                area_tolerance=area_tol,
                target_depth=target_depth,
                depth_tolerance=depth_tolerance,
            )
            if bool(self.get_parameter('use_robot_frame_distance').value) and distance.source != 'robot_frame':
                self.stop_robot()
                if now - last_log > 0.45:
                    self.log_alignment_state(
                        label=label,
                        det=det,
                        distance=distance,
                        center_error=det.cx_ratio - desired_center,
                        center_tolerance=center_tolerance,
                        twist=None,
                        prefix='visual-servo waiting robot-frame pose',
                    )
                    last_log = now
                time.sleep(period)
                continue
            center_error = (
                distance.lateral_error
                if distance.lateral_error is not None
                else det.cx_ratio - desired_center
            )
            active_center_tol = (
                distance.lateral_tolerance
                if distance.lateral_tolerance is not None
                else center_tolerance
            )
            if area_tolerance is not None and abs(center_error) <= active_center_tol and distance.aligned:
                self.stop_robot()
                if now - last_log > 0.45:
                    self.log_alignment_state(
                        label=label,
                        det=det,
                        distance=distance,
                        center_error=center_error,
                        center_tolerance=active_center_tol,
                        twist=None,
                        prefix='visual-servo holding',
                    )
                    last_log = now
                time.sleep(period)
                continue
            twist = self.compute_visual_servo_twist(distance, center_error, active_center_tol)
            if now - last_log > 0.45:
                self.log_alignment_state(
                    label=label,
                    det=det,
                    distance=distance,
                    center_error=center_error,
                    center_tolerance=active_center_tol,
                    twist=twist,
                    prefix='visual-servo',
                )
                last_log = now
            self.publish_control_pulse(twist, period)

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

    def publish_servo_row(self, row: Sequence[int], wait: bool = True, duration_scale: float = 1.0) -> None:
        if len(row) < 8:
            raise RuntimeError(f'invalid action row: {row!r}')
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = max(0.05, float(row[1]) / 1000.0 * max(0.1, float(duration_scale)))
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

    def publish_single_servo_position(self, servo_id: int, position: float, duration: float, label: str) -> None:
        if ServosPosition is None:
            raise RuntimeError('servo_controller_msgs is not available')
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = max(0.05, float(duration))
        msg.position = [self.make_servo_position(servo_id, position)]
        if self.arm_controller is None:
            raise RuntimeError('arm controller is not initialized')
        self.get_logger().info(f'{label}: servo={servo_id} position={position:.0f} duration={msg.duration:.3f}s')
        self.arm_controller.servo_controller_pub.publish(msg)
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

    def release_payload(self, label: str) -> None:
        release_action = str(self.get_parameter('release_action').value).strip()
        if release_action:
            self.run_action_group(release_action, label)
            return
        self.open_gripper_for_release(label)

    def open_gripper_for_release(self, label: str) -> None:
        self.open_gripper_to_position(
            position=float(self.get_parameter('release_gripper_position').value),
            duration=max(0.05, float(self.get_parameter('release_gripper_duration').value)),
            settle=max(0.0, float(self.get_parameter('release_settle_seconds').value)),
            label=label,
        )

    def open_gripper_to_position(self, position: float, duration: float, settle: float, label: str) -> None:
        if not self.use_arm:
            self.get_logger().warn(f'use_arm=false: skip {label} gripper command')
            return
        if self.dry_run:
            self.get_logger().info(
                f'dry-run {label}: gripper id={int(self.get_parameter("gripper_servo_id").value)} '
                f'position={position:.0f}, duration={duration:.2f}s'
            )
            return
        if ServosPosition is None:
            raise RuntimeError('servo_controller_msgs is not available')
        if self.arm_controller is None:
            raise RuntimeError('arm controller is not initialized')

        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = duration
        msg.position = [self.make_servo_position(int(self.get_parameter('gripper_servo_id').value), position)]
        self.stop_robot()
        self.get_logger().info(f'{label}: gripper position={position:.0f}, duration={duration:.2f}s')
        self.arm_controller.servo_controller_pub.publish(msg)
        time.sleep(duration + settle)

    def drive_body_x(self, distance_m: float, speed_mps: float, label: str) -> None:
        distance = float(distance_m)
        if abs(distance) < 1e-6:
            self.get_logger().info(f'{label}: post-pick advance distance is zero; skip')
            return
        speed = abs(float(speed_mps))
        if speed < 1e-4:
            raise RuntimeError(f'{label}: post_pick_advance_speed must be positive')
        duration = abs(distance) / speed
        if self.dry_run:
            self.get_logger().info(f'dry-run {label}: drive x={distance:.3f}m speed={speed:.3f}m/s duration={duration:.2f}s')
            return
        if self.nav_mode == 'odom' and bool(self.get_parameter('post_pick_advance_use_odom').value):
            self.drive_body_x_odom(distance, speed, label)
            return

        twist = Twist()
        twist.linear.x = math.copysign(speed, distance)
        self.get_logger().info(f'{label}: drive x={distance:.3f}m speed={twist.linear.x:.3f}m/s duration={duration:.2f}s')
        deadline = time.monotonic() + duration
        try:
            while rclpy.ok() and not self.shutdown_requested and time.monotonic() < deadline:
                self.cmd_pub.publish(twist)
                time.sleep(0.05)
        finally:
            self.stop_robot()
        if self.shutdown_requested:
            raise RuntimeError('stop requested during post-pick advance')

    def drive_body_x_odom(self, distance_m: float, speed_mps: float, label: str) -> None:
        start = self.current_odom_pose()
        target_x = start.x + float(distance_m) * math.cos(start.yaw)
        target_y = start.y + float(distance_m) * math.sin(start.yaw)
        self.get_logger().info(
            f'{label}: odom body-x advance from x={start.x:.3f}, y={start.y:.3f}, yaw={start.yaw:.3f} '
            f'to x={target_x:.3f}, y={target_y:.3f}, yaw={start.yaw:.3f}'
        )
        self.navigate_to_odom_pose(
            target_x,
            target_y,
            start.yaw,
            label,
            max_linear_speed=abs(float(speed_mps)),
        )

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

    def publish_control_pulse(self, twist: Twist, period: float) -> None:
        if not rclpy.ok():
            return
        command_seconds = min(self.visual_servo_command_seconds(), max(0.01, period))
        try:
            self.cmd_pub.publish(twist)
            self.last_twist = twist
            time.sleep(command_seconds)
            zero = Twist()
            for _ in range(2):
                self.cmd_pub.publish(zero)
                time.sleep(0.01)
            self.last_twist = zero
            remaining = max(0.0, period - command_seconds - 0.02)
            if remaining > 1e-6:
                time.sleep(remaining)
        except Exception as exc:
            self.get_logger().debug(f'ignored control pulse publish after shutdown: {exc}')

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

    def visual_servo_period_seconds(self) -> float:
        configured = self.clamp(float(self.get_parameter('visual_servo_period').value), 0.001, 5.0)
        if not bool(self.get_parameter('adaptive_servo_timing').value):
            return configured
        with self.detection_lock:
            ema = self.detection_period_ema
        if ema <= 0.0:
            return configured
        min_period = self.clamp(float(self.get_parameter('visual_servo_min_period').value), 0.001, 5.0)
        max_period = self.clamp(float(self.get_parameter('visual_servo_max_period').value), min_period, 5.0)
        scale = self.clamp(float(self.get_parameter('visual_servo_period_scale').value), 0.1, 5.0)
        return self.clamp(ema * scale, min_period, max_period)

    def visual_servo_command_seconds(self) -> float:
        return self.clamp(float(self.get_parameter('visual_servo_command_seconds').value), 0.001, 5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompetitionPickPlace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as exc:
        message = str(exc)
        if rclpy.ok() or 'context is not valid' not in message:
            raise
    finally:
        node.shutdown_requested = True
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
