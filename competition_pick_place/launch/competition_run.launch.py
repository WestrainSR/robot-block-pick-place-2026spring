import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _package_path(package_name):
    if os.environ.get('need_compile', 'False') == 'True':
        return get_package_share_directory(package_name)
    return os.path.join('/home/ubuntu/ros2_ws/src', package_name)


def _csv_list(value, fallback):
    items = [part.strip() for part in str(value or '').split(',') if part.strip()]
    return items or list(fallback)


def launch_setup(context):
    competition_path = _package_path('competition_pick_place')
    navigation_path = _package_path('navigation')
    slam_path = _package_path('slam')
    peripherals_path = _package_path('peripherals')

    target_class = LaunchConfiguration('target_class')
    target_sequence = LaunchConfiguration('target_sequence')
    target_aliases = LaunchConfiguration('target_aliases')
    place_class = LaunchConfiguration('place_class')
    dry_run = LaunchConfiguration('dry_run')
    exit_on_done = LaunchConfiguration('exit_on_done')
    stop_after_pick = LaunchConfiguration('stop_after_pick')
    use_nav = LaunchConfiguration('use_nav')
    use_arm = LaunchConfiguration('use_arm')
    map_name = LaunchConfiguration('map_name')
    start_navigation = LaunchConfiguration('start_navigation')
    start_base = LaunchConfiguration('start_base')
    start_camera = LaunchConfiguration('start_camera')
    start_yolo = LaunchConfiguration('start_yolo')
    yolo_model = LaunchConfiguration('yolo_model')
    yolo_classes = _csv_list(
        LaunchConfiguration('yolo_classes').perform(context),
        ['gray', 'yellow', 'grass', 'blue'],
    )
    yolo_conf = LaunchConfiguration('yolo_conf')
    min_score = LaunchConfiguration('min_score')
    waypoints_yaml = LaunchConfiguration('waypoints_yaml')
    init_action = LaunchConfiguration('init_action')
    pick_action = LaunchConfiguration('pick_action')
    place_action = LaunchConfiguration('place_action')
    search_timeout = LaunchConfiguration('search_timeout')
    align_timeout = LaunchConfiguration('align_timeout')
    wait_for_detection_stream = LaunchConfiguration('wait_for_detection_stream')
    detection_stream_timeout = LaunchConfiguration('detection_stream_timeout')
    detection_ready_min_messages = LaunchConfiguration('detection_ready_min_messages')
    wait_for_target_before_search = LaunchConfiguration('wait_for_target_before_search')
    allow_search_rotation = LaunchConfiguration('allow_search_rotation')
    use_depth_distance = LaunchConfiguration('use_depth_distance')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    use_robot_frame_distance = LaunchConfiguration('use_robot_frame_distance')
    camera_tilt_deg = LaunchConfiguration('camera_tilt_deg')
    camera_height_m = LaunchConfiguration('camera_height_m')
    camera_offset_x_m = LaunchConfiguration('camera_offset_x_m')
    depth_roi_pixels = LaunchConfiguration('depth_roi_pixels')
    depth_stale_seconds = LaunchConfiguration('depth_stale_seconds')
    depth_unit_scale = LaunchConfiguration('depth_unit_scale')
    depth_roi_scale = LaunchConfiguration('depth_roi_scale')
    depth_sample_grid = LaunchConfiguration('depth_sample_grid')
    depth_min_valid_samples = LaunchConfiguration('depth_min_valid_samples')
    depth_min_m = LaunchConfiguration('depth_min_m')
    depth_max_m = LaunchConfiguration('depth_max_m')
    pick_target_depth_m = LaunchConfiguration('pick_target_depth_m')
    pick_target_robot_x_m = LaunchConfiguration('pick_target_robot_x_m')
    pick_target_robot_y_m = LaunchConfiguration('pick_target_robot_y_m')
    pick_robot_x_tolerance_m = LaunchConfiguration('pick_robot_x_tolerance_m')
    pick_robot_y_tolerance_m = LaunchConfiguration('pick_robot_y_tolerance_m')
    place_target_robot_x_m = LaunchConfiguration('place_target_robot_x_m')
    place_target_robot_y_m = LaunchConfiguration('place_target_robot_y_m')
    place_robot_x_tolerance_m = LaunchConfiguration('place_robot_x_tolerance_m')
    place_robot_y_tolerance_m = LaunchConfiguration('place_robot_y_tolerance_m')
    pick_depth_tolerance_m = LaunchConfiguration('pick_depth_tolerance_m')
    pick_preclose_target_depth_m = LaunchConfiguration('pick_preclose_target_depth_m')
    desired_center_x_ratio = LaunchConfiguration('desired_center_x_ratio')
    center_tolerance_ratio = LaunchConfiguration('center_tolerance_ratio')
    pick_target_area_ratio = LaunchConfiguration('pick_target_area_ratio')
    place_target_area_ratio = LaunchConfiguration('place_target_area_ratio')
    area_tolerance_ratio = LaunchConfiguration('area_tolerance_ratio')
    stable_frames = LaunchConfiguration('stable_frames')
    control_mode = LaunchConfiguration('control_mode')
    closed_loop_pick = LaunchConfiguration('closed_loop_pick')
    pick_visual_servo_timeout = LaunchConfiguration('pick_visual_servo_timeout')
    visual_servo_period = LaunchConfiguration('visual_servo_period')
    visual_servo_command_seconds = LaunchConfiguration('visual_servo_command_seconds')
    adaptive_servo_timing = LaunchConfiguration('adaptive_servo_timing')
    visual_servo_min_period = LaunchConfiguration('visual_servo_min_period')
    visual_servo_max_period = LaunchConfiguration('visual_servo_max_period')
    visual_servo_period_scale = LaunchConfiguration('visual_servo_period_scale')
    require_fresh_detection_for_control = LaunchConfiguration('require_fresh_detection_for_control')
    pick_pregrasp_visual_servo = LaunchConfiguration('pick_pregrasp_visual_servo')
    open_gripper_before_approach = LaunchConfiguration('open_gripper_before_approach')
    gripper_open_position = LaunchConfiguration('gripper_open_position')
    gripper_open_duration = LaunchConfiguration('gripper_open_duration')
    pick_pregrasp_time_scale = LaunchConfiguration('pick_pregrasp_time_scale')
    pick_pregrasp_min_step_seconds = LaunchConfiguration('pick_pregrasp_min_step_seconds')
    pick_pregrasp_settle_seconds = LaunchConfiguration('pick_pregrasp_settle_seconds')
    pick_pregrasp_post_step_seconds = LaunchConfiguration('pick_pregrasp_post_step_seconds')
    pick_preclose_required = LaunchConfiguration('pick_preclose_required')
    pick_preclose_fail_on_timeout = LaunchConfiguration('pick_preclose_fail_on_timeout')
    pick_preclose_center_x_ratio = LaunchConfiguration('pick_preclose_center_x_ratio')
    pick_preclose_target_area_ratio = LaunchConfiguration('pick_preclose_target_area_ratio')
    pick_preclose_center_tolerance_ratio = LaunchConfiguration('pick_preclose_center_tolerance_ratio')
    pick_preclose_area_tolerance_ratio = LaunchConfiguration('pick_preclose_area_tolerance_ratio')
    pick_preclose_stable_frames = LaunchConfiguration('pick_preclose_stable_frames')
    pick_pregrasp_steps = LaunchConfiguration('pick_pregrasp_steps')
    pick_close_steps = LaunchConfiguration('pick_close_steps')
    pick_lift_steps = LaunchConfiguration('pick_lift_steps')
    place_steps = LaunchConfiguration('place_steps')
    hold_after_place = LaunchConfiguration('hold_after_place')
    hold_place_steps = LaunchConfiguration('hold_place_steps')
    pick_retry_attempts = LaunchConfiguration('pick_retry_attempts')
    grasp_check_enabled = LaunchConfiguration('grasp_check_enabled')
    gripper_state_topic = LaunchConfiguration('gripper_state_topic')
    gripper_servo_id = LaunchConfiguration('gripper_servo_id')
    gripper_empty_close_position = LaunchConfiguration('gripper_empty_close_position')
    gripper_grasp_min_gap = LaunchConfiguration('gripper_grasp_min_gap')
    gripper_check_delay = LaunchConfiguration('gripper_check_delay')
    gripper_feedback_timeout = LaunchConfiguration('gripper_feedback_timeout')
    angular_k = LaunchConfiguration('angular_k')
    angular_sign = LaunchConfiguration('angular_sign')
    linear_sign = LaunchConfiguration('linear_sign')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    search_angular_speed = LaunchConfiguration('search_angular_speed')
    mpc_horizon = LaunchConfiguration('mpc_horizon')
    mpc_dt = LaunchConfiguration('mpc_dt')
    mpc_center_response = LaunchConfiguration('mpc_center_response')
    mpc_area_response = LaunchConfiguration('mpc_area_response')
    mpc_center_weight = LaunchConfiguration('mpc_center_weight')
    mpc_area_weight = LaunchConfiguration('mpc_area_weight')
    mpc_velocity_weight = LaunchConfiguration('mpc_velocity_weight')
    mpc_delta_weight = LaunchConfiguration('mpc_delta_weight')
    mpc_terminal_weight = LaunchConfiguration('mpc_terminal_weight')
    mpc_center_gate_ratio = LaunchConfiguration('mpc_center_gate_ratio')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_path, 'launch/navigation.launch.py')),
        condition=IfCondition(start_navigation),
        launch_arguments={'map': map_name}.items(),
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_path, 'launch/include/robot.launch.py')),
        condition=IfCondition(PythonExpression([
            "'", start_base, "' == 'true' and '", start_navigation, "' != 'true'",
        ])),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(peripherals_path, 'launch/depth_camera.launch.py')),
        condition=IfCondition(start_camera),
    )

    yolo_node = Node(
        package='yolov11_detect',
        executable='yolov11_node',
        name='yolo_node',
        output='screen',
        condition=IfCondition(start_yolo),
        parameters=[
            {'classes': yolo_classes},
            {'model': yolo_model, 'conf': ParameterValue(yolo_conf, value_type=float), 'start': True},
        ],
    )

    competition_node = Node(
        package='competition_pick_place',
        executable='competition_node',
        output='screen',
        parameters=[
            {
                'target_class': target_class,
                'target_sequence': target_sequence,
                'target_aliases': target_aliases,
                'place_class': place_class,
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'min_score': ParameterValue(min_score, value_type=float),
                'exit_on_done': ParameterValue(exit_on_done, value_type=bool),
                'stop_after_pick': ParameterValue(stop_after_pick, value_type=bool),
                'use_nav': ParameterValue(use_nav, value_type=bool),
                'use_arm': ParameterValue(use_arm, value_type=bool),
                'waypoints_yaml': waypoints_yaml,
                'init_action': init_action,
                'pick_action': pick_action,
                'place_action': place_action,
                'search_timeout': ParameterValue(search_timeout, value_type=float),
                'align_timeout': ParameterValue(align_timeout, value_type=float),
                'wait_for_detection_stream': ParameterValue(wait_for_detection_stream, value_type=bool),
                'detection_stream_timeout': ParameterValue(detection_stream_timeout, value_type=float),
                'detection_ready_min_messages': ParameterValue(detection_ready_min_messages, value_type=int),
                'wait_for_target_before_search': ParameterValue(wait_for_target_before_search, value_type=bool),
                'allow_search_rotation': ParameterValue(allow_search_rotation, value_type=bool),
                'use_depth_distance': ParameterValue(use_depth_distance, value_type=bool),
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'use_robot_frame_distance': ParameterValue(use_robot_frame_distance, value_type=bool),
                'camera_tilt_deg': ParameterValue(camera_tilt_deg, value_type=float),
                'camera_height_m': ParameterValue(camera_height_m, value_type=float),
                'camera_offset_x_m': ParameterValue(camera_offset_x_m, value_type=float),
                'depth_roi_pixels': ParameterValue(depth_roi_pixels, value_type=int),
                'depth_stale_seconds': ParameterValue(depth_stale_seconds, value_type=float),
                'depth_unit_scale': ParameterValue(depth_unit_scale, value_type=float),
                'depth_roi_scale': ParameterValue(depth_roi_scale, value_type=float),
                'depth_sample_grid': ParameterValue(depth_sample_grid, value_type=int),
                'depth_min_valid_samples': ParameterValue(depth_min_valid_samples, value_type=int),
                'depth_min_m': ParameterValue(depth_min_m, value_type=float),
                'depth_max_m': ParameterValue(depth_max_m, value_type=float),
                'pick_target_depth_m': ParameterValue(pick_target_depth_m, value_type=float),
                'pick_target_robot_x_m': ParameterValue(pick_target_robot_x_m, value_type=float),
                'pick_target_robot_y_m': ParameterValue(pick_target_robot_y_m, value_type=float),
                'pick_robot_x_tolerance_m': ParameterValue(pick_robot_x_tolerance_m, value_type=float),
                'pick_robot_y_tolerance_m': ParameterValue(pick_robot_y_tolerance_m, value_type=float),
                'place_target_robot_x_m': ParameterValue(place_target_robot_x_m, value_type=float),
                'place_target_robot_y_m': ParameterValue(place_target_robot_y_m, value_type=float),
                'place_robot_x_tolerance_m': ParameterValue(place_robot_x_tolerance_m, value_type=float),
                'place_robot_y_tolerance_m': ParameterValue(place_robot_y_tolerance_m, value_type=float),
                'pick_depth_tolerance_m': ParameterValue(pick_depth_tolerance_m, value_type=float),
                'pick_preclose_target_depth_m': ParameterValue(pick_preclose_target_depth_m, value_type=float),
                'desired_center_x_ratio': ParameterValue(desired_center_x_ratio, value_type=float),
                'center_tolerance_ratio': ParameterValue(center_tolerance_ratio, value_type=float),
                'pick_target_area_ratio': ParameterValue(pick_target_area_ratio, value_type=float),
                'place_target_area_ratio': ParameterValue(place_target_area_ratio, value_type=float),
                'area_tolerance_ratio': ParameterValue(area_tolerance_ratio, value_type=float),
                'stable_frames': ParameterValue(stable_frames, value_type=int),
                'control_mode': control_mode,
                'closed_loop_pick': ParameterValue(closed_loop_pick, value_type=bool),
                'pick_visual_servo_timeout': ParameterValue(pick_visual_servo_timeout, value_type=float),
                'visual_servo_period': ParameterValue(visual_servo_period, value_type=float),
                'visual_servo_command_seconds': ParameterValue(visual_servo_command_seconds, value_type=float),
                'adaptive_servo_timing': ParameterValue(adaptive_servo_timing, value_type=bool),
                'visual_servo_min_period': ParameterValue(visual_servo_min_period, value_type=float),
                'visual_servo_max_period': ParameterValue(visual_servo_max_period, value_type=float),
                'visual_servo_period_scale': ParameterValue(visual_servo_period_scale, value_type=float),
                'require_fresh_detection_for_control': ParameterValue(require_fresh_detection_for_control, value_type=bool),
                'pick_pregrasp_visual_servo': ParameterValue(pick_pregrasp_visual_servo, value_type=bool),
                'open_gripper_before_approach': ParameterValue(open_gripper_before_approach, value_type=bool),
                'gripper_open_position': ParameterValue(gripper_open_position, value_type=int),
                'gripper_open_duration': ParameterValue(gripper_open_duration, value_type=float),
                'pick_pregrasp_time_scale': ParameterValue(pick_pregrasp_time_scale, value_type=float),
                'pick_pregrasp_min_step_seconds': ParameterValue(pick_pregrasp_min_step_seconds, value_type=float),
                'pick_pregrasp_settle_seconds': ParameterValue(pick_pregrasp_settle_seconds, value_type=float),
                'pick_pregrasp_post_step_seconds': ParameterValue(pick_pregrasp_post_step_seconds, value_type=float),
                'pick_preclose_required': ParameterValue(pick_preclose_required, value_type=bool),
                'pick_preclose_fail_on_timeout': ParameterValue(pick_preclose_fail_on_timeout, value_type=bool),
                'pick_preclose_center_x_ratio': ParameterValue(pick_preclose_center_x_ratio, value_type=float),
                'pick_preclose_target_area_ratio': ParameterValue(pick_preclose_target_area_ratio, value_type=float),
                'pick_preclose_center_tolerance_ratio': ParameterValue(pick_preclose_center_tolerance_ratio, value_type=float),
                'pick_preclose_area_tolerance_ratio': ParameterValue(pick_preclose_area_tolerance_ratio, value_type=float),
                'pick_preclose_stable_frames': ParameterValue(pick_preclose_stable_frames, value_type=int),
                'pick_pregrasp_steps': pick_pregrasp_steps,
                'pick_close_steps': pick_close_steps,
                'pick_lift_steps': pick_lift_steps,
                'place_steps': place_steps,
                'hold_after_place': ParameterValue(hold_after_place, value_type=bool),
                'hold_place_steps': hold_place_steps,
                'pick_retry_attempts': ParameterValue(pick_retry_attempts, value_type=int),
                'grasp_check_enabled': ParameterValue(grasp_check_enabled, value_type=bool),
                'gripper_state_topic': gripper_state_topic,
                'gripper_servo_id': ParameterValue(gripper_servo_id, value_type=int),
                'gripper_empty_close_position': ParameterValue(gripper_empty_close_position, value_type=int),
                'gripper_grasp_min_gap': ParameterValue(gripper_grasp_min_gap, value_type=int),
                'gripper_check_delay': ParameterValue(gripper_check_delay, value_type=float),
                'gripper_feedback_timeout': ParameterValue(gripper_feedback_timeout, value_type=float),
                'angular_k': ParameterValue(angular_k, value_type=float),
                'angular_sign': ParameterValue(angular_sign, value_type=float),
                'linear_sign': ParameterValue(linear_sign, value_type=float),
                'max_linear_speed': ParameterValue(max_linear_speed, value_type=float),
                'max_angular_speed': ParameterValue(max_angular_speed, value_type=float),
                'search_angular_speed': ParameterValue(search_angular_speed, value_type=float),
                'mpc_horizon': ParameterValue(mpc_horizon, value_type=int),
                'mpc_dt': ParameterValue(mpc_dt, value_type=float),
                'mpc_center_response': ParameterValue(mpc_center_response, value_type=float),
                'mpc_area_response': ParameterValue(mpc_area_response, value_type=float),
                'mpc_center_weight': ParameterValue(mpc_center_weight, value_type=float),
                'mpc_area_weight': ParameterValue(mpc_area_weight, value_type=float),
                'mpc_velocity_weight': ParameterValue(mpc_velocity_weight, value_type=float),
                'mpc_delta_weight': ParameterValue(mpc_delta_weight, value_type=float),
                'mpc_terminal_weight': ParameterValue(mpc_terminal_weight, value_type=float),
                'mpc_center_gate_ratio': ParameterValue(mpc_center_gate_ratio, value_type=float),
            },
        ],
    )

    return [
        navigation_launch,
        base_launch,
        camera_launch,
        yolo_node,
        competition_node,
    ]


def generate_launch_description():
    default_waypoints = os.path.join(
        _package_path('competition_pick_place'),
        'config',
        'competition_waypoints.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('target_class', default_value='grass'),
        DeclareLaunchArgument('target_sequence', default_value=''),
        DeclareLaunchArgument('target_aliases', default_value=''),
        DeclareLaunchArgument('place_class', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('exit_on_done', default_value='false'),
        DeclareLaunchArgument('stop_after_pick', default_value='false'),
        DeclareLaunchArgument('use_nav', default_value='true'),
        DeclareLaunchArgument('use_arm', default_value='true'),
        DeclareLaunchArgument('map_name', default_value='competition_map'),
        DeclareLaunchArgument('start_navigation', default_value='false'),
        DeclareLaunchArgument('start_base', default_value='false'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_yolo', default_value='false'),
        DeclareLaunchArgument('yolo_model', default_value='tongji'),
        DeclareLaunchArgument('yolo_classes', default_value='gray,yellow,grass,blue'),
        DeclareLaunchArgument('yolo_conf', default_value='0.20'),
        DeclareLaunchArgument('min_score', default_value='0.20'),
        DeclareLaunchArgument('waypoints_yaml', default_value=default_waypoints),
        DeclareLaunchArgument('init_action', default_value='navigation_pick_init'),
        DeclareLaunchArgument('pick_action', default_value='navigation_pick'),
        DeclareLaunchArgument('place_action', default_value='navigation_place'),
        DeclareLaunchArgument('search_timeout', default_value='18.0'),
        DeclareLaunchArgument('align_timeout', default_value='24.0'),
        DeclareLaunchArgument('wait_for_detection_stream', default_value='true'),
        DeclareLaunchArgument('detection_stream_timeout', default_value='20.0'),
        DeclareLaunchArgument('detection_ready_min_messages', default_value='1'),
        DeclareLaunchArgument('wait_for_target_before_search', default_value='true'),
        DeclareLaunchArgument('allow_search_rotation', default_value='false'),
        DeclareLaunchArgument('use_depth_distance', default_value='true'),
        DeclareLaunchArgument('depth_topic', default_value='/ascamera/camera_publisher/depth0/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/ascamera/camera_publisher/rgb0/camera_info'),
        DeclareLaunchArgument('use_robot_frame_distance', default_value='true'),
        DeclareLaunchArgument('camera_tilt_deg', default_value='45.0'),
        DeclareLaunchArgument('camera_height_m', default_value='0.22'),
        DeclareLaunchArgument('camera_offset_x_m', default_value='0.06'),
        DeclareLaunchArgument('depth_roi_pixels', default_value='15'),
        DeclareLaunchArgument('depth_stale_seconds', default_value='0.8'),
        DeclareLaunchArgument('depth_unit_scale', default_value='0.001'),
        DeclareLaunchArgument('depth_roi_scale', default_value='0.45'),
        DeclareLaunchArgument('depth_sample_grid', default_value='5'),
        DeclareLaunchArgument('depth_min_valid_samples', default_value='20'),
        DeclareLaunchArgument('depth_min_m', default_value='0.08'),
        DeclareLaunchArgument('depth_max_m', default_value='1.50'),
        DeclareLaunchArgument('pick_target_depth_m', default_value='0.32'),
        DeclareLaunchArgument('pick_target_robot_x_m', default_value='0.32'),
        DeclareLaunchArgument('pick_target_robot_y_m', default_value='0.0'),
        DeclareLaunchArgument('pick_robot_x_tolerance_m', default_value='0.025'),
        DeclareLaunchArgument('pick_robot_y_tolerance_m', default_value='0.025'),
        DeclareLaunchArgument('place_target_robot_x_m', default_value='0.145'),
        DeclareLaunchArgument('place_target_robot_y_m', default_value='0.0'),
        DeclareLaunchArgument('place_robot_x_tolerance_m', default_value='0.015'),
        DeclareLaunchArgument('place_robot_y_tolerance_m', default_value='0.015'),
        DeclareLaunchArgument('pick_depth_tolerance_m', default_value='0.025'),
        DeclareLaunchArgument('pick_preclose_target_depth_m', default_value='-1.0'),
        DeclareLaunchArgument('desired_center_x_ratio', default_value='0.50'),
        DeclareLaunchArgument('center_tolerance_ratio', default_value='0.055'),
        DeclareLaunchArgument('pick_target_area_ratio', default_value='0.043'),
        DeclareLaunchArgument('place_target_area_ratio', default_value='0.043'),
        DeclareLaunchArgument('area_tolerance_ratio', default_value='0.018'),
        DeclareLaunchArgument('stable_frames', default_value='5'),
        DeclareLaunchArgument('control_mode', default_value='p'),
        DeclareLaunchArgument('closed_loop_pick', default_value='false'),
        DeclareLaunchArgument('pick_visual_servo_timeout', default_value='10.0'),
        DeclareLaunchArgument('visual_servo_period', default_value='0.06'),
        DeclareLaunchArgument('visual_servo_command_seconds', default_value='0.05'),
        DeclareLaunchArgument('adaptive_servo_timing', default_value='true'),
        DeclareLaunchArgument('visual_servo_min_period', default_value='0.035'),
        DeclareLaunchArgument('visual_servo_max_period', default_value='0.16'),
        DeclareLaunchArgument('visual_servo_period_scale', default_value='1.05'),
        DeclareLaunchArgument('require_fresh_detection_for_control', default_value='false'),
        DeclareLaunchArgument('pick_pregrasp_visual_servo', default_value='true'),
        DeclareLaunchArgument('open_gripper_before_approach', default_value='true'),
        DeclareLaunchArgument('gripper_open_position', default_value='200'),
        DeclareLaunchArgument('gripper_open_duration', default_value='0.30'),
        DeclareLaunchArgument('pick_pregrasp_time_scale', default_value='1.0'),
        DeclareLaunchArgument('pick_pregrasp_min_step_seconds', default_value='0.0'),
        DeclareLaunchArgument('pick_pregrasp_settle_seconds', default_value='0.0'),
        DeclareLaunchArgument('pick_pregrasp_post_step_seconds', default_value='0.0'),
        DeclareLaunchArgument('pick_preclose_required', default_value='false'),
        DeclareLaunchArgument('pick_preclose_fail_on_timeout', default_value='false'),
        DeclareLaunchArgument('pick_preclose_center_x_ratio', default_value='0.50'),
        DeclareLaunchArgument('pick_preclose_target_area_ratio', default_value='-1.0'),
        DeclareLaunchArgument('pick_preclose_center_tolerance_ratio', default_value='-1.0'),
        DeclareLaunchArgument('pick_preclose_area_tolerance_ratio', default_value='-1.0'),
        DeclareLaunchArgument('pick_preclose_stable_frames', default_value='2'),
        DeclareLaunchArgument('pick_pregrasp_steps', default_value='1,2'),
        DeclareLaunchArgument('pick_close_steps', default_value='3,4'),
        DeclareLaunchArgument('pick_lift_steps', default_value='5,6'),
        DeclareLaunchArgument('place_steps', default_value=''),
        DeclareLaunchArgument('hold_after_place', default_value='true'),
        DeclareLaunchArgument('hold_place_steps', default_value='1,2'),
        DeclareLaunchArgument('pick_retry_attempts', default_value='3'),
        DeclareLaunchArgument('grasp_check_enabled', default_value='true'),
        DeclareLaunchArgument('gripper_state_topic', default_value='/controller_manager/servo_states'),
        DeclareLaunchArgument('gripper_servo_id', default_value='10'),
        DeclareLaunchArgument('gripper_empty_close_position', default_value='500'),
        DeclareLaunchArgument('gripper_grasp_min_gap', default_value='30'),
        DeclareLaunchArgument('gripper_check_delay', default_value='0.35'),
        DeclareLaunchArgument('gripper_feedback_timeout', default_value='2.0'),
        DeclareLaunchArgument('angular_k', default_value='1.35'),
        DeclareLaunchArgument('angular_sign', default_value='-1.0'),
        DeclareLaunchArgument('linear_sign', default_value='1.0'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.10'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.45'),
        DeclareLaunchArgument('search_angular_speed', default_value='0.22'),
        DeclareLaunchArgument('mpc_horizon', default_value='10'),
        DeclareLaunchArgument('mpc_dt', default_value='0.06'),
        DeclareLaunchArgument('mpc_center_response', default_value='1.05'),
        DeclareLaunchArgument('mpc_area_response', default_value='0.24'),
        DeclareLaunchArgument('mpc_center_weight', default_value='8.0'),
        DeclareLaunchArgument('mpc_area_weight', default_value='26.0'),
        DeclareLaunchArgument('mpc_velocity_weight', default_value='0.08'),
        DeclareLaunchArgument('mpc_delta_weight', default_value='0.16'),
        DeclareLaunchArgument('mpc_terminal_weight', default_value='2.2'),
        DeclareLaunchArgument('mpc_center_gate_ratio', default_value='0.12'),
        OpaqueFunction(function=launch_setup),
    ])
