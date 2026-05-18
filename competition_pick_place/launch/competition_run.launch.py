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
    yolo_conf = LaunchConfiguration('yolo_conf')
    waypoints_yaml = LaunchConfiguration('waypoints_yaml')
    search_timeout = LaunchConfiguration('search_timeout')
    align_timeout = LaunchConfiguration('align_timeout')
    desired_center_x_ratio = LaunchConfiguration('desired_center_x_ratio')
    center_tolerance_ratio = LaunchConfiguration('center_tolerance_ratio')
    pick_target_area_ratio = LaunchConfiguration('pick_target_area_ratio')
    area_tolerance_ratio = LaunchConfiguration('area_tolerance_ratio')
    stable_frames = LaunchConfiguration('stable_frames')
    angular_k = LaunchConfiguration('angular_k')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    search_angular_speed = LaunchConfiguration('search_angular_speed')

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
            {'classes': ['red', 'green', 'blue']},
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
                'exit_on_done': ParameterValue(exit_on_done, value_type=bool),
                'stop_after_pick': ParameterValue(stop_after_pick, value_type=bool),
                'use_nav': ParameterValue(use_nav, value_type=bool),
                'use_arm': ParameterValue(use_arm, value_type=bool),
                'waypoints_yaml': waypoints_yaml,
                'search_timeout': ParameterValue(search_timeout, value_type=float),
                'align_timeout': ParameterValue(align_timeout, value_type=float),
                'desired_center_x_ratio': ParameterValue(desired_center_x_ratio, value_type=float),
                'center_tolerance_ratio': ParameterValue(center_tolerance_ratio, value_type=float),
                'pick_target_area_ratio': ParameterValue(pick_target_area_ratio, value_type=float),
                'area_tolerance_ratio': ParameterValue(area_tolerance_ratio, value_type=float),
                'stable_frames': ParameterValue(stable_frames, value_type=int),
                'angular_k': ParameterValue(angular_k, value_type=float),
                'max_linear_speed': ParameterValue(max_linear_speed, value_type=float),
                'max_angular_speed': ParameterValue(max_angular_speed, value_type=float),
                'search_angular_speed': ParameterValue(search_angular_speed, value_type=float),
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
        DeclareLaunchArgument('target_class', default_value='red'),
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
        DeclareLaunchArgument('yolo_model', default_value='competition_blocks'),
        DeclareLaunchArgument('yolo_conf', default_value='0.70'),
        DeclareLaunchArgument('waypoints_yaml', default_value=default_waypoints),
        DeclareLaunchArgument('search_timeout', default_value='18.0'),
        DeclareLaunchArgument('align_timeout', default_value='24.0'),
        DeclareLaunchArgument('desired_center_x_ratio', default_value='0.50'),
        DeclareLaunchArgument('center_tolerance_ratio', default_value='0.055'),
        DeclareLaunchArgument('pick_target_area_ratio', default_value='0.095'),
        DeclareLaunchArgument('area_tolerance_ratio', default_value='0.018'),
        DeclareLaunchArgument('stable_frames', default_value='5'),
        DeclareLaunchArgument('angular_k', default_value='1.35'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.10'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.45'),
        DeclareLaunchArgument('search_angular_speed', default_value='0.22'),
        OpaqueFunction(function=launch_setup),
    ])
