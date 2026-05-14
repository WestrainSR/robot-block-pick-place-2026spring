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
    target_aliases = LaunchConfiguration('target_aliases')
    place_class = LaunchConfiguration('place_class')
    dry_run = LaunchConfiguration('dry_run')
    exit_on_done = LaunchConfiguration('exit_on_done')
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
                'target_aliases': target_aliases,
                'place_class': place_class,
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'exit_on_done': ParameterValue(exit_on_done, value_type=bool),
                'use_nav': ParameterValue(use_nav, value_type=bool),
                'use_arm': ParameterValue(use_arm, value_type=bool),
                'waypoints_yaml': waypoints_yaml,
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
        DeclareLaunchArgument('target_aliases', default_value=''),
        DeclareLaunchArgument('place_class', default_value=''),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('exit_on_done', default_value='false'),
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
        OpaqueFunction(function=launch_setup),
    ])
