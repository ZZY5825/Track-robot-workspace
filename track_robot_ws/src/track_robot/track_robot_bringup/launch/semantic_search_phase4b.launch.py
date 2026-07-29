"""One-command Phase 0-4B semantic navigation bringup.

The default is SEMANTIC_SHADOW with no base driver and no motion servers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package),
            'launch',
            launch_file,
        ])),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    phase4a = _include(
        'track_robot_bringup',
        'semantic_search_phase4a.launch.py',
        {
            'start_rviz': 'false',
            'configure_network': LaunchConfiguration('configure_network'),
            'network_interface': LaunchConfiguration('network_interface'),
            'host_ip': LaunchConfiguration('host_ip'),
            'host_cidr': LaunchConfiguration('host_cidr'),
            'driver_start_delay': LaunchConfiguration('driver_start_delay'),
            'extrinsic_mode': LaunchConfiguration('extrinsic_mode'),
            'extrinsic_file': LaunchConfiguration('extrinsic_file'),
            'lidar_config_path': LaunchConfiguration('lidar_config_path'),
            'yolo_runtime_path': LaunchConfiguration('yolo_runtime_path'),
            'clip_runtime_path': LaunchConfiguration('clip_runtime_path'),
            'yolo_checkpoint': LaunchConfiguration('yolo_checkpoint'),
            'clip_checkpoint': LaunchConfiguration('clip_checkpoint'),
            'dino_repo_path': LaunchConfiguration('dino_repo_path'),
            'dino_checkpoint': LaunchConfiguration('dino_checkpoint'),
        },
    )
    platform = _include(
        'track_robot_bringup',
        'semantic_search_platform.launch.py',
        {
            'start_base': LaunchConfiguration('start_base'),
            'start_imu': 'false',
        },
    )
    navigation = _include(
        'track_robot_navigation',
        'phase4b_navigation.launch.py',
        {
            'runtime_mode': LaunchConfiguration('runtime_mode'),
            'enable_semantic_execution':
                LaunchConfiguration('enable_semantic_execution'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            # Phase 4A already owns this independent safety map instance.
            'start_obstacle_map': 'false',
        },
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='semantic_search_phase4b_rviz',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('track_robot_bringup'),
                'rviz',
                'semantic_search_phase4b.rviz',
            ]),
        ],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'runtime_mode', default_value='SEMANTIC_SHADOW'),
        DeclareLaunchArgument(
            'enable_semantic_execution', default_value='false'),
        DeclareLaunchArgument('start_base', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument('driver_start_delay', default_value='1.0'),
        DeclareLaunchArgument('extrinsic_mode', default_value='prototype'),
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument(
            'lidar_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_sensor_bringup'),
                'config',
                'rslidar_track_robot.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'yolo_runtime_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/r0c_runtime/python')),
        DeclareLaunchArgument(
            'clip_runtime_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/phase1_runtime/python')),
        DeclareLaunchArgument(
            'yolo_checkpoint',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/r0c/yolov8s-worldv2.pt')),
        DeclareLaunchArgument(
            'clip_checkpoint',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/phase1/ViT-B-32.pt')),
        DeclareLaunchArgument(
            'dino_repo_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'src/track_robot_core/third_party/dinov3_py38')),
        DeclareLaunchArgument(
            'dino_checkpoint',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/dinov3_vits16plus_pretrain_lvd1689m.pth')),
        phase4a,
        platform,
        navigation,
        rviz,
    ])
