"""Stationary Phase 0-4A semantic-search integration test.

This launch intentionally has no base, IMU, odometry, controller, navigation
action, or motion-supervisor interface.  It produces inspection products and
human-readable approach advice only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
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


def _launch_runtime(context):
    search_config = PathJoinSubstitution([
        FindPackageShare('track_robot_semantic_search'),
        'config',
        'semantic_search_phase4a.yaml',
    ])
    memory_config = PathJoinSubstitution([
        FindPackageShare('track_robot_semantic_memory'),
        'config',
        'phase4a_test.yaml',
    ])
    obstacle_config = PathJoinSubstitution([
        FindPackageShare('track_robot_safety'),
        'config',
        'local_obstacle_map.yaml',
    ])
    actions = [
        _include(
            'track_robot_bringup',
            'semantic_search_sensors.launch.py',
            {
                'start_camera': 'true',
                'start_lidar': 'true',
                'start_base': 'false',
                'start_imu': 'false',
                'configure_network':
                    LaunchConfiguration('configure_network'),
                'network_interface':
                    LaunchConfiguration('network_interface'),
                'host_ip': LaunchConfiguration('host_ip'),
                'host_cidr': LaunchConfiguration('host_cidr'),
                'driver_start_delay':
                    LaunchConfiguration('driver_start_delay'),
                'publish_base_lidar_tf': 'true',
                'lidar_config_path':
                    LaunchConfiguration('lidar_config_path'),
                'extrinsic_mode': LaunchConfiguration('extrinsic_mode'),
                'extrinsic_file': LaunchConfiguration('extrinsic_file'),
                'allow_degraded': 'true',
                'camera_depth_mode': 'PERFORMANCE',
            },
        ),
        Node(
            package='track_robot_semantic_search',
            executable='semantic_search_phase4a_fixed_base',
            name='phase4a_fixed_base_session',
            output='screen',
            parameters=[search_config],
        ),
        _include(
            'track_robot_semantic_search',
            'semantic_search_yolo_world.launch.py',
            {
                'config_file': PathJoinSubstitution([
                    FindPackageShare('track_robot_semantic_search'),
                    'config',
                    'semantic_search_yolo_world.yaml',
                ]),
                'start_perception': 'true',
                'runtime_path':
                    LaunchConfiguration('yolo_runtime_path').perform(context),
                'clip_runtime_path':
                    LaunchConfiguration('clip_runtime_path').perform(context),
                'world_checkpoint':
                    LaunchConfiguration('yolo_checkpoint').perform(context),
                'clip_checkpoint':
                    LaunchConfiguration('clip_checkpoint').perform(context),
                'dino_local_repo':
                    LaunchConfiguration('dino_repo_path').perform(context),
                'dino_checkpoint':
                    LaunchConfiguration('dino_checkpoint').perform(context),
                'dino_enabled': LaunchConfiguration('dino_enabled'),
            },
        ),
        Node(
            package='track_robot_semantic_search',
            executable='semantic_search_spatial_observation',
            name='semantic_depth_enricher',
            output='screen',
            parameters=[search_config],
        ),
        _include(
            'track_robot_lidar_tracking',
            'semantic_memory_lidar_tracklets.launch.py',
            {
                'config_file': PathJoinSubstitution([
                    FindPackageShare('track_robot_lidar_tracking'),
                    'config',
                    'semantic_memory_lidar_tracklets.yaml',
                ]),
            },
        ),
        _include(
            'track_robot_semantic_memory',
            'semantic_memory_phase2.launch.py',
            {
                'config_file': memory_config,
                'enable_test_camera_attachment': 'true',
                'allow_degraded_calibration': 'true',
                # Phase 4B only visualizes the selected target and approach;
                # drawing every memory object creates irrelevant box clutter.
                'start_visualizer': 'false',
            },
        ),
        Node(
            package='track_robot_safety',
            executable='local_obstacle_map_node',
            name='local_obstacle_map_node',
            output='screen',
            parameters=[obstacle_config],
        ),
        Node(
            package='track_robot_semantic_search',
            executable='semantic_search_phase4a_selector',
            name='phase4a_target_selector',
            output='screen',
            parameters=[search_config],
        ),
        Node(
            package='track_robot_semantic_search',
            executable='semantic_search_phase4_planner',
            name='phase4_approach_planner',
            output='screen',
            parameters=[search_config],
        ),
        Node(
            package='track_robot_semantic_search',
            executable='semantic_search_phase4a_advisor',
            name='phase4a_advisor',
            output='screen',
            parameters=[search_config],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='semantic_search_phase4a_rviz',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('track_robot_bringup'),
                    'rviz',
                    'semantic_search_phase4.rviz',
                ]),
            ],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument('driver_start_delay', default_value='1.0'),
        DeclareLaunchArgument('extrinsic_mode', default_value='prototype'),
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument('dino_enabled', default_value='false'),
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
        OpaqueFunction(function=_launch_runtime),
    ])
