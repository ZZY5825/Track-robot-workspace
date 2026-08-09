"""One-command Phase 0-5A bounded active-search bringup.

PASSIVE_ONLY is the default.  SEARCH_SHADOW records decisions without motion.
ROTATION_SUPERVISED uses the full Phase 4B Nav2 runtime so a confirmed target
can be handed to supervised approach without restarting navigation. Finding
remains limited to Nav2 Spin through the existing safety and velocity gate.
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
            FindPackageShare(package), 'launch', launch_file,
        ])),
        launch_arguments=arguments.items(),
    )


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _validate_mode(context):
    search_mode = LaunchConfiguration('search_mode').perform(context)
    runtime_mode = LaunchConfiguration('rotation_runtime_mode').perform(
        context)
    execution = _as_bool(
        LaunchConfiguration('enable_rotation_execution').perform(context))
    supported = ('PASSIVE_ONLY', 'SEARCH_SHADOW', 'ROTATION_SUPERVISED')
    if search_mode not in supported:
        raise ValueError('unsupported Phase 5A search_mode: {}'.format(
            search_mode))
    if search_mode == 'ROTATION_SUPERVISED':
        if runtime_mode != 'SEMANTIC_ACTIVE' or not execution:
            raise ValueError(
                'ROTATION_SUPERVISED requires SEMANTIC_ACTIVE and the '
                'separate enable_rotation_execution gate')
    elif runtime_mode != 'PLANNING_ONLY' or execution:
        raise ValueError(
            '{} must remain PLANNING_ONLY with rotation execution disabled'
            .format(search_mode))
    return []


def generate_launch_description():
    search_config = PathJoinSubstitution([
        FindPackageShare('track_robot_semantic_search'),
        'config', 'semantic_search_phase5a.yaml',
    ])
    platform = _include(
        'track_robot_bringup',
        'semantic_search_platform.launch.py',
        {
            'start_base': LaunchConfiguration('start_base'),
            'start_imu': 'false',
            'base_frame': 'robot_bottom',
        },
    )
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
            'dino_enabled': LaunchConfiguration('dino_enabled'),
        },
    )
    navigation = _include(
        'track_robot_navigation',
        'phase4b_navigation.launch.py',
        {
            'runtime_mode': LaunchConfiguration('rotation_runtime_mode'),
            'enable_semantic_execution':
                LaunchConfiguration('enable_rotation_execution'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            # Phase 4A already owns this independent safety map instance.
            'start_obstacle_map': 'false',
        },
    )
    search_adapter = Node(
        package='track_robot_navigation',
        executable='search_motion_adapter',
        name='search_motion_adapter',
        output='screen',
        parameters=[LaunchConfiguration('search_motion_config')],
        condition=IfCondition(LaunchConfiguration(
            'enable_rotation_execution')),
    )
    manager = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_active_manager',
        name='active_search_manager',
        output='screen',
        parameters=[
            search_config,
            {
                # Keep these gates out of the node-specific YAML.  On Foxy,
                # exact-name YAML values override launch's /** parameters.
                'search_mode': LaunchConfiguration('search_mode'),
                'active_search_execution_enabled':
                    LaunchConfiguration('enable_rotation_execution'),
            },
        ],
    )
    overlay = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_live_overlay',
        name='semantic_search_live_overlay',
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='semantic_search_phase5a_rviz',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'rviz', 'semantic_search_phase5a.rviz',
        ])],
        condition=IfCondition(LaunchConfiguration('start_phase5a_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('search_mode', default_value='PASSIVE_ONLY'),
        DeclareLaunchArgument(
            'rotation_runtime_mode', default_value='PLANNING_ONLY'),
        DeclareLaunchArgument(
            'enable_rotation_execution', default_value='false'),
        DeclareLaunchArgument('start_base', default_value='false'),
        DeclareLaunchArgument('start_phase5a_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'search_motion_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_navigation'),
                'config', 'active_search_motion.yaml',
            ])),
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument('driver_start_delay', default_value='1.0'),
        DeclareLaunchArgument('extrinsic_mode', default_value='prototype'),
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument('dino_enabled', default_value='true'),
        DeclareLaunchArgument(
            'lidar_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_sensor_bringup'),
                'config', 'rslidar_track_robot.yaml',
            ])),
        DeclareLaunchArgument(
            'yolo_runtime_path',
            default_value=(
                '/home/track-robot/track_robot_ws/models/r0c_runtime/python')),
        DeclareLaunchArgument(
            'clip_runtime_path',
            default_value=(
                '/home/track-robot/track_robot_ws/models/phase1_runtime/python')),
        DeclareLaunchArgument(
            'yolo_checkpoint',
            default_value=(
                '/home/track-robot/track_robot_ws/models/r0c/'
                'yolov8s-worldv2.pt')),
        DeclareLaunchArgument(
            'clip_checkpoint',
            default_value=(
                '/home/track-robot/track_robot_ws/models/phase1/'
                'ViT-B-32.pt')),
        DeclareLaunchArgument(
            'dino_repo_path',
            default_value=(
                '/home/track-robot/track_robot_ws/src/track_robot_core/'
                'third_party/dinov3_py38')),
        DeclareLaunchArgument(
            'dino_checkpoint',
            default_value=(
                '/home/track-robot/track_robot_ws/models/'
                'dinov3_vits16plus_pretrain_lvd1689m.pth')),
        OpaqueFunction(function=_validate_mode),
        platform,
        phase4a,
        navigation,
        search_adapter,
        manager,
        overlay,
        rviz,
    ])
