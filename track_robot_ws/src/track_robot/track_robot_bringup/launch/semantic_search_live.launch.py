from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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


def _sensor_arguments(stage):
    requested = {
        'start_camera': LaunchConfiguration('start_camera'),
        'start_lidar': LaunchConfiguration('start_lidar'),
        'start_base': LaunchConfiguration('start_base'),
        'start_imu': LaunchConfiguration('start_imu'),
    }
    if stage == 'phase1':
        requested.update({
            'start_lidar': 'false',
            'start_base': 'false',
            'start_imu': 'false',
        })
    requested.update({
        'configure_network': LaunchConfiguration('configure_network'),
        'network_interface': LaunchConfiguration('network_interface'),
        'host_ip': LaunchConfiguration('host_ip'),
        'host_cidr': LaunchConfiguration('host_cidr'),
        'driver_start_delay': LaunchConfiguration('driver_start_delay'),
        'lidar_config_path': LaunchConfiguration('lidar_config_path'),
        'publish_base_lidar_tf':
            LaunchConfiguration('publish_base_lidar_tf'),
        'extrinsic_mode': LaunchConfiguration('extrinsic_mode'),
        'extrinsic_file': LaunchConfiguration('extrinsic_file'),
        'allow_degraded': LaunchConfiguration('allow_degraded'),
    })
    return requested


def _launch_stage(context):
    stage = LaunchConfiguration('stage').perform(context)
    if stage not in {'sensors', 'phase0', 'phase1', 'phase2', 'phase3'}:
        raise RuntimeError(
            'unknown semantic-search stage {!r}; expected sensors or '
            'phase0..phase3'.format(stage))

    actions = []
    if stage != 'phase0':
        actions.append(_include(
            'bunker_pro2',
            'description.launch.py',
            {},
        ))
        actions.append(_include(
            'track_robot_bringup',
            'semantic_search_sensors.launch.py',
            _sensor_arguments(stage),
        ))
    if stage in ('phase0', 'phase2', 'phase3'):
        actions.append(_include(
            'track_robot_semantic_search',
            'semantic_search_phase0.launch.py',
            {
                'start_evaluator': 'false',
                'config_file': PathJoinSubstitution([
                    FindPackageShare('track_robot_semantic_search'),
                    'config',
                    'semantic_search_phase0.yaml',
                ]),
            },
        ))
    if stage in ('phase1', 'phase2', 'phase3'):
        actions.append(_include(
            'track_robot_semantic_search',
            'semantic_search_yolo_world.launch.py',
            {
                'config_file': PathJoinSubstitution([
                    FindPackageShare('track_robot_semantic_search'),
                    'config',
                    'semantic_search_yolo_world.yaml',
                ]),
                'start_perception': 'true',
                # Resolve model paths before entering the nested launch.
                # ROS 2 Foxy may otherwise leak the child's ``runtime_path``
                # value into its sibling ``clip_runtime_path`` argument.
                'runtime_path':
                    LaunchConfiguration('yolo_runtime_path').perform(context),
                'clip_runtime_path':
                    LaunchConfiguration('runtime_path').perform(context),
                'world_checkpoint':
                    LaunchConfiguration(
                        'yolo_checkpoint_path').perform(context),
                'clip_checkpoint':
                    LaunchConfiguration('checkpoint_path').perform(context),
                'dino_local_repo':
                    LaunchConfiguration('dino_repo_path').perform(context),
                'dino_checkpoint':
                    LaunchConfiguration(
                        'dino_checkpoint_path').perform(context),
                'dino_enabled': 'true',
            },
        ))
    if stage in ('phase2', 'phase3'):
        memory_config = (
            'phase123_test.yaml' if stage == 'phase3'
            else 'semantic_memory.yaml')
        actions.extend([
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
                    'config_file': PathJoinSubstitution([
                        FindPackageShare('track_robot_semantic_memory'),
                        'config',
                        memory_config,
                    ]),
                    'enable_test_camera_attachment':
                        'true' if stage == 'phase3' else 'false',
                    'allow_degraded_calibration':
                        'true' if stage == 'phase3' else 'false',
                },
            ),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage', default_value='phase1'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument(
            'runtime_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/phase1_runtime/python')),
        DeclareLaunchArgument(
            'checkpoint_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/phase1/ViT-B-32.pt')),
        DeclareLaunchArgument(
            'yolo_runtime_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/r0c_runtime/python')),
        DeclareLaunchArgument(
            'yolo_checkpoint_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/r0c/yolov8s-worldv2.pt')),
        DeclareLaunchArgument(
            'dino_repo_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'src/track_robot_core/third_party/dinov3_py38')),
        DeclareLaunchArgument(
            'dino_checkpoint_path',
            default_value=(
                '/home/track-robot/track_robot_ws/'
                'models/dinov3_vits16plus_pretrain_lvd1689m.pth')),
        DeclareLaunchArgument(
            'extrinsic_mode', default_value='robot_description'),
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument('allow_degraded', default_value='false'),
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument('driver_start_delay', default_value='1.0'),
        DeclareLaunchArgument(
            'publish_base_lidar_tf',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'lidar_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_sensor_bringup'),
                'config',
                'rslidar_track_robot.yaml',
            ]),
        ),
        OpaqueFunction(function=_launch_stage),
    ])
