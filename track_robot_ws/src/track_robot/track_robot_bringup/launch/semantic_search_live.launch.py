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
    if stage not in {'sensors', 'phase1', 'phase2'}:
        raise RuntimeError(
            'unknown semantic-search stage {!r}; expected sensors, phase1, '
            'or phase2'.format(stage))

    actions = [
        _include(
            'track_robot_bringup',
            'semantic_search_sensors.launch.py',
            _sensor_arguments(stage),
        ),
    ]
    if stage == 'phase2':
        actions.append(_include(
            'track_robot_semantic_search',
            'semantic_search_phase0.launch.py',
            {'start_evaluator': 'false'},
        ))
    if stage in ('phase1', 'phase2'):
        actions.append(_include(
            'track_robot_semantic_search',
            'semantic_search_phase1.launch.py',
            {
                'start_perception': 'true',
                'runtime_path': LaunchConfiguration('runtime_path'),
                'checkpoint_path': LaunchConfiguration('checkpoint_path'),
            },
        ))
    if stage == 'phase2':
        actions.extend([
            _include(
                'track_robot_lidar_tracking',
                'semantic_memory_lidar_tracklets.launch.py',
                {},
            ),
            _include(
                'track_robot_semantic_memory',
                'semantic_memory_phase2.launch.py',
                {},
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
        DeclareLaunchArgument('extrinsic_mode', default_value='none'),
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument('allow_degraded', default_value='false'),
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument('driver_start_delay', default_value='1.0'),
        DeclareLaunchArgument(
            'publish_base_lidar_tf',
            default_value='true',
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
