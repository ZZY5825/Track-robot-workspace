from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'launch',
            'semantic_search_camera.launch.py',
        ])),
        launch_arguments={
            'start_camera': LaunchConfiguration('start_camera'),
            'extrinsic_mode': LaunchConfiguration('extrinsic_mode'),
            'extrinsic_file': LaunchConfiguration('extrinsic_file'),
            'allow_degraded': LaunchConfiguration('allow_degraded'),
            'depth_mode': LaunchConfiguration('camera_depth_mode'),
        }.items(),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'launch',
            'rslidar_with_tf.launch.py',
        ])),
        launch_arguments={
            'configure_network': LaunchConfiguration('configure_network'),
            'network_interface': LaunchConfiguration('network_interface'),
            'host_ip': LaunchConfiguration('host_ip'),
            'host_cidr': LaunchConfiguration('host_cidr'),
            'driver_start_delay': LaunchConfiguration('driver_start_delay'),
            'config_path': LaunchConfiguration('lidar_config_path'),
            'publish_base_lidar_tf':
                LaunchConfiguration('publish_base_lidar_tf'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar')),
    )
    platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_bringup'),
            'launch',
            'semantic_search_platform.launch.py',
        ])),
        launch_arguments={
            'start_base': LaunchConfiguration('start_base'),
            'start_imu': LaunchConfiguration('start_imu'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
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
        DeclareLaunchArgument('extrinsic_mode', default_value='none'),
        DeclareLaunchArgument('extrinsic_file', default_value=''),
        DeclareLaunchArgument('allow_degraded', default_value='false'),
        DeclareLaunchArgument('camera_depth_mode', default_value='NONE'),
        camera,
        lidar,
        platform,
    ])
