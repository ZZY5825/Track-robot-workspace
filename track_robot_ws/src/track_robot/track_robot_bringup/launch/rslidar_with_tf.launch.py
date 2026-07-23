from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    configure_network = LaunchConfiguration('configure_network')
    network_interface = LaunchConfiguration('network_interface')
    host_ip = LaunchConfiguration('host_ip')
    host_cidr = LaunchConfiguration('host_cidr')
    driver_start_delay = LaunchConfiguration('driver_start_delay')
    publish_base_lidar_tf = LaunchConfiguration('publish_base_lidar_tf')
    config_path = LaunchConfiguration('config_path')

    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('track_robot_sensor_bringup'),
            'launch',
            'rslidar_with_tf.launch.py',
        ])),
        launch_arguments={
            'configure_network': configure_network,
            'network_interface': network_interface,
            'host_ip': host_ip,
            'host_cidr': host_cidr,
            'driver_start_delay': driver_start_delay,
            'publish_base_lidar_tf': publish_base_lidar_tf,
            'config_path': config_path,
        }.items(),
    )

    return LaunchDescription([
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
            'config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_sensor_bringup'),
                'config',
                'rslidar_track_robot.yaml',
            ]),
        ),
        sensor_bringup,
    ])
