from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    rslidar_config_file = LaunchConfiguration('rslidar_config_file')
    configure_network = LaunchConfiguration('configure_network')
    network_interface = LaunchConfiguration('network_interface')
    host_ip = LaunchConfiguration('host_ip')
    host_cidr = LaunchConfiguration('host_cidr')

    network_setup = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            [
                'sudo -n ip link set ',
                network_interface,
                ' up && sudo -n ip addr replace ',
                host_ip,
                '/',
                host_cidr,
                ' dev ',
                network_interface,
            ],
        ],
        output='screen',
        condition=IfCondition(configure_network),
    )

    rslidar_node = Node(
        namespace='rslidar_sdk',
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': rslidar_config_file}],
        condition=IfCondition(LaunchConfiguration('start_lidar')),
    )

    delayed_lidar_start = TimerAction(
        period=1.0,
        actions=[
            rslidar_node,
        ],
    )

    imu_node = Node(
        package='track_robot_perception',
        executable='phidget_spatial_imu_node',
        name='phidget_spatial_imu',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('track_robot_perception'),
            'config',
            'phidget_imu.yaml',
        ])],
        condition=IfCondition(LaunchConfiguration('start_imu')),
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[config_file, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('fast_lio'),
            'rviz',
            'fastlio.rviz',
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_perception'),
                'config',
                'fast_lio_rshelios.yaml',
            ]),
        ),
        DeclareLaunchArgument('start_imu', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        DeclareLaunchArgument('configure_network', default_value='false'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument(
            'rslidar_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_sensor_bringup'),
                'config',
                'rslidar_track_robot.yaml',
            ]),
        ),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        network_setup,
        delayed_lidar_start,
        imu_node,
        fast_lio_node,
        rviz_node,
    ])
