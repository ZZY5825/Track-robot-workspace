from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    configure_network = LaunchConfiguration('configure_network')
    network_interface = LaunchConfiguration('network_interface')
    host_ip = LaunchConfiguration('host_ip')
    host_cidr = LaunchConfiguration('host_cidr')
    config_path = LaunchConfiguration('config_path')

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
        parameters=[{'config_path': config_path}],
    )

    base_to_rslidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_rslidar_tf',
        arguments=[
            '0', '0', '0.7',
            '0', '0', '0',
            'base_link', 'rslidar'
        ]
    )

    delayed_lidar_start = TimerAction(
        period=1.0,
        actions=[
            rslidar_node,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('configure_network', default_value='true'),
        DeclareLaunchArgument('network_interface', default_value='eth0'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('host_cidr', default_value='24'),
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_bringup'),
                'config',
                'rslidar_track_robot.yaml',
            ]),
        ),
        network_setup,
        delayed_lidar_start,
        base_to_rslidar_tf
    ])
