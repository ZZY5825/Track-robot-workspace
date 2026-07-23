import re
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _runtime_rslidar_config(config_path, host_ip):
    config_text = Path(config_path).read_text()
    replacement = f'host_address: {host_ip}'

    if re.search(r'^(\s*)host_address:\s*\S+', config_text, flags=re.MULTILINE):
        config_text = re.sub(
            r'^(\s*)host_address:\s*\S+',
            rf'\1{replacement}',
            config_text,
            count=1,
            flags=re.MULTILINE,
        )
    else:
        config_text = re.sub(
            r'^(\s*difop_port:\s*\S+.*)$',
            rf'\1\n      {replacement}',
            config_text,
            count=1,
            flags=re.MULTILINE,
        )

    safe_host = re.sub(r'[^A-Za-z0-9_.-]', '_', host_ip)
    runtime_path = Path(tempfile.gettempdir()) / f'rslidar_track_robot_{safe_host}.yaml'
    runtime_path.write_text(config_text)
    return str(runtime_path)


def _launch_setup(context, *args, **kwargs):
    configure_network = LaunchConfiguration('configure_network')
    network_interface = LaunchConfiguration('network_interface')
    host_ip = LaunchConfiguration('host_ip')
    host_cidr = LaunchConfiguration('host_cidr')
    config_path = LaunchConfiguration('config_path')
    driver_start_delay = LaunchConfiguration('driver_start_delay')

    host_ip_value = host_ip.perform(context)
    driver_start_delay_value = float(driver_start_delay.perform(context))
    runtime_config_path = _runtime_rslidar_config(
        config_path.perform(context),
        host_ip_value,
    )

    network_setup = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            [
                'set -e; ',
                'sudo -n ip addr flush dev ',
                network_interface,
                '; sudo -n ip addr add ',
                host_ip,
                '/',
                host_cidr,
                ' dev ',
                network_interface,
                '; sudo -n ip link set ',
                network_interface,
                ' up; ',
                'for i in $(seq 1 20); do ',
                'ip -4 addr show dev ',
                network_interface,
                ' | grep -q "inet ',
                host_ip,
                '/" && exit 0; ',
                'sleep 0.1; ',
                'done; ',
                'echo "LiDAR interface did not get expected IP ',
                host_ip,
                ' on ',
                network_interface,
                '" >&2; exit 1',
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
        parameters=[{'config_path': runtime_config_path}],
    )

    base_to_rslidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_rslidar_tf',
        arguments=[
            '0', '0', '0.7',
            '0', '0', '0',
            'base_link', 'rslidar'
        ],
        condition=IfCondition(LaunchConfiguration('publish_base_lidar_tf')),
    )

    delayed_lidar_start = TimerAction(
        period=driver_start_delay_value,
        actions=[
            rslidar_node,
        ],
    )

    def start_driver_after_successful_network_setup(event, context):
        if event.returncode != 0:
            return []
        return [delayed_lidar_start]

    driver_after_network_setup = RegisterEventHandler(
        OnProcessExit(
            target_action=network_setup,
            on_exit=start_driver_after_successful_network_setup,
        ),
        condition=IfCondition(configure_network),
    )

    driver_without_network_setup = TimerAction(
        period=driver_start_delay_value,
        actions=[rslidar_node],
        condition=UnlessCondition(configure_network),
    )

    return [
        network_setup,
        driver_after_network_setup,
        driver_without_network_setup,
        base_to_rslidar_tf,
    ]


def generate_launch_description():
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
        OpaqueFunction(function=_launch_setup),
    ])
