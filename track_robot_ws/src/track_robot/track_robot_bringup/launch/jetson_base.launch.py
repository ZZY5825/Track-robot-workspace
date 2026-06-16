from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    bringup_dir = get_package_share_directory('track_robot_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'cmd_vel_gate.yaml')

    # 启动 bunker_base
    bunker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bunker_base'),
                'launch',
                'bunker_base.launch.py'
            )
        )
    )

    # 启动 cmd_vel_gate
    cmd_vel_gate = Node(
        package='track_robot_core',
        executable='cmd_vel_gate',
        name='cmd_vel_gate',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        bunker_launch,
        cmd_vel_gate
    ])
