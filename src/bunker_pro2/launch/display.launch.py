from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = Path(get_package_share_directory('bunker_pro2'))
    rviz_path = package_share / 'rviz' / 'bunker_pro2.rviz'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('bunker_pro2'),
                    'launch',
                    'description.launch.py',
                ])
            )
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='bunker_pro2_world_to_robot_bottom',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0', 'world', 'robot_bottom'
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='bunker_pro2_rviz2',
            output='screen',
            arguments=['-d', str(rviz_path)],
        ),
    ])
