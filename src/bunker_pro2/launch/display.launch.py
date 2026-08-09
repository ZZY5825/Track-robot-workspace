from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('bunker_pro2'))
    urdf_path = package_share / 'urdf' / 'bunker_pro2.urdf'
    rviz_path = package_share / 'rviz' / 'bunker_pro2.rviz'
    robot_description = urdf_path.read_text(encoding='utf-8')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='bunker_pro2_world_to_base_link',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0', 'world', 'base_link'
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='bunker_pro2_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='bunker_pro2_rviz2',
            output='screen',
            arguments=['-d', str(rviz_path)],
        ),
    ])
