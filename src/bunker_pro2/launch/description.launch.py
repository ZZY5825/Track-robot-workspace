from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('bunker_pro2'))
    urdf_path = package_share / 'urdf' / 'bunker_pro2.urdf'
    robot_description = urdf_path.read_text(encoding='utf-8')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='bunker_pro2_robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
