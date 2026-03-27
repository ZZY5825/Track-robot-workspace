from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('track_robot_bringup')
    teleop_config = os.path.join(bringup_dir, 'config', 'keyboard_teleop.yaml')

    keyboard_teleop = Node(
        package='track_robot_teleop',
        executable='keyboard_teleop_node',
        name='keyboard_teleop_node',
        output='screen',
        parameters=[teleop_config]
    )

    return LaunchDescription([
        keyboard_teleop
    ])
